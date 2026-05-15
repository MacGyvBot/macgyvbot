#!/usr/bin/env python3
from __future__ import annotations

import json
from pathlib import Path
from typing import Optional

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from macgyvbot.config.hand_grasp import (
    HAND_GRASP_LOCK_ON_STATUS,
    HAND_GRASP_ML_CONFIDENCE,
)
from macgyvbot.config.models import (
    HAND_GRASP_MODEL_NAME,
    HAND_GRASP_SAM_CHECKPOINT_NAME,
)
from macgyvbot.config.topics import (
    HAND_GRASP_MASK_LOCK_TOPIC,
    ROBOT_STATUS_TOPIC,
)
from macgyvbot.perception.hand_grasp.hand_detector import (
    HandDetector,
)
from macgyvbot.perception.hand_grasp.hand_center import (
    extract_hand_center_pixel,
)
from macgyvbot.perception.hand_grasp.tool_detector import (
    DEFAULT_MODEL_PATH,
    DEFAULT_TOOL_CLASSES,
    ToolDetection,
    ToolDetector,
)
from macgyvbot.perception.hand_grasp.calculations import (
    build_depth_grasp_info,
    depth_to_mm,
    select_active_hand,
)
from macgyvbot.perception.hand_grasp.ml_grasp_classifier import (
    MLHandGraspClassifier,
    MLGraspResult,
    disabled_ml_result,
)
from macgyvbot.perception.hand_grasp.sam_tool_mask import (
    BBoxPromptSegmenter,
    LockedToolMask,
    compute_mask_contact,
    create_bbox_locked_mask,
)
from macgyvbot.perception.hand_grasp.visualization import (
    draw_grasp_overlay,
)


class HandGraspDetectionNode(Node):
    """Detect whether a human hand is grasping a detected tool."""

    def __init__(self) -> None:
        super().__init__("hand_grasp_detection_node")

        self.bridge = CvBridge()
        self.latest_depth_mm = None
        self.last_state = None
        self.lock_requested = False
        self.locked_tool: Optional[LockedToolMask] = None
        self.latest_tool_detection: Optional[ToolDetection] = None
        self.latest_tool_mask: Optional[LockedToolMask] = None
        self.mask_tracking_active = False
        self.frame_index = 0

        self.color_topic = self.declare_parameter(
            "color_topic",
            "/camera/camera/color/image_raw",
        ).value
        self.depth_topic = self.declare_parameter(
            "depth_topic",
            "/camera/camera/aligned_depth_to_color/image_raw",
        ).value
        self.result_topic = self.declare_parameter(
            "result_topic",
            "/human_grasped_tool",
        ).value
        self.annotated_topic = self.declare_parameter(
            "annotated_topic",
            "/hand_grasp_detection/annotated_image",
        ).value
        self.use_depth = self._as_bool(self.declare_parameter("use_depth", True).value)
        self.publish_annotated = bool(
            self._as_bool(self.declare_parameter("publish_annotated", True).value)
        )
        self.display = self._as_bool(self.declare_parameter("display", False).value)
        self.robot_status_topic = self.declare_parameter(
            "robot_status_topic",
            ROBOT_STATUS_TOPIC,
        ).value
        self.mask_lock_topic = self.declare_parameter(
            "mask_lock_topic",
            HAND_GRASP_MASK_LOCK_TOPIC,
        ).value
        self.lock_on_status = str(
            self.declare_parameter(
                "lock_on_status",
                HAND_GRASP_LOCK_ON_STATUS,
            ).value
        )
        self.ml_min_confidence = float(
            self.declare_parameter(
                "ml_min_confidence",
                HAND_GRASP_ML_CONFIDENCE,
            ).value
        )
        self.grasp_model_path = str(
            self.declare_parameter(
                "grasp_model",
                str(Path("weights") / HAND_GRASP_MODEL_NAME),
            ).value
        )
        self.sam_enabled = self._as_bool(
            self.declare_parameter("sam_enabled", False).value
        )
        self.sam_backend = str(self.declare_parameter("sam_backend", "mobile_sam").value)
        self.sam_checkpoint = str(
            self.declare_parameter(
                "sam_checkpoint",
                str(Path("weights") / HAND_GRASP_SAM_CHECKPOINT_NAME),
            ).value
        )
        self.sam_model_type = str(self.declare_parameter("sam_model_type", "vit_t").value)
        self.sam_device = str(self.declare_parameter("sam_device", "cuda").value)
        self.sam_track_interval = int(
            self.declare_parameter("sam_track_interval", 10).value
        )
        self.sam_track_margin = int(
            self.declare_parameter("sam_track_margin", 12).value
        )
        self.allow_bbox_lock = self._as_bool(
            self.declare_parameter("allow_bbox_lock", True).value
        )
        self.mask_contact_radius = int(
            self.declare_parameter("mask_contact_radius", 6).value
        )
        self.mask_min_contact_landmarks = int(
            self.declare_parameter("mask_min_contact_landmarks", 2).value
        )
        self.mask_proximity_threshold = float(
            self.declare_parameter("mask_proximity_threshold", 45.0).value
        )

        yolo_model = str(
            self.declare_parameter("yolo_model", DEFAULT_MODEL_PATH).value
        )
        tool_classes = str(
            self.declare_parameter(
                "tool_classes",
                ",".join(DEFAULT_TOOL_CLASSES),
            ).value
        )
        yolo_conf = float(self.declare_parameter("yolo_conf", 0.20).value)
        yolo_imgsz = int(self.declare_parameter("yolo_imgsz", 640).value)
        max_hands = int(self.declare_parameter("max_hands", 2).value)
        self.depth_diff_threshold_mm = float(
            self.declare_parameter("depth_diff_threshold_mm", 50.0).value
        )
        self.depth_min_contact_landmarks = int(
            self.declare_parameter("depth_min_contact_landmarks", 2).value
        )

        self.hand_detector = HandDetector(max_num_hands=max_hands)
        self.tool_detector = self._create_tool_detector(
            model_path=yolo_model,
            tool_classes=tool_classes,
            confidence_threshold=yolo_conf,
            image_size=yolo_imgsz,
        )
        self.ml_classifier = self._create_ml_classifier()
        self.sam_segmenter = self._create_sam_segmenter()

        self.result_pub = self.create_publisher(String, self.result_topic, 10)
        self.mask_lock_pub = self.create_publisher(String, self.mask_lock_topic, 10)
        self.annotated_pub = (
            self.create_publisher(Image, self.annotated_topic, 10)
            if self.publish_annotated
            else None
        )

        if self.use_depth:
            self.create_subscription(Image, self.depth_topic, self._depth_cb, 10)
            self.get_logger().info(f"Depth recognition enabled: {self.depth_topic}")
        else:
            self.get_logger().warn("Depth recognition disabled by parameter.")

        self.create_subscription(Image, self.color_topic, self._color_cb, 10)
        self.create_subscription(String, self.robot_status_topic, self._robot_status_cb, 10)
        self.get_logger().info(f"Color image topic: {self.color_topic}")
        self.get_logger().info(f"Result topic: {self.result_topic}")
        self.get_logger().info(f"Robot status topic: {self.robot_status_topic}")
        self.get_logger().info(f"Tool mask lock topic: {self.mask_lock_topic}")

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, str):
            return value.strip().lower() in {"1", "true", "yes", "on"}
        return bool(value)

    def _create_tool_detector(
        self,
        model_path: str,
        tool_classes: str,
        confidence_threshold: float,
        image_size: int,
    ) -> Optional[ToolDetector]:
        target_classes = [
            name.strip()
            for name in tool_classes.split(",")
            if name.strip()
        ]
        try:
            detector = ToolDetector(
                model_path=model_path,
                target_classes=target_classes,
                confidence_threshold=confidence_threshold,
                image_size=image_size,
            )
        except Exception as exc:
            self.get_logger().error(f"YOLO tool detector init failed: {exc}")
            return None

        self.get_logger().info(
            f"YOLO tool detector enabled: model={detector.model_path}, "
            f"classes={target_classes or 'ANY'}"
        )
        return detector

    def _create_ml_classifier(self) -> Optional[MLHandGraspClassifier]:
        try:
            classifier = MLHandGraspClassifier(self.grasp_model_path)
        except Exception as exc:
            self.get_logger().error(f"ML grasp classifier init failed: {exc}")
            return None

        self.get_logger().info(f"ML grasp classifier enabled: {classifier.path}")
        return classifier

    def _create_sam_segmenter(self) -> Optional[BBoxPromptSegmenter]:
        if not self.sam_enabled:
            self.get_logger().info("SAM tool mask disabled. Using bbox lock fallback.")
            return None

        try:
            segmenter = BBoxPromptSegmenter(
                backend=self.sam_backend,
                checkpoint_path=self.sam_checkpoint,
                model_type=self.sam_model_type,
                device=self.sam_device,
            )
        except Exception as exc:
            self.get_logger().error(f"SAM tool mask init failed: {exc}")
            return None

        self.get_logger().info(
            f"SAM tool mask enabled: backend={self.sam_backend}, "
            f"checkpoint={self.sam_checkpoint}"
        )
        return segmenter

    def _robot_status_cb(self, msg: String) -> None:
        try:
            status = json.loads(msg.data).get("status")
        except json.JSONDecodeError:
            self.get_logger().warn(f"Robot status JSON parse failed: {msg.data}")
            return

        if status == self.lock_on_status:
            self.lock_requested = True
            self.mask_tracking_active = False
            self.locked_tool = None
            if self.ml_classifier is not None:
                self.ml_classifier.reset()
            self.get_logger().info("Robot grasp success received. Tool mask lock requested.")
            return

        if status in {"accepted", "searching", "picking", "grasping"}:
            if status == "accepted":
                self.lock_requested = False
                self.locked_tool = None
                self.latest_tool_detection = None
                self.latest_tool_mask = None
            self.mask_tracking_active = True
            return

        if status in {"returned", "done", "failed", "cancelled"}:
            self.lock_requested = False
            self.locked_tool = None
            self.latest_tool_detection = None
            self.latest_tool_mask = None
            self.mask_tracking_active = False
            if self.ml_classifier is not None:
                self.ml_classifier.reset()

    def _depth_cb(self, msg: Image) -> None:
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        except Exception as exc:
            self.get_logger().warn(f"Depth image conversion failed: {exc}")
            return

        self.latest_depth_mm = depth_to_mm(depth_image, msg.encoding)

    def _color_cb(self, msg: Image) -> None:
        self.frame_index += 1
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as exc:
            self.get_logger().warn(f"Color image conversion failed: {exc}")
            return

        tool_detection = (
            self.tool_detector.detect(frame)
            if self.tool_detector is not None
            else None
        )
        if tool_detection is not None:
            self.latest_tool_detection = tool_detection
            if self.mask_tracking_active:
                self._update_prelock_tool_mask(frame, tool_detection)
        elif self.mask_tracking_active and self.latest_tool_mask is not None:
            self._track_prelock_tool_mask(frame)

        if self.lock_requested and self.locked_tool is None:
            self.locked_tool = self._lock_tool_mask(
                frame,
                tool_detection or self.latest_tool_detection,
            )
            self._publish_mask_lock_result(self.locked_tool)

        tool_roi = (
            self.locked_tool.roi
            if self.locked_tool is not None
            else tool_detection.roi if tool_detection is not None else None
        )
        hand_infos = self.hand_detector.detect_all(frame)
        active_hand = select_active_hand(hand_infos, tool_roi)
        hand_for_state = active_hand if active_hand is not None else (
            hand_infos[0] if hand_infos else None
        )
        depth_info = self._build_depth_info(hand_for_state, tool_roi)
        mask_result = compute_mask_contact(
            hand_info=hand_for_state,
            locked_tool=self.locked_tool,
            contact_radius=self.mask_contact_radius,
            min_contact_landmarks=self.mask_min_contact_landmarks,
            proximity_threshold=self.mask_proximity_threshold,
        )
        ml_result = self._update_ml_grasp(hand_for_state)
        result = self._compose_result(
            hand_info=hand_for_state,
            tool_roi=tool_roi,
            depth_info=depth_info,
            mask_result=mask_result,
            ml_result=ml_result,
        )

        self._publish_result(result, tool_detection, active_hand, hand_infos)

        if self.publish_annotated or self.display:
            annotated = frame.copy()
            draw_grasp_overlay(
                annotated,
                hand_infos,
                active_hand,
                tool_detection,
                result,
                self.locked_tool,
            )
            if self.annotated_pub is not None:
                self.annotated_pub.publish(
                    self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
                )
            if self.display:
                cv2.imshow("Hand Grasp Detection", annotated)
                cv2.waitKey(1)

        if result["state"] != self.last_state:
            self.get_logger().info(
                "grasp state={state}, human_grasped_tool={human_grasped_tool}, "
                "ml={ml_stable_state}/{ml_raw_state}, mask={mask_source}, "
                "mask_contact_count={mask_contact_count}".format(
                    **result
                )
            )
            self.last_state = result["state"]

    def _build_depth_info(self, hand_info: Optional[dict], tool_roi):
        if not self.use_depth:
            return None
        if self.latest_depth_mm is None or hand_info is None or tool_roi is None:
            return None

        return build_depth_grasp_info(
            hand_info=hand_info,
            tool_roi=tool_roi,
            depth_mm=self.latest_depth_mm,
            depth_diff_threshold_mm=self.depth_diff_threshold_mm,
            min_depth_contact_landmarks=self.depth_min_contact_landmarks,
        )

    def _update_prelock_tool_mask(self, frame, tool_detection: ToolDetection) -> None:
        if self.locked_tool is not None:
            return

        if self.sam_segmenter is None:
            self.latest_tool_mask = create_bbox_locked_mask(
                tool_detection.roi,
                frame.shape[:2],
            )
            return

        if self.frame_index % max(1, self.sam_track_interval) != 0:
            if self.latest_tool_mask is None:
                self.latest_tool_mask = create_bbox_locked_mask(
                    tool_detection.roi,
                    frame.shape[:2],
                )
            return

        mask = self.sam_segmenter.segment(frame, tool_detection.roi)
        if mask is not None and int(mask.sum()) > 0:
            self.latest_tool_mask = LockedToolMask(
                roi=tool_detection.roi,
                mask=mask,
                source="SAM_TRACKED",
            )

    def _track_prelock_tool_mask(self, frame) -> None:
        if self.locked_tool is not None or self.sam_segmenter is None:
            return

        if self.frame_index % max(1, self.sam_track_interval) != 0:
            return

        tracked = self.sam_segmenter.track_from_mask(
            frame,
            self.latest_tool_mask,
            margin=self.sam_track_margin,
        )
        if tracked is not None:
            self.latest_tool_mask = tracked

    def _lock_tool_mask(
        self,
        frame,
        tool_detection: Optional[ToolDetection],
    ) -> Optional[LockedToolMask]:
        if self.latest_tool_mask is not None:
            return LockedToolMask(
                roi=self.latest_tool_mask.roi,
                mask=self.latest_tool_mask.mask.copy(),
                source=(
                    "SAM_LOCKED"
                    if self.latest_tool_mask.source == "SAM_TRACKED"
                    else self.latest_tool_mask.source
                ),
            )

        if tool_detection is None:
            self.get_logger().warn("Tool mask lock requested, but no YOLO tool ROI is available.")
            return None

        if self.sam_segmenter is not None:
            mask = self.sam_segmenter.segment(frame, tool_detection.roi)
            if mask is not None and int(mask.sum()) > 0:
                self.get_logger().info(
                    f"Locked SAM tool mask: label={tool_detection.label}, "
                    f"roi={tool_detection.roi}"
                )
                return LockedToolMask(
                    roi=tool_detection.roi,
                    mask=mask,
                    source="SAM_LOCKED",
                )
            self.get_logger().warn("SAM returned an empty tool mask; falling back if allowed.")

        if self.allow_bbox_lock:
            locked = create_bbox_locked_mask(tool_detection.roi, frame.shape[:2])
            self.get_logger().info(
                f"Locked bbox tool mask: label={tool_detection.label}, roi={locked.roi}"
            )
            return locked

        return None

    def _publish_mask_lock_result(self, locked_tool: Optional[LockedToolMask]) -> None:
        payload = {
            "locked": locked_tool is not None,
            "mask_source": locked_tool.source if locked_tool is not None else "NONE",
            "tool_roi": locked_tool.roi if locked_tool is not None else None,
        }
        if locked_tool is None:
            payload["reason"] = "tool_mask_unavailable"

        self.mask_lock_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _update_ml_grasp(self, hand_info: Optional[dict]) -> MLGraspResult:
        if self.ml_classifier is None:
            return disabled_ml_result("model_unavailable")

        try:
            return self.ml_classifier.update(hand_info)
        except Exception as exc:
            self.get_logger().warn(f"ML grasp update failed: {exc}")
            self.ml_classifier.reset()
            return disabled_ml_result("model_error")

    def _compose_result(
        self,
        hand_info: Optional[dict],
        tool_roi,
        depth_info: Optional[dict],
        mask_result,
        ml_result: MLGraspResult,
    ) -> dict:
        confidence_ok = (
            ml_result.enabled
            and (
                ml_result.confidence is None
                or ml_result.confidence >= self.ml_min_confidence
            )
        )
        mask_available = self.locked_tool is not None
        mask_near_or_contact = mask_result.near_or_contact if mask_available else False
        locked_mask_grasp_ok = bool(mask_available and mask_result.mask_contact_confirmed)
        depth_info = depth_info or {}
        depth_grasp_ok = bool(depth_info.get("depth_grasp_confirmed", False))
        human_grasped_tool = bool(
            ml_result.is_grasp
            and confidence_ok
            and depth_grasp_ok
            and locked_mask_grasp_ok
        )

        result = {
            "state": self._combined_grasp_state(
                hand_info=hand_info,
                tool_roi=tool_roi,
                ml_result=ml_result,
                confidence_ok=confidence_ok,
                depth_grasp_ok=depth_grasp_ok,
                locked_mask_grasp_ok=locked_mask_grasp_ok,
                human_grasped_tool=human_grasped_tool,
            ),
            "human_grasped_tool": human_grasped_tool,
            "grasp_counter": 1 if human_grasped_tool else 0,
            "grasp_score": 1 if human_grasped_tool else 0,
            "depth_available": depth_info.get("depth_available", False),
            "tool_depth_mm": depth_info.get("tool_depth_mm"),
            "min_hand_tool_depth_diff_mm": depth_info.get("min_hand_tool_depth_diff_mm"),
            "depth_contact_count": depth_info.get("depth_contact_count", 0),
            "depth_grasp_confirmed": depth_info.get("depth_grasp_confirmed", False),
            "min_landmark_to_tool_distance": mask_result.min_landmark_to_tool_distance,
        }
        result["ml_raw_state"] = ml_result.raw_state
        result["ml_stable_state"] = ml_result.stable_state
        result["ml_confidence"] = ml_result.confidence
        result["ml_grasp_confirmed"] = ml_result.is_grasp
        result["ml_confidence_ok"] = confidence_ok
        result["ml_required"] = True
        result["depth_required"] = True
        result["depth_grasp_ok"] = depth_grasp_ok
        result["mask_source"] = self.locked_tool.source if self.locked_tool else "NONE"
        result["mask_locked"] = mask_available
        result["mask_contact_count"] = mask_result.mask_contact_count
        result["mask_contact_confirmed"] = mask_result.mask_contact_confirmed
        result["mask_proximity_ok"] = mask_result.mask_proximity_ok
        result["mask_near_or_contact"] = mask_near_or_contact
        result["locked_mask_grasp_ok"] = locked_mask_grasp_ok

        return result

    @staticmethod
    def _combined_grasp_state(
        hand_info: Optional[dict],
        tool_roi,
        ml_result: MLGraspResult,
        confidence_ok: bool,
        depth_grasp_ok: bool,
        locked_mask_grasp_ok: bool,
        human_grasped_tool: bool,
    ) -> str:
        if human_grasped_tool:
            return "HUMAN_GRASPED_TOOL"
        if not ml_result.enabled:
            return "ML_GRASP_UNAVAILABLE"
        if hand_info is None:
            return "NO_HAND"
        if tool_roi is None:
            return "TOOL_NOT_DETECTED"
        if ml_result.is_grasp and not confidence_ok:
            return "ML_GRASP_LOW_CONF"
        if not ml_result.is_grasp:
            if ml_result.stable_state == "grasp":
                return "ML_GRASP_BUFFER_NOT_RAW"
            if ml_result.raw_state == "open" or ml_result.stable_state == "open":
                return "ML_OPEN_HAND"
            return "ML_NOT_GRASP"
        if not depth_grasp_ok:
            return "DEPTH_GRASP_NOT_CONFIRMED"
        if not locked_mask_grasp_ok:
            return "LOCKED_MASK_GRASP_NOT_CONFIRMED"
        if ml_result.stable_state == "grasp":
            return "ML_GRASP_BUFFER_NOT_RAW"
        return "ML_NOT_GRASP"

    def _publish_result(
        self,
        result: dict,
        tool_detection: Optional[ToolDetection],
        active_hand: Optional[dict],
        hand_infos: list[dict],
    ) -> None:
        hand_pixel = extract_hand_center_pixel(active_hand, hand_infos)
        payload = {
            "state": result["state"],
            "human_grasped_tool": result["human_grasped_tool"],
            "grasp_counter": result["grasp_counter"],
            "grasp_score": result["grasp_score"],
            "depth_available": result["depth_available"],
            "tool_depth_mm": result["tool_depth_mm"],
            "min_hand_tool_depth_diff_mm": result["min_hand_tool_depth_diff_mm"],
            "depth_contact_count": result["depth_contact_count"],
            "depth_grasp_confirmed": result["depth_grasp_confirmed"],
            "ml_raw_state": result["ml_raw_state"],
            "ml_stable_state": result["ml_stable_state"],
            "ml_confidence": result["ml_confidence"],
            "ml_grasp_confirmed": result["ml_grasp_confirmed"],
            "ml_confidence_ok": result["ml_confidence_ok"],
            "ml_required": result["ml_required"],
            "depth_required": result["depth_required"],
            "depth_grasp_ok": result["depth_grasp_ok"],
            "mask_source": result["mask_source"],
            "mask_locked": result["mask_locked"],
            "mask_contact_count": result["mask_contact_count"],
            "mask_contact_confirmed": result["mask_contact_confirmed"],
            "mask_proximity_ok": result["mask_proximity_ok"],
            "mask_near_or_contact": result["mask_near_or_contact"],
            "locked_mask_grasp_ok": result["locked_mask_grasp_ok"],
            "min_landmark_to_tool_distance": result["min_landmark_to_tool_distance"],
            "active_hand_index": active_hand["hand_index"] if active_hand else None,
            "active_handedness": active_hand["handedness"] if active_hand else None,
            "tool_label": tool_detection.label if tool_detection else None,
            "tool_confidence": tool_detection.confidence if tool_detection else None,
            "tool_roi": (
                self.locked_tool.roi
                if self.locked_tool is not None
                else tool_detection.roi if tool_detection else None
            ),
            "hand_pixel": hand_pixel,
        }
        self.result_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def destroy_node(self) -> bool:
        self.hand_detector.close()
        if self.display:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = HandGraspDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
