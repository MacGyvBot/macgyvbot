#!/usr/bin/env python3
from __future__ import annotations

import json
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .grasp_detection.grasp_detector import GraspDetector
from .grasp_detection.hand_detector import HandDetector
from .grasp_detection.tool_detector import (
    DEFAULT_MODEL_PATH,
    DEFAULT_TOOL_CLASSES,
    ToolDetection,
    ToolDetector,
)
from .grasp_detection.utils import build_depth_grasp_info, draw_text, point_to_rect_distance


class HandGraspDetectionNode(Node):
    """Detect whether a human hand is grasping a detected tool."""

    def __init__(self) -> None:
        super().__init__("hand_grasp_detection_node")

        self.bridge = CvBridge()
        self.latest_depth_mm = None
        self.last_state = None

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
        self.use_depth = bool(self.declare_parameter("use_depth", True).value)
        self.publish_annotated = bool(
            self.declare_parameter("publish_annotated", True).value
        )
        self.display = bool(self.declare_parameter("display", False).value)

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
        self.grasp_detector = GraspDetector()
        self.tool_detector = self._create_tool_detector(
            model_path=yolo_model,
            tool_classes=tool_classes,
            confidence_threshold=yolo_conf,
            image_size=yolo_imgsz,
        )

        self.result_pub = self.create_publisher(String, self.result_topic, 10)
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
        self.get_logger().info(f"Color image topic: {self.color_topic}")
        self.get_logger().info(f"Result topic: {self.result_topic}")

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
        resolved_model_path = self._resolve_model_path(model_path)

        try:
            detector = ToolDetector(
                model_path=resolved_model_path,
                target_classes=target_classes,
                confidence_threshold=confidence_threshold,
                image_size=image_size,
            )
        except Exception as exc:
            self.get_logger().error(f"YOLO tool detector init failed: {exc}")
            return None

        self.get_logger().info(
            f"YOLO tool detector enabled: model={resolved_model_path}, "
            f"classes={target_classes or 'ANY'}"
        )
        return detector

    def _resolve_model_path(self, model_path: str) -> str:
        path = Path(model_path).expanduser()
        if path.exists() or path.is_absolute():
            return str(path)

        package_share = Path(get_package_share_directory("macgyvbot"))
        candidates = [
            package_share / "models" / model_path,
            package_share / model_path,
        ]
        for candidate in candidates:
            if candidate.exists():
                return str(candidate)

        return model_path

    def _depth_cb(self, msg: Image) -> None:
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        except Exception as exc:
            self.get_logger().warn(f"Depth image conversion failed: {exc}")
            return

        self.latest_depth_mm = self._depth_to_mm(depth_image, msg.encoding)

    def _color_cb(self, msg: Image) -> None:
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
        tool_roi = tool_detection.roi if tool_detection is not None else None
        hand_infos = self.hand_detector.detect_all(frame)
        active_hand = self._select_active_hand(hand_infos, tool_roi)
        hand_for_state = active_hand if active_hand is not None else (
            hand_infos[0] if hand_infos else None
        )
        depth_info = self._build_depth_info(hand_for_state, tool_roi)
        result = self.grasp_detector.update(hand_for_state, tool_roi, depth_info)

        self._publish_result(result, tool_detection, active_hand)

        if self.publish_annotated or self.display:
            annotated = frame.copy()
            self._draw_overlay(annotated, hand_infos, active_hand, tool_detection, result)
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
                "depth_available={depth_available}, depth_contact_count={depth_contact_count}".format(
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

    def _publish_result(
        self,
        result: dict,
        tool_detection: Optional[ToolDetection],
        active_hand: Optional[dict],
    ) -> None:
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
            "active_hand_index": active_hand["hand_index"] if active_hand else None,
            "active_handedness": active_hand["handedness"] if active_hand else None,
            "tool_label": tool_detection.label if tool_detection else None,
            "tool_confidence": tool_detection.confidence if tool_detection else None,
            "tool_roi": tool_detection.roi if tool_detection else None,
        }
        self.result_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _draw_overlay(
        self,
        frame,
        hand_infos: list[dict],
        active_hand: Optional[dict],
        tool_detection: Optional[ToolDetection],
        result: dict,
    ) -> None:
        if tool_detection is not None:
            x1, y1, x2, y2 = tool_detection.roi
            color = (0, 255, 0) if result["human_grasped_tool"] else (255, 120, 0)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            draw_text(
                frame,
                f"{tool_detection.label} {tool_detection.confidence:.2f}",
                (x1, max(24, y1 - 10)),
                color,
                scale=0.55,
            )

        active_hand_index = active_hand["hand_index"] if active_hand else None
        for hand_info in hand_infos:
            is_active = hand_info["hand_index"] == active_hand_index
            point_color = (0, 255, 255) if is_active else (255, 200, 0)
            for point in hand_info["landmarks"].values():
                cv2.circle(frame, point, 4 if is_active else 3, point_color, -1)

        draw_text(frame, f"State: {result['state']}", (20, 32))
        draw_text(
            frame,
            f"human_grasped_tool: {result['human_grasped_tool']}",
            (20, 62),
            scale=0.6,
        )
        draw_text(
            frame,
            f"depth_contact: {result['depth_contact_count']}",
            (20, 92),
            scale=0.6,
        )

    @staticmethod
    def _select_active_hand(hand_infos: list[dict], tool_roi) -> Optional[dict]:
        if not hand_infos or tool_roi is None:
            return None

        return min(
            hand_infos,
            key=lambda hand_info: point_to_rect_distance(
                hand_info["palm_center"],
                tool_roi,
            ),
        )

    @staticmethod
    def _depth_to_mm(depth_image, encoding: str):
        depth = np.asarray(depth_image)
        if encoding == "32FC1":
            depth = np.nan_to_num(depth.astype(np.float32), nan=0.0)
            return depth * 1000.0

        if np.issubdtype(depth.dtype, np.floating):
            depth = np.nan_to_num(depth.astype(np.float32), nan=0.0)
            return depth * 1000.0 if float(np.nanmax(depth)) < 20.0 else depth

        return depth.astype(np.float32)

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
