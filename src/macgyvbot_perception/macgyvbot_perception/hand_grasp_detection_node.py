#!/usr/bin/env python3
from __future__ import annotations

import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from macgyvbot_config.hand_grasp import (
    HAND_GRASP_ALLOW_BBOX_LOCK,
    HAND_GRASP_DEPTH_DIFF_THRESHOLD_MM,
    HAND_GRASP_DEPTH_LOCK_DURATION_SEC,
    HAND_GRASP_DEPTH_LOCK_MAX_AREA_RATIO,
    HAND_GRASP_DEPTH_LOCK_MIN_AREA_RATIO,
    HAND_GRASP_DEPTH_LOCK_MIN_FRAMES,
    HAND_GRASP_DEPTH_LOCK_MIN_VALID_RATIO,
    HAND_GRASP_DEPTH_LOCK_STABILITY_MM,
    HAND_GRASP_DEPTH_MIN_CONTACT_LANDMARKS,
    HAND_GRASP_DEPTH_TOOL_MASK_ENABLED,
    HAND_GRASP_DEPTH_TOOL_MASK_MARGIN,
    HAND_GRASP_DEPTH_TOOL_MASK_TOLERANCE_MM,
    HAND_GRASP_DISPLAY_DEFAULT,
    HAND_GRASP_LOCK_ON_STATUS,
    HAND_GRASP_MASK_CONTACT_RADIUS,
    HAND_GRASP_MASK_MIN_CONTACT_LANDMARKS,
    HAND_GRASP_MASK_PROXIMITY_THRESHOLD,
    HAND_GRASP_MAX_HANDS,
    HAND_GRASP_ML_CONFIDENCE,
    HAND_GRASP_PRE_GRASP_MASK_MAX_SCALE,
    HAND_GRASP_PRE_GRASP_MASK_MIN_SCALE,
    HAND_GRASP_PUBLISH_ANNOTATED_DEFAULT,
    HAND_GRASP_SAM_RESEED_FROM_YOLO,
    HAND_GRASP_SAM_TRACK_INTERVAL,
    HAND_GRASP_SAM_TRACK_MARGIN,
    HAND_GRASP_SAM_TRACK_MAX_AREA_RATIO,
    HAND_GRASP_SAM_TRACK_MAX_CENTER_SHIFT_PX,
    HAND_GRASP_SAM_TRACK_MIN_AREA_RATIO,
    HAND_GRASP_SAM_YOLO_MAX_CENTER_SHIFT_PX,
    HAND_GRASP_SAM_YOLO_MIN_IOU,
    HAND_GRASP_SAM_YOLO_MISSING_GRACE_FRAMES,
    HAND_GRASP_SHOW_RETURN_CLOSE_ROI_DEFAULT,
    HAND_GRASP_USE_DEPTH_DEFAULT,
    HAND_GRASP_YOLO_CONFIDENCE,
    HAND_GRASP_YOLO_IMAGE_SIZE,
)
from macgyvbot_config.structured_logging import (
    format_structured_log,
)
from macgyvbot_config.models import (
    HAND_GRASP_MODEL_NAME,
    HAND_GRASP_SAM_CHECKPOINT_NAME,
)
from macgyvbot_config.topics import (
    CAMERA_COLOR_TOPIC,
    CAMERA_DEPTH_TOPIC,
    HAND_GRASP_IMAGE_TOPIC,
    HAND_GRASP_MASK_LOCK_TOPIC,
    HAND_GRASP_TOPIC,
    ROBOT_STATUS_TOPIC,
)
from macgyvbot_config.vlm import (
    SAM_BACKEND_DEFAULT,
    SAM_DEPTH_EXPAND_ITERATIONS,
    SAM_DEPTH_MIN_VALID_RATIO,
    SAM_DEPTH_TOLERANCE_MM,
    SAM_DEVICE_DEFAULT,
    SAM_ENABLED_DEFAULT,
    SAM_MODEL_TYPE_DEFAULT,
)
from macgyvbot_interfaces.msg import HumanGraspResult, RobotTaskStatus, ToolMaskLock
from macgyvbot_domain.mask_models import LockedToolMask
from macgyvbot_perception.hand_tool_grasp.hand_detector import (
    HandDetector,
)
from macgyvbot_perception.hand_tool_grasp.hand_center import (
    extract_hand_center_pixel,
)
from macgyvbot_perception.hand_tool_grasp.tool_detector import (
    DEFAULT_MODEL_PATH,
    DEFAULT_TOOL_CLASSES,
    ToolDetection,
    ToolDetector,
)
from macgyvbot_perception.hand_tool_grasp.calculations import (
    build_depth_grasp_info,
    depth_to_mm,
    median_depth_in_rect,
    select_active_hand,
)
from macgyvbot_perception.hand_tool_grasp.ml_grasp_classifier import (
    MLHandGraspClassifier,
    MLGraspResult,
    disabled_ml_result,
    extract_ml_features,
)
from macgyvbot_perception.hand_tool_grasp.sam_tool_mask import (
    BBoxPromptSegmenter,
    create_bbox_locked_mask,
    create_depth_locked_mask,
    mask_to_roi,
    rect_area,
    rect_center,
    rect_iou,
    scale_locked_mask,
    validate_tracked_mask,
)
from macgyvbot_perception.hand_tool_grasp.visualization import (
    draw_pick_overlay,
    draw_return_overlay,
)

def _format_pipeline_log(*, svc, pipe, step, event, msg="", **fields):
    return format_structured_log(
        svc=svc,
        pipe=pipe,
        step=step,
        event=event,
        msg=msg,
        **fields,
    )


class _StructuredLoggerAdapter:
    def __init__(self, logger, svc, pipe):
        self._logger = logger
        self._svc = svc
        self._pipe = pipe

    def debug(self, message):
        self._logger.debug(self._format(message))

    def info(self, message):
        self._logger.info(self._format(message))

    def warn(self, message):
        self._logger.warn(self._format(message))

    def warning(self, message):
        self.warn(message)

    def error(self, message):
        self._logger.error(self._format(message))

    def _format(self, message):
        text = str(message or "")
        if text.startswith("[pkg] ") or text.startswith("pkg="):
            return text
        return _format_pipeline_log(
            svc=self._svc,
            pipe=self._pipe,
            step="log",
            event="status",
            msg=text,
        )


class HandGraspDetectionNode(Node):
    """Detect whether a human hand is grasping a detected tool."""

    def __init__(self) -> None:
        super().__init__("hand_grasp_detection_node")

        self.bridge = CvBridge()
        self.latest_depth_mm = None
        self.robot_status = ""
        self.last_state = None
        self.lock_requested = False
        self.locked_tool: Optional[LockedToolMask] = None
        self.latest_tool_detection: Optional[ToolDetection] = None
        self.tool_mask_anchor: Optional[LockedToolMask] = None
        self.latest_tool_mask: Optional[LockedToolMask] = None
        self.mask_tracking_active = False
        self.active_tool_label = ""
        self.last_yolo_seen_frame = 0
        self.pre_grasp_lock_requested = False
        self.pre_grasp_tool_mask: Optional[LockedToolMask] = None
        self.pre_grasp_tool_roi = None
        self.pre_grasp_tool_depth_mm = None
        self.depth_lock_active = False
        self.depth_lock_published = False
        self.depth_lock_failed = False
        self.depth_lock_roi = None
        self.depth_lock_prev = None
        self.depth_lock_prev_valid = None
        self.depth_lock_delta_stable_count = None
        self.depth_lock_pair_valid_count = None
        self.depth_lock_frame_count = 0
        self.depth_lock_started_sec = None
        self.depth_lock_preview_tool: Optional[LockedToolMask] = None
        self.frame_index = 0

        self.color_topic = self.declare_parameter(
            "color_topic",
            CAMERA_COLOR_TOPIC,
        ).value
        self.depth_topic = self.declare_parameter(
            "depth_topic",
            CAMERA_DEPTH_TOPIC,
        ).value
        self.result_topic = self.declare_parameter(
            "result_topic",
            HAND_GRASP_TOPIC,
        ).value
        self.annotated_topic = self.declare_parameter(
            "annotated_topic",
            HAND_GRASP_IMAGE_TOPIC,
        ).value
        self.use_depth = self._as_bool(
            self.declare_parameter(
                "use_depth",
                HAND_GRASP_USE_DEPTH_DEFAULT,
            ).value
        )
        self.publish_annotated = bool(
            self._as_bool(
                self.declare_parameter(
                    "publish_annotated",
                    HAND_GRASP_PUBLISH_ANNOTATED_DEFAULT,
                ).value
            )
        )
        self.display = self._as_bool(
            self.declare_parameter("display", HAND_GRASP_DISPLAY_DEFAULT).value
        )
        self.show_return_close_roi = self._as_bool(
            self.declare_parameter(
                "show_return_close_roi",
                HAND_GRASP_SHOW_RETURN_CLOSE_ROI_DEFAULT,
            ).value
        )
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
        ).strip().lower()
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
            self.declare_parameter("sam_enabled", SAM_ENABLED_DEFAULT).value
        )
        self.sam_backend = str(
            self.declare_parameter("sam_backend", SAM_BACKEND_DEFAULT).value
        )
        self.sam_checkpoint = str(
            self.declare_parameter(
                "sam_checkpoint",
                str(Path("weights") / HAND_GRASP_SAM_CHECKPOINT_NAME),
            ).value
        )
        self.sam_model_type = str(
            self.declare_parameter("sam_model_type", SAM_MODEL_TYPE_DEFAULT).value
        )
        self.sam_device = str(
            self.declare_parameter("sam_device", SAM_DEVICE_DEFAULT).value
        )
        self.sam_track_interval = int(
            self.declare_parameter(
                "sam_track_interval",
                HAND_GRASP_SAM_TRACK_INTERVAL,
            ).value
        )
        self.sam_track_margin = int(
            self.declare_parameter(
                "sam_track_margin",
                HAND_GRASP_SAM_TRACK_MARGIN,
            ).value
        )
        self.sam_reseed_from_yolo = self._as_bool(
            self.declare_parameter(
                "sam_reseed_from_yolo",
                HAND_GRASP_SAM_RESEED_FROM_YOLO,
            ).value
        )
        self.sam_track_max_center_shift_px = float(
            self.declare_parameter(
                "sam_track_max_center_shift_px",
                HAND_GRASP_SAM_TRACK_MAX_CENTER_SHIFT_PX,
            ).value
        )
        self.sam_track_min_area_ratio = float(
            self.declare_parameter(
                "sam_track_min_area_ratio",
                HAND_GRASP_SAM_TRACK_MIN_AREA_RATIO,
            ).value
        )
        self.sam_track_max_area_ratio = float(
            self.declare_parameter(
                "sam_track_max_area_ratio",
                HAND_GRASP_SAM_TRACK_MAX_AREA_RATIO,
            ).value
        )
        self.sam_yolo_missing_grace_frames = int(
            self.declare_parameter(
                "sam_yolo_missing_grace_frames",
                HAND_GRASP_SAM_YOLO_MISSING_GRACE_FRAMES,
            ).value
        )
        self.sam_yolo_min_iou = float(
            self.declare_parameter(
                "sam_yolo_min_iou",
                HAND_GRASP_SAM_YOLO_MIN_IOU,
            ).value
        )
        self.sam_yolo_max_center_shift_px = float(
            self.declare_parameter(
                "sam_yolo_max_center_shift_px",
                HAND_GRASP_SAM_YOLO_MAX_CENTER_SHIFT_PX,
            ).value
        )
        self.pre_grasp_mask_min_scale = float(
            self.declare_parameter(
                "pre_grasp_mask_min_scale",
                HAND_GRASP_PRE_GRASP_MASK_MIN_SCALE,
            ).value
        )
        self.pre_grasp_mask_max_scale = float(
            self.declare_parameter(
                "pre_grasp_mask_max_scale",
                HAND_GRASP_PRE_GRASP_MASK_MAX_SCALE,
            ).value
        )
        self.depth_tool_mask_enabled = self._as_bool(
            self.declare_parameter(
                "depth_tool_mask_enabled",
                HAND_GRASP_DEPTH_TOOL_MASK_ENABLED,
            ).value
        )
        self.depth_tool_mask_tolerance_mm = float(
            self.declare_parameter(
                "depth_tool_mask_tolerance_mm",
                HAND_GRASP_DEPTH_TOOL_MASK_TOLERANCE_MM,
            ).value
        )
        self.depth_tool_mask_margin = int(
            self.declare_parameter(
                "depth_tool_mask_margin",
                HAND_GRASP_DEPTH_TOOL_MASK_MARGIN,
            ).value
        )
        self.sam_depth_tolerance_mm = float(
            self.declare_parameter(
                "sam_depth_tolerance_mm",
                SAM_DEPTH_TOLERANCE_MM,
            ).value
        )
        self.sam_depth_min_valid_ratio = float(
            self.declare_parameter(
                "sam_depth_min_valid_ratio",
                SAM_DEPTH_MIN_VALID_RATIO,
            ).value
        )
        self.sam_depth_expand_iterations = int(
            self.declare_parameter(
                "sam_depth_expand_iterations",
                SAM_DEPTH_EXPAND_ITERATIONS,
            ).value
        )
        self.allow_bbox_lock = self._as_bool(
            self.declare_parameter("allow_bbox_lock", HAND_GRASP_ALLOW_BBOX_LOCK).value
        )
        self.mask_contact_radius = int(
            self.declare_parameter(
                "mask_contact_radius",
                HAND_GRASP_MASK_CONTACT_RADIUS,
            ).value
        )
        self.mask_min_contact_landmarks = int(
            self.declare_parameter(
                "mask_min_contact_landmarks",
                HAND_GRASP_MASK_MIN_CONTACT_LANDMARKS,
            ).value
        )
        self.mask_proximity_threshold = float(
            self.declare_parameter(
                "mask_proximity_threshold",
                HAND_GRASP_MASK_PROXIMITY_THRESHOLD,
            ).value
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
        yolo_conf = float(
            self.declare_parameter("yolo_conf", HAND_GRASP_YOLO_CONFIDENCE).value
        )
        yolo_imgsz = int(
            self.declare_parameter("yolo_imgsz", HAND_GRASP_YOLO_IMAGE_SIZE).value
        )
        max_hands = int(
            self.declare_parameter("max_hands", HAND_GRASP_MAX_HANDS).value
        )
        self.depth_diff_threshold_mm = float(
            self.declare_parameter(
                "depth_diff_threshold_mm",
                HAND_GRASP_DEPTH_DIFF_THRESHOLD_MM,
            ).value
        )
        self.depth_min_contact_landmarks = int(
            self.declare_parameter(
                "depth_min_contact_landmarks",
                HAND_GRASP_DEPTH_MIN_CONTACT_LANDMARKS,
            ).value
        )
        self.depth_lock_min_frames = int(
            self.declare_parameter(
                "depth_lock_min_frames",
                HAND_GRASP_DEPTH_LOCK_MIN_FRAMES,
            ).value
        )
        self.depth_lock_duration_sec = float(
            self.declare_parameter(
                "depth_lock_duration_sec",
                HAND_GRASP_DEPTH_LOCK_DURATION_SEC,
            ).value
        )
        self.depth_lock_stability_mm = float(
            self.declare_parameter(
                "depth_lock_stability_mm",
                HAND_GRASP_DEPTH_LOCK_STABILITY_MM,
            ).value
        )
        self.depth_lock_min_valid_ratio = float(
            self.declare_parameter(
                "depth_lock_min_valid_ratio",
                HAND_GRASP_DEPTH_LOCK_MIN_VALID_RATIO,
            ).value
        )
        self.depth_lock_min_area_ratio = float(
            self.declare_parameter(
                "depth_lock_min_area_ratio",
                HAND_GRASP_DEPTH_LOCK_MIN_AREA_RATIO,
            ).value
        )
        self.depth_lock_max_area_ratio = float(
            self.declare_parameter(
                "depth_lock_max_area_ratio",
                HAND_GRASP_DEPTH_LOCK_MAX_AREA_RATIO,
            ).value
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

        self.result_pub = self.create_publisher(
            HumanGraspResult, self.result_topic, 10
        )
        self.mask_lock_pub = self.create_publisher(
            ToolMaskLock, self.mask_lock_topic, 10
        )
        self.annotated_pub = (
            self.create_publisher(Image, self.annotated_topic, 10)
            if self.publish_annotated
            else None
        )

        if self.use_depth:
            self.create_subscription(Image, self.depth_topic, self._depth_cb, 10)
            self._log(
                "info",
                "depth recognition enabled",
                step="startup",
                event="ready",
                depth_topic=self.depth_topic,
            )
        else:
            self._log(
                "warn",
                "depth recognition disabled",
                step="startup",
                event="disabled",
                reason="parameter_disabled",
            )

        self.create_subscription(Image, self.color_topic, self._color_cb, 10)
        self.create_subscription(
            RobotTaskStatus, self.robot_status_topic, self._robot_status_cb, 10
        )
        self._log(
            "info",
            "node ready",
            step="startup",
            event="ready",
            color_topic=self.color_topic,
            result_topic=self.result_topic,
            robot_status_topic=self.robot_status_topic,
            mask_lock_topic=self.mask_lock_topic,
        )

    def _log(self, level, message, **fields):
        text = _format_pipeline_log(
            svc="perception",
            pipe="hand_grasp",
            step=fields.pop("step", "log"),
            event=fields.pop("event", "status"),
            msg=message,
            **fields,
        )
        logger = self.get_logger()
        if level == "debug":
            logger.debug(text)
        elif level == "error":
            logger.error(text)
        elif level in ("warn", "warning"):
            logger.warn(text)
        else:
            logger.info(text)

    def get_logger(self):
        return _StructuredLoggerAdapter(
            super().get_logger(),
            svc="perception",
            pipe="hand_grasp",
        )

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
            self._log(
                "error",
                "YOLO tool detector init failed",
                step="tool_detector",
                event="fail",
                reason=str(exc) or type(exc).__name__,
            )
            return None

        self._log(
            "info",
            "YOLO tool detector enabled",
            step="tool_detector",
            event="ready",
            model=Path(detector.model_path).name,
            classes=target_classes or "ANY",
        )
        return detector

    def _create_ml_classifier(self) -> Optional[MLHandGraspClassifier]:
        try:
            classifier = MLHandGraspClassifier(self.grasp_model_path)
        except Exception as exc:
            self._log(
                "error",
                "ML grasp classifier init failed",
                step="ml_classifier",
                event="fail",
                reason=str(exc) or type(exc).__name__,
            )
            return None

        self._log(
            "info",
            "ML grasp classifier enabled",
            step="ml_classifier",
            event="ready",
            model=Path(classifier.path).name,
        )
        return classifier

    def _create_sam_segmenter(self) -> Optional[BBoxPromptSegmenter]:
        if not self.sam_enabled:
            self._log(
                "info",
                "SAM tool mask disabled",
                step="sam",
                event="disabled",
                reason="parameter_disabled",
            )
            return None

        try:
            segmenter = BBoxPromptSegmenter(
                backend=self.sam_backend,
                checkpoint_path=self.sam_checkpoint,
                model_type=self.sam_model_type,
                device=self.sam_device,
            )
        except Exception as exc:
            self._log(
                "error",
                "SAM tool mask init failed",
                step="sam",
                event="fail",
                reason=str(exc) or type(exc).__name__,
            )
            return None

        self._log(
            "info",
            "SAM tool mask enabled",
            step="sam",
            event="ready",
            backend=self.sam_backend,
            checkpoint=Path(self.sam_checkpoint).name,
        )
        return segmenter

    def _robot_status_cb(self, msg: RobotTaskStatus) -> None:
        status = str(msg.status or "").strip().lower()
        self.robot_status = status
        requested_tool_label = self._status_tool_label(msg)
        reason = str(getattr(msg, "reason", "") or "").strip().lower()

        if status == "tool_dropped" or (
            status == "recovering"
            and (
                reason.startswith("drop_recovery")
                or reason.startswith("moving_to_recovery")
                or reason.startswith("detecting_recovery")
            )
        ):
            self._reset_tool_mask_state()
            self.mask_tracking_active = False
            if self.ml_classifier is not None:
                self.ml_classifier.reset()
            self.get_logger().info(
                "Drop recovery status received. Tool mask/depth lock cleared."
            )
            return

        if status == self.lock_on_status:
            self._set_active_tool_label(requested_tool_label)
            self._start_depth_tool_lock()
            self.mask_tracking_active = False
            self.locked_tool = None
            if self.ml_classifier is not None:
                self.ml_classifier.reset()
            self.get_logger().info("Robot grasp success received. Tool mask lock requested.")
            return

        if status == "released_to_human":
            self._reset_tool_mask_state()
            self.mask_tracking_active = False
            if self.ml_classifier is not None:
                self.ml_classifier.reset()
            self._publish_tool_mask_unlock("released_to_human")
            self.get_logger().info(
                "Tool handoff release received. Tool mask/depth lock cleared."
            )
            return

        if status in {"accepted", "opening_drawer", "observing_drawer"}:
            self._set_active_tool_label(requested_tool_label)
            self._reset_tool_mask_state(clear_active_label=False)
            self.mask_tracking_active = False
            return

        if status in {"observing_pick_target", "grasping"}:
            self._set_active_tool_label(requested_tool_label)
            self.mask_tracking_active = bool(self.active_tool_label)
            return

        if status == "pre_grasp_mask_lock":
            self._set_active_tool_label(requested_tool_label)
            self.mask_tracking_active = False
            return

        if status in {"returned", "done", "failed", "cancelled"}:
            if status == "returned" and reason == "drop_recovery_succeeded":
                self.get_logger().info(
                    "Drop recovery succeeded. Keeping tool mask/depth lock for resumed task."
                )
                return
            self._reset_tool_mask_state()
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
            self.tool_detector.detect(frame, target_label=self.active_tool_label)
            if self.tool_detector is not None
            else None
        )
        grasp_point_detection = (
            self.tool_detector.last_grasp_point_detection
            if self.tool_detector is not None
            else None
        )
        if tool_detection is not None:
            self.latest_tool_detection = tool_detection
            self.last_yolo_seen_frame = self.frame_index

        if self.depth_lock_active:
            self._update_depth_tool_lock(frame, tool_detection or self.latest_tool_detection)

        detected_tool_roi = (
            self.locked_tool.roi
            if self.locked_tool is not None
            else tool_detection.roi if tool_detection is not None else None
        )
        hand_infos = self.hand_detector.detect_all(frame)
        active_hand = self._select_active_hand(hand_infos, detected_tool_roi)
        if active_hand is None and hand_infos:
            active_hand = hand_infos[0]
        hand_for_state = active_hand if active_hand is not None else (
            hand_infos[0] if hand_infos else None
        )
        observation_roi = detected_tool_roi
        depth_info = (
            self._build_locked_tool_depth_info(hand_for_state)
            if self.locked_tool is not None
            else self._build_depth_info(hand_for_state, observation_roi)
        )
        ml_result = self._update_ml_grasp(hand_for_state)
        result = self._compose_result(
            hand_info=hand_for_state,
            tool_roi=observation_roi,
            depth_info=depth_info,
            ml_result=ml_result,
        )

        self._publish_result(result, tool_detection, active_hand, hand_infos)

        if self.publish_annotated or self.display:
            annotated = frame.copy()
            if self._uses_return_overlay():
                draw_return_overlay(
                    frame=annotated,
                    hand_infos=hand_infos,
                    active_hand=active_hand,
                    tool_detection=tool_detection,
                    result=result,
                    grasp_point_detection=grasp_point_detection,
                    depth_mm=self.latest_depth_mm,
                )
            else:
                draw_pick_overlay(
                    frame=annotated,
                    hand_infos=hand_infos,
                    active_hand=active_hand,
                    tool_detection=tool_detection,
                    result=result,
                    grasp_point_detection=grasp_point_detection,
                    locked_tool=self.locked_tool,
                    candidate_tool_mask=self.depth_lock_preview_tool,
                    depth_mm=self.latest_depth_mm,
                )
            if self.annotated_pub is not None:
                self.annotated_pub.publish(
                    self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
                )
            if self.display:
                cv2.imshow("Hand Grasp Detection", annotated)
                cv2.waitKey(1)

        if result["state"] != self.last_state:
            self._log(
                "debug",
                "grasp state changed",
                step="state",
                event="changed",
                state=result.get("state"),
                human_grasped_tool=result.get("human_grasped_tool"),
                ml=f"{result.get('ml_stable_state')}/{result.get('ml_raw_state')}",
                depth_contact_count=result.get("depth_contact_count"),
            )
            self.last_state = result["state"]

    def _build_depth_info(self, hand_info: Optional[dict], tool_roi):
        if not self.use_depth:
            return None
        if self.latest_depth_mm is None or tool_roi is None:
            return None
        if hand_info is None:
            tool_depth = median_depth_in_rect(self.latest_depth_mm, tool_roi)
            return {
                "depth_available": tool_depth is not None,
                "tool_depth_mm": tool_depth,
                "min_hand_tool_depth_diff_mm": None,
                "depth_contact_count": 0,
                "depth_grasp_confirmed": False,
            }

        return build_depth_grasp_info(
            hand_info=hand_info,
            tool_roi=tool_roi,
            depth_mm=self.latest_depth_mm,
            depth_diff_threshold_mm=self.depth_diff_threshold_mm,
            min_depth_contact_landmarks=self.depth_min_contact_landmarks,
        )

    def _build_locked_tool_depth_info(self, hand_info: Optional[dict]):
        if not self.use_depth or self.latest_depth_mm is None:
            return None
        if self.locked_tool is None:
            return None

        mask = self.locked_tool.mask.astype(bool)
        tool_values = self.latest_depth_mm[(self.latest_depth_mm > 0) & mask]
        tool_depth = float(np.median(tool_values)) if tool_values.size else None
        if tool_depth is None:
            return {
                "depth_available": False,
                "tool_depth_mm": None,
                "min_hand_tool_depth_diff_mm": None,
                "depth_contact_count": 0,
                "depth_grasp_confirmed": False,
            }

        if hand_info is None:
            return {
                "depth_available": True,
                "tool_depth_mm": tool_depth,
                "min_hand_tool_depth_diff_mm": None,
                "depth_contact_count": 0,
                "depth_grasp_confirmed": False,
            }

        contact_mask = self._dilated_locked_tool_mask()
        valid_diffs = []
        depth_contact_count = 0
        height, width = self.latest_depth_mm.shape[:2]
        for x, y in hand_info["landmarks"].values():
            if not (0 <= x < width and 0 <= y < height):
                continue
            if not contact_mask[y, x]:
                continue
            hand_depth = self._median_depth_at_point((x, y))
            if hand_depth is None:
                continue
            diff = abs(hand_depth - tool_depth)
            valid_diffs.append(diff)
            if diff <= self.depth_diff_threshold_mm:
                depth_contact_count += 1

        min_depth_diff = min(valid_diffs) if valid_diffs else None
        return {
            "depth_available": True,
            "tool_depth_mm": tool_depth,
            "min_hand_tool_depth_diff_mm": min_depth_diff,
            "depth_contact_count": depth_contact_count,
            "depth_grasp_confirmed": (
                depth_contact_count >= self.depth_min_contact_landmarks
            ),
        }

    def _dilated_locked_tool_mask(self):
        mask = self.locked_tool.mask.astype(np.uint8)
        radius = max(1, int(self.mask_contact_radius))
        kernel = np.ones((radius * 2 + 1, radius * 2 + 1), np.uint8)
        return cv2.dilate(mask, kernel).astype(bool)

    def _median_depth_at_point(self, point, radius=3):
        x, y = point
        height, width = self.latest_depth_mm.shape[:2]
        x1 = max(0, min(width, int(x) - radius))
        y1 = max(0, min(height, int(y) - radius))
        x2 = max(0, min(width, int(x) + radius + 1))
        y2 = max(0, min(height, int(y) + radius + 1))
        if x2 <= x1 or y2 <= y1:
            return None
        values = self.latest_depth_mm[y1:y2, x1:x2]
        values = values[values > 0]
        if values.size == 0:
            return None
        return float(np.median(values))

    @staticmethod
    def _observation_roi(
        tool_roi: Optional[tuple[int, int, int, int]],
        active_hand: Optional[dict],
    ) -> Optional[tuple[int, int, int, int]]:
        if tool_roi is not None:
            return tool_roi
        if active_hand is None:
            return None
        return active_hand.get("hand_rect")

    def _uses_return_overlay(self) -> bool:
        return self.show_return_close_roi or "return" in self.robot_status

    @staticmethod
    def _normalize_tool_label(label) -> str:
        return str(label or "").strip().lower()

    def _status_tool_label(self, msg: RobotTaskStatus) -> str:
        if str(msg.action or "").strip().lower() != "bring":
            return ""
        return self._normalize_tool_label(msg.tool_name)

    def _set_active_tool_label(self, tool_label: str) -> None:
        normalized = self._normalize_tool_label(tool_label)
        if not normalized or normalized == "unknown":
            return
        if normalized == self.active_tool_label:
            return

        self.active_tool_label = normalized
        self.latest_tool_detection = None
        self.tool_mask_anchor = None
        self.latest_tool_mask = None
        self.get_logger().info(
            f"Tool mask tracking target label set: {self.active_tool_label}"
        )

    def _reset_tool_mask_state(self, clear_active_label=True) -> None:
        self.lock_requested = False
        self.locked_tool = None
        self.latest_tool_detection = None
        self.tool_mask_anchor = None
        self.latest_tool_mask = None
        self.last_yolo_seen_frame = 0
        self.pre_grasp_lock_requested = False
        self.pre_grasp_tool_mask = None
        self.pre_grasp_tool_roi = None
        self.pre_grasp_tool_depth_mm = None
        self._reset_depth_tool_lock(clear_locked_tool=False)
        if clear_active_label:
            self.active_tool_label = ""

    def _start_depth_tool_lock(self) -> None:
        self._reset_depth_tool_lock(clear_locked_tool=True)
        self.depth_lock_active = True
        self.depth_lock_started_sec = time.monotonic()
        self.get_logger().info("Depth locked tool mask accumulation started.")

    def _reset_depth_tool_lock(self, clear_locked_tool=True) -> None:
        self.depth_lock_active = False
        self.depth_lock_published = False
        self.depth_lock_failed = False
        self.depth_lock_roi = None
        self.depth_lock_prev = None
        self.depth_lock_prev_valid = None
        self.depth_lock_delta_stable_count = None
        self.depth_lock_pair_valid_count = None
        self.depth_lock_frame_count = 0
        self.depth_lock_started_sec = None
        self.depth_lock_preview_tool = None
        if clear_locked_tool:
            self.locked_tool = None

    def _update_depth_tool_lock(
        self,
        frame,
        _tool_detection: Optional[ToolDetection],
    ) -> None:
        if self.latest_depth_mm is None:
            return

        sam_depth_locked = self._create_sam_depth_tool_mask(frame, _tool_detection)
        if sam_depth_locked is not None:
            self.locked_tool = sam_depth_locked
            self.depth_lock_active = False
            self.depth_lock_preview_tool = None
            self._publish_mask_lock_result(sam_depth_locked)
            self.get_logger().info(
                f"SAM+Depth tool mask ready: source={sam_depth_locked.source}, "
                f"roi={sam_depth_locked.roi}"
            )
            return

        if self.depth_lock_roi is None:
            height, width = self.latest_depth_mm.shape[:2]
            self.depth_lock_roi = (0, 0, width, height)
            self.get_logger().info(
                f"Depth lock ROI initialized from full depth frame: roi={self.depth_lock_roi}"
            )

        x1, y1, x2, y2 = self.depth_lock_roi
        if x2 <= x1 or y2 <= y1:
            self._publish_depth_tool_lock_failure("invalid_depth_lock_roi")
            return

        crop = self.latest_depth_mm[y1:y2, x1:x2].astype(np.float32)
        valid = crop > 0
        if not valid.any():
            self._handle_depth_lock_no_sample()
            return

        if self.depth_lock_prev is None:
            self.depth_lock_prev = crop
            self.depth_lock_prev_valid = valid
            self.depth_lock_frame_count += 1
            self.depth_lock_preview_tool = self._depth_lock_preview_from_accumulator()
            return

        pair_valid = valid & self.depth_lock_prev_valid
        stable_delta = (
            pair_valid
            & (np.abs(crop - self.depth_lock_prev) <= self.depth_lock_stability_mm)
        )
        if self.depth_lock_delta_stable_count is None:
            self.depth_lock_delta_stable_count = stable_delta.astype(np.int32)
            self.depth_lock_pair_valid_count = pair_valid.astype(np.int32)
        else:
            self.depth_lock_delta_stable_count += stable_delta.astype(np.int32)
            self.depth_lock_pair_valid_count += pair_valid.astype(np.int32)
        self.depth_lock_prev = crop
        self.depth_lock_prev_valid = valid

        self.depth_lock_frame_count += 1
        self.depth_lock_preview_tool = self._depth_lock_preview_from_accumulator()
        locked = self._depth_locked_tool_from_accumulator()
        if locked is not None:
            self.locked_tool = locked
            self.depth_lock_active = False
            self.depth_lock_preview_tool = None
            self._publish_mask_lock_result(locked)
            self.get_logger().info(
                f"Depth locked tool mask ready: frames={self.depth_lock_frame_count}, "
                f"roi={locked.roi}"
            )
            return

        if self._depth_lock_elapsed_sec() >= self.depth_lock_duration_sec:
            self._publish_depth_tool_lock_failure("depth_lock_unstable")

    def _create_sam_depth_tool_mask(
        self,
        frame,
        tool_detection: Optional[ToolDetection],
    ) -> Optional[LockedToolMask]:
        if tool_detection is None or self.sam_segmenter is None:
            return None

        sam_mask = self.sam_segmenter.segment(frame, tool_detection.roi)
        if sam_mask is None or int(sam_mask.sum()) <= 0:
            return None

        refined = self._refine_sam_mask_with_depth(sam_mask)
        roi = mask_to_roi(refined)
        if roi is None:
            return None

        self.depth_lock_preview_tool = LockedToolMask(
            roi=roi,
            mask=refined,
            source="DEPTH_SAM_LOCKING",
        )
        return LockedToolMask(
            roi=roi,
            mask=refined,
            source="DEPTH_SAM_LOCKED_TOOL",
        )

    def _refine_sam_mask_with_depth(self, sam_mask: np.ndarray) -> np.ndarray:
        sam_bool = sam_mask.astype(bool)
        if self.latest_depth_mm is None or not sam_bool.any():
            return self._expand_binary_mask(
                sam_bool.astype(np.uint8),
                iterations=self.sam_depth_expand_iterations,
            ).astype(bool)

        valid = (self.latest_depth_mm > 0) & sam_bool
        sam_area = max(1, int(sam_bool.sum()))
        valid_ratio = int(valid.sum()) / sam_area
        if valid_ratio < self.sam_depth_min_valid_ratio:
            return self._expand_binary_mask(
                sam_bool.astype(np.uint8),
                iterations=self.sam_depth_expand_iterations,
            ).astype(bool)

        target_depth = float(np.median(self.latest_depth_mm[valid]))
        close_depth = (
            (self.latest_depth_mm > 0)
            & (np.abs(self.latest_depth_mm - target_depth) <= self.sam_depth_tolerance_mm)
        )
        refined = sam_bool & (close_depth | ~valid)
        if not refined.any():
            refined = sam_bool
        return self._expand_binary_mask(
            refined.astype(np.uint8),
            iterations=self.sam_depth_expand_iterations,
        ).astype(bool)

    def _depth_lock_preview_from_accumulator(self) -> Optional[LockedToolMask]:
        if (
            self.depth_lock_roi is None
            or self.depth_lock_delta_stable_count is None
            or self.depth_lock_pair_valid_count is None
            or self.latest_depth_mm is None
        ):
            return None

        pair_frame_count = max(1, self.depth_lock_frame_count - 1)
        required_valid_count = max(
            1,
            int(round(pair_frame_count * self.depth_lock_min_valid_ratio)),
        )
        preview = (
            (self.depth_lock_pair_valid_count >= required_valid_count)
            & (self.depth_lock_delta_stable_count >= required_valid_count)
        )
        if not preview.any():
            return None

        x1, y1, x2, y2 = self.depth_lock_roi
        full_mask = np.zeros(self.latest_depth_mm.shape[:2], dtype=np.uint8)
        full_mask[y1:y2, x1:x2] = preview.astype(np.uint8)
        full_mask = self._expand_binary_mask(full_mask).astype(bool)
        roi = mask_to_roi(full_mask)
        if roi is None:
            return None
        return LockedToolMask(roi=roi, mask=full_mask, source="DEPTH_LOCKING")

    def _depth_locked_tool_from_accumulator(self) -> Optional[LockedToolMask]:
        if self.depth_lock_frame_count < self.depth_lock_min_frames:
            return None
        if self._depth_lock_elapsed_sec() < self.depth_lock_duration_sec:
            return None
        if (
            self.depth_lock_delta_stable_count is None
            or self.depth_lock_pair_valid_count is None
        ):
            return None

        pair_frame_count = max(1, self.depth_lock_frame_count - 1)
        required_valid_count = max(
            1,
            int(round(pair_frame_count * self.depth_lock_min_valid_ratio)),
        )
        stable = (
            (self.depth_lock_pair_valid_count >= required_valid_count)
            & (self.depth_lock_delta_stable_count >= required_valid_count)
        )

        x1, y1, x2, y2 = self.depth_lock_roi
        bbox_area = max(1, (x2 - x1) * (y2 - y1))
        mask_area = int(stable.sum())
        area_ratio = mask_area / bbox_area
        if (
            mask_area <= 0
            or area_ratio < self.depth_lock_min_area_ratio
            or area_ratio > self.depth_lock_max_area_ratio
        ):
            return None

        full_mask = np.zeros(self.latest_depth_mm.shape[:2], dtype=np.uint8)
        full_mask[y1:y2, x1:x2] = stable.astype(np.uint8)
        full_mask = self._expand_binary_mask(full_mask).astype(bool)
        roi = mask_to_roi(full_mask)
        if roi is None:
            return None
        return LockedToolMask(roi=roi, mask=full_mask, source="DEPTH_LOCKED_TOOL")

    def _handle_depth_lock_no_sample(self) -> None:
        self.depth_lock_frame_count += 1
        if self._depth_lock_elapsed_sec() >= self.depth_lock_duration_sec:
            self._publish_depth_tool_lock_failure("depth_lock_no_valid_depth")

    def _publish_depth_tool_lock_failure(self, reason: str) -> None:
        if self.depth_lock_failed:
            return
        self.depth_lock_active = False
        self.depth_lock_failed = True
        self.depth_lock_preview_tool = None
        msg = ToolMaskLock()
        msg.locked = False
        msg.mask_source = "DEPTH_LOCKED_TOOL"
        msg.has_tool_roi = False
        msg.tool_roi = [0, 0, 0, 0]
        msg.has_reason = True
        msg.reason = reason
        self.mask_lock_pub.publish(msg)

    def _publish_tool_mask_unlock(self, reason: str) -> None:
        msg = ToolMaskLock()
        msg.locked = False
        msg.mask_source = "NONE"
        msg.has_tool_roi = False
        msg.tool_roi = [0, 0, 0, 0]
        msg.has_reason = True
        msg.reason = reason
        self.mask_lock_pub.publish(msg)

    def _depth_lock_elapsed_sec(self) -> float:
        if self.depth_lock_started_sec is None:
            return 0.0
        return max(0.0, time.monotonic() - self.depth_lock_started_sec)

    @staticmethod
    def _expand_binary_mask(mask: np.ndarray, iterations: int = 1) -> np.ndarray:
        kernel = np.ones((3, 3), np.uint8)
        expanded = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_CLOSE, kernel)
        return cv2.dilate(expanded, kernel, iterations=max(0, int(iterations)))

    def _update_prelock_tool_mask(
        self,
        frame,
        tool_detection: Optional[ToolDetection],
    ) -> None:
        if self.locked_tool is not None:
            return

        if tool_detection is not None:
            self._update_mask_from_yolo_anchor(frame, tool_detection)
            return

        if self.latest_tool_mask is None:
            return

        if not self._recent_yolo_available():
            self.get_logger().warn(
                "Clearing SAM candidate mask because requested YOLO target "
                "has not been seen recently."
            )
            self.latest_tool_mask = None
            self.tool_mask_anchor = None
            return

        self._track_prelock_tool_mask(frame)

    def _update_mask_from_yolo_anchor(
        self,
        frame,
        tool_detection: ToolDetection,
    ) -> None:
        if self.latest_tool_mask is None:
            self._seed_prelock_tool_mask(frame, tool_detection)
            return

        if not self._mask_agrees_with_yolo(self.latest_tool_mask, tool_detection.roi):
            self._seed_prelock_tool_mask(frame, tool_detection, force=True)
            return

        self._track_prelock_tool_mask(frame)

    def _seed_prelock_tool_mask(
        self,
        frame,
        tool_detection: Optional[ToolDetection],
        force=False,
    ) -> None:
        if tool_detection is None:
            return

        depth_mask = self._create_depth_tool_mask(tool_detection, frame.shape[:2])
        if depth_mask is not None:
            self.latest_tool_mask = depth_mask
            self.tool_mask_anchor = depth_mask
            return

        if self.sam_segmenter is None:
            self.latest_tool_mask = create_bbox_locked_mask(
                tool_detection.roi,
                frame.shape[:2],
            )
            self.tool_mask_anchor = self.latest_tool_mask
            return

        if not force and self.frame_index % max(1, self.sam_track_interval) != 0:
            return

        mask = self.sam_segmenter.segment(frame, tool_detection.roi)
        if mask is not None and int(mask.sum()) > 0:
            roi = mask_to_roi(mask) or tool_detection.roi
            self.latest_tool_mask = LockedToolMask(
                roi=roi,
                mask=mask,
                source="SAM_TRACKED",
            )
            self.tool_mask_anchor = self.latest_tool_mask

    def _track_prelock_tool_mask(self, frame) -> None:
        if self.locked_tool is not None or self.sam_segmenter is None:
            return

        if self.latest_tool_mask is None:
            return

        if self.frame_index % max(1, self.sam_track_interval) != 0:
            return

        tracked = self.sam_segmenter.track_from_mask(
            frame,
            self.latest_tool_mask,
            margin=self.sam_track_margin,
        )
        if tracked is None:
            return

        validation = validate_tracked_mask(
            previous=self.latest_tool_mask,
            tracked=tracked,
            max_center_shift_px=self.sam_track_max_center_shift_px,
            min_area_ratio=self.sam_track_min_area_ratio,
            max_area_ratio=self.sam_track_max_area_ratio,
        )
        if not validation.accepted:
            self.get_logger().warn(
                f"Rejected SAM tracked tool mask: reason={validation.reason}, "
                f"previous_roi={self.latest_tool_mask.roi}, tracked_roi={tracked.roi}"
            )
            return

        self.latest_tool_mask = tracked

    def _capture_pre_grasp_tool_mask(
        self,
        frame,
        tool_detection: Optional[ToolDetection],
    ) -> None:
        self.pre_grasp_lock_requested = False
        locked = self._pre_grasp_sam_mask(frame, tool_detection)
        if locked is None:
            locked = self._lock_from_yolo_or_candidate(frame, tool_detection)
        if locked is None:
            self.get_logger().warn("Pre-grasp tool mask lock failed.")
            return

        self.pre_grasp_tool_mask = LockedToolMask(
            roi=locked.roi,
            mask=locked.mask.copy(),
            source="PRE_GRASP_LOCKED",
        )
        self.pre_grasp_tool_roi = tool_detection.roi if tool_detection else locked.roi
        self.pre_grasp_tool_depth_mm = self._tool_depth_mm(self.pre_grasp_tool_roi)
        self.latest_tool_mask = self.pre_grasp_tool_mask
        self.tool_mask_anchor = self.pre_grasp_tool_mask
        self.get_logger().info(
            "Pre-grasp tool mask locked: "
            f"source={locked.source}, roi={locked.roi}, "
            f"depth_mm={self.pre_grasp_tool_depth_mm}"
        )

    def _pre_grasp_sam_mask(
        self,
        frame,
        tool_detection: Optional[ToolDetection],
    ) -> Optional[LockedToolMask]:
        if tool_detection is None or self.sam_segmenter is None:
            return None

        mask = self.sam_segmenter.segment(frame, tool_detection.roi)
        if mask is None or int(mask.sum()) <= 0:
            self.get_logger().warn("Pre-grasp SAM returned an empty tool mask.")
            return None

        roi = mask_to_roi(mask) or tool_detection.roi
        self.get_logger().info(
            f"Pre-grasp SAM mask locked: label={tool_detection.label}, roi={roi}"
        )
        return LockedToolMask(
            roi=roi,
            mask=mask,
            source="PRE_GRASP_SAM_LOCKED",
        )

    def _lock_tool_mask(
        self,
        frame,
        tool_detection: Optional[ToolDetection],
    ) -> Optional[LockedToolMask]:
        scaled = self._scaled_pre_grasp_tool_mask(frame, tool_detection)
        if scaled is not None:
            return scaled

        return self._lock_from_yolo_or_candidate(frame, tool_detection)

    def _lock_from_yolo_or_candidate(
        self,
        frame,
        tool_detection: Optional[ToolDetection],
    ) -> Optional[LockedToolMask]:
        if tool_detection is not None:
            depth_mask = self._create_depth_tool_mask(
                tool_detection,
                frame.shape[:2],
                source="DEPTH_LOCKED",
            )
            if depth_mask is not None:
                self.get_logger().info(
                    f"Locked depth tool mask: label={tool_detection.label}, "
                    f"roi={depth_mask.roi}"
                )
                return depth_mask

        if (
            self.latest_tool_mask is not None
            and self.latest_tool_mask.source in {"SAM_TRACKED", "DEPTH_TRACKED"}
        ):
            return LockedToolMask(
                roi=self.latest_tool_mask.roi,
                mask=self.latest_tool_mask.mask.copy(),
                source=self._locked_source(self.latest_tool_mask.source),
            )

        if (
            self.tool_mask_anchor is not None
            and self.tool_mask_anchor.source in {"SAM_TRACKED", "DEPTH_TRACKED"}
        ):
            return LockedToolMask(
                roi=self.tool_mask_anchor.roi,
                mask=self.tool_mask_anchor.mask.copy(),
                source=self._locked_source(self.tool_mask_anchor.source),
            )

        if tool_detection is None:
            if self.latest_tool_mask is not None and self.allow_bbox_lock:
                self.get_logger().warn(
                    "Tool mask lock requested without YOLO ROI; "
                    "using latest bbox fallback."
                )
                return LockedToolMask(
                    roi=self.latest_tool_mask.roi,
                    mask=self.latest_tool_mask.mask.copy(),
                    source=self.latest_tool_mask.source,
                )
            self.get_logger().warn(
                "Tool mask lock requested, but no YOLO tool ROI is available."
            )
            return None

        if self.sam_segmenter is not None and self.sam_reseed_from_yolo:
            mask = self.sam_segmenter.segment(frame, tool_detection.roi)
            if mask is not None and int(mask.sum()) > 0:
                roi = mask_to_roi(mask) or tool_detection.roi
                self.get_logger().info(
                    f"Locked SAM tool mask: label={tool_detection.label}, "
                    f"roi={roi}"
                )
                return LockedToolMask(
                    roi=roi,
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

    def _create_depth_tool_mask(
        self,
        tool_detection: ToolDetection,
        frame_shape: tuple[int, int],
        source="DEPTH_TRACKED",
    ) -> Optional[LockedToolMask]:
        if not self.depth_tool_mask_enabled:
            return None
        return create_depth_locked_mask(
            self.latest_depth_mm,
            tool_detection.roi,
            frame_shape,
            tolerance_mm=self.depth_tool_mask_tolerance_mm,
            margin=self.depth_tool_mask_margin,
            source=source,
        )

    @staticmethod
    def _locked_source(source: str) -> str:
        if source == "DEPTH_TRACKED":
            return "DEPTH_LOCKED"
        if source == "SAM_TRACKED":
            return "SAM_LOCKED"
        return source

    def _scaled_pre_grasp_tool_mask(
        self,
        frame,
        tool_detection: Optional[ToolDetection],
    ) -> Optional[LockedToolMask]:
        if self.pre_grasp_tool_mask is None:
            return None

        scale = self._pre_grasp_scale(tool_detection)
        center = rect_center(tool_detection.roi) if tool_detection else None
        scaled = scale_locked_mask(
            self.pre_grasp_tool_mask,
            frame.shape[:2],
            scale,
            center=center,
            source="PRE_GRASP_SCALED_LOCKED",
        )
        if scaled is None:
            return None

        if tool_detection is not None and not self._mask_agrees_with_yolo(
            scaled,
            tool_detection.roi,
        ):
            self.get_logger().warn(
                "Rejected scaled pre-grasp mask because it disagrees with YOLO: "
                f"mask_roi={scaled.roi}, yolo_roi={tool_detection.roi}"
            )
            return None

        self.get_logger().info(
            "Using scaled pre-grasp mask for final lock: "
            f"scale={scale:.3f}, roi={scaled.roi}"
        )
        return scaled

    def _pre_grasp_scale(self, tool_detection: Optional[ToolDetection]) -> float:
        scale = None
        if tool_detection is not None and self.pre_grasp_tool_roi is not None:
            previous_area = rect_area(self.pre_grasp_tool_roi)
            current_area = rect_area(tool_detection.roi)
            if previous_area > 0 and current_area > 0:
                scale = (current_area / previous_area) ** 0.5

        if scale is None:
            current_depth = self._tool_depth_mm(
                tool_detection.roi
                if tool_detection is not None
                else self.pre_grasp_tool_mask.roi
            )
            if (
                self.pre_grasp_tool_depth_mm is not None
                and current_depth is not None
                and current_depth > 0
            ):
                scale = self.pre_grasp_tool_depth_mm / current_depth

        if scale is None:
            scale = 1.0

        return max(
            self.pre_grasp_mask_min_scale,
            min(self.pre_grasp_mask_max_scale, float(scale)),
        )

    def _tool_depth_mm(self, roi):
        if self.latest_depth_mm is None or roi is None:
            return None
        return median_depth_in_rect(self.latest_depth_mm, roi)

    def _recent_yolo_available(self) -> bool:
        return (
            self.last_yolo_seen_frame > 0
            and self.frame_index - self.last_yolo_seen_frame
            <= self.sam_yolo_missing_grace_frames
        )

    def _mask_agrees_with_yolo(self, mask: LockedToolMask, yolo_roi) -> bool:
        if rect_iou(mask.roi, yolo_roi) >= self.sam_yolo_min_iou:
            return True

        mask_center = rect_center(mask.roi)
        yolo_center = rect_center(yolo_roi)
        center_shift = float(
            ((mask_center[0] - yolo_center[0]) ** 2 + (mask_center[1] - yolo_center[1]) ** 2)
            ** 0.5
        )
        return center_shift <= self.sam_yolo_max_center_shift_px

    def _publish_mask_lock_result(self, locked_tool: Optional[LockedToolMask]) -> None:
        payload = {
            "locked": locked_tool is not None,
            "mask_source": locked_tool.source if locked_tool is not None else "NONE",
            "tool_roi": locked_tool.roi if locked_tool is not None else None,
        }
        if locked_tool is None:
            payload["reason"] = "tool_mask_unavailable"

        msg = ToolMaskLock()
        msg.locked = bool(payload["locked"])
        msg.mask_source = str(payload["mask_source"])
        roi = payload.get("tool_roi")
        msg.has_tool_roi = roi is not None
        msg.tool_roi = [int(value) for value in roi] if roi is not None else [0, 0, 0, 0]
        msg.has_reason = "reason" in payload
        msg.reason = str(payload.get("reason", ""))
        self.mask_lock_pub.publish(msg)

    def _update_ml_grasp(self, hand_info: Optional[dict]) -> MLGraspResult:
        if self.ml_classifier is None:
            return disabled_ml_result("model_unavailable")

        try:
            return self.ml_classifier.update(hand_info)
        except Exception as exc:
            self.get_logger().warn(f"ML grasp update failed: {exc}")
            self.ml_classifier.reset()
            return disabled_ml_result("model_error")

    def _select_active_hand(self, hand_infos: list[dict], tool_roi):
        ml_hand = self._select_highest_ml_grasp_hand(hand_infos)
        if ml_hand is not None:
            return ml_hand
        return select_active_hand(hand_infos, tool_roi)

    def _select_highest_ml_grasp_hand(self, hand_infos: list[dict]):
        if self.ml_classifier is None or not hand_infos:
            return None

        scored_hands = []
        for hand_info in hand_infos:
            score = self._raw_ml_grasp_score(hand_info)
            if score is None:
                continue
            scored_hands.append((score, hand_info))

        if not scored_hands:
            return None

        score, hand_info = max(scored_hands, key=lambda item: item[0])
        hand_info["ml_grasp_candidate_score"] = score
        return hand_info

    def _raw_ml_grasp_score(self, hand_info: dict):
        try:
            features = extract_ml_features(hand_info)
            model = self.ml_classifier.model
            raw_state = str(model.predict([features])[0])
            if raw_state != "grasp":
                return 0.0
            if not hasattr(model, "predict_proba"):
                return 1.0

            probabilities = model.predict_proba([features])[0]
            classes = list(getattr(model, "classes_", []))
            if "grasp" in classes:
                return float(probabilities[classes.index("grasp")])
            return float(max(probabilities))
        except Exception as exc:
            self.get_logger().warn(f"ML grasp 후보 점수 계산 실패: {exc}")
            return None

    def _compose_result(
        self,
        hand_info: Optional[dict],
        tool_roi,
        depth_info: Optional[dict],
        ml_result: MLGraspResult,
    ) -> dict:
        confidence_ok = (
            ml_result.enabled
            and (
                ml_result.confidence is None
                or ml_result.confidence >= self.ml_min_confidence
            )
        )
        depth_info = depth_info or {}
        depth_grasp_ok = bool(depth_info.get("depth_grasp_confirmed", False))
        human_grasped_tool = bool(
            ml_result.is_grasp
            and confidence_ok
            and depth_grasp_ok
        )

        result = {
            "state": self._combined_grasp_state(
                hand_info=hand_info,
                tool_roi=tool_roi,
                ml_result=ml_result,
                confidence_ok=confidence_ok,
                depth_grasp_ok=depth_grasp_ok,
                human_grasped_tool=human_grasped_tool,
            ),
            "hand_present": hand_info is not None,
            "human_grasped_tool": human_grasped_tool,
            "grasp_counter": 1 if human_grasped_tool else 0,
            "grasp_score": 1 if human_grasped_tool else 0,
            "depth_available": depth_info.get("depth_available", False),
            "tool_depth_mm": depth_info.get("tool_depth_mm"),
            "min_hand_tool_depth_diff_mm": depth_info.get("min_hand_tool_depth_diff_mm"),
            "depth_contact_count": depth_info.get("depth_contact_count", 0),
            "depth_grasp_confirmed": depth_info.get("depth_grasp_confirmed", False),
            "min_landmark_to_tool_distance": None,
        }
        result["tool_roi"] = tool_roi
        result["ml_raw_state"] = ml_result.raw_state
        result["ml_stable_state"] = ml_result.stable_state
        result["ml_confidence"] = ml_result.confidence
        result["ml_grasp_confirmed"] = ml_result.is_grasp
        result["ml_confidence_ok"] = confidence_ok
        result["ml_required"] = True
        result["depth_required"] = True
        result["depth_grasp_ok"] = depth_grasp_ok
        result["mask_source"] = (
            self.locked_tool.source if self.locked_tool is not None else "DEPTH_GRASP_CHECK"
        )
        result["mask_locked"] = self.locked_tool is not None
        result["mask_contact_count"] = 0
        result["mask_contact_confirmed"] = False
        result["mask_proximity_ok"] = False
        result["mask_near_or_contact"] = False
        result["locked_mask_grasp_ok"] = depth_grasp_ok

        return result

    @staticmethod
    def _combined_grasp_state(
        hand_info: Optional[dict],
        tool_roi,
        ml_result: MLGraspResult,
        confidence_ok: bool,
        depth_grasp_ok: bool,
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
        hand_pixel = extract_hand_center_pixel(
            active_hand,
            hand_infos,
            tool_mask=self.locked_tool.mask if self.locked_tool is not None else None,
        )
        payload = {
            "state": result["state"],
            "hand_present": bool(hand_infos),
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
            "tool_roi": result.get("tool_roi"),
            "hand_pixel": hand_pixel,
        }
        msg = HumanGraspResult()
        msg.state = str(payload["state"])
        msg.hand_present = bool(payload["hand_present"])
        msg.human_grasped_tool = bool(payload["human_grasped_tool"])
        msg.grasp_counter = int(payload["grasp_counter"])
        msg.grasp_score = float(payload["grasp_score"])
        msg.ml_required = bool(payload["ml_required"])
        msg.ml_grasp_confirmed = bool(payload["ml_grasp_confirmed"])
        msg.ml_confidence_ok = bool(payload["ml_confidence_ok"])
        msg.ml_raw_state = str(payload["ml_raw_state"])
        msg.ml_stable_state = str(payload["ml_stable_state"])
        msg.has_ml_confidence = payload["ml_confidence"] is not None
        msg.ml_confidence = (
            float(payload["ml_confidence"]) if payload["ml_confidence"] is not None else 0.0
        )
        msg.depth_required = bool(payload["depth_required"])
        msg.depth_available = bool(payload["depth_available"])
        msg.depth_grasp_ok = bool(payload["depth_grasp_ok"])
        msg.depth_grasp_confirmed = bool(payload["depth_grasp_confirmed"])
        msg.has_tool_depth_mm = payload["tool_depth_mm"] is not None
        msg.tool_depth_mm = (
            float(payload["tool_depth_mm"]) if payload["tool_depth_mm"] is not None else 0.0
        )
        depth_diff = payload["min_hand_tool_depth_diff_mm"]
        msg.has_min_hand_tool_depth_diff_mm = depth_diff is not None
        msg.min_hand_tool_depth_diff_mm = float(depth_diff) if depth_diff is not None else 0.0
        msg.depth_contact_count = int(payload["depth_contact_count"])
        msg.mask_locked = bool(payload["mask_locked"])
        msg.locked_mask_grasp_ok = bool(payload["locked_mask_grasp_ok"])
        msg.mask_contact_confirmed = bool(payload["mask_contact_confirmed"])
        msg.mask_proximity_ok = bool(payload["mask_proximity_ok"])
        msg.mask_near_or_contact = bool(payload["mask_near_or_contact"])
        msg.mask_source = str(payload["mask_source"])
        msg.mask_contact_count = int(payload["mask_contact_count"])
        distance = payload["min_landmark_to_tool_distance"]
        msg.has_min_landmark_to_tool_distance = distance is not None
        msg.min_landmark_to_tool_distance = float(distance) if distance is not None else 0.0
        label = payload["tool_label"]
        msg.has_tool_label = label is not None
        msg.tool_label = str(label) if label is not None else ""
        tool_confidence = payload["tool_confidence"]
        msg.has_tool_confidence = tool_confidence is not None
        msg.tool_confidence = (
            float(tool_confidence) if tool_confidence is not None else 0.0
        )
        roi = payload["tool_roi"]
        msg.has_tool_roi = roi is not None
        msg.tool_roi = [int(value) for value in roi] if roi is not None else [0, 0, 0, 0]
        hand_pixel = payload["hand_pixel"]
        msg.has_hand_pixel = hand_pixel is not None
        if hand_pixel is not None:
            msg.hand_u = int(hand_pixel["u"])
            msg.hand_v = int(hand_pixel["v"])
            msg.hand_pixel_source = str(hand_pixel.get("source", "unknown"))
        active_hand_index = payload["active_hand_index"]
        msg.has_active_hand_index = active_hand_index is not None
        msg.active_hand_index = (
            int(active_hand_index) if active_hand_index is not None else 0
        )
        handedness = payload["active_handedness"]
        msg.has_active_handedness = handedness is not None
        msg.active_handedness = str(handedness) if handedness is not None else ""
        self.result_pub.publish(msg)

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
