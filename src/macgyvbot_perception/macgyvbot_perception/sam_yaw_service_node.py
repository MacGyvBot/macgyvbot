#!/usr/bin/env python3
"""Dedicated ROS2 service node for SAM-based grasp yaw estimation."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
import math
import time

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node

from macgyvbot_config.models import HAND_GRASP_SAM_CHECKPOINT_NAME
from macgyvbot_config.structured_logging import format_structured_log
from macgyvbot_config.vlm import (
    SAM_BACKEND_DEFAULT,
    SAM_DEPTH_EXPAND_ITERATIONS,
    SAM_DEPTH_MIN_VALID_RATIO,
    SAM_DEPTH_TOLERANCE_MM,
    SAM_DEVICE_DEFAULT,
    SAM_ENABLED_DEFAULT,
    SAM_MODEL_TYPE_DEFAULT,
    SAM_YAW_SERVICE_NAME,
    VLM_DATA_ROOT,
    VLM_YAW_PCA_DEBUG_DIR,
)
from macgyvbot_interfaces.srv import SAMYaw
from macgyvbot_perception.grasp_point.grasp_method import (
    estimate_yaw_from_binary_crop,
)
from macgyvbot_perception.grasp_point.mask_image_for_grasp_detection import (
    generate_sam_depth_mask_image_for_grasp_detection,
)
from macgyvbot_perception.hand_tool_grasp.calculations import depth_to_mm
from macgyvbot_perception.hand_tool_grasp.sam_tool_mask import BBoxPromptSegmenter


class SAMYawServiceNode(Node):
    """Serve SAM depth-mask PCA yaw requests for pick grasp detection."""

    def __init__(self) -> None:
        super().__init__("sam_yaw_service_node")

        self.bridge = CvBridge()
        self.declare_parameter("sam_yaw_service_name", SAM_YAW_SERVICE_NAME)
        self.declare_parameter("sam_enabled", SAM_ENABLED_DEFAULT)
        self.declare_parameter(
            "sam_checkpoint",
            str(Path("weights") / HAND_GRASP_SAM_CHECKPOINT_NAME),
        )
        self.declare_parameter("sam_backend", SAM_BACKEND_DEFAULT)
        self.declare_parameter("sam_model_type", SAM_MODEL_TYPE_DEFAULT)
        self.declare_parameter("sam_device", SAM_DEVICE_DEFAULT)
        self.declare_parameter("sam_depth_tolerance_mm", SAM_DEPTH_TOLERANCE_MM)
        self.declare_parameter(
            "sam_depth_min_valid_ratio",
            SAM_DEPTH_MIN_VALID_RATIO,
        )
        self.declare_parameter(
            "sam_depth_expand_iterations",
            SAM_DEPTH_EXPAND_ITERATIONS,
        )

        self.sam_enabled = self._as_bool(self.get_parameter("sam_enabled").value)
        self.sam_checkpoint = str(self.get_parameter("sam_checkpoint").value).strip()
        self.sam_backend = str(self.get_parameter("sam_backend").value).strip()
        self.sam_model_type = str(self.get_parameter("sam_model_type").value).strip()
        self.sam_device = str(self.get_parameter("sam_device").value).strip()
        self.sam_depth_tolerance_mm = float(
            self.get_parameter("sam_depth_tolerance_mm").value
        )
        self.sam_depth_min_valid_ratio = float(
            self.get_parameter("sam_depth_min_valid_ratio").value
        )
        self.sam_depth_expand_iterations = int(
            self.get_parameter("sam_depth_expand_iterations").value
        )
        self._segmenter = None

        service_name = str(self.get_parameter("sam_yaw_service_name").value).strip()
        self.service = self.create_service(SAMYaw, service_name, self._handle_request)
        self._preload_segmenter()
        self.get_logger().info(
            _log(
                "SAM yaw service ready",
                step="startup",
                event="ready",
                service=service_name,
                checkpoint=Path(self.sam_checkpoint).name,
            )
        )

    def _handle_request(self, request, response):
        request_start = time.monotonic()
        self.get_logger().info(
            _log(
                "SAM yaw service request received",
                step="request",
                event="received",
                target=request.target_label,
                received_at=self._timestamp(),
            )
        )

        segmenter = self._sam_segmenter()
        if segmenter is None:
            response.success = False
            response.error_message = "SAM segmenter unavailable"
            return response

        try:
            color_image = self.bridge.imgmsg_to_cv2(request.color_image, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(
                request.depth_image,
                desired_encoding="passthrough",
            )
        except Exception as exc:
            response.success = False
            response.error_message = f"image conversion failed: {exc}"
            self.get_logger().warn(
                _log(
                    "SAM yaw service image conversion failed",
                    step="request",
                    event="fail",
                    reason=str(exc) or type(exc).__name__,
                )
            )
            return response

        bbox = [float(value) for value in request.bbox_xyxy]
        if len(bbox) < 4:
            response.success = False
            response.error_message = "bbox_xyxy must contain four values"
            self.get_logger().warn(
                _log(
                    "SAM yaw service request bbox is invalid",
                    step="request",
                    event="fail",
                    reason="invalid_bbox",
                )
            )
            return response

        images = generate_sam_depth_mask_image_for_grasp_detection(
            color_image=color_image,
            depth_mm=depth_to_mm(depth_image, "passthrough"),
            bbox_xyxy=bbox,
            sam_segmenter=segmenter,
            data_root=Path(VLM_DATA_ROOT),
            filename_prefix=f"pick_sam_yaw_{request.target_label}",
            sam_depth_tolerance_mm=self.sam_depth_tolerance_mm,
            sam_depth_min_valid_ratio=self.sam_depth_min_valid_ratio,
            sam_depth_expand_iterations=self.sam_depth_expand_iterations,
        )
        if not images:
            response.success = False
            response.error_message = "SAM depth mask generation failed"
            self.get_logger().warn(
                _log(
                    "SAM yaw service mask generation failed",
                    step="sam_mask",
                    event="fail",
                    target=request.target_label,
                    reason="sam_depth_mask_failed",
                )
            )
            return response

        yaw_deg, debug = estimate_yaw_from_binary_crop(images[0])
        if yaw_deg is None:
            response.success = False
            response.error_message = str(debug.get("reason") or "PCA yaw failed")
            self.get_logger().warn(
                _log(
                    "SAM yaw service PCA yaw failed",
                    step="pca_yaw",
                    event="fail",
                    target=request.target_label,
                    reason=response.error_message,
                    pixels=debug.get("num_pixels"),
                )
            )
            return response

        response.success = True
        response.yaw_deg = float(yaw_deg)
        response.debug_image_path = VLM_YAW_PCA_DEBUG_DIR
        self.get_logger().info(
            _log(
                "SAM yaw service response ready",
                step="response",
                event="done",
                target=request.target_label,
                yaw_deg=f"{response.yaw_deg:.1f}",
                latency_sec=f"{time.monotonic() - request_start:.3f}",
                pixels=debug.get("num_pixels"),
            )
        )
        return response

    def _preload_segmenter(self):
        started = time.monotonic()
        self.get_logger().info(
            _log(
                "SAM yaw service preload started",
                step="preload",
                event="start",
                checkpoint=Path(self.sam_checkpoint).name,
            )
        )
        self._sam_segmenter()
        self.get_logger().info(
            _log(
                "SAM yaw service preload completed",
                step="preload",
                event="done",
                elapsed_sec=f"{time.monotonic() - started:.3f}",
            )
        )

    def _sam_segmenter(self):
        if self._segmenter is not None:
            return self._segmenter
        if not self.sam_enabled:
            self.get_logger().warn(
                _log(
                    "SAM yaw service disabled",
                    step="sam_mask",
                    event="disabled",
                )
            )
            return None
        try:
            self._segmenter = BBoxPromptSegmenter(
                backend=self.sam_backend,
                checkpoint_path=self.sam_checkpoint,
                model_type=self.sam_model_type,
                device=self.sam_device,
            )
        except Exception as exc:
            self.get_logger().warn(
                _log(
                    "SAM yaw service init failed",
                    step="sam_mask",
                    event="fail",
                    reason=str(exc) or type(exc).__name__,
                )
            )
            return None
        return self._segmenter

    @staticmethod
    def _timestamp():
        return datetime.now().isoformat(timespec="milliseconds")

    @staticmethod
    def _as_bool(value):
        if isinstance(value, str):
            return value.strip().lower() in {"1", "true", "yes", "on"}
        return bool(value)


def _log(message, **fields):
    return format_structured_log(
        pkg="perception",
        pipe="sam_yaw_service",
        msg=message,
        **fields,
    )


def main() -> None:
    rclpy.init()
    node = SAMYawServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
