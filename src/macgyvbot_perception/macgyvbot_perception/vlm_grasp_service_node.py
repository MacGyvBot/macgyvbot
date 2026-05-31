#!/usr/bin/env python3
"""Dedicated ROS2 service node for VLM grasp inference."""

from __future__ import annotations

from datetime import datetime
import math
import time

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node

from macgyvbot_config.vlm import (
    DEFAULT_GRASP_POINT_MODE,
    GRASP_POINT_MODE_VLM,
    VLM_GRASP_SERVICE_NAME,
    VLM_ONLY_MODEL_BY_MODE,
    VLM_ONLY_MODES,
)
from macgyvbot_interfaces.srv import VLMGrasp
from macgyvbot_perception.grasp_point.vlm_grasp_point_selector import (
    VLMGraspPointSelector,
)
from macgyvbot_perception.grasp_point.vlm_only_grasp_point_selector import (
    VLMOnlyGraspPointSelector,
)


class VLMGraspServiceNode(Node):
    """Serve grasp-point inference requests using the existing VLM selectors."""

    def __init__(self) -> None:
        super().__init__("vlm_grasp_service_node")

        self.bridge = CvBridge()
        self.declare_parameter("vlm_service_name", VLM_GRASP_SERVICE_NAME)
        self.declare_parameter("grasp_point_mode", DEFAULT_GRASP_POINT_MODE)
        self.declare_parameter("sam_enabled", True)
        self.declare_parameter("sam_checkpoint", "")
        self.declare_parameter("sam_backend", "mobile_sam")
        self.declare_parameter("sam_model_type", "vit_t")
        self.declare_parameter("sam_device", "cuda")

        self.grasp_point_mode = str(self.get_parameter("grasp_point_mode").value).strip()
        self.sam_kwargs = {
            "sam_enabled": self._as_bool(self.get_parameter("sam_enabled").value),
            "sam_checkpoint": str(self.get_parameter("sam_checkpoint").value).strip(),
            "sam_backend": str(self.get_parameter("sam_backend").value).strip(),
            "sam_model_type": str(self.get_parameter("sam_model_type").value).strip(),
            "sam_device": str(self.get_parameter("sam_device").value).strip(),
        }
        self._vlm_selector = None
        self._vlm_only_selectors = {}

        service_name = str(self.get_parameter("vlm_service_name").value).strip()
        self.service = self.create_service(
            VLMGrasp,
            service_name,
            self._handle_request,
        )
        self._preload_default_mode()
        self.get_logger().info(
            "VLM grasp service ready: "
            f"service={service_name}, default_mode={self.grasp_point_mode}"
        )

    def _handle_request(self, request, response):
        request_received_wall = self._timestamp()
        request_received_mono = time.monotonic()
        mode = str(request.mode or self.grasp_point_mode).strip()
        self.get_logger().info(
            "VLM service request received: "
            f"received_at={request_received_wall}, mode={mode}, "
            f"target={request.target_label}, label={request.label}"
        )

        try:
            color_image = self.bridge.imgmsg_to_cv2(request.color_image, "bgr8")
        except Exception as exc:
            response.success = False
            response.error_message = f"image conversion failed: {exc}"
            self.get_logger().warn(f"VLM service image conversion failed: {exc}")
            return response

        bbox = [float(value) for value in request.bbox_xyxy]
        if len(bbox) < 4:
            response.success = False
            response.error_message = "bbox_xyxy must contain four values"
            self.get_logger().warn("VLM service request bbox is invalid.")
            return response

        selector = self._selector_for_mode(mode)
        if selector is None:
            response.success = False
            response.error_message = f"unsupported VLM service mode: {mode}"
            self.get_logger().warn(response.error_message)
            return response

        infer_start_wall = self._timestamp()
        infer_start_mono = time.monotonic()
        self.get_logger().info(
            "VLM service inference started: "
            f"started_at={infer_start_wall}, mode={mode}"
        )
        try:
            selected = selector.select_grasp_pixel(
                bbox,
                request.label,
                color_image,
                None,
                None,
                request.target_label,
            )
        except Exception as exc:
            response.success = False
            response.error_message = f"VLM inference failed: {exc}"
            self.get_logger().warn(response.error_message)
            return response

        infer_end_wall = self._timestamp()
        infer_end_mono = time.monotonic()
        self.get_logger().info(
            "VLM service inference completed: "
            f"completed_at={infer_end_wall}, "
            f"inference_latency_sec={infer_end_mono - infer_start_mono:.3f}"
        )

        if selected is None:
            response.success = False
            response.error_message = "VLM selector returned no grasp result"
            return response

        pixel_u, pixel_v, source, orientation_rpy_deg = selected
        response.success = True
        response.pixel_u = int(pixel_u)
        response.pixel_v = int(pixel_v)
        response.source = str(source)
        response.orientation_rpy_deg = (
            [float(value) for value in orientation_rpy_deg]
            if orientation_rpy_deg is not None
            else []
        )
        response.grasp_yaw_deg = self._extract_yaw(orientation_rpy_deg)
        response.result_text = (
            f"pixel=({response.pixel_u}, {response.pixel_v}), "
            f"yaw_deg={response.grasp_yaw_deg:.1f}, source={response.source}"
        )
        self.get_logger().info(
            "VLM service request completed: "
            f"total_latency_sec={time.monotonic() - request_received_mono:.3f}, "
            f"result={response.result_text}"
        )
        return response

    def _selector_for_mode(self, mode):
        if mode == GRASP_POINT_MODE_VLM:
            if self._vlm_selector is None:
                self.get_logger().info(
                    "Preloading VLM service selector: "
                    f"mode={mode}, model=grid_vlm_default"
                )
                self._vlm_selector = VLMGraspPointSelector(
                    self.get_logger(),
                    **self.sam_kwargs,
                )
                self._vlm_selector.preload()
            return self._vlm_selector

        if mode in VLM_ONLY_MODES:
            selector = self._vlm_only_selectors.get(mode)
            if selector is None:
                model_id = VLM_ONLY_MODEL_BY_MODE[mode]
                self.get_logger().info(
                    "Preloading VLM service selector: "
                    f"mode={mode}, model_id={model_id}"
                )
                selector = VLMOnlyGraspPointSelector(
                    self.get_logger(),
                    **self.sam_kwargs,
                    model_id=model_id,
                    mode=mode,
                )
                selector.preload()
                self._vlm_only_selectors[mode] = selector
            return selector

        return None

    def _preload_default_mode(self):
        mode = self.grasp_point_mode
        if mode != GRASP_POINT_MODE_VLM and mode not in VLM_ONLY_MODES:
            self.get_logger().info(
                "VLM service preload skipped: "
                f"default_mode={mode} does not use a local VLM selector"
            )
            return

        preload_started = time.monotonic()
        self.get_logger().info(
            "VLM service preload started: "
            f"mode={mode}, started_at={self._timestamp()}"
        )
        self._selector_for_mode(mode)
        self.get_logger().info(
            "VLM service preload completed: "
            f"mode={mode}, elapsed_sec={time.monotonic() - preload_started:.3f}"
        )

    @staticmethod
    def _extract_yaw(orientation_rpy_deg):
        if orientation_rpy_deg is None or len(orientation_rpy_deg) < 3:
            return math.nan
        try:
            return float(orientation_rpy_deg[2])
        except (TypeError, ValueError):
            return math.nan

    @staticmethod
    def _timestamp():
        return datetime.now().isoformat(timespec="milliseconds")

    @staticmethod
    def _as_bool(value):
        if isinstance(value, str):
            return value.strip().lower() in {"1", "true", "yes", "on"}
        return bool(value)


def main() -> None:
    rclpy.init()
    node = VLMGraspServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
