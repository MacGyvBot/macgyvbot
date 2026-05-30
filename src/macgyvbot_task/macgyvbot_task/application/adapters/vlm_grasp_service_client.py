"""Client wrapper for remote VLM grasp inference."""

from __future__ import annotations

from datetime import datetime
import time

from macgyvbot_config.vlm import (
    VLM_GRASP_SERVICE_NAME,
    VLM_SERVICE_RESPONSE_TIMEOUT_SEC,
    VLM_SERVICE_WAIT_TIMEOUT_SEC,
)
from macgyvbot_domain.logging import MacGyvbotLogger, exception_log_fields
from macgyvbot_interfaces.srv import VLMGrasp


class VLMGraspServiceClient:
    """Request VLM grasp inference from a dedicated perception service node."""

    def __init__(
        self,
        node,
        bridge,
        service_name=VLM_GRASP_SERVICE_NAME,
        wait_timeout_sec=VLM_SERVICE_WAIT_TIMEOUT_SEC,
        response_timeout_sec=VLM_SERVICE_RESPONSE_TIMEOUT_SEC,
    ):
        self.node = node
        self.bridge = bridge
        self.service_name = service_name
        self.wait_timeout_sec = float(wait_timeout_sec)
        self.response_timeout_sec = float(response_timeout_sec)
        self.client = node.create_client(VLMGrasp, service_name)
        if hasattr(node, "service_log"):
            self.log = node.service_log.bind("vlm_client")
        else:
            self.log = MacGyvbotLogger(node.get_logger(), svc="task", pipe="vlm_client")

    def infer_grasp(
        self,
        color_image,
        bbox_xyxy,
        label,
        target_label,
        mode,
        interrupted=None,
    ):
        interrupted = interrupted or (lambda: False)
        request_created_mono = time.monotonic()
        request_created_wall = self._timestamp()
        self.log.info(
            "request",
            "created",
            created_at=request_created_wall,
            service=self.service_name,
            mode=mode,
            target=target_label,
            label=label,
            bbox=list(bbox_xyxy),
        )

        if not self.client.wait_for_service(timeout_sec=self.wait_timeout_sec):
            self.log.warn(
                "service",
                "unavailable",
                service=self.service_name,
                timeout_sec=self.wait_timeout_sec,
            )
            return None

        request = VLMGrasp.Request()
        request.color_image = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
        request.bbox_xyxy = [float(value) for value in bbox_xyxy]
        request.label = str(label)
        request.target_label = str(target_label)
        request.mode = str(mode)

        send_wall = self._timestamp()
        send_mono = time.monotonic()
        self.log.info(
            "request",
            "sent",
            sent_at=send_wall,
            queue_wait_sec=f"{send_mono - request_created_mono:.3f}",
        )
        future = self.client.call_async(request)

        deadline = send_mono + self.response_timeout_sec
        while not future.done():
            if interrupted():
                future.cancel()
                self.log.warn("request", "interrupted")
                return None
            if time.monotonic() >= deadline:
                future.cancel()
                self.log.warn(
                    "response",
                    "timeout",
                    service=self.service_name,
                    timeout_sec=self.response_timeout_sec,
                )
                return None
            time.sleep(0.02)

        response_wall = self._timestamp()
        response_mono = time.monotonic()
        service_latency = response_mono - send_mono
        total_latency = response_mono - request_created_mono
        self.log.info(
            "response",
            "received",
            received_at=response_wall,
            service_latency_sec=f"{service_latency:.3f}",
            total_latency_sec=f"{total_latency:.3f}",
        )

        try:
            response = future.result()
        except Exception as exc:
            self.log.warn("response", "future_failed", **exception_log_fields(exc))
            return None

        if response is None:
            self.log.warn("response", "empty")
            return None
        if not response.success:
            self.log.warn(
                "response",
                "failed",
                reason=response.error_message or response.result_text,
            )
            return None

        return response

    @staticmethod
    def _timestamp():
        return datetime.now().isoformat(timespec="milliseconds")
