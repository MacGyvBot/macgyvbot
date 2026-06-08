"""Client wrapper for remote VLM grasp inference."""

from __future__ import annotations

from datetime import datetime
import time

from macgyvbot_config.timing import ROS_FUTURE_POLL_SEC
from macgyvbot_config.vlm import (
    VLM_GRASP_SERVICE_NAME,
    VLM_SERVICE_RESPONSE_TIMEOUT_SEC,
    VLM_SERVICE_WAIT_TIMEOUT_SEC,
)
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
        self._log().info(
            "VLM service request created",
            step="request",
            event="created",
            created_at=request_created_wall,
            service=self.service_name,
            mode=mode,
            target=target_label,
            label=label,
            bbox=list(bbox_xyxy),
        )

        if not self.client.wait_for_service(timeout_sec=self.wait_timeout_sec):
            self._log().warn(
                "VLM service unavailable within timeout",
                step="service",
                event="timeout",
                service=self.service_name,
                timeout_sec=self.wait_timeout_sec,
                reason="service_unavailable",
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
        self._log().info(
            "VLM service request sent",
            step="request",
            event="sent",
            sent_at=send_wall,
            queue_wait_sec=f"{send_mono - request_created_mono:.3f}",
        )
        future = self.client.call_async(request)

        deadline = send_mono + self.response_timeout_sec
        while not future.done():
            if interrupted():
                future.cancel()
                self._log().warn(
                    "VLM service request interrupted while waiting for response",
                    step="response",
                    event="interrupted",
                    reason="interrupted",
                )
                return None
            if time.monotonic() >= deadline:
                future.cancel()
                self._log().warn(
                    "VLM service response timeout",
                    step="response",
                    event="timeout",
                    service=self.service_name,
                    timeout_sec=self.response_timeout_sec,
                    reason="response_timeout",
                )
                return None
            time.sleep(ROS_FUTURE_POLL_SEC)

        response_wall = self._timestamp()
        response_mono = time.monotonic()
        service_latency = response_mono - send_mono
        total_latency = response_mono - request_created_mono
        self._log().info(
            "VLM service response received",
            step="response",
            event="received",
            received_at=response_wall,
            service_latency_sec=f"{service_latency:.3f}",
            total_latency_sec=f"{total_latency:.3f}",
        )

        try:
            response = future.result()
        except Exception as exc:
            self._log().warn(
                "VLM service future failed",
                step="response",
                event="fail",
                reason=str(exc) or type(exc).__name__,
            )
            return None

        if response is None:
            self._log().warn(
                "VLM service returned no response object",
                step="response",
                event="empty",
                reason="empty_response",
            )
            return None
        if not response.success:
            self._log().warn(
                "VLM service reported failure",
                step="response",
                event="fail",
                reason=response.error_message or response.result_text,
            )
            return None

        return response

    def _log(self):
        task_log = getattr(self.node, "_task_log", None)
        if task_log is not None:
            return task_log("vlm", quiet_info=True)
        return _PlainLoggerAdapter(self.node.get_logger())

    @staticmethod
    def _timestamp():
        return datetime.now().isoformat(timespec="milliseconds")


class _PlainLoggerAdapter:
    """Accept structured fields when a task logger is not available."""

    def __init__(self, logger):
        self._logger = logger

    def info(self, message, **fields):
        self._logger.info(_format_fallback(message, fields))

    def warn(self, message, **fields):
        self._logger.warn(_format_fallback(message, fields))


def _format_fallback(message, fields):
    if not fields:
        return str(message)
    field_text = " ".join(
        f"{key}={value}" for key, value in fields.items() if value is not None
    )
    return f"{message} {field_text}"
