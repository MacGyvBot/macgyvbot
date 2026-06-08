"""Client wrapper for remote SAM yaw estimation."""

from __future__ import annotations

from datetime import datetime
import time

from macgyvbot_config.vlm import (
    SAM_YAW_SERVICE_NAME,
    SAM_YAW_SERVICE_RESPONSE_TIMEOUT_SEC,
    SAM_YAW_SERVICE_WAIT_TIMEOUT_SEC,
)
from macgyvbot_interfaces.srv import SAMYaw


class SAMYawServiceClient:
    """Request SAM-based grasp yaw estimation from perception."""

    def __init__(
        self,
        node,
        bridge,
        service_name=SAM_YAW_SERVICE_NAME,
        wait_timeout_sec=SAM_YAW_SERVICE_WAIT_TIMEOUT_SEC,
        response_timeout_sec=SAM_YAW_SERVICE_RESPONSE_TIMEOUT_SEC,
    ):
        self.node = node
        self.bridge = bridge
        self.service_name = service_name
        self.wait_timeout_sec = float(wait_timeout_sec)
        self.response_timeout_sec = float(response_timeout_sec)
        self.client = node.create_client(SAMYaw, service_name)

    def estimate_yaw(
        self,
        color_image,
        depth_image,
        bbox_xyxy,
        target_label,
        interrupted=None,
    ):
        interrupted = interrupted or (lambda: False)
        created_mono = time.monotonic()
        self._log().info(
            "SAM yaw service request created",
            step="request",
            event="created",
            created_at=self._timestamp(),
            service=self.service_name,
            target=target_label,
            bbox=list(bbox_xyxy),
        )

        if not self.client.wait_for_service(timeout_sec=self.wait_timeout_sec):
            self._log().warn(
                "SAM yaw service unavailable within timeout",
                step="service",
                event="timeout",
                service=self.service_name,
                timeout_sec=self.wait_timeout_sec,
                reason="service_unavailable",
            )
            return None

        request = SAMYaw.Request()
        request.color_image = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
        request.depth_image = self.bridge.cv2_to_imgmsg(
            depth_image,
            encoding="passthrough",
        )
        request.bbox_xyxy = [float(value) for value in bbox_xyxy]
        request.target_label = str(target_label)

        send_mono = time.monotonic()
        self._log().info(
            "SAM yaw service request sent",
            step="request",
            event="sent",
            sent_at=self._timestamp(),
            queue_wait_sec=f"{send_mono - created_mono:.3f}",
        )
        future = self.client.call_async(request)

        deadline = send_mono + self.response_timeout_sec
        while not future.done():
            if interrupted():
                future.cancel()
                self._log().warn(
                    "SAM yaw service request interrupted while waiting for response",
                    step="response",
                    event="interrupted",
                    reason="interrupted",
                )
                return None
            if time.monotonic() >= deadline:
                future.cancel()
                self._log().warn(
                    "SAM yaw service response timeout",
                    step="response",
                    event="timeout",
                    service=self.service_name,
                    timeout_sec=self.response_timeout_sec,
                    reason="response_timeout",
                )
                return None
            time.sleep(0.02)

        response_mono = time.monotonic()
        self._log().info(
            "SAM yaw service response received",
            step="response",
            event="received",
            received_at=self._timestamp(),
            service_latency_sec=f"{response_mono - send_mono:.3f}",
            total_latency_sec=f"{response_mono - created_mono:.3f}",
        )

        try:
            response = future.result()
        except Exception as exc:
            self._log().warn(
                "SAM yaw service future failed",
                step="response",
                event="fail",
                reason=str(exc) or type(exc).__name__,
            )
            return None

        if response is None:
            self._log().warn(
                "SAM yaw service returned no response object",
                step="response",
                event="empty",
                reason="empty_response",
            )
            return None
        if not response.success:
            self._log().warn(
                "SAM yaw service reported failure",
                step="response",
                event="fail",
                reason=response.error_message,
            )
            return None

        return response

    def _log(self):
        task_log = getattr(self.node, "_task_log", None)
        if task_log is not None:
            return task_log("perception", quiet_info=True)
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
