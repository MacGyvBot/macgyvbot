"""Status reporting for return workflow."""

from __future__ import annotations

from macgyvbot_task.application.logging_utils import log_error


class ReturnStatusReporter:
    """Build and publish return workflow status payloads."""

    def __init__(self, state):
        self.state = state

    def publish(self, status, tool_name, message, command, reason=""):
        publish_payload = getattr(self.state, "_publish_status_payload", None)
        if publish_payload is None:
            return

        payload = {
            "status": status,
            "task": "return_tool",
            "tool_name": tool_name,
            "action": "return",
            "message": message,
            "command": command,
        }
        if reason:
            payload["reason"] = reason
        publish_payload(payload)

    def fail(self, tool_name, message, reason, command, logger):
        log_error(
            logger,
            "return flow failed",
            step="return",
            event="fail",
            tool=tool_name,
            reason=reason,
            detail=message,
        )
        self.publish(
            "failed",
            tool_name,
            message,
            command,
            reason=reason,
        )
