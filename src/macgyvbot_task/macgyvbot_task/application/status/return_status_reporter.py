"""Status reporting for return workflow."""

from __future__ import annotations


class ReturnStatusReporter:
    """Build and publish return workflow status payloads."""

    def __init__(self, publish_payload):
        self._publish_payload = publish_payload

    def publish(self, status, tool_name, message, command, reason=""):
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
        self._publish_payload(payload)

    def fail(self, tool_name, message, reason, command, logger):
        logger.error(message)
        self.publish(
            "failed",
            tool_name,
            message,
            command,
            reason=reason,
        )
