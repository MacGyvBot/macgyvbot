"""Status reporting for return workflow."""

from __future__ import annotations

import json

from std_msgs.msg import String


class ReturnStatusReporter:
    """Build and publish return workflow status payloads."""

    def __init__(self, state):
        self.state = state

    def publish(self, status, tool_name, message, command, reason=""):
        publisher = getattr(self.state, "robot_status_pub", None)
        if publisher is None:
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
        publisher.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def fail(self, tool_name, message, reason, command, logger):
        logger.error(message)
        self.publish(
            "failed",
            tool_name,
            message,
            command,
            reason=reason,
        )
