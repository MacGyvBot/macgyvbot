"""Status payload builder for robot task progress."""

from __future__ import annotations


class RobotStatusPublisher:
    """Build robot status payloads and pass them to an output adapter."""

    def __init__(self, publish_payload, target_label_provider=None):
        self.publish_payload = publish_payload
        self.target_label_provider = target_label_provider or (lambda: None)

    def publish(
        self,
        status,
        tool_name=None,
        action=None,
        message="",
        reason="",
        command=None,
    ):
        payload = {
            "status": status,
            "tool_name": tool_name or self.target_label_provider() or "unknown",
            "action": action or (command or {}).get("action", "unknown"),
            "message": message,
        }
        if reason:
            payload["reason"] = reason
        if command is not None:
            payload["command"] = command

        self.publish_payload(payload)
