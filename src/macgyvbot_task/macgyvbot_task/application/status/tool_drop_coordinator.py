"""Coordinate tool drop monitoring with task status reporting."""

from __future__ import annotations

from macgyvbot_manipulation.tool_drop_monitor import ToolDropMonitor


class ToolDropCoordinator:
    """Own drop monitoring and translate events to application status payloads."""

    def __init__(self, gripper, publish_drop_payload, publish_status_payload):
        self.gripper = gripper
        self.publish_drop_payload = publish_drop_payload
        self.publish_status_payload = publish_status_payload
        self.monitor = ToolDropMonitor(gripper, self._publish_event)

    def start(self, tool_name, action, command=None):
        self.monitor.start(tool_name, action, command)

    def stop(self, reason="intentional_release"):
        self.monitor.stop(reason)

    def release_gripper(self, reason="manual_release"):
        self.stop(reason)
        self.gripper.open_gripper()

    def _publish_event(self, payload):
        self.publish_drop_payload(payload)
        if payload.get("event") != "tool_dropped":
            return

        self.publish_status_payload(
            {
                "status": "tool_dropped",
                "tool_name": payload.get("tool_name", "unknown"),
                "action": payload.get("action", "unknown"),
                "message": "그리퍼 grasp 성공 이후 공구 drop이 감지되었습니다.",
                "reason": payload.get("reason", "grip_lost_after_grasp_success"),
                "width_mm": payload.get("width_mm"),
                "gripper_status": payload.get("status"),
                "command": payload.get("command"),
            }
        )
