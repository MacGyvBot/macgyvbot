"""Shared runtime state for the main task node and workflow runners."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable


@dataclass
class TaskRuntimeState:
    """Mutable state shared by ROS callbacks and task workflow runners."""

    logger_provider: Callable[[], Any]
    publish_robot_status: Callable[..., None]
    publish_status_payload: Callable[[dict], None]

    color_image: Any = None
    depth_image: Any = None
    intrinsics: dict | None = None
    picking: bool = False
    target_label: str | None = None
    pending_pick_thread: Any = None
    pending_return_thread: Any = None
    human_grasped_tool: bool = False
    last_grasp_result: dict | None = None
    tool_mask_locked: bool = False
    last_tool_mask_lock_result: dict | None = None
    hand_grasp_image: Any = None
    latest_wrench: Any = None
    home_xyz: tuple | None = None
    home_ori: dict | None = None
    current_command: dict | None = None
    _last_search_status_target: str | None = field(default=None, repr=False)

    def logger(self):
        return self.logger_provider()

    def get_logger(self):
        return self.logger_provider()

    def _publish_robot_status(
        self,
        status,
        tool_name=None,
        action=None,
        message="",
        reason="",
        command=None,
    ):
        self.publish_robot_status(
            status,
            tool_name=tool_name,
            action=action,
            message=message,
            reason=reason,
            command=command,
        )

    def _publish_status_payload(self, payload):
        self.publish_status_payload(payload)
