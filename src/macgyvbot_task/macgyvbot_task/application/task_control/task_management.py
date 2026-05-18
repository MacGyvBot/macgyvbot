"""Task control side effects for stop, pause, and resume requests."""

from __future__ import annotations


class TaskManagement:
    """Apply task-control requests outside the queue execution loop."""

    def __init__(
        self,
        state,
        coordinator,
        motion_cancel_client,
        stop_event,
        pause_event,
        resume_event,
        logger_provider,
    ):
        self.state = state
        self.coordinator = coordinator
        self.motion_cancel_client = motion_cancel_client
        self.stop_event = stop_event
        self.pause_event = pause_event
        self.resume_event = resume_event
        self.logger_provider = logger_provider

    def handle_control(self, action, reason=""):
        action = (action or "").strip().lower()

        if action == "stop":
            return self._handle_stop(reason)
        if action == "pause":
            return self._handle_pause(reason)
        if action == "resume":
            return self._handle_resume(reason)

        self.logger().warn(f"지원하지 않는 task control action: {action}")
        return False

    def _handle_stop(self, reason):
        self.stop_event.set()
        self.pause_event.clear()
        self.resume_event.clear()
        self._clear_task_queue()
        self._cancel_motion("stop")
        self.state._publish_robot_status(
            "cancelled",
            message="사용자 요청으로 작업을 중단합니다.",
            reason=reason or "stop_requested",
            command=self.state.current_command,
        )
        return True

    def _handle_pause(self, reason):
        if self.stop_event.is_set():
            return False

        self.pause_event.set()
        self.resume_event.clear()
        self.state._publish_robot_status(
            "paused",
            message="사용자 요청으로 작업을 일시정지합니다.",
            reason=reason or "pause_requested",
            command=self.state.current_command,
        )
        self._cancel_motion("pause")
        return True

    def _handle_resume(self, reason):
        self.pause_event.clear()
        self.resume_event.clear()
        self.state._publish_robot_status(
            "resumed",
            message="작업을 재개합니다.",
            reason=reason or "resume_requested",
            command=self.state.current_command,
        )
        return True

    def _cancel_motion(self, action):
        if self.motion_cancel_client is None:
            self.logger().warn(f"{action}: motion cancel client가 없습니다.")
            return False
        return self.motion_cancel_client.cancel_all_goals(reason=action)

    def _clear_task_queue(self):
        self.coordinator.clear_queue()

    def logger(self):
        return self.logger_provider()
