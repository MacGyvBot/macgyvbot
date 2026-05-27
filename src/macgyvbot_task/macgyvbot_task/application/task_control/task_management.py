"""Task control side effects for exit, pause, and resume requests."""

from __future__ import annotations


class TaskManagement:
    """Apply task-control requests outside the queue execution loop."""

    def __init__(
        self,
        state,
        coordinator,
        exit_event,
        pause_event,
        resume_event,
        logger_provider,
    ):
        self.state = state
        self.coordinator = coordinator
        self.exit_event = exit_event
        self.pause_event = pause_event
        self.resume_event = resume_event
        self.logger_provider = logger_provider

    def handle_control(self, action, reason="", publish_status=True):
        action = (action or "").strip().lower()

        if action == "exit":
            return self._handle_exit(reason, publish_status=publish_status)

        if action == "pause":
            return self._handle_pause(reason)
        if action == "resume":
            return self._handle_resume(reason)

        self.logger().warn(f"지원하지 않는 task control action: {action}")
        return False

    def _handle_exit(self, reason, publish_status=True):
        self.exit_event.set()
        self.pause_event.clear()
        self.resume_event.clear()
        self._clear_task_queue()
        if publish_status:
            self.state._publish_robot_status(
                "cancelled",
                message="사용자 요청으로 작업을 중단합니다.",
                reason=reason or "exit_requested",
                command=self.state.current_command,
            )
        return True

    def _handle_pause(self, reason):
        if self.exit_event.is_set():
            return False

        self.pause_event.set()
        self.resume_event.clear()
        self.state._publish_robot_status(
            "paused",
            message="사용자 요청으로 작업을 일시정지합니다.",
            reason=reason or "pause_requested",
            command=self.state.current_command,
        )
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

    def _clear_task_queue(self):
        self.coordinator.clear_queue()

    def logger(self):
        return self.logger_provider()
