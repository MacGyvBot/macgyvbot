"""Application-level command routing for robot tasks."""

from __future__ import annotations


class ToolCommandController:
    """Route parsed tool commands to pick, return, release, or stop actions."""

    def __init__(
        self,
        logger,
        status_publisher,
        is_busy,
        set_target,
        clear_target,
        reset_search_status,
        start_return,
        release_gripper,
    ):
        self.logger = logger
        self.status = status_publisher
        self.is_busy = is_busy
        self.set_target = set_target
        self.clear_target = clear_target
        self.reset_search_status = reset_search_status
        self.start_return = start_return
        self.release_gripper = release_gripper

    def handle_target_label(self, tool_name, source="/target_label", command=None):
        val = (tool_name or "").strip()

        if not val or val == "unknown":
            self.status.publish(
                "rejected",
                tool_name=val or "unknown",
                message="대상 공구가 비어 있거나 unknown입니다.",
                reason="unknown_tool",
                command=command,
            )
            return False

        if self.is_busy():
            self.logger.warn(
                f"현재 pick 동작 중이라 새 타겟 '{val}' 입력은 무시합니다."
            )
            self.status.publish(
                "busy",
                tool_name=val,
                message=f"현재 pick 동작 중이라 새 타겟 '{val}' 입력은 무시합니다.",
                reason="already_picking",
                command=command,
            )
            return False

        self.set_target(val, command)
        self.reset_search_status()
        self.logger.info(f"타겟 객체 설정: {val} ({source})")
        self.status.publish(
            "accepted",
            tool_name=val,
            action="bring",
            message=f"{val} 탐색을 시작합니다.",
            command=command,
        )
        return True

    def handle_command(self, command):
        action = command.get("action", "unknown")
        tool_name = command.get("tool_name", "unknown")

        if action == "bring":
            self.handle_target_label(tool_name, source="/tool_command", command=command)
            return

        if action == "return":
            self.start_return(command)
            return

        if action == "release":
            self._handle_release(tool_name, action, command)
            return

        if action == "stop":
            self._handle_stop(tool_name, action, command)
            return

        self.logger.warn(f"지원하지 않는 action: {action}")
        self.status.publish(
            "rejected",
            tool_name=tool_name,
            action=action,
            message="지원하지 않는 명령입니다.",
            reason="unsupported_action",
            command=command,
        )

    def _handle_release(self, tool_name, action, command):
        if self.is_busy():
            self.logger.warn("pick 동작 중 release 명령은 수동 실행하지 않습니다.")
            self.status.publish(
                "busy",
                tool_name=tool_name,
                action=action,
                message="pick 동작 중에는 자동 핸드오프 절차가 그리퍼를 제어합니다.",
                reason="handoff_controls_release",
                command=command,
            )
            return

        self.logger.info("release 명령 수신: 그리퍼를 엽니다.")
        self.release_gripper()
        self.status.publish(
            "done",
            tool_name=tool_name,
            action=action,
            message="그리퍼를 열었습니다.",
            command=command,
        )

    def _handle_stop(self, tool_name, action, command):
        self.logger.warn("stop 명령 수신")
        if self.is_busy():
            self.status.publish(
                "busy",
                tool_name=tool_name,
                action=action,
                message=(
                    "이미 실행 중인 MoveIt 동작은 안전 중단을 지원하지 않아 "
                    "완료를 기다립니다."
                ),
                reason="active_motion_not_interruptible",
                command=command,
            )
            return

        self.clear_target()
        self.status.publish(
            "cancelled",
            tool_name=tool_name,
            action=action,
            message="대기 중인 pick 요청을 취소했습니다.",
            command=command,
        )
