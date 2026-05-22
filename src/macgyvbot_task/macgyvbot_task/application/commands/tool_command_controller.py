"""Application-level command routing for robot tasks."""

from __future__ import annotations

import time


class ToolCommandController:
    """Route parsed tool commands to pick, return, release, or pause actions."""

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
        move_home,
    ):
        self.logger = logger
        self.status = status_publisher
        self.is_busy = is_busy
        self.set_target = set_target
        self.clear_target = clear_target
        self.reset_search_status = reset_search_status
        self.start_return = start_return
        self.release_gripper = release_gripper
        self.move_home = move_home
        self._last_busy_command_key = None
        self._last_busy_command_sec = 0.0
        self._busy_command_dedupe_sec = 2.0
        self._recent_command_ids = {}
        self._last_command_signature = None
        self._last_command_signature_sec = 0.0
        self._command_dedupe_sec = 20.0

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
            if self._is_duplicate_busy_command("bring", val):
                return False
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

    def _is_duplicate_busy_command(self, action, tool_name):
        key = (action, tool_name)
        now = time.monotonic()
        if (
            key == self._last_busy_command_key
            and now - self._last_busy_command_sec < self._busy_command_dedupe_sec
        ):
            return True
        self._last_busy_command_key = key
        self._last_busy_command_sec = now
        return False

    def handle_command(self, command):
        if self._is_duplicate_command(command):
            return

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

        if action == "pause":
            self._handle_stop(tool_name, action, command)
            return

        if action == "home":
            self._handle_home(tool_name, action, command)
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

    def _is_duplicate_command(self, command):
        now = time.monotonic()
        stale_before = now - self._command_dedupe_sec
        self._recent_command_ids = {
            command_id: stamp
            for command_id, stamp in self._recent_command_ids.items()
            if stamp >= stale_before
        }

        command_id = command.get("command_id")
        if command_id:
            if command_id in self._recent_command_ids:
                self.logger.warn(f"중복 command_id 무시: {command_id}")
                return True
            self._recent_command_ids[command_id] = now
            return False

        signature = (
            command.get("action", "unknown"),
            command.get("tool_name", "unknown"),
            command.get("target_mode", "unknown"),
            command.get("raw_text", ""),
            command.get("match_method", ""),
        )
        if (
            signature == self._last_command_signature
            and now - self._last_command_signature_sec < self._command_dedupe_sec
        ):
            self.logger.warn(f"중복 command signature 무시: {signature}")
            return True
        self._last_command_signature = signature
        self._last_command_signature_sec = now
        return False

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

    def _handle_home(self, tool_name, action, command):
        if self.is_busy():
            self.logger.warn("로봇 동작 중 home 명령은 즉시 실행하지 않습니다.")
            self.status.publish(
                "busy",
                tool_name=tool_name,
                action=action,
                message="로봇이 작업 중입니다. 먼저 정지한 뒤 Home 복귀를 요청해주세요.",
                reason="already_picking",
                command=command,
            )
            return

        self.logger.info("home 명령 수신: Home 위치로 복귀합니다.")
        self.status.publish(
            "returning_home",
            tool_name=tool_name,
            action=action,
            message="Home 위치로 복귀하는 중입니다.",
            command=command,
        )
        ok = self.move_home()
        if not ok:
            self.status.publish(
                "failed",
                tool_name=tool_name,
                action=action,
                message="Home 위치 복귀에 실패했습니다.",
                reason="home_motion_failed",
                command=command,
            )
            return

        self.release_gripper(reason="home_return")
        self.status.publish(
            "done",
            tool_name=tool_name,
            action=action,
            message="Home 위치로 복귀하고 그리퍼를 열었습니다.",
            command=command,
        )

    def _handle_stop(self, tool_name, action, command):
        self.logger.warn("pause 명령 수신")
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
