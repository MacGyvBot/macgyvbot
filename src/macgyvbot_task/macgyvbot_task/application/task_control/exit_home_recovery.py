"""Home recovery after an exit task-control request."""

from __future__ import annotations

import threading
import time


class ExitHomeRecovery:
    """Move the robot Home after exit has stopped the active task queue."""

    def __init__(
        self,
        motion,
        exit_event,
        task_coordinator,
        publish_robot_status,
        logger_provider,
        current_command_provider,
        release_gripper=None,
    ):
        self.motion = motion
        self.exit_event = exit_event
        self.task_coordinator = task_coordinator
        self.publish_robot_status = publish_robot_status
        self.logger_provider = logger_provider
        self.current_command_provider = current_command_provider
        self.release_gripper = release_gripper
        self._lock = threading.Lock()
        self._thread = None

    def move_home_after_exit(self, reason):
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                self.logger().info("exit Home 복귀가 이미 진행 중입니다.")
                return True

            self._thread = threading.Thread(
                target=self._move_home_after_exit,
                args=(reason,),
                name="exit_home_recovery",
                daemon=True,
            )
            self._thread.start()
        return True

    def _move_home_after_exit(self, reason):
        logger = self.logger()
        command = self.current_command_provider()
        if not self._wait_for_task_queue_to_stop(logger):
            self.publish_robot_status(
                "failed",
                action="exit",
                message="종료 요청 후 작업 큐가 멈추지 않아 Home 복귀를 시작하지 못했습니다.",
                reason="exit_queue_shutdown_timeout",
                command=command,
            )
            return False

        self.exit_event.clear()
        logger.info("종료 요청 후 Home 위치로 복귀합니다.")
        self.publish_robot_status(
            "returning_home",
            action="exit",
            message="종료 요청 후 Home 위치로 복귀합니다.",
            reason=reason or "exit_requested",
            command=command,
        )

        ok = self.motion.move_to_home_joints(logger)
        if ok:
            if not self._release_gripper_after_home(logger):
                self.publish_robot_status(
                    "failed",
                    action="exit",
                    message="종료 요청 후 Home 복귀는 완료했지만 그리퍼 열기에 실패했습니다.",
                    reason="exit_gripper_release_failed",
                    command=command,
                )
                return False

            self.publish_robot_status(
                "done",
                action="exit",
                message="종료 요청 후 Home 위치로 복귀하고 그리퍼를 열었습니다.",
                reason=reason or "exit_home_completed",
                command=command,
            )
            return True

        self.publish_robot_status(
            "failed",
            action="exit",
            message="종료 요청 후 Home 위치 복귀에 실패했습니다.",
            reason="exit_home_failed",
            command=command,
        )
        return False

    def _wait_for_task_queue_to_stop(self, logger, timeout_sec=60.0):
        deadline = time.monotonic() + timeout_sec
        while self.task_coordinator.is_running() and time.monotonic() < deadline:
            time.sleep(0.02)

        if self.task_coordinator.is_running():
            logger.warn("exit 요청 후 task queue 종료 대기 시간이 초과되었습니다.")
            return False

        return True

    def _release_gripper_after_home(self, logger):
        if self.release_gripper is None:
            logger.warn("exit Home 복귀 후 실행할 그리퍼 release callback이 없습니다.")
            return False

        try:
            self.release_gripper("exit_home_completed")
        except Exception as exc:
            logger.warn(f"exit Home 복귀 후 그리퍼 열기 실패: {exc}")
            return False

        logger.info("exit Home 복귀 후 그리퍼를 열었습니다.")
        return True

    def logger(self):
        return self.logger_provider()
