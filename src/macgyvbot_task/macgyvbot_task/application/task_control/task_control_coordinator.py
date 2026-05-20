"""Task queue coordinator for pick and return workflows."""

from __future__ import annotations

from collections import deque
import threading
import time
import traceback

import rclpy

from macgyvbot_config.timing import SEQUENCE_WAIT_POLL_SEC
from macgyvbot_task.application.task_control.task_step import TaskStep


class TaskControlCoordinator:
    """Load workflow steps into a queue and execute them in a worker thread."""

    def __init__(
        self,
        pick_sequence,
        return_sequence,
        state,
        stop_event,
        pause_event,
        resume_event,
        logger_provider,
    ):
        self.pick_sequence = pick_sequence
        self.return_sequence = return_sequence
        self.state = state
        self.stop_event = stop_event
        self.pause_event = pause_event
        self.resume_event = resume_event
        self.logger_provider = logger_provider
        self._queue = deque()
        self._lock = threading.Lock()
        self._thread = None

    def start_pick(self, bx, by, bz, z_m, vlm_yaw_deg=None):
        return self._start(
            task_name="pick",
            step_builder=lambda: self.pick_sequence.build_steps(
                bx,
                by,
                bz,
                z_m,
                vlm_yaw_deg,
            ),
        )

    def start_return(self, command):
        return self._start(
            task_name="return",
            step_builder=lambda: self.return_sequence.build_steps(command),
        )

    def clear_queue(self):
        """Clear queued steps when requested by task management."""
        with self._lock:
            self._queue.clear()

    def is_running(self):
        return self._thread is not None and self._thread.is_alive()

    def _start(self, task_name, step_builder):
        if self.is_running():
            self.logger().warn(
                f"이미 task coordinator가 실행 중이라 {task_name} 요청을 무시합니다."
            )
            return False

        self.stop_event.clear()
        self.resume_event.clear()
        self._thread = threading.Thread(
            target=self._run,
            args=(task_name, step_builder),
            daemon=True,
        )
        self._thread.start()
        return True

    def _run(self, task_name, step_builder):
        log = self.logger()
        completed = False
        stopped = False
        failed = False
        failure_status_published = False

        try:
            try:
                steps = list(step_builder())
            except Exception as exc:
                failed = True
                self._log_step_exception(
                    log,
                    task_name,
                    step_name=None,
                    exc=exc,
                )
                failure_status_published = self._publish_task_exception_status(
                    task_name,
                    step_name=None,
                    exc=exc,
                    reason="task_step_build_exception",
                )
                return

            with self._lock:
                self._queue.clear()
                self._queue.extend(steps)

            log.info(f"{task_name} task queue 로딩 완료: {len(steps)} steps")
            while rclpy.ok():
                if self.stop_event.is_set():
                    stopped = True
                    break

                if self.pause_event.is_set():
                    time.sleep(SEQUENCE_WAIT_POLL_SEC)
                    continue

                step = self._pop_step()
                if step is None:
                    completed = True
                    break

                log.info(f"task step 시작: {step.name}")
                try:
                    ok = step.execute()
                except Exception as exc:
                    failed = True
                    self._log_step_exception(
                        log,
                        task_name,
                        step_name=step.name,
                        exc=exc,
                    )
                    failure_status_published = self._publish_task_exception_status(
                        task_name,
                        step_name=step.name,
                        exc=exc,
                        reason="task_step_exception",
                    )
                    break

                if ok:
                    log.info(f"task step 완료: {step.name}")
                    continue

                if self.pause_event.is_set() and step.retry_on_pause:
                    log.info(f"task step 일시정지: {step.name}")
                    self._push_front(step)
                    continue

                if self.stop_event.is_set():
                    stopped = True
                    break

                log.error(f"task step 실패: {step.name}")
                failed = True
                break

        finally:
            if stopped:
                log.info(f"{task_name} task queue 중단")
            elif failed:
                if not failure_status_published:
                    log.warn(
                        f"{task_name} task queue 실패 상태는 step 내부 publish에 위임됨"
                    )
                log.info(f"{task_name} task queue 실패로 종료")
            elif completed:
                log.info(f"{task_name} task queue 완료")

            self._clear_task_state()
            self.stop_event.clear()
            self.resume_event.clear()

    def _pop_step(self):
        with self._lock:
            if not self._queue:
                return None
            return self._queue.popleft()

    def _push_front(self, step: TaskStep):
        with self._lock:
            self._queue.appendleft(step)

    def _clear_task_state(self):
        self.state.picking = False
        self.state.target_label = None
        self.state.human_grasped_tool = False
        self.state.current_command = None

    def _publish_task_exception_status(self, task_name, step_name, exc, reason):
        publish = getattr(self.state, "_publish_robot_status", None)
        if publish is None:
            return False

        action = "bring" if task_name == "pick" else task_name
        step_label = step_name or "build_steps"
        exc_type = type(exc).__name__
        publish(
            "failed",
            tool_name=self.state.target_label,
            action=action,
            message=(
                f"{task_name} task step 예외로 작업을 중단합니다: "
                f"{step_label} ({exc_type})"
            ),
            reason=reason,
            command=self.state.current_command,
        )
        return True

    @staticmethod
    def _log_step_exception(log, task_name, step_name, exc):
        step_label = step_name or "build_steps"
        log.error(
            f"{task_name} task step 예외 발생: "
            f"step={step_label}, error={type(exc).__name__}: {exc}"
        )
        log.error(traceback.format_exc())

    def logger(self):
        return self.logger_provider()
