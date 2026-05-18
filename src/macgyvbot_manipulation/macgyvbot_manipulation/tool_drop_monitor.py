"""Monitor gripper state for unexpected tool drops after grasp success."""

from __future__ import annotations

import threading
import time

from macgyvbot_config.grasp import (
    DROP_MONITOR_GRACE_SEC,
    DROP_MONITOR_POLL_SEC,
    DROP_MONITOR_STABLE_COUNT,
    DROP_MONITOR_STOP_TIMEOUT_SEC,
)
from macgyvbot_manipulation.grasp_verifier import read_grasp_confirmation


class ToolDropMonitor:
    """Publish one event when a confirmed gripper grasp is unexpectedly lost."""

    def __init__(self, gripper, publish_event, wait_fn=None, ok_fn=None):
        self.gripper = gripper
        self.publish_event = publish_event
        self.wait_fn = wait_fn or _sleep_wait
        self.ok_fn = ok_fn or (lambda: True)
        self._lock = threading.Lock()
        self._stop_event = None
        self._thread = None
        self._session_id = 0

    def start(self, tool_name, action, command=None):
        """Start monitoring the current grasp until stopped or a drop is found."""
        self.stop("restart")
        with self._lock:
            self._session_id += 1
            session_id = self._session_id
            self._stop_event = threading.Event()
            self._thread = threading.Thread(
                target=self._run,
                args=(session_id, self._stop_event, tool_name, action, command),
                daemon=True,
            )
            self._thread.start()

    def stop(self, reason="intentional_release", wait=True):
        """Stop monitoring before an intentional gripper open or workflow cleanup."""
        with self._lock:
            stop_event = self._stop_event
            thread = self._thread
            self._stop_event = None
            self._thread = None
        if stop_event is not None:
            stop_event.set()
        if (
            wait
            and thread is not None
            and thread is not threading.current_thread()
        ):
            thread.join(timeout=DROP_MONITOR_STOP_TIMEOUT_SEC)

    def _run(self, session_id, stop_event, tool_name, action, command):
        logger = _EventLogger()
        start_time = time.monotonic()
        lost_count = 0
        width_error_reported = False

        while self.ok_fn() and not stop_event.is_set():
            if time.monotonic() - start_time < DROP_MONITOR_GRACE_SEC:
                self.wait_fn(DROP_MONITOR_POLL_SEC)
                continue

            try:
                confirmed, busy, status, width_mm = read_grasp_confirmation(
                    self.gripper,
                    logger,
                    log_success=False,
                )
            except Exception as exc:
                if self._is_current_session(session_id, stop_event):
                    self.publish_event(
                        {
                            "event": "tool_drop_monitor_error",
                            "tool_name": tool_name or "unknown",
                            "action": action or "unknown",
                            "reason": "gripper_state_read_failed",
                            "error": str(exc),
                            "command": command,
                        }
                    )
                return

            if width_mm is None:
                lost_count = 0
                if not width_error_reported:
                    width_error_reported = True
                    self._publish_monitor_error(
                        session_id,
                        stop_event,
                        tool_name,
                        action,
                        command,
                        "gripper_width_read_failed",
                        "gripper width is unavailable",
                    )
                self.wait_fn(DROP_MONITOR_POLL_SEC)
                continue

            width_error_reported = False

            if busy or confirmed:
                lost_count = 0
            else:
                lost_count += 1

            if lost_count >= DROP_MONITOR_STABLE_COUNT:
                if self._is_current_session(session_id, stop_event):
                    self.publish_event(
                        {
                            "event": "tool_dropped",
                            "tool_name": tool_name or "unknown",
                            "action": action or "unknown",
                            "reason": "grip_lost_after_grasp_success",
                            "width_mm": width_mm,
                            "status": status,
                            "command": command,
                        }
                    )
                    self.stop("drop_detected", wait=False)
                return

            self.wait_fn(DROP_MONITOR_POLL_SEC)

    def _publish_monitor_error(
        self,
        session_id,
        stop_event,
        tool_name,
        action,
        command,
        reason,
        error,
    ):
        if not self._is_current_session(session_id, stop_event):
            return

        self.publish_event(
            {
                "event": "tool_drop_monitor_error",
                "tool_name": tool_name or "unknown",
                "action": action or "unknown",
                "reason": reason,
                "error": error,
                "command": command,
            }
        )

    def _is_current_session(self, session_id, stop_event):
        with self._lock:
            return (
                self._session_id == session_id
                and self._stop_event is stop_event
            )


class _EventLogger:
    def info(self, message):
        pass

    def warn(self, message):
        pass


def _sleep_wait(duration_sec):
    time.sleep(max(0.0, float(duration_sec)))
