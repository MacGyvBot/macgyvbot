"""Monitor gripper state for unexpected tool drops after grasp success."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

from macgyvbot_config.grasp import (
    DROP_MONITOR_GRACE_SEC,
    DROP_MONITOR_POLL_SEC,
    DROP_MONITOR_STABLE_COUNT,
    DROP_MONITOR_STOP_TIMEOUT_SEC,
    DROP_MONITOR_WIDTH_RELEASE_DELTA_MM,
    GRIPPER_EMPTY_CLOSED_WIDTH_THRESHOLD_MM,
)


@dataclass(frozen=True)
class GripperHoldState:
    busy: bool
    grip_detected: bool
    status: list
    width_mm: float | None


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
        baseline_width_mm = _read_width_mm(self.gripper, logger)
        lost_count = 0
        loss_reason = "grip_lost_after_grasp_success"
        width_error_reported = False

        while self.ok_fn() and not stop_event.is_set():
            if time.monotonic() - start_time < DROP_MONITOR_GRACE_SEC:
                self.wait_fn(DROP_MONITOR_POLL_SEC)
                continue

            try:
                state = read_gripper_hold_state(
                    self.gripper,
                    logger,
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

            if state.width_mm is None:
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

            current_loss_reason = classify_tool_drop_sample(
                state,
                baseline_width_mm,
            )
            if current_loss_reason is None:
                lost_count = 0
            else:
                loss_reason = current_loss_reason
                lost_count += 1

            if lost_count >= DROP_MONITOR_STABLE_COUNT:
                if self._is_current_session(session_id, stop_event):
                    self.publish_event(
                        {
                            "event": "tool_dropped",
                            "tool_name": tool_name or "unknown",
                            "action": action or "unknown",
                            "reason": loss_reason,
                            "width_mm": state.width_mm,
                            "baseline_width_mm": baseline_width_mm,
                            "empty_closed_width_threshold_mm": (
                                GRIPPER_EMPTY_CLOSED_WIDTH_THRESHOLD_MM
                            ),
                            "status": state.status,
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


def read_gripper_hold_state(gripper, logger):
    """Return the raw gripper state needed for hold/drop monitoring."""
    status = gripper.get_status()
    grip_detected = len(status) > 1 and bool(status[1])
    busy = bool(status[0]) if status else False
    width_mm = _read_width_mm(gripper, logger)
    return GripperHoldState(
        busy=busy,
        grip_detected=grip_detected,
        status=status,
        width_mm=width_mm,
    )


def classify_tool_drop_sample(
    state,
    baseline_width_mm,
    empty_closed_width_threshold_mm=GRIPPER_EMPTY_CLOSED_WIDTH_THRESHOLD_MM,
    release_delta_mm=DROP_MONITOR_WIDTH_RELEASE_DELTA_MM,
):
    """Return a drop reason for one monitor sample, or None while held."""
    if state.busy:
        return None

    if not state.grip_detected:
        return "grip_detected_signal_lost"

    width_mm = state.width_mm
    if width_mm is None or baseline_width_mm is None:
        return None

    if width_mm <= empty_closed_width_threshold_mm:
        if baseline_width_mm > empty_closed_width_threshold_mm:
            return "width_reached_empty_closed_threshold"
        return None

    if baseline_width_mm - width_mm >= release_delta_mm:
        return "width_reduced_from_held_baseline"

    return None


def _read_width_mm(gripper, logger):
    try:
        return float(gripper.get_width())
    except Exception as exc:
        logger.warn(f"그리퍼 폭 읽기 실패: {exc}")
        return None


def _sleep_wait(duration_sec):
    time.sleep(max(0.0, float(duration_sec)))
