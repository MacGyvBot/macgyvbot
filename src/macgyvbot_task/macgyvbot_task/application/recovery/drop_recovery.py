"""Common drop recovery orchestration."""

from __future__ import annotations

from macgyvbot_task.application.task_control.task_step import TaskStep
from macgyvbot_task.application.recovery.recovery_utils import (
    RecoveryConfig,
    attempt_grasp,
    cleanup_after_recovery,
    clear_recovery_perception_lock,
    close_open_drawer_after_recovery_failure,
    detect_target_tool,
    is_graspable,
    log_recovery_event,
    move_to_inspection_pose,
    normalize_tool_name,
    open_gripper_after_inspection,
    publish_recovery_grasp_success,
    publish_recovery_status,
    recovery_restart_requested,
    run_recovery_step_with_pause_retry,
    set_recovery_mode,
    wait_for_recovery_tool_mask_lock,
)


class DropRecoverySequenceRunner:
    """Build queue steps for drop recovery so pause/resume uses task queue semantics."""

    def __init__(
        self,
        status,
        motion_controller,
        gripper,
        config,
        logger,
        task_type,
    ):
        self.status = status
        self.motion_controller = motion_controller
        self.gripper = gripper
        self.config = config
        self.logger = logger
        self.task_type = task_type
        self.target_tool = _select_target_tool(status, config, task_type)
        self.detection = None

    def build_steps(self):
        return [
            TaskStep(
                "recovery/start",
                self._start,
                retry_on_pause=False,
            ),
            TaskStep(
                "recovery/move_to_inspection_pose",
                self._move_to_inspection_pose,
            ),
            TaskStep(
                "recovery/open_gripper_after_inspection",
                self._open_gripper_after_inspection,
            ),
            TaskStep(
                "recovery/detect_target",
                self._detect_initial_target,
            ),
            TaskStep(
                "recovery/move_to_target_observe",
                self._move_to_target_observe,
            ),
            TaskStep(
                "recovery/redetect_target",
                self._redetect_target,
            ),
            TaskStep(
                "recovery/check_graspability",
                self._check_graspability,
            ),
            TaskStep(
                "recovery/attempt_grasp",
                self._attempt_grasp,
            ),
            TaskStep(
                "recovery/wait_tool_mask_lock",
                self._wait_tool_mask_lock,
            ),
            TaskStep(
                "recovery/cleanup",
                self._cleanup,
            ),
        ]

    def _start(self):
        log_recovery_event(
            self.logger,
            "RECOVERY_STARTED",
            f"{self.task_type} recovery started",
            {"task_type": self.task_type, "target_tool": self.target_tool},
        )
        set_recovery_mode(self.status, True)
        clear_recovery_perception_lock(self.status, self.logger)
        publish_recovery_status(
            self.status,
            "recovering",
            self.target_tool,
            "drop recovery started.",
            reason="drop_recovery_started",
        )
        return True

    def _move_to_inspection_pose(self):
        publish_recovery_status(
            self.status,
            "recovering",
            self.target_tool,
            "Moving to inspection pose to find the dropped tool.",
            reason="moving_to_recovery_inspection",
        )
        return self._run_step(
            "moving_to_recovery_inspection",
            lambda: move_to_inspection_pose(self.motion_controller, self.config),
            "motion_planning_failed",
            "PLANNING_FAILED",
            f"{self.task_type} recovery inspection pose move failed",
        )

    def _open_gripper_after_inspection(self):
        return self._run_step(
            "inspection_gripper_open",
            lambda: open_gripper_after_inspection(
                self.gripper,
                self.config,
                self.logger,
            ),
            "inspection_gripper_open_failed",
            "RECOVERY_FAILED",
            f"{self.task_type} recovery gripper open after inspection failed",
        )

    def _detect_initial_target(self):
        publish_recovery_status(
            self.status,
            "recovering",
            self.target_tool,
            "Searching dropped tool bbox again.",
            reason="detecting_recovery_target",
        )

        def detect_initial_target():
            self.detection = _detect_target(self.config, self.target_tool)
            return self.detection is not None

        ok = self._run_step(
            "detecting_recovery_target",
            detect_initial_target,
            "target_detection_failed",
            "TARGET_NOT_FOUND",
            f"{self.task_type} recovery target not found",
        )
        if not ok:
            self.detection = None
        return ok

    def _move_to_target_observe(self):
        if self.config.target_observe_fn is None:
            return True

        publish_recovery_status(
            self.status,
            "recovering",
            self.target_tool,
            "Moving to target observe pose.",
            reason="moving_to_recovery_target_observe",
        )
        return self._run_step(
            "moving_to_recovery_target_observe",
            lambda: self.config.target_observe_fn(
                self.detection,
                self.target_tool,
                self.logger,
            ),
            "target_observe_move_failed",
            "PLANNING_FAILED",
            f"{self.task_type} recovery target observe pose move failed",
        )

    def _redetect_target(self):
        if self.config.target_observe_fn is None:
            return True

        if self.task_type == "return":
            self.target_tool = _update_return_target_from_observed_label(
                self.status,
                self.config,
                self.logger,
                self.target_tool,
            )

        def redetect_target():
            self.detection = _redetect_target(self.config, self.target_tool)
            return self.detection is not None

        ok = self._run_step(
            "redetecting_recovery_target",
            redetect_target,
            "target_redetection_failed",
            "TARGET_NOT_FOUND",
            f"{self.task_type} recovery target not found at observe pose",
        )
        if not ok:
            self.detection = None
            return False

        publish_recovery_status(
            self.status,
            "recovering",
            self.target_tool,
            "Fresh bbox/depth target and yaw PCA were recomputed.",
            reason="recovery_target_redetected",
        )
        return True

    def _check_graspability(self):
        return self._run_step(
            "checking_recovery_graspability",
            lambda: is_graspable(
                self.detection,
                self.motion_controller,
                None,
                self.config,
            ),
            "graspability_check_failed",
            "GRASPABILITY_CHECK_FAILED",
            f"{self.task_type} recovery target is not graspable",
        )

    def _attempt_grasp(self):
        publish_recovery_status(
            self.status,
            "recovering",
            self.target_tool,
            "Applying yaw PCA and attempting recovery grasp.",
            reason="recovery_grasp_started",
        )
        ok = self._run_step(
            "recovery_grasp_started",
            lambda: attempt_grasp(
                self.detection,
                self.motion_controller,
                self.gripper,
                config=self.config,
                target_tool=self.target_tool,
            ),
            "grasp_execution_failed",
            "GRASP_ATTEMPT_FAILED",
            f"{self.task_type} recovery grasp failed",
        )
        if not ok:
            return False

        self.status.held_tool = self.target_tool
        self.status.gripper_holding = True
        publish_recovery_grasp_success(self.status, self.config, self.target_tool)
        return True

    def _wait_tool_mask_lock(self):
        ok = self._run_step(
            "recovery_tool_mask_lock",
            lambda: wait_for_recovery_tool_mask_lock(
                self.config,
                self.logger,
                self.target_tool,
            ),
            "tool_mask_lock_failed",
            "RECOVERY_FAILED",
            f"{self.task_type} recovery tool mask lock failed after grasp",
        )
        if not ok:
            return False

        if self.config.tool_hold_monitor is not None:
            self.config.tool_hold_monitor.start(
                self.target_tool,
                _monitor_action(self.task_type),
                self.config.command,
            )
        return True

    def _cleanup(self):
        cleanup_ok = run_recovery_step_with_pause_retry(
            self.config,
            self.status,
            self.logger,
            self.target_tool,
            "recovery_cleanup",
            lambda: cleanup_after_recovery(
                self.status,
                self.motion_controller,
                self.config,
                self.logger,
                self.task_type,
                self.target_tool,
                reason="recovery_succeeded",
            ),
        )
        log_recovery_event(
            self.logger,
            "RECOVERY_SUCCEEDED" if cleanup_ok else "RECOVERY_FAILED",
            f"{self.task_type} recovery finished",
            {
                "task_type": self.task_type,
                "target_tool": self.target_tool,
                "cleanup_ok": cleanup_ok,
            },
        )
        return bool(cleanup_ok)

    def _run_step(self, reason, step_fn, failure_reason, event_type, message):
        if run_recovery_step_with_pause_retry(
            self.config,
            self.status,
            self.logger,
            self.target_tool,
            reason,
            step_fn,
        ):
            return True

        return _fail(
            self.status,
            self.motion_controller,
            self.config,
            self.logger,
            self.task_type,
            self.target_tool,
            failure_reason,
            event_type,
            message,
        )


def build_drop_recovery_steps(
    status,
    motion_controller,
    gripper,
    config,
    logger,
    task_type,
):
    runner = DropRecoverySequenceRunner(
        status,
        motion_controller,
        gripper,
        config,
        logger,
        task_type,
    )
    return runner.build_steps()


def run_drop_recovery(
    status,
    motion_controller,
    gripper,
    config,
    logger,
    task_type,
) -> bool:
    """Recover a dropped tool by re-grasping it, then returning home."""
    target_tool = _select_target_tool(status, config, task_type)

    log_recovery_event(
        logger,
        "RECOVERY_STARTED",
        f"{task_type} recovery started",
        {"task_type": task_type, "target_tool": target_tool},
    )
    set_recovery_mode(status, True)
    clear_recovery_perception_lock(status, logger)
    publish_recovery_status(
        status,
        "recovering",
        target_tool,
        "drop recovery started.",
        reason="drop_recovery_started",
    )

    publish_recovery_status(
        status,
        "recovering",
        target_tool,
        "Moving to inspection pose to find the dropped tool.",
        reason="moving_to_recovery_inspection",
    )
    if not run_recovery_step_with_pause_retry(
        config,
        status,
        logger,
        target_tool,
        "moving_to_recovery_inspection",
        lambda: move_to_inspection_pose(motion_controller, config),
    ):
        return _fail(
            status,
            motion_controller,
            config,
            logger,
            task_type,
            target_tool,
            "motion_planning_failed",
            "PLANNING_FAILED",
            f"{task_type} recovery inspection pose move failed",
        )

    if not run_recovery_step_with_pause_retry(
        config,
        status,
        logger,
        target_tool,
        "inspection_gripper_open",
        lambda: open_gripper_after_inspection(gripper, config, logger),
    ):
        return _fail(
            status,
            motion_controller,
            config,
            logger,
            task_type,
            target_tool,
            "inspection_gripper_open_failed",
            "RECOVERY_FAILED",
            f"{task_type} recovery gripper open after inspection failed",
        )

    publish_recovery_status(
        status,
        "recovering",
        target_tool,
        "Searching dropped tool bbox again.",
        reason="detecting_recovery_target",
    )
    detection = None

    def detect_initial_target():
        nonlocal detection
        detection = _detect_target(config, target_tool)
        return detection is not None

    detection_ok = run_recovery_step_with_pause_retry(
        config,
        status,
        logger,
        target_tool,
        "detecting_recovery_target",
        detect_initial_target,
    )
    if not detection_ok:
        detection = None
    if detection is None:
        return _fail(
            status,
            motion_controller,
            config,
            logger,
            task_type,
            target_tool,
            "target_detection_failed",
            "TARGET_NOT_FOUND",
            f"{task_type} recovery target not found",
        )

    if config.target_observe_fn is not None:
        publish_recovery_status(
            status,
            "recovering",
            target_tool,
            "Moving to target observe pose.",
            reason="moving_to_recovery_target_observe",
        )
        if not run_recovery_step_with_pause_retry(
            config,
            status,
            logger,
            target_tool,
            "moving_to_recovery_target_observe",
            lambda: config.target_observe_fn(detection, target_tool, logger),
        ):
            return _fail(
                status,
                motion_controller,
                config,
                logger,
                task_type,
                target_tool,
                "target_observe_move_failed",
                "PLANNING_FAILED",
                f"{task_type} recovery target observe pose move failed",
            )

        if task_type == "return":
            target_tool = _update_return_target_from_observed_label(
                status,
                config,
                logger,
                target_tool,
            )

        detection = None

        def redetect_target():
            nonlocal detection
            detection = _redetect_target(config, target_tool)
            return detection is not None

        redetect_ok = run_recovery_step_with_pause_retry(
            config,
            status,
            logger,
            target_tool,
            "redetecting_recovery_target",
            redetect_target,
        )
        if not redetect_ok:
            detection = None
        if detection is None:
            return _fail(
                status,
                motion_controller,
                config,
                logger,
                task_type,
                target_tool,
                "target_redetection_failed",
                "TARGET_NOT_FOUND",
                f"{task_type} recovery target not found at observe pose",
            )
        publish_recovery_status(
            status,
            "recovering",
            target_tool,
            "Fresh bbox/depth target and yaw PCA were recomputed.",
            reason="recovery_target_redetected",
        )

    if not run_recovery_step_with_pause_retry(
        config,
        status,
        logger,
        target_tool,
        "checking_recovery_graspability",
        lambda: is_graspable(detection, motion_controller, None, config),
    ):
        return _fail(
            status,
            motion_controller,
            config,
            logger,
            task_type,
            target_tool,
            "graspability_check_failed",
            "GRASPABILITY_CHECK_FAILED",
            f"{task_type} recovery target is not graspable",
        )

    publish_recovery_status(
        status,
        "recovering",
        target_tool,
        "Applying yaw PCA and attempting recovery grasp.",
        reason="recovery_grasp_started",
    )
    if not run_recovery_step_with_pause_retry(
        config,
        status,
        logger,
        target_tool,
        "recovery_grasp_started",
        lambda: attempt_grasp(
            detection,
            motion_controller,
            gripper,
            config=config,
            target_tool=target_tool,
        ),
    ):
        return _fail(
            status,
            motion_controller,
            config,
            logger,
            task_type,
            target_tool,
            "grasp_execution_failed",
            "GRASP_ATTEMPT_FAILED",
            f"{task_type} recovery grasp failed",
        )

    status.held_tool = target_tool
    status.gripper_holding = True
    publish_recovery_grasp_success(status, config, target_tool)
    if not run_recovery_step_with_pause_retry(
        config,
        status,
        logger,
        target_tool,
        "recovery_tool_mask_lock",
        lambda: wait_for_recovery_tool_mask_lock(config, logger, target_tool),
    ):
        return _fail(
            status,
            motion_controller,
            config,
            logger,
            task_type,
            target_tool,
            "tool_mask_lock_failed",
            "RECOVERY_FAILED",
            f"{task_type} recovery tool mask lock failed after grasp",
        )

    if config.tool_hold_monitor is not None:
        config.tool_hold_monitor.start(target_tool, _monitor_action(task_type), config.command)

    cleanup_ok = run_recovery_step_with_pause_retry(
        config,
        status,
        logger,
        target_tool,
        "recovery_cleanup",
        lambda: cleanup_after_recovery(
            status,
            motion_controller,
            config,
            logger,
            task_type,
            target_tool,
            reason="recovery_succeeded",
        ),
    )
    log_recovery_event(
        logger,
        "RECOVERY_SUCCEEDED" if cleanup_ok else "RECOVERY_FAILED",
        f"{task_type} recovery finished",
        {"task_type": task_type, "target_tool": target_tool, "cleanup_ok": cleanup_ok},
    )
    return bool(cleanup_ok)


def _detect_target(config, target_tool):
    return detect_target_tool(
        config.initial_detect_target_fn or config.detect_target_fn,
        target_tool,
        max_retry=config.max_detection_retry,
        retry_wait_sec=config.detection_retry_wait_sec,
        timeout_sec=config.detection_timeout_sec,
        wait_fn=config.wait_fn,
    )
def _redetect_target(config, target_tool):
    return detect_target_tool(
        config.detect_target_fn,
        target_tool,
        max_retry=config.max_detection_retry,
        retry_wait_sec=config.detection_retry_wait_sec,
        timeout_sec=config.detection_timeout_sec,
        wait_fn=config.wait_fn,
    )


def _update_return_target_from_observed_label(status, config, logger, target_tool):
    observed_tool = _observed_tool_label(config)
    if observed_tool == "unknown" or observed_tool == target_tool:
        return target_tool

    log_recovery_event(
        logger,
        "TARGET_SELECTED",
        "return recovery target updated from observed label",
        {
            "task_type": "return",
            "target_tool": observed_tool,
            "previous_target_tool": target_tool,
        },
    )
    status.held_tool = observed_tool
    return observed_tool


def _observed_tool_label(config):
    if config.observed_tool_label_fn is None:
        return "unknown"
    return normalize_tool_name(config.observed_tool_label_fn())


def _select_target_tool(status, config, task_type):
    if task_type == "return":
        held_tool = normalize_tool_name(getattr(status, "held_tool", None))
        if held_tool != "unknown":
            return held_tool

    command = getattr(config, "command", None) or {}
    candidates = (
        getattr(status, "target_tool", None),
        getattr(status, "target_label", None),
        command.get("tool_name"),
    )
    for candidate in candidates:
        tool_name = normalize_tool_name(candidate)
        if tool_name != "unknown":
            return tool_name
    return "unknown"


def _fail(
    status,
    motion_controller,
    config,
    logger,
    task_type,
    target_tool,
    reason,
    event_type,
    message,
):
    if recovery_restart_requested(config):
        log_recovery_event(
            logger,
            "RECOVERY_RESTART_REQUESTED",
            "drop recovery restart requested by a new drop event",
            {"task_type": task_type, "target_tool": target_tool, "reason": reason},
        )
        set_recovery_mode(status, False)
        return False

    failure_message = _recovery_failure_message_for_event(
        reason,
        event_type,
        drawer_open=bool(getattr(status, "drawer_open", False)),
    )
    if failure_message is not None:
        publish_recovery_status(
            status,
            "recovering",
            target_tool,
            failure_message,
            reason=reason,
        )
        close_open_drawer_after_recovery_failure(
            status,
            config,
            logger,
            target_tool,
            reason,
        )

    log_recovery_event(
        logger,
        event_type,
        message,
        {"task_type": task_type, "target_tool": target_tool, "reason": reason},
    )
    cleanup_after_recovery(
        status,
        motion_controller,
        config,
        logger,
        task_type,
        target_tool,
        reason=reason,
    )
    log_recovery_event(
        logger,
        "RECOVERY_FAILED",
        f"{task_type} recovery failed",
        {"task_type": task_type, "target_tool": target_tool, "reason": reason},
    )
    return False


def _monitor_action(task_type):
    if task_type == "return":
        return "return"
    return "bring"


def _recovery_failure_message(reason):
    if reason in {"target_detection_failed", "target_redetection_failed"}:
        return "공구를 못찾겠습니다. 서랍을 닫고 홈 위치로 복귀합니다."
    if reason in {
        "motion_planning_failed",
        "target_observe_move_failed",
        "graspability_check_failed",
        "grasp_execution_failed",
    }:
        return "공구를 못잡겠습니다. 서랍을 닫고 홈 위치로 복귀합니다."
    return None


def _recovery_failure_message_for_event(reason, event_type, drawer_open=False):
    if (
        reason in {"target_detection_failed", "target_redetection_failed"}
        or event_type == "TARGET_NOT_FOUND"
    ):
        return "공구를 못찾겠습니다. 서랍을 닫고 홈 위치로 복귀합니다."
    if (
        reason in {
            "motion_planning_failed",
            "target_observe_move_failed",
            "graspability_check_failed",
            "grasp_execution_failed",
        }
        or event_type in {
            "PLANNING_FAILED",
            "GRASPABILITY_CHECK_FAILED",
            "GRASP_ATTEMPT_FAILED",
        }
    ):
        return "공구를 못잡겠습니다. 서랍을 닫고 홈 위치로 복귀합니다."
    if drawer_open:
        return "recovery가 실패했습니다. 서랍을 닫고 홈 위치로 복귀합니다."
    return _recovery_failure_message(reason)
