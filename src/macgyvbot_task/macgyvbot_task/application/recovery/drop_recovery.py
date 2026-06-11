"""Common drop recovery orchestration."""

from __future__ import annotations

from macgyvbot_task.application.recovery.recovery_utils import (
    RecoveryConfig,
    attempt_grasp,
    cleanup_after_recovery,
    clear_recovery_perception_lock,
    clear_remaining_tasks,
    detect_target_tool,
    is_graspable,
    log_recovery_event,
    move_to_inspection_pose,
    normalize_tool_name,
    open_gripper_after_inspection,
    publish_recovery_status,
    set_recovery_mode,
)


def run_drop_recovery(
    status,
    task_queue,
    yolo_detector,
    motion_controller,
    grasp_planner,
    gripper,
    drawer_controller,
    config,
    logger,
    task_type,
) -> bool:
    """Recover a dropped tool by re-grasping it, then returning home."""
    task_type = _normalize_task_type(task_type)
    config = _ensure_config(config, status, task_type)
    target_tool = _select_target_tool(status, config, task_type)

    log_recovery_event(
        logger,
        "RECOVERY_STARTED",
        f"{task_type} recovery started",
        {"task_type": task_type, "target_tool": target_tool},
    )
    clear_remaining_tasks(task_queue)
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
    if not move_to_inspection_pose(motion_controller, config):
        return _fail(
            status,
            drawer_controller,
            motion_controller,
            config,
            logger,
            task_type,
            target_tool,
            "motion_planning_failed",
            "PLANNING_FAILED",
            f"{task_type} recovery inspection pose move failed",
        )

    if not open_gripper_after_inspection(gripper, config, logger):
        return _fail(
            status,
            drawer_controller,
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
    detection = _detect_target(config, yolo_detector, target_tool)
    if detection is None:
        return _fail(
            status,
            drawer_controller,
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
        if not config.target_observe_fn(detection, target_tool, logger):
            return _fail(
                status,
                drawer_controller,
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

        detection = _redetect_target(config, yolo_detector, target_tool)
        if detection is None:
            return _fail(
                status,
                drawer_controller,
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

    if not is_graspable(detection, motion_controller, grasp_planner, config):
        return _fail(
            status,
            drawer_controller,
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
    if not attempt_grasp(
        detection,
        motion_controller,
        grasp_planner,
        gripper,
        max_retry=config.max_grasp_retry,
        config=config,
        target_tool=target_tool,
    ):
        return _fail(
            status,
            drawer_controller,
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
    if config.tool_hold_monitor is not None:
        config.tool_hold_monitor.start(target_tool, _monitor_action(task_type), config.command)

    cleanup_ok = cleanup_after_recovery(
        status,
        drawer_controller,
        motion_controller,
        config,
        logger,
        task_type,
        target_tool,
        reason="recovery_succeeded",
    )
    log_recovery_event(
        logger,
        "RECOVERY_SUCCEEDED" if cleanup_ok else "RECOVERY_FAILED",
        f"{task_type} recovery finished",
        {"task_type": task_type, "target_tool": target_tool, "cleanup_ok": cleanup_ok},
    )
    return bool(cleanup_ok)


def _detect_target(config, yolo_detector, target_tool):
    return detect_target_tool(
        config.initial_detect_target_fn or config.detect_target_fn or yolo_detector,
        target_tool,
        max_retry=config.max_detection_retry,
        retry_wait_sec=config.detection_retry_wait_sec,
        timeout_sec=config.detection_timeout_sec,
        wait_fn=config.wait_fn,
    )


def _redetect_target(config, yolo_detector, target_tool):
    return detect_target_tool(
        config.detect_target_fn or yolo_detector,
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
    drawer_controller,
    motion_controller,
    config,
    logger,
    task_type,
    target_tool,
    reason,
    event_type,
    message,
):
    log_recovery_event(
        logger,
        event_type,
        message,
        {"task_type": task_type, "target_tool": target_tool, "reason": reason},
    )
    cleanup_after_recovery(
        status,
        drawer_controller,
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


def _ensure_config(config, status, task_type):
    if config is None:
        return RecoveryConfig(state=status, task_type=task_type)
    if isinstance(config, RecoveryConfig):
        if config.state is None:
            config.state = status
        config.task_type = task_type
        return config
    return RecoveryConfig(state=status, task_type=task_type)


def _monitor_action(task_type):
    if task_type == "return":
        return "return"
    return "bring"


def _normalize_task_type(task_type):
    task_type = str(task_type or "").strip().lower()
    if task_type == "return":
        return "return"
    return "pick"
