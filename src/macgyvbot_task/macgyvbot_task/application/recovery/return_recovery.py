"""Return/drop recovery orchestration."""

from __future__ import annotations

from macgyvbot_task.application.recovery.recovery_utils import (
    RecoveryConfig,
    attempt_grasp,
    cleanup_after_recovery,
    clear_remaining_tasks,
    detect_target_tool,
    is_graspable,
    log_recovery_event,
    move_to_inspection_pose,
    normalize_tool_name,
    return_home,
    set_recovery_mode,
)


def run_return_recovery(
    status,
    task_queue,
    yolo_detector,
    motion_controller,
    grasp_planner,
    gripper,
    drawer_controller,
    tool_drawer_map,
    config,
    logger,
) -> bool:
    """Recover a dropped return tool and store it in the mapped drawer."""
    config = _ensure_config(config, status, "return")
    target_tool = _select_return_target(status)

    log_recovery_event(
        logger,
        "RECOVERY_STARTED",
        "return recovery started",
        {"task_type": "return", "target_tool": target_tool},
    )
    clear_remaining_tasks(task_queue)
    set_recovery_mode(status, True)
    # TODO: later interrupt/resume support should snapshot the interrupted
    # return step and drawer state here.

    if not move_to_inspection_pose(motion_controller, config):
        return _fail(
            status,
            drawer_controller,
            motion_controller,
            config,
            logger,
            target_tool,
            "motion_planning_failed",
            "PLANNING_FAILED",
            "return recovery inspection pose move failed",
        )

    detection = detect_target_tool(
        config.initial_detect_target_fn or config.detect_target_fn or yolo_detector,
        target_tool,
        max_retry=config.max_detection_retry,
    )
    if detection is None:
        return _fail(
            status,
            drawer_controller,
            motion_controller,
            config,
            logger,
            target_tool,
            "target_detection_failed",
            "TARGET_NOT_FOUND",
            "return recovery target not found",
        )

    if config.target_observe_fn is not None:
        if not config.target_observe_fn(detection, target_tool, logger):
            return _fail(
                status,
                drawer_controller,
                motion_controller,
                config,
                logger,
                target_tool,
                "target_observe_move_failed",
                "PLANNING_FAILED",
                "return recovery target observe pose move failed",
            )
        observed_tool = _observed_tool_label(config)
        if observed_tool != "unknown" and observed_tool != target_tool:
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
            target_tool = observed_tool
            status.held_tool = target_tool

    drawer_id = _drawer_id_for_tool(drawer_controller, tool_drawer_map, target_tool)
    if drawer_id is None:
        return _fail(
            status,
            drawer_controller,
            motion_controller,
            config,
            logger,
            target_tool,
            "unknown_drawer_mapping",
            "RECOVERY_FAILED",
            "return recovery drawer mapping missing",
        )

    marker_target = None
    drawer_already_open = _drawer_is_open(status, drawer_controller, drawer_id)
    if drawer_already_open:
        status.drawer_open = True
        status.opened_drawer_id = drawer_id
        log_recovery_event(
            logger,
            "DRAWER_OPEN_STARTED",
            "return recovery drawer already open; skipping drawer open before grasp",
            {
                "task_type": "return",
                "target_tool": target_tool,
                "drawer_id": drawer_id,
                "reason": "drawer_already_open",
            },
        )
    elif drawer_controller is not None:
        log_recovery_event(
            logger,
            "DRAWER_OPEN_STARTED",
            "return recovery drawer open started",
            {
                "task_type": "return",
                "target_tool": target_tool,
                "drawer_id": drawer_id,
            },
        )
        if not drawer_controller.open_drawer(drawer_id, logger):
            return _fail(
                status,
                drawer_controller,
                motion_controller,
                config,
                logger,
                target_tool,
                "drawer_open_failed",
                "DRAWER_OPEN_FAILED",
                "return recovery drawer open failed",
            )
        status.drawer_open = True
        status.opened_drawer_id = drawer_id

        observe_drawer = getattr(drawer_controller, "observe_drawer", None)
        if observe_drawer is not None and not observe_drawer(drawer_id, logger):
            return _fail(
                status,
                drawer_controller,
                motion_controller,
                config,
                logger,
                target_tool,
                "drawer_observe_failed",
                "PLANNING_FAILED",
                "return recovery drawer observe failed",
            )

        marker_target = _resolve_drawer_marker(config, drawer_id, target_tool, logger)
        if marker_target is None:
            return _fail(
                status,
                drawer_controller,
                motion_controller,
                config,
                logger,
                target_tool,
                "drawer_marker_not_found",
                "TARGET_NOT_FOUND",
                "return recovery drawer marker not found",
            )

    if config.target_observe_fn is not None:
        if not config.target_observe_fn(detection, target_tool, logger):
            return _fail(
                status,
                drawer_controller,
                motion_controller,
                config,
                logger,
                target_tool,
                "target_observe_move_failed",
                "PLANNING_FAILED",
                "return recovery target observe remount failed",
            )

    detection = detect_target_tool(
        config.detect_target_fn or yolo_detector,
        target_tool,
        max_retry=config.max_detection_retry,
    )
    if detection is None:
        return _fail(
            status,
            drawer_controller,
            motion_controller,
            config,
            logger,
            target_tool,
            "target_redetection_failed",
            "TARGET_NOT_FOUND",
            "return recovery target not found at grasp observe pose",
        )
    if not is_graspable(detection, motion_controller, grasp_planner, config):
        return _fail(
            status,
            drawer_controller,
            motion_controller,
            config,
            logger,
            target_tool,
            "graspability_check_failed",
            "GRASPABILITY_CHECK_FAILED",
            "return recovery target is not graspable",
        )

    log_recovery_event(
        logger,
        "GRASP_ATTEMPT_STARTED",
        "return recovery grasp started",
        {
            "task_type": "return",
            "target_tool": target_tool,
            "drawer_id": drawer_id,
        },
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
            target_tool,
            "grasp_execution_failed",
            "GRASP_ATTEMPT_FAILED",
            "return recovery grasp failed",
        )

    status.held_tool = target_tool
    status.gripper_holding = True
    if config.tool_hold_monitor is not None:
        config.tool_hold_monitor.start(target_tool, "return", config.command)

    if marker_target is None:
        observe_drawer = getattr(drawer_controller, "observe_drawer", None)
        if observe_drawer is not None and not observe_drawer(drawer_id, logger):
            return _fail(
                status,
                drawer_controller,
                motion_controller,
                config,
                logger,
                target_tool,
                "drawer_observe_failed",
                "PLANNING_FAILED",
                "return recovery drawer observe after grasp failed",
            )
        marker_target = _resolve_drawer_marker(config, drawer_id, target_tool, logger)
        if marker_target is None:
            return _fail(
                status,
                drawer_controller,
                motion_controller,
                config,
                logger,
                target_tool,
                "drawer_marker_not_found",
                "TARGET_NOT_FOUND",
                "return recovery drawer marker not found after grasp",
            )

    if not _place_recovered_tool(config, drawer_id, target_tool, logger, marker_target):
        return _fail(
            status,
            drawer_controller,
            motion_controller,
            config,
            logger,
            target_tool,
            "drawer_place_failed",
            "RECOVERY_FAILED",
            "return recovery drawer placement failed",
        )

    status.held_tool = None
    status.gripper_holding = False
    drawer_closed = _close_drawer(drawer_controller, status, drawer_id, target_tool, logger)
    home_ok = return_home(motion_controller, config)
    set_recovery_mode(status, False)

    log_recovery_event(
        logger,
        "RECOVERY_SUCCEEDED" if drawer_closed and home_ok else "RECOVERY_FAILED",
        "return recovery finished",
        {
            "task_type": "return",
            "target_tool": target_tool,
            "drawer_id": drawer_id,
            "drawer_closed": drawer_closed,
            "home_ok": home_ok,
        },
    )
    return bool(drawer_closed and home_ok)


def _observed_tool_label(config):
    if config.observed_tool_label_fn is None:
        return "unknown"
    return normalize_tool_name(config.observed_tool_label_fn())


def _select_return_target(status):
    held_tool = normalize_tool_name(getattr(status, "held_tool", None))
    if held_tool != "unknown":
        return held_tool
    target_tool = normalize_tool_name(
        getattr(status, "target_tool", None) or getattr(status, "target_label", None)
    )
    if target_tool != "unknown":
        return target_tool
    return "unknown"


def _drawer_id_for_tool(drawer_controller, tool_drawer_map, target_tool):
    if drawer_controller is not None:
        resolver = getattr(drawer_controller, "drawer_id_for_tool", None)
        if resolver is not None:
            drawer_id = resolver(target_tool)
            if drawer_id is not None:
                return drawer_id
    if tool_drawer_map is None:
        return None
    return tool_drawer_map.get(target_tool)


def _drawer_is_open(status, drawer_controller, drawer_id):
    if getattr(status, "drawer_open", False) and (
        getattr(status, "opened_drawer_id", None) in (None, drawer_id)
    ):
        return True
    opened = getattr(drawer_controller, "_opened_drawers", None)
    return bool(opened and drawer_id in opened)


def _resolve_drawer_marker(config, drawer_id, target_tool, logger):
    if config.drawer_marker_target_fn is None or config.place_tool_fn is None:
        log_recovery_event(
            logger,
            "RECOVERY_FAILED",
            "return recovery drawer placement adapter missing",
            {
                "task_type": "return",
                "target_tool": target_tool,
                "drawer_id": drawer_id,
                "reason": "drawer_place_adapter_missing",
            },
        )
        return None

    marker_target = config.drawer_marker_target_fn(drawer_id)
    if marker_target is None or not getattr(marker_target, "found", False):
        log_recovery_event(
            logger,
            "TARGET_NOT_FOUND",
            "return recovery drawer marker target not found",
            {
                "task_type": "return",
                "target_tool": target_tool,
                "drawer_id": drawer_id,
                "reason": getattr(marker_target, "reason", "drawer_marker_not_found"),
            },
        )
        return None
    return marker_target


def _place_recovered_tool(config, drawer_id, target_tool, logger, marker_target):
    if config.place_tool_fn is None:
        return False
    return bool(config.place_tool_fn(marker_target, target_tool, logger))


def _close_drawer(drawer_controller, status, drawer_id, target_tool, logger):
    if drawer_controller is None:
        status.drawer_open = False
        status.opened_drawer_id = None
        return True

    log_recovery_event(
        logger,
        "DRAWER_CLOSE_STARTED",
        "return recovery drawer close started",
        {
            "task_type": "return",
            "target_tool": target_tool,
            "drawer_id": drawer_id,
        },
    )
    drawer_closed = bool(drawer_controller.close_drawer(drawer_id, logger))
    if drawer_closed:
        status.drawer_open = False
        status.opened_drawer_id = None
    else:
        log_recovery_event(
            logger,
            "DRAWER_CLOSE_FAILED",
            "return recovery drawer close failed",
            {
                "task_type": "return",
                "target_tool": target_tool,
                "drawer_id": drawer_id,
                "reason": "drawer_close_failed",
            },
        )
    return drawer_closed


def _fail(
    status,
    drawer_controller,
    motion_controller,
    config,
    logger,
    target_tool,
    reason,
    event_type,
    message,
):
    log_recovery_event(
        logger,
        event_type,
        message,
        {"task_type": "return", "target_tool": target_tool, "reason": reason},
    )
    cleanup_after_recovery(
        status,
        drawer_controller,
        motion_controller,
        config,
        logger,
        "return",
        target_tool,
        reason=reason,
    )
    log_recovery_event(
        logger,
        "RECOVERY_FAILED",
        "return recovery failed",
        {"task_type": "return", "target_tool": target_tool, "reason": reason},
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
