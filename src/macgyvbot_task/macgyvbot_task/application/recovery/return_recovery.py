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

    if target_tool == "unknown":
        log_recovery_event(
            logger,
            "TARGET_NORMALIZED_TO_UNKNOWN",
            "return recovery 대상 공구를 알 수 없어 unknown으로 처리합니다.",
            {"task_type": "return", "target_tool": target_tool},
        )

    log_recovery_event(
        logger,
        "RECOVERY_STARTED",
        "return recovery를 시작합니다.",
        {"task_type": "return", "target_tool": target_tool},
    )
    clear_remaining_tasks(task_queue)
    set_recovery_mode(status, True)
    # TODO: interrupt/resume recovery에서는 중단된 return step과 drawer 상태를
    # 여기서 snapshot으로 보관한 뒤 재개 정책을 선택합니다.

    if not move_to_inspection_pose(motion_controller, config):
        return _fail(
            status,
            drawer_controller,
            motion_controller,
            config,
            logger,
            "return",
            target_tool,
            "motion_planning_failed",
            "PLANNING_FAILED",
            "복구 관찰 자세로 이동하지 못했습니다.",
        )

    log_recovery_event(
        logger,
        "TARGET_SELECTED",
        "return recovery 대상을 선택했습니다.",
        {"task_type": "return", "target_tool": target_tool},
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
            "return",
            target_tool,
            "target_detection_failed",
            "TARGET_NOT_FOUND",
            "복구 대상 공구를 찾지 못했습니다.",
        )

    log_recovery_event(
        logger,
        "TARGET_DETECTED",
        "복구 대상 공구를 감지했습니다.",
        {
            "task_type": "return",
            "target_tool": target_tool,
            "confidence": getattr(detection, "confidence", None),
        },
    )
    log_recovery_event(
        logger,
        "GRASPABILITY_CHECK_STARTED",
        "복구 grasp 가능 여부를 확인합니다.",
        {"task_type": "return", "target_tool": target_tool},
    )
    if not is_graspable(detection, motion_controller, grasp_planner, config):
        return _fail(
            status,
            drawer_controller,
            motion_controller,
            config,
            logger,
            "return",
            target_tool,
            "graspability_check_failed",
            "GRASPABILITY_CHECK_FAILED",
            "복구 대상 공구가 현재 자세에서 grasp 가능하지 않습니다.",
        )

    drawer_id = _drawer_id_for_tool(drawer_controller, tool_drawer_map, target_tool)
    if drawer_id is None:
        return _fail(
            status,
            drawer_controller,
            motion_controller,
            config,
            logger,
            "return",
            target_tool,
            "unknown_drawer_mapping",
            "RECOVERY_FAILED",
            "복구 대상 공구에 대응하는 서랍을 찾지 못했습니다.",
        )

    log_recovery_event(
        logger,
        "DRAWER_OPEN_STARTED",
        "복구 대상 서랍을 엽니다.",
        {"task_type": "return", "target_tool": target_tool, "drawer_id": drawer_id},
    )
    if drawer_controller is not None and not drawer_controller.open_drawer(
        drawer_id,
        logger,
    ):
        return _fail(
            status,
            drawer_controller,
            motion_controller,
            config,
            logger,
            "return",
            target_tool,
            "drawer_open_failed",
            "DRAWER_OPEN_FAILED",
            "복구 대상 서랍 열기에 실패했습니다.",
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
            "return",
            target_tool,
            "drawer_observe_failed",
            "PLANNING_FAILED",
            "복구 대상 서랍 관찰 자세 이동에 실패했습니다.",
        )
    marker_target = _resolve_drawer_marker(config, drawer_id, target_tool, logger)
    if marker_target is None:
        return _fail(
            status,
            drawer_controller,
            motion_controller,
            config,
            logger,
            "return",
            target_tool,
            "drawer_marker_not_found",
            "TARGET_NOT_FOUND",
            "서랍 marker target을 찾지 못했습니다.",
        )

    log_recovery_event(
        logger,
        "GRASP_ATTEMPT_STARTED",
        "복구 grasp를 시도합니다.",
        {"task_type": "return", "target_tool": target_tool, "drawer_id": drawer_id},
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
            "return",
            target_tool,
            "grasp_execution_failed",
            "GRASP_ATTEMPT_FAILED",
            "복구 grasp에 실패했습니다.",
        )

    status.held_tool = target_tool
    status.gripper_holding = True
    if config.tool_hold_monitor is not None:
        config.tool_hold_monitor.start(target_tool, "return", config.command)
    log_recovery_event(
        logger,
        "GRASP_SUCCEEDED",
        "복구 grasp가 성공했습니다.",
        {"task_type": "return", "target_tool": target_tool, "drawer_id": drawer_id},
    )

    if not _place_recovered_tool(config, drawer_id, target_tool, logger, marker_target):
        return _fail(
            status,
            drawer_controller,
            motion_controller,
            config,
            logger,
            "return",
            target_tool,
            "drawer_place_failed",
            "RECOVERY_FAILED",
            "복구 대상 공구를 서랍에 배치하지 못했습니다.",
        )

    status.held_tool = None
    status.gripper_holding = False
    drawer_closed = True
    if drawer_controller is not None:
        log_recovery_event(
            logger,
            "DRAWER_CLOSE_STARTED",
            "복구 대상 서랍을 닫습니다.",
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
                "복구 대상 서랍 닫기에 실패했습니다.",
                {
                    "task_type": "return",
                    "target_tool": target_tool,
                    "drawer_id": drawer_id,
                    "reason": "drawer_close_failed",
                },
            )
    else:
        status.drawer_open = False
        status.opened_drawer_id = None

    home_ok = return_home(motion_controller, config)
    set_recovery_mode(status, False)
    log_recovery_event(
        logger,
        "RECOVERY_SUCCEEDED" if drawer_closed and home_ok else "RECOVERY_FAILED",
        "return recovery가 완료되었습니다.",
        {
            "task_type": "return",
            "target_tool": target_tool,
            "drawer_id": drawer_id,
            "drawer_closed": drawer_closed,
            "home_ok": home_ok,
        },
    )
    log_recovery_event(
        logger,
        "RECOVERY_FINISHED",
        "복구 절차를 종료했습니다.",
        {
            "task_type": "return",
            "target_tool": target_tool,
            "drawer_id": drawer_id,
        },
    )
    return bool(drawer_closed and home_ok)


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


def _resolve_drawer_marker(config, drawer_id, target_tool, logger):
    if config.drawer_marker_target_fn is None or config.place_tool_fn is None:
        log_recovery_event(
            logger,
            "RECOVERY_FAILED",
            "서랍 배치 adapter가 없어 return recovery를 완료할 수 없습니다.",
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
            "서랍 marker target을 찾지 못했습니다.",
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
        log_recovery_event(
            logger,
            "RECOVERY_FAILED",
            "서랍 배치 adapter가 없어 return recovery를 완료할 수 없습니다.",
            {
                "task_type": "return",
                "target_tool": target_tool,
                "drawer_id": drawer_id,
                "reason": "drawer_place_adapter_missing",
            },
        )
        return False
    return bool(config.place_tool_fn(marker_target, target_tool, logger))


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
        "return recovery가 실패했습니다.",
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
