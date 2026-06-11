"""Pick/drop recovery orchestration."""

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
    set_recovery_mode,
)
from macgyvbot_task.application.drawer_store_motion import rotate_wrist_for_drawer_store


def run_pick_recovery(
    status,
    task_queue,
    yolo_detector,
    motion_controller,
    grasp_planner,
    gripper,
    drawer_controller,
    config,
    logger,
) -> bool:
    """Recover a dropped delivery tool by finding and re-grasping it."""
    config = _ensure_config(config, status, "pick")
    command = getattr(config, "command", None) or {}
    target_tool = normalize_tool_name(
        getattr(status, "target_tool", None)
        or getattr(status, "target_label", None)
        or command.get("tool_name")
    )
    if target_tool == "unknown":
        log_recovery_event(
            logger,
            "TARGET_NORMALIZED_TO_UNKNOWN",
            "pick recovery 대상 공구를 알 수 없어 unknown으로 처리합니다.",
            {"task_type": "pick", "target_tool": target_tool},
        )

    log_recovery_event(
        logger,
        "RECOVERY_STARTED",
        "pick recovery를 시작합니다.",
        {"task_type": "pick", "target_tool": target_tool},
    )
    clear_remaining_tasks(task_queue)
    set_recovery_mode(status, True)
    clear_recovery_perception_lock(status, logger)
    # TODO: interrupt/resume recovery에서는 기존 queue snapshot을 여기서 보관합니다.

    if not move_to_inspection_pose(motion_controller, config):
        return _fail(
            status,
            drawer_controller,
            motion_controller,
            config,
            logger,
            "pick",
            target_tool,
            "motion_planning_failed",
            "PLANNING_FAILED",
            "복구 관찰 자세로 이동하지 못했습니다.",
        )

    if not open_gripper_after_inspection(gripper, config, logger):
        return _fail(
            status,
            drawer_controller,
            motion_controller,
            config,
            logger,
            "pick",
            target_tool,
            "inspection_gripper_open_failed",
            "RECOVERY_FAILED",
            "pick recovery gripper open after inspection failed",
        )

    log_recovery_event(
        logger,
        "TARGET_SELECTED",
        "pick recovery 대상을 선택했습니다.",
        {"task_type": "pick", "target_tool": target_tool},
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
            "pick",
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
            "task_type": "pick",
            "target_tool": target_tool,
            "confidence": getattr(detection, "confidence", None),
        },
    )
    log_recovery_event(
        logger,
        "GRASPABILITY_CHECK_STARTED",
        "복구 grasp 가능 여부를 확인합니다.",
        {"task_type": "pick", "target_tool": target_tool},
    )
    if config.target_observe_fn is not None:
        log_recovery_event(
            logger,
            "PLANNING_STARTED",
            "복구 대상 위 관찰 자세로 이동합니다.",
            {"task_type": "pick", "target_tool": target_tool},
        )
        if not config.target_observe_fn(detection, target_tool, logger):
            log_recovery_event(
                logger,
                "GRASPABILITY_CHECK_FAILED",
                "복구 대상 공구를 잡을 수 없습니다. 관찰 자세 planning에 실패했습니다.",
                {
                    "task_type": "pick",
                    "target_tool": target_tool,
                    "reason": "target_observe_move_failed",
                },
            )
            return _fail(
                status,
                drawer_controller,
                motion_controller,
                config,
                logger,
                "pick",
                target_tool,
                "target_observe_move_failed",
                "PLANNING_FAILED",
                "복구 대상 관찰 자세 이동에 실패했습니다.",
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
                "pick",
                target_tool,
                "target_redetection_failed",
                "TARGET_NOT_FOUND",
                "복구 대상 관찰 자세에서 공구를 다시 찾지 못했습니다.",
            )
    if not is_graspable(detection, motion_controller, grasp_planner, config):
        log_recovery_event(
            logger,
            "GRASPABILITY_CHECK_FAILED",
            "복구 대상 공구를 잡을 수 없습니다. grasp planning이 실패했습니다.",
            {
                "task_type": "pick",
                "target_tool": target_tool,
                "reason": "graspability_check_failed",
            },
        )
        return _fail(
            status,
            drawer_controller,
            motion_controller,
            config,
            logger,
            "pick",
            target_tool,
            "graspability_check_failed",
            "GRASPABILITY_CHECK_FAILED",
            "복구 대상 공구가 현재 자세에서 grasp 가능하지 않습니다.",
        )

    log_recovery_event(
        logger,
        "GRASP_ATTEMPT_STARTED",
        "복구 grasp를 시도합니다.",
        {"task_type": "pick", "target_tool": target_tool},
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
            "pick",
            target_tool,
            "grasp_execution_failed",
            "GRASP_ATTEMPT_FAILED",
            "복구 grasp에 실패했습니다.",
        )

    status.held_tool = target_tool
    status.gripper_holding = True
    if config.tool_hold_monitor is not None:
        config.tool_hold_monitor.start(target_tool, "bring", config.command)
    log_recovery_event(
        logger,
        "GRASP_SUCCEEDED",
        "복구 grasp가 성공했습니다.",
        {"task_type": "pick", "target_tool": target_tool},
    )
    if not _store_recovered_pick_tool(
        status,
        drawer_controller,
        motion_controller,
        config,
        logger,
        target_tool,
    ):
        return _fail(
            status,
            drawer_controller,
            motion_controller,
            config,
            logger,
            "pick",
            target_tool,
            "pick_recovery_store_failed",
            "RECOVERY_FAILED",
            "복구한 공구를 원래 서랍에 넣지 못했습니다.",
        )

    cleanup_ok = cleanup_after_recovery(
        status,
        drawer_controller,
        motion_controller,
        config,
        logger,
        "pick",
        target_tool,
        reason="recovery_succeeded",
    )
    log_recovery_event(
        logger,
        "RECOVERY_SUCCEEDED",
        "pick recovery가 완료되었습니다.",
        {"task_type": "pick", "target_tool": target_tool, "cleanup_ok": cleanup_ok},
    )
    return True


def _store_recovered_pick_tool(
    status,
    drawer_controller,
    motion_controller,
    config,
    logger,
    target_tool,
):
    drawer_id = _drawer_id_for_tool(drawer_controller, target_tool)
    if drawer_id is None:
        log_recovery_event(
            logger,
            "RECOVERY_FAILED",
            "복구한 공구의 원래 서랍 mapping을 찾지 못했습니다.",
            {
                "task_type": "pick",
                "target_tool": target_tool,
                "reason": "unknown_drawer_mapping",
            },
        )
        return False

    if drawer_controller is None:
        return False

    if not _drawer_is_open(drawer_controller, drawer_id):
        log_recovery_event(
            logger,
            "DRAWER_OPEN_STARTED",
            "복구한 공구를 넣기 위해 원래 서랍을 엽니다.",
            {"task_type": "pick", "target_tool": target_tool, "drawer_id": drawer_id},
        )
        if not drawer_controller.open_drawer(drawer_id, logger):
            return False
    status.drawer_open = True
    status.opened_drawer_id = drawer_id

    if not rotate_wrist_for_drawer_store(
        motion_controller,
        logger,
        label="pick_recovery_store",
    ):
        return False

    observe_drawer = getattr(drawer_controller, "observe_drawer", None)
    if observe_drawer is not None and not observe_drawer(drawer_id, logger):
        return False

    if config.drawer_marker_target_fn is None or config.place_tool_fn is None:
        return False

    marker_target = config.drawer_marker_target_fn(drawer_id)
    if marker_target is None or not getattr(marker_target, "found", False):
        return False

    if not config.place_tool_fn(marker_target, target_tool, logger):
        return False

    status.held_tool = None
    status.gripper_holding = False
    log_recovery_event(
        logger,
        "DRAWER_CLOSE_STARTED",
        "복구한 공구를 넣은 뒤 서랍을 닫습니다.",
        {"task_type": "pick", "target_tool": target_tool, "drawer_id": drawer_id},
    )
    if not drawer_controller.close_drawer(drawer_id, logger):
        return False

    status.drawer_open = False
    status.opened_drawer_id = None
    return True


def _drawer_id_for_tool(drawer_controller, target_tool):
    resolver = getattr(drawer_controller, "drawer_id_for_tool", None)
    if resolver is None:
        return None
    return resolver(target_tool)


def _drawer_is_open(drawer_controller, drawer_id):
    opened = getattr(drawer_controller, "_opened_drawers", None)
    return bool(opened and drawer_id in opened)


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
        "pick recovery가 실패했습니다.",
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
