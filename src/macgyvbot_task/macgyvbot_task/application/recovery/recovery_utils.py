"""Shared first-version drop recovery orchestration utilities."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import time
from typing import Any

from macgyvbot_config.grasp import GRASP_RETRY_LIMIT
from macgyvbot_config.handoff import OBSERVATION_TIMEOUT_SEC
from macgyvbot_manipulation.grasp_verifier import GraspVerifier
from macgyvbot_manipulation.handover_targeting import move_to_observation_pose
from macgyvbot_manipulation.robot_pose import current_ee_orientation, make_safe_pose
from macgyvbot_manipulation.timing import cooperative_wait
from macgyvbot_task.application.logging_utils import (
    log_error,
    log_info,
    log_warn,
)
from macgyvbot_task.application.pick_flow.pick_target_planner import PickTargetPlanner


@dataclass
class RecoveryConfig:
    """Runtime adapters used by recovery without owning new robot algorithms."""

    robot: Any = None
    state: Any = None
    command: dict | None = None
    task_type: str = "unknown"
    inspection_pose_fn: Callable[[Any, Any, Any], bool] | None = None
    home_fn: Callable[[Any, Any], bool] | None = None
    initial_detect_target_fn: Callable[[str], Any] | None = None
    detect_target_fn: Callable[[str], Any] | None = None
    target_observe_fn: Callable[[Any, str, Any], bool] | None = None
    observed_tool_label_fn: Callable[[], str | None] | None = None
    grasp_fn: Callable[[Any, str, Any], bool] | None = None
    drawer_marker_target_fn: Callable[[int], Any] | None = None
    place_tool_fn: Callable[[Any, str, Any], bool] | None = None
    tool_hold_monitor: Any = None
    wait_fn: Callable[[float], None] = cooperative_wait
    max_detection_retry: int = 3
    max_grasp_retry: int = GRASP_RETRY_LIMIT
    detection_retry_wait_sec: float = 0.2
    recovery_timeout_sec: float = OBSERVATION_TIMEOUT_SEC


def clear_remaining_tasks(task_queue):
    """Clear queued work using the existing queue or cancel interface."""
    if task_queue is None:
        return True

    if hasattr(task_queue, "clear_remaining_tasks"):
        return bool(task_queue.clear_remaining_tasks())
    if hasattr(task_queue, "cancel_remaining_tasks"):
        return bool(task_queue.cancel_remaining_tasks())
    if callable(task_queue):
        return bool(task_queue())

    if isinstance(task_queue, tuple) and len(task_queue) == 2:
        queue, lock = task_queue
        with lock:
            queue.clear()
        return True

    if hasattr(task_queue, "clear"):
        task_queue.clear()
        return True

    return False


def set_recovery_mode(status, enabled: bool):
    if status is not None:
        status.recovery_mode = bool(enabled)


def clear_recovery_perception_lock(status, logger=None):
    """Drop recovery must ignore stale SAM/depth lock state and re-detect by bbox."""
    if status is None:
        return

    for attr in (
        "last_tool_mask_lock_result",
        "grasp_detection_mask_images",
        "grasp_detection_mask_target",
        "grasp_detection_yaw_deg",
        "grasp_detection_yaw_target",
        "hand_grasp_image",
        "last_grasp_result",
    ):
        if hasattr(status, attr):
            setattr(status, attr, None)
    if hasattr(status, "tool_mask_locked"):
        status.tool_mask_locked = False

    log_info(
        logger or _logger(None),
        "recovery perception lock cleared",
        step="recovery_detect",
        event="reset",
        reason="drop_recovery_uses_bbox_center",
    )


def move_to_inspection_pose(motion_controller, config: RecoveryConfig):
    """Move to the existing hand inspection pose used by return handoff."""
    logger = _logger(config)
    if config.inspection_pose_fn is not None:
        return bool(config.inspection_pose_fn(motion_controller, config, logger))

    if config.robot is None:
        log_error(
            logger,
            "inspection pose unavailable",
            step="recovery",
            event="fail",
            reason="robot_missing",
        )
        return False

    ok, _start_pose = move_to_observation_pose(
        motion_controller,
        config.robot,
        logger,
    )
    return bool(ok)


def open_gripper_after_inspection(gripper, config: RecoveryConfig, logger=None):
    """Open the gripper after the robot reaches the drop inspection pose."""
    logger = logger or _logger(config)
    open_gripper = getattr(gripper, "open_gripper", None)
    if open_gripper is None:
        log_warn(
            logger,
            "recovery gripper open unavailable after inspection",
            step="recovery",
            event="fail",
            reason="gripper_open_missing",
        )
        return False

    try:
        open_gripper()
    except Exception as exc:
        log_warn(
            logger,
            "recovery gripper open failed after inspection",
            step="recovery",
            event="fail",
            reason=str(exc) or type(exc).__name__,
        )
        return False

    log_info(
        logger,
        "recovery gripper opened after inspection",
        step="recovery",
        event="gripper_opened",
        task_type=getattr(config, "task_type", "unknown"),
    )
    return True


def detect_target_tool(yolo_detector, target_tool, max_retry: int = 3):
    """Find a tool through an existing detector/resolver callable."""
    target = normalize_tool_name(target_tool)
    for attempt in range(1, int(max_retry) + 1):
        detection = _call_detector(yolo_detector, target)
        if _detection_found(detection):
            return detection
        time.sleep(0.2)
    return None


def is_graspable(detection, motion_controller, grasp_planner, config) -> bool:
    """First-version graspability check delegates to existing target planning."""
    if detection is None or not _detection_found(detection):
        return False
    base_xyz = getattr(detection, "base_xyz", None)
    if base_xyz is None:
        return False

    planner = grasp_planner
    if planner is None and getattr(config, "robot", None) is not None:
        planner = PickTargetPlanner(config.robot)
    if planner is None:
        # TODO: Replace with a richer graspability adapter when interrupt/resume
        # recovery records the original grasp-planning context.
        return True

    try:
        planner.plan(*base_xyz, _logger(config))
    except Exception as exc:
        log_warn(
            _logger(config),
            "recovery graspability planning failed",
            step="recovery",
            event="fail",
            reason=str(exc) or type(exc).__name__,
        )
        return False
    return True


def attempt_grasp(
    detection,
    motion_controller,
    grasp_planner,
    gripper,
    max_retry: int = 3,
    config: RecoveryConfig | None = None,
    target_tool: str = "unknown",
) -> bool:
    """Attempt a recovery grasp using existing planner and verifier helpers."""
    logger = _logger(config)
    if config is not None and config.grasp_fn is not None:
        return bool(config.grasp_fn(detection, target_tool, logger))

    base_xyz = getattr(detection, "base_xyz", None)
    if base_xyz is None:
        return False

    robot = getattr(config, "robot", None)
    if robot is None:
        log_error(
            logger,
            "recovery grasp failed",
            step="recovery",
            event="fail",
            reason="robot_missing",
        )
        return False

    planner = grasp_planner or PickTargetPlanner(robot)
    plan = planner.plan(*base_xyz, logger)

    yaw_deg = getattr(detection, "yaw_deg", None)
    if yaw_deg is not None and hasattr(motion_controller, "rotate_wrist_by_yaw_deg"):
        log_info(
            logger,
            "recovery grasp yaw applied",
            step="recovery",
            event="yaw",
            target=target_tool,
            yaw_deg=yaw_deg,
        )
        if not motion_controller.rotate_wrist_by_yaw_deg(
            yaw_deg,
            logger,
            collision_scene_key="recovery/apply_wrist_yaw",
        ):
            return False

    ori = current_ee_orientation(robot)

    if hasattr(gripper, "open_gripper"):
        gripper.open_gripper()

    if not motion_controller.plan_and_execute(
        logger,
        pose_goal=make_safe_pose(
            plan.target_x,
            plan.target_y,
            plan.drawer_wall_clearance_z,
            ori,
            logger,
        ),
        collision_scene_key="recovery/grasp_xy_align",
    ):
        return False

    if plan.should_descend_to_grasp and not motion_controller.plan_and_execute(
        logger,
        pose_goal=make_safe_pose(
            plan.target_x,
            plan.target_y,
            plan.grasp_z,
            ori,
            logger,
        ),
        collision_scene_key="recovery/grasp_descent",
    ):
        return False

    verifier = GraspVerifier(
        gripper,
        getattr(config, "wait_fn", cooperative_wait),
        interrupted=lambda: False,
    )
    # GraspVerifier owns the package retry policy. max_retry is kept in the
    # wrapper signature so a future interrupt/resume recovery issue can pass a
    # per-recovery limit without changing callers.
    _ = max_retry
    return bool(verifier.try_grasp(logger, failure_prefix="recovery grasp"))


def close_drawer_if_open(drawer_controller, status):
    """Close an open drawer if status or drawer controller records one."""
    if drawer_controller is None:
        return True

    drawer_id = getattr(status, "opened_drawer_id", None)
    if drawer_id is None:
        opened = getattr(drawer_controller, "_opened_drawers", None)
        if opened:
            drawer_id = next(iter(opened))

    if drawer_id is None:
        return True

    logger = status.logger() if hasattr(status, "logger") else None
    if logger is None:
        logger = _NullLogger()
    ok = bool(drawer_controller.close_drawer(drawer_id, logger))
    if ok and status is not None:
        status.drawer_open = False
        status.opened_drawer_id = None
    return ok


def return_home(motion_controller, config):
    logger = _logger(config)
    if getattr(config, "home_fn", None) is not None:
        return bool(config.home_fn(motion_controller, logger))
    return bool(motion_controller.move_to_home_joints(logger))


def log_recovery_event(logger, event_type, message, extra=None):
    fields = dict(extra or {})
    level = "error" if event_type.endswith("FAILED") else "info"
    fields.setdefault("step", "recovery")
    fields.setdefault("event", event_type)
    if level == "error":
        log_error(logger, message, **fields)
    else:
        log_info(logger, message, **fields)


def publish_recovery_status(
    state,
    status,
    tool_name,
    message,
    reason="",
):
    if state is None:
        return
    publisher = getattr(state, "_publish_robot_status", None)
    if publisher is None:
        return
    publisher(
        status,
        tool_name=tool_name,
        message=message,
        reason=reason,
    )


def normalize_tool_name(tool_name) -> str:
    if tool_name is None or str(tool_name).strip() == "":
        return "unknown"
    return str(tool_name).strip()


def cleanup_after_recovery(
    status,
    drawer_controller,
    motion_controller,
    config,
    logger,
    task_type,
    target_tool,
    reason=None,
):
    drawer_closed = close_drawer_if_open(drawer_controller, status)
    if not drawer_closed:
        log_recovery_event(
            logger,
            "DRAWER_CLOSE_FAILED",
            "복구 중 열린 서랍 닫기에 실패했습니다.",
            {
                "task_type": task_type,
                "target_tool": target_tool,
                "reason": "drawer_close_failed",
            },
        )
    home_ok = return_home(motion_controller, config)
    if not home_ok:
        log_recovery_event(
            logger,
            "RECOVERY_FAILED",
            "복구 후 Home 복귀에 실패했습니다.",
            {
                "task_type": task_type,
                "target_tool": target_tool,
                "reason": "return_home_failed",
            },
        )
    set_recovery_mode(status, False)
    if reason is not None:
        log_recovery_event(
            logger,
            "RECOVERY_FINISHED",
            "복구 절차를 종료했습니다.",
            {
                "task_type": task_type,
                "target_tool": target_tool,
                "reason": reason,
                "drawer_closed": drawer_closed,
                "home_ok": home_ok,
            },
        )
    return drawer_closed and home_ok


def _call_detector(detector, target):
    if detector is None:
        return None
    if callable(detector):
        return detector(target)
    for method_name in ("detect_target_tool", "resolve_store_tool_target", "resolve_target"):
        method = getattr(detector, method_name, None)
        if method is not None:
            return method(target)
    return None


def _detection_found(detection):
    return detection is not None and bool(getattr(detection, "found", True))


def _logger(config):
    if config is not None and getattr(config, "state", None) is not None:
        state = config.state
        if hasattr(state, "logger"):
            return state.logger()
    if config is not None and hasattr(config, "logger"):
        return config.logger
    return _NullLogger()


class _NullLogger:
    def debug(self, *_args, **_kwargs):
        pass

    def info(self, *_args, **_kwargs):
        pass

    def warn(self, *_args, **_kwargs):
        pass

    def warning(self, *_args, **_kwargs):
        pass

    def error(self, *_args, **_kwargs):
        pass
