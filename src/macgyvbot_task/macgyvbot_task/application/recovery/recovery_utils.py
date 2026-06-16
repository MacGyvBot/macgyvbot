"""Shared first-version drop recovery orchestration utilities."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import time
from typing import Any

from moveit.core.robot_state import RobotState

from macgyvbot_config.grasp import GRASP_RETRY_LIMIT
from macgyvbot_config.hand_grasp import HAND_GRASP_MASK_LOCK_TIMEOUT_SEC
from macgyvbot_config.handoff import HANDOFF_WAIT_POLL_SEC
from macgyvbot_config.pick import PICK_PREGRASP_REOPEN_WAIT_SEC
from macgyvbot_config.recovery import (
    RECOVERY_UNREACHABLE_X_MIN_M,
    RECOVERY_UNREACHABLE_Y_MIN_M,
)
from macgyvbot_config.robot import RECOVERY_INSPECTION_JOINTS
from macgyvbot_manipulation.grasp_verifier import GraspVerifier
from macgyvbot_manipulation.handover_targeting import move_to_observation_pose
from macgyvbot_manipulation.robot_safezone import SAFE_Z_MIN
from macgyvbot_manipulation.robot_pose import current_ee_orientation, make_safe_pose
from macgyvbot_manipulation.timing import cooperative_wait
from macgyvbot_task.application.logging_utils import (
    log_error,
    log_info,
    log_warn,
)
from macgyvbot_task.application.pick_flow.pick_grasp_flow import (
    PickGraspFlow,
    calculate_limited_grasp_width_mm,
    calculate_pregrasp_extra_descent,
)
from macgyvbot_task.application.pick_flow.pick_target_planner import PickTargetPlanner


RECOVERY_INSPECTION_SEARCH_TIMEOUT_SEC = 4.0


@dataclass
class RecoveryConfig:
    """Runtime adapters used by recovery without owning new robot algorithms."""

    robot: Any = None
    state: Any = None
    command: dict | None = None
    task_type: str = "unknown"
    initial_detect_target_fn: Callable[[str], Any] | None = None
    detect_target_fn: Callable[[str], Any] | None = None
    target_observe_fn: Callable[[Any, str, Any], bool] | None = None
    observed_tool_label_fn: Callable[[], str | None] | None = None
    close_open_drawer_fn: Callable[[Any], bool] | None = None
    tool_hold_monitor: Any = None
    pause_event: Any = None
    resume_event: Any = None
    drop_event: Any = None
    exit_event: Any = None
    wait_fn: Callable[[float], None] = cooperative_wait
    max_detection_retry: int = 3
    max_grasp_retry: int = GRASP_RETRY_LIMIT
    detection_retry_wait_sec: float = 0.2
    detection_timeout_sec: float = RECOVERY_INSPECTION_SEARCH_TIMEOUT_SEC
    pause_poll_sec: float = 0.1

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
        "grasp_detection_width_mm",
        "grasp_detection_width_target",
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
    """Move to the recovery-specific inspection joint pose."""
    logger = _logger(config)
    if config.robot is None:
        log_error(
            logger,
            "inspection pose unavailable",
            step="recovery",
            event="fail",
            reason="robot_missing",
        )
        return False

    state_goal = RobotState(config.robot.get_robot_model())
    state_goal.joint_positions = dict(RECOVERY_INSPECTION_JOINTS)
    state_goal.update()

    log_info(
        logger,
        "move to recovery inspection pose",
        step="recovery",
        event="start",
        joints="j1=0,j2=-25.38,j3=44.24,j4=0,j5=133.39,j6=90.0",
    )
    return bool(
        motion_controller.plan_and_execute(
            logger,
            state_goal=state_goal,
            collision_scene_key="recovery/inspection_pose",
        )
    )


def run_recovery_step_with_pause_retry(
    config: RecoveryConfig,
    status,
    logger,
    target_tool,
    reason,
    step_fn,
):
    """Run one recovery action, retrying it when pause interrupted the action."""
    while True:
        if recovery_restart_requested(config):
            return False

        if not wait_for_recovery_resume_if_paused(
            config,
            status,
            logger,
            target_tool,
            reason,
        ):
            return False

        step_ok = bool(step_fn())
        if recovery_restart_requested(config):
            return False
        if step_ok:
            return True

        if not _event_is_set(getattr(config, "pause_event", None)):
            return False


def wait_for_recovery_resume_if_paused(
    config: RecoveryConfig,
    status,
    logger,
    target_tool,
    reason,
):
    pause_event = getattr(config, "pause_event", None)
    if not _event_is_set(pause_event):
        return True

    log_recovery_event(
        logger,
        "RECOVERY_PAUSED",
        "drop recovery paused",
        {"target_tool": target_tool, "reason": reason},
    )
    publish_recovery_status(
        status,
        "paused",
        target_tool,
        "drop recovery paused.",
        reason=reason,
    )

    wait_fn = getattr(config, "wait_fn", cooperative_wait)
    while _event_is_set(pause_event):
        if _event_is_set(getattr(config, "exit_event", None)):
            return False
        wait_fn(float(getattr(config, "pause_poll_sec", 0.1)))

    if _event_is_set(getattr(config, "resume_event", None)):
        getattr(config, "resume_event").clear()

    log_recovery_event(
        logger,
        "RECOVERY_RESUMED",
        "drop recovery resumed",
        {"target_tool": target_tool, "reason": "resume_requested"},
    )
    publish_recovery_status(
        status,
        "recovering",
        target_tool,
        "drop recovery resumed.",
        reason="drop_recovery_resumed",
    )
    return True


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


def detect_target_tool(
    yolo_detector,
    target_tool,
    max_retry: int = 3,
    retry_wait_sec: float = 0.2,
    timeout_sec: float | None = None,
    wait_fn: Callable[[float], None] = time.sleep,
):
    """Find a tool through an existing detector/resolver callable."""
    target = normalize_tool_name(target_tool)
    deadline = None
    if timeout_sec is not None:
        deadline = time.monotonic() + max(0.0, float(timeout_sec))

    attempt = 0
    while True:
        attempt += 1
        detection = _call_detector(yolo_detector, target)
        if _detection_found(detection):
            return detection

        if deadline is None:
            if attempt >= int(max_retry):
                return None
        elif time.monotonic() >= deadline:
            return None

        wait_fn(float(retry_wait_sec))
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


def recovery_target_is_unreachable(detection) -> bool:
    """Return True when a detected recovery target must be cleared manually."""
    base_xyz = getattr(detection, "base_xyz", None)
    if base_xyz is None or len(base_xyz) < 2:
        return False

    try:
        x = float(base_xyz[0])
        y = float(base_xyz[1])
    except (TypeError, ValueError):
        return False

    return x >= RECOVERY_UNREACHABLE_X_MIN_M and y >= RECOVERY_UNREACHABLE_Y_MIN_M


def log_recovery_target_coordinates(logger, detection, target_tool, reason):
    """Log recovery target coordinates in the robot base frame when available."""
    base_xyz = getattr(detection, "base_xyz", None)
    if base_xyz is None or len(base_xyz) < 3:
        log_warn(
            logger,
            "recovery target coordinates unavailable",
            step="recovery_observe",
            event="target_coordinates_missing",
            target=target_tool,
            reason=reason,
        )
        return

    try:
        base_x = float(base_xyz[0])
        base_y = float(base_xyz[1])
        base_z = float(base_xyz[2])
    except (TypeError, ValueError):
        log_warn(
            logger,
            "recovery target coordinates invalid",
            step="recovery_observe",
            event="target_coordinates_invalid",
            target=target_tool,
            reason=reason,
            base_xyz=base_xyz,
        )
        return

    log_info(
        logger,
        "recovery target coordinates",
        step="recovery_observe",
        event="target_coordinates",
        target=target_tool,
        reason=reason,
        base_x=base_x,
        base_y=base_y,
        base_z=base_z,
    )


def attempt_grasp(
    detection,
    motion_controller,
    gripper,
    config: RecoveryConfig | None = None,
    target_tool: str = "unknown",
) -> bool:
    """Attempt a recovery grasp using existing planner and verifier helpers."""
    logger = _logger(config)
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

    planner = PickTargetPlanner(robot)
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

    if not _set_limited_gripper_width_for_recovery(
        gripper,
        config,
        target_tool,
        logger,
    ) and hasattr(gripper, "open_gripper"):
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

    if not _pregrasp_depth_adjust_for_recovery(
        plan,
        ori,
        motion_controller,
        gripper,
        config,
        target_tool,
        logger,
    ):
        return False

    verifier = GraspVerifier(
        gripper,
        getattr(config, "wait_fn", cooperative_wait),
        interrupted=lambda: _recovery_interrupted(config),
    )
    return bool(verifier.try_grasp(logger, failure_prefix="recovery grasp"))


def _pregrasp_depth_adjust_for_recovery(
    plan,
    ori,
    motion_controller,
    gripper,
    config,
    target_tool,
    logger,
):
    grasp_flow = PickGraspFlow(
        gripper,
        getattr(config, "state", None),
        getattr(config, "wait_fn", cooperative_wait),
        interrupted=lambda: _recovery_interrupted(config),
    )
    measurement = grasp_flow.measure_pregrasp_depth(logger)
    if measurement is None:
        return False

    depth_mm = measurement.get("depth_mm")
    extra_descent_m = calculate_pregrasp_extra_descent(depth_mm)
    if extra_descent_m is None:
        log_warn(
            logger,
            "recovery pregrasp depth unavailable",
            step="recovery_pregrasp",
            event="fail",
            reason="depth_missing",
        )
        return False

    recovery_safe_z_min = SAFE_Z_MIN
    redescend_min_z = recovery_safe_z_min - extra_descent_m
    target_z = max(plan.grasp_z - extra_descent_m, redescend_min_z)
    actual_descent_m = plan.grasp_z - target_z
    log_info(
        logger,
        "recovery pregrasp descent calculated",
        step="recovery_pregrasp",
        event="calculated",
        width_mm=measurement.get("width_mm"),
        depth_mm=depth_mm,
        extra_descent_m=extra_descent_m,
        target_z=target_z,
        recovery_safe_z_min=recovery_safe_z_min,
        redescend_min_z=redescend_min_z,
    )

    if not _set_limited_gripper_width_for_recovery(
        gripper,
        config,
        target_tool,
        logger,
    ) and hasattr(gripper, "open_gripper"):
        gripper.open_gripper()
    getattr(config, "wait_fn", cooperative_wait)(PICK_PREGRASP_REOPEN_WAIT_SEC)

    if actual_descent_m <= 0.0:
        log_info(
            logger,
            "recovery pregrasp descent skipped",
            step="recovery_pregrasp",
            event="skip",
            reason="z_limit",
        )
        return True

    return motion_controller.plan_and_execute(
        logger,
        pose_goal=make_safe_pose(
            plan.target_x,
            plan.target_y,
            target_z,
            ori,
            logger,
        ),
        min_z=redescend_min_z,
        collision_scene_key="recovery/pregrasp_depth_adjust",
    )


def _set_limited_gripper_width_for_recovery(gripper, config, target_tool, logger):
    width_mm = _current_recovery_limited_grasp_width_mm(
        gripper,
        config,
        target_tool,
    )
    if width_mm is None:
        return False

    move_gripper = getattr(gripper, "move_gripper", None)
    if move_gripper is None:
        return False

    try:
        move_gripper(int(round(float(width_mm) * 10.0)))
    except Exception as exc:
        log_warn(
            logger,
            "recovery limited grasp width command failed",
            step="recovery_gripper_width",
            event="fail",
            reason=str(exc) or type(exc).__name__,
            width_mm=width_mm,
        )
        return False

    log_info(
        logger,
        "recovery limited grasp width applied",
        step="recovery_gripper_width",
        event="applied",
        target=target_tool,
        width_mm=width_mm,
    )
    return True


def _current_recovery_limited_grasp_width_mm(gripper, config, target_tool):
    state = getattr(config, "state", None)
    if state is None:
        return None
    width_target = getattr(state, "grasp_detection_width_target", None)
    if width_target != target_tool:
        return None
    mask_width_mm = getattr(state, "grasp_detection_width_mm", None)
    if mask_width_mm is None:
        return None
    max_width_mm = float(getattr(gripper, "max_width", 0) or 0) / 10.0
    return calculate_limited_grasp_width_mm(mask_width_mm, max_width_mm)


def return_home(motion_controller, config):
    logger = _logger(config)
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


def publish_recovery_grasp_success(state, config, tool_name):
    if state is None:
        return
    publisher = getattr(state, "_publish_robot_status", None)
    if publisher is None:
        return

    if hasattr(state, "tool_mask_locked"):
        state.tool_mask_locked = False
    if hasattr(state, "last_tool_mask_lock_result"):
        state.last_tool_mask_lock_result = None

    command = getattr(config, "command", None) or getattr(state, "current_command", None)
    action = None
    if isinstance(command, dict):
        action = command.get("action")

    publisher(
        "grasp_success",
        tool_name=tool_name,
        action=action,
        message="recovery grasp succeeded.",
        reason="drop_recovery_grasp_success",
        command=command,
    )


def wait_for_recovery_tool_mask_lock(config: RecoveryConfig, logger, target_tool) -> bool:
    state = getattr(config, "state", None)
    if state is None:
        return False

    start_time = time.monotonic()
    wait_fn = getattr(config, "wait_fn", cooperative_wait)
    while True:
        if _recovery_interrupted(config):
            return False

        result = getattr(state, "last_tool_mask_lock_result", None)
        if getattr(state, "tool_mask_locked", False) and result is not None:
            log_info(
                logger,
                "recovery tool mask lock confirmed",
                step="recovery_tool_mask_lock",
                event="done",
                target=target_tool,
                source=result.get("mask_source"),
                roi=result.get("tool_roi"),
            )
            return True

        if result is not None and result.get("locked") is False:
            log_warn(
                logger,
                "recovery tool mask lock failed",
                step="recovery_tool_mask_lock",
                event="fail",
                target=target_tool,
                reason=result.get("reason", "unknown"),
            )
            return False

        if time.monotonic() - start_time >= HAND_GRASP_MASK_LOCK_TIMEOUT_SEC:
            log_warn(
                logger,
                "recovery tool mask lock timed out",
                step="recovery_tool_mask_lock",
                event="timeout",
                target=target_tool,
                timeout_sec=HAND_GRASP_MASK_LOCK_TIMEOUT_SEC,
            )
            return False

        wait_fn(HANDOFF_WAIT_POLL_SEC)


def close_open_drawer_after_recovery_failure(
    status,
    config: RecoveryConfig,
    logger,
    target_tool,
    reason,
):
    drawer_is_open = bool(getattr(status, "drawer_open", False))
    if not drawer_is_open:
        return True

    close_drawer = getattr(config, "close_open_drawer_fn", None)
    if close_drawer is None:
        log_warn(
            logger,
            "recovery failure drawer close unavailable",
            step="recovery_drawer_close",
            event="skip",
            target=target_tool,
            reason=reason,
        )
        return False

    log_info(
        logger,
        "close drawer after recovery failure",
        step="recovery_drawer_close",
        event="start",
        target=target_tool,
        reason=reason,
        drawer_id=getattr(status, "opened_drawer_id", None),
    )
    return bool(close_drawer(logger))


def normalize_tool_name(tool_name) -> str:
    if tool_name is None or str(tool_name).strip() == "":
        return "unknown"
    return str(tool_name).strip()


def cleanup_after_recovery(
    status,
    motion_controller,
    config,
    logger,
    task_type,
    target_tool,
    reason=None,
    finish_recovery_mode=True,
):
    """Finish recovery by returning home."""
    home_ok = return_home(motion_controller, config)
    interrupted = _recovery_interrupted(config)
    if not home_ok and not interrupted:
        log_recovery_event(
            logger,
            "RECOVERY_FAILED",
            "recovery Home return failed",
            {
                "task_type": task_type,
                "target_tool": target_tool,
                "reason": "return_home_failed",
            },
        )
    if finish_recovery_mode and not interrupted:
        set_recovery_mode(status, False)
    if reason is not None:
        log_recovery_event(
            logger,
            "RECOVERY_FINISHED",
            "recovery cleanup finished",
            {
                "task_type": task_type,
                "target_tool": target_tool,
                "reason": reason,
                "home_ok": home_ok,
                "interrupted": interrupted,
                "gripper_holding": getattr(status, "gripper_holding", False),
            },
        )
    return bool(home_ok)

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


def _event_is_set(event):
    return event is not None and bool(event.is_set())


def _recovery_interrupted(config):
    return (
        _event_is_set(getattr(config, "pause_event", None))
        or _event_is_set(getattr(config, "drop_event", None))
        or _event_is_set(getattr(config, "exit_event", None))
    )


def recovery_restart_requested(config):
    return _event_is_set(getattr(config, "drop_event", None))


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
