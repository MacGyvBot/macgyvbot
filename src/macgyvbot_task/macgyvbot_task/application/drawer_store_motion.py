"""Shared drawer-store placement motion helpers."""

from __future__ import annotations

from macgyvbot_config.drawer import (
    DRAWER_STORE_MARKER_EXIT_OFFSET_XYZ_M,
    DRAWER_WALL_CLEARANCE_Z_OFFSET_M,
)
from macgyvbot_config.return_flow import RETURN_DRAWER_PLACE_WRIST_YAW_DEG
from macgyvbot_manipulation.robot_pose import make_safe_pose
from macgyvbot_manipulation.robot_safezone import safe_z_min_for_drawer
from macgyvbot_task.application.logging_utils import log_error, log_info, log_warn


def drawer_wall_clearance_z_for_drawer(drawer_id):
    return safe_z_min_for_drawer(drawer_id) + DRAWER_WALL_CLEARANCE_Z_OFFSET_M


def drawer_store_exit_target(target_x, target_y, drawer_wall_clearance_z):
    return (
        target_x + DRAWER_STORE_MARKER_EXIT_OFFSET_XYZ_M[0],
        target_y + DRAWER_STORE_MARKER_EXIT_OFFSET_XYZ_M[1],
        drawer_wall_clearance_z + DRAWER_STORE_MARKER_EXIT_OFFSET_XYZ_M[2],
    )


def rotate_wrist_for_drawer_store(motion, logger, label="drawer_store"):
    """Rotate J6 into the drawer-store orientation used by return placement."""
    rotate_wrist = getattr(motion, "rotate_wrist_by_yaw_deg", None)
    if rotate_wrist is None:
        log_warn(
            logger,
            "drawer store wrist rotation unavailable",
            step="drawer_store",
            event="fail",
            reason="rotate_wrist_missing",
            label=label,
        )
        return False

    log_info(
        logger,
        "drawer store wrist rotation started",
        step="drawer_store",
        event="wrist_rotation",
        yaw_deg=RETURN_DRAWER_PLACE_WRIST_YAW_DEG,
        label=label,
    )
    try:
        ok = rotate_wrist(RETURN_DRAWER_PLACE_WRIST_YAW_DEG, logger)
    except Exception as exc:
        log_warn(
            logger,
            "drawer store wrist rotation failed",
            step="drawer_store",
            event="fail",
            reason=str(exc) or type(exc).__name__,
            label=label,
        )
        return False
    return bool(ok)


def move_to_drawer_store_exit(
    motion,
    logger,
    target_x,
    target_y,
    drawer_wall_clearance_z,
    ori,
    label,
    error_message,
):
    exit_x, exit_y, exit_z = drawer_store_exit_target(
        target_x,
        target_y,
        drawer_wall_clearance_z,
    )
    log_info(
        logger,
        "move to drawer store exit",
        step="drawer_exit",
        event="start",
        offset_x=DRAWER_STORE_MARKER_EXIT_OFFSET_XYZ_M[0],
        offset_y=DRAWER_STORE_MARKER_EXIT_OFFSET_XYZ_M[1],
        offset_z=DRAWER_STORE_MARKER_EXIT_OFFSET_XYZ_M[2],
        target_x=exit_x,
        target_y=exit_y,
        target_z=exit_z,
        label=label,
    )
    ok = motion.plan_and_execute(
        logger,
        pose_goal=make_safe_pose(
            exit_x,
            exit_y,
            exit_z,
            ori,
            logger,
        ),
        collision_scene_key="drawer/store_exit",
    )
    if not ok:
        log_error(
            logger,
            "drawer store exit move failed",
            step="drawer_exit",
            event="fail",
            reason=error_message,
        )
    return ok
