"""Shared drawer-store placement motion helpers."""

from __future__ import annotations

from macgyvbot_config.drawer import (
    DRAWER_STORE_MARKER_EXIT_OFFSET_XYZ_M,
    DRAWER_WALL_CLEARANCE_Z_OFFSET_M,
)
from macgyvbot_manipulation.robot_pose import make_safe_pose
from macgyvbot_manipulation.robot_safezone import safe_z_min_for_drawer
from macgyvbot_task.application.logging_utils import log_error, log_info


def drawer_wall_clearance_z_for_drawer(drawer_id):
    return safe_z_min_for_drawer(drawer_id) + DRAWER_WALL_CLEARANCE_Z_OFFSET_M


def drawer_store_exit_target(target_x, target_y, drawer_wall_clearance_z):
    return (
        target_x + DRAWER_STORE_MARKER_EXIT_OFFSET_XYZ_M[0],
        target_y + DRAWER_STORE_MARKER_EXIT_OFFSET_XYZ_M[1],
        drawer_wall_clearance_z + DRAWER_STORE_MARKER_EXIT_OFFSET_XYZ_M[2],
    )


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
