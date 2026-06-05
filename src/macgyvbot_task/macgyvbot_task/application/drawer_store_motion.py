"""Shared drawer-store placement motion helpers."""

from __future__ import annotations

from macgyvbot_config.drawer import (
    DRAWER_STORE_MARKER_EXIT_OFFSET_XYZ_M,
    DRAWER_WALL_CLEARANCE_Z_OFFSET_M,
)
from macgyvbot_manipulation.robot_pose import make_safe_pose
from macgyvbot_manipulation.robot_safezone import safe_z_min_for_drawer


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
    logger.info(
        f"{label}: offset=({DRAWER_STORE_MARKER_EXIT_OFFSET_XYZ_M[0]:.3f}, "
        f"{DRAWER_STORE_MARKER_EXIT_OFFSET_XYZ_M[1]:.3f}, "
        f"{DRAWER_STORE_MARKER_EXIT_OFFSET_XYZ_M[2]:.3f}), "
        f"target=({exit_x:.3f}, {exit_y:.3f}, {exit_z:.3f})"
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
    )
    if not ok:
        logger.error(error_message)
    return ok
