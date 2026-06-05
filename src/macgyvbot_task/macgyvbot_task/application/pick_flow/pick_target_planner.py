"""Pick target planning helpers."""

from __future__ import annotations

from macgyvbot_config.drawer import DRAWER_WALL_CLEARANCE_Z_OFFSET_M
from macgyvbot_manipulation.robot_safezone import SAFE_Z_MIN
from macgyvbot_domain.pick_models import PickMotionPlan
from macgyvbot_task.application.logging_utils import log_warn


class PickTargetPlanner:
    """Convert a perceived object position into safe pick waypoints."""

    def __init__(self, _robot):
        pass

    def plan(self, bx, by, bz, logger, safe_z_min=SAFE_Z_MIN):
        grasp_z = max(safe_z_min, float(bz))

        target_x = bx
        target_y = by
        drawer_wall_clearance_z = safe_z_min + DRAWER_WALL_CLEARANCE_Z_OFFSET_M

        if float(bz) < safe_z_min:
            log_warn(
                logger,
                "pick grasp z clamped to safe minimum",
                step="plan",
                event="clamp",
                raw_bz=float(bz),
                safe_z_min=safe_z_min,
            )

        return PickMotionPlan(
            target_x=target_x,
            target_y=target_y,
            drawer_wall_clearance_z=drawer_wall_clearance_z,
            grasp_z=grasp_z,
            should_descend_to_grasp=abs(drawer_wall_clearance_z - grasp_z) > 0.005,
        )
