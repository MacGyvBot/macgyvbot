"""Pick target planning helpers."""

from __future__ import annotations

from macgyvbot_config.drawer import DRAWER_WALL_CLEARANCE_Z_OFFSET_M
from macgyvbot_manipulation.robot_pose import get_ee_matrix
from macgyvbot_manipulation.robot_safezone import SAFE_Z_MIN
from macgyvbot_domain.pick_models import PickMotionPlan


class PickTargetPlanner:
    """Convert a perceived object position into safe pick waypoints."""

    def __init__(self, robot):
        self.robot = robot

    def plan(self, bx, by, bz, logger, safe_z_min=SAFE_Z_MIN):
        grasp_z = max(safe_z_min, float(bz))

        current_pose = get_ee_matrix(self.robot)
        current_x = current_pose[0, 3]
        current_y = current_pose[1, 3]

        target_x = bx
        target_y = by
        drawer_wall_clearance_z = safe_z_min + DRAWER_WALL_CLEARANCE_Z_OFFSET_M

        if float(bz) < safe_z_min:
            logger.warn(
                f"계산된 grasp_z({float(bz):.3f})가 "
                f"safe_z_min({safe_z_min:.3f})보다 낮아 "
                "safe_z_min으로 맞춥니다."
            )

        return PickMotionPlan(
            target_x=target_x,
            target_y=target_y,
            drawer_wall_clearance_z=drawer_wall_clearance_z,
            grasp_z=grasp_z,
            current_x=current_x,
            current_y=current_y,
            should_descend_to_grasp=abs(drawer_wall_clearance_z - grasp_z) > 0.005,
        )
