"""Pick target planning helpers."""

from __future__ import annotations

from macgyvbot.config.pick import (
    GRASP_Z_OFFSET,
    OBJECT_Z_HEIGHT_BIAS_M,
    SAFE_Z,
)
from macgyvbot.control.robot_pose import get_ee_matrix
from macgyvbot.control.robot_safezone import SAFE_Z_MIN
from macgyvbot.domain.pick_models import PickMotionPlan


class PickTargetPlanner:
    """Convert a perceived object position into safe pick waypoints."""

    def __init__(self, robot):
        self.robot = robot

    def plan(self, bx, by, bz, logger):
        corrected_bz = bz - OBJECT_Z_HEIGHT_BIAS_M
        grasp_z = SAFE_Z_MIN + corrected_bz - GRASP_Z_OFFSET
        approach_z = grasp_z - GRASP_Z_OFFSET

        current_pose = get_ee_matrix(self.robot)
        current_x = current_pose[0, 3]
        current_y = current_pose[1, 3]

        target_x = bx
        target_y = by
        travel_z = SAFE_Z

        if grasp_z > approach_z:
            logger.warn(
                f"계산된 grasp_z({grasp_z:.3f})가 "
                f"approach_z({approach_z:.3f})보다 높아 approach_z로 맞춥니다."
            )
            grasp_z = approach_z

        return PickMotionPlan(
            target_x=target_x,
            target_y=target_y,
            travel_z=travel_z,
            approach_z=approach_z,
            grasp_z=grasp_z,
            current_x=current_x,
            current_y=current_y,
            corrected_bz=corrected_bz,
            should_descend_to_grasp=abs(approach_z - grasp_z) > 0.005,
        )
