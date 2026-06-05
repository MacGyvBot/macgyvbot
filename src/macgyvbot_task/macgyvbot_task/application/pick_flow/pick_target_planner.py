"""Pick target planning helpers."""

from __future__ import annotations

from macgyvbot_config.pick import (
    APPROACH_Z_OFFSET,
    GRASP_Z_OFFSET,
    OBJECT_Z_HEIGHT_BIAS_M,
    PICK_TRAVEL_Z_CLEARANCE_M,
    SAFE_Z,
)
from macgyvbot_manipulation.robot_pose import get_ee_matrix
from macgyvbot_manipulation.robot_safezone import SAFE_Z_MIN
from macgyvbot_domain.pick_models import PickMotionPlan
from macgyvbot_task.application.logging_utils import log_warn


class PickTargetPlanner:
    """Convert a perceived object position into safe pick waypoints."""

    def __init__(self, robot):
        self.robot = robot

    def plan(self, bx, by, bz, logger, safe_z_min=SAFE_Z_MIN):
        corrected_bz = bz - OBJECT_Z_HEIGHT_BIAS_M
        grasp_z = SAFE_Z_MIN + corrected_bz - GRASP_Z_OFFSET
        approach_z = grasp_z + APPROACH_Z_OFFSET

        current_pose = get_ee_matrix(self.robot)
        current_x = current_pose[0, 3]
        current_y = current_pose[1, 3]

        target_x = bx
        target_y = by
        travel_z = safe_z_min + PICK_TRAVEL_Z_CLEARANCE_M

        if approach_z < grasp_z:
            log_warn(
                logger,
                "approach z clamped to grasp z",
                step="plan",
                event="clamp",
                approach_z=approach_z,
                grasp_z=grasp_z,
            )
            approach_z = grasp_z

        if grasp_z < safe_z_min:
            log_warn(
                logger,
                "grasp z clamped to safe minimum",
                step="plan",
                event="clamp",
                grasp_z=grasp_z,
                safe_z_min=safe_z_min,
            )
            grasp_z = safe_z_min

        if approach_z < safe_z_min:
            log_warn(
                logger,
                "approach z clamped to safe minimum",
                step="plan",
                event="clamp",
                approach_z=approach_z,
                safe_z_min=safe_z_min,
            )
            approach_z = safe_z_min

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
