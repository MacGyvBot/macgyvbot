"""Pick target planning helpers."""

from __future__ import annotations

from macgyvbot_config.drawer import (
    DRAWER_PICK_APPROACH_LIFT_M,
    DRAWER_PICK_GRASP_FROM_HANDLE_Z_M,
    get_drawer_floor_min_grasp_z,
)
from macgyvbot_config.pick import (
    GRASP_Z_OFFSET,
    OBJECT_Z_HEIGHT_BIAS_M,
    SAFE_Z,
)
from macgyvbot_config.robot import SAFE_Z_MIN
from macgyvbot_manipulation.robot_pose import get_ee_matrix
from macgyvbot_domain.pick_models import PickMotionPlan


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

    def plan_drawer(self, bx, by, drawer_handle_z, floor, logger):
        floor_min_grasp_z = get_drawer_floor_min_grasp_z(floor)
        grasp_z = max(floor_min_grasp_z, drawer_handle_z + DRAWER_PICK_GRASP_FROM_HANDLE_Z_M)
        approach_z = grasp_z + DRAWER_PICK_APPROACH_LIFT_M

        current_pose = get_ee_matrix(self.robot)

        logger.info(
            f"[서랍pick] plan_drawer — "
            f"drawer_handle_z={drawer_handle_z:.3f} "
            f"floor={floor} floor_min_grasp_z={floor_min_grasp_z:.3f} "
            f"grasp_z={grasp_z:.3f} approach_z={approach_z:.3f}"
        )

        return PickMotionPlan(
            target_x=bx,
            target_y=by,
            travel_z=SAFE_Z,
            approach_z=approach_z,
            grasp_z=grasp_z,
            current_x=float(current_pose[0, 3]),
            current_y=float(current_pose[1, 3]),
            corrected_bz=0.0,
            should_descend_to_grasp=True,
        )
