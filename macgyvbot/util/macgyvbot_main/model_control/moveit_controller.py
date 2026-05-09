"""MoveIt planning helpers and controller-level motion operations."""

import math

import numpy as np
from moveit.core.robot_state import RobotState

from macgyvbot.util.macgyvbot_main.model_control.robot_safezone import (
    clamp_to_safe_workspace,
)

from macgyvbot.config.config import GROUP_NAME, EE_LINK, WRIST_JOINT_NAME
from macgyvbot.util.macgyvbot_main.model_control.robot_pose import normalize_angle_deg


def plan_and_execute(
    robot,
    arm,
    logger,
    pose_goal=None,
    state_goal=None,
    params=None,
):
    arm.set_start_state_to_current_state()

    if pose_goal:
        x = pose_goal.pose.position.x
        y = pose_goal.pose.position.y
        z = pose_goal.pose.position.z
        sx, sy, sz = clamp_to_safe_workspace(x, y, z, logger)
        pose_goal.pose.position.x = sx
        pose_goal.pose.position.y = sy
        pose_goal.pose.position.z = sz

        arm.set_goal_state(
            pose_stamped_msg=pose_goal,
            pose_link=EE_LINK,
        )
    elif state_goal:
        arm.set_goal_state(robot_state=state_goal)

    plan_result = arm.plan(parameters=params) if params else arm.plan()

    if plan_result:
        robot.execute(
            GROUP_NAME,
            plan_result.trajectory,
            blocking=True,
        )
        return True

    logger.error("Planning 실패")
    return False


class MoveItController:
    """Thin adapter around MoveItPy for reusable robot-arm operations."""

    def __init__(self, robot, arm, params):
        self.robot = robot
        self.arm = arm
        self.params = params

    def plan_and_execute(self, logger, pose_goal=None, state_goal=None):
        return plan_and_execute(
            self.robot,
            self.arm,
            logger,
            pose_goal=pose_goal,
            state_goal=state_goal,
            params=self.params,
        )

    def rotate_wrist_by_yaw_deg(self, yaw_deg, logger):
        if yaw_deg is None:
            return True

        try:
            yaw_deg = float(yaw_deg)
        except (TypeError, ValueError):
            logger.warn(f"유효하지 않은 yaw 값이라 J6 회전을 생략합니다: {yaw_deg}")
            return False

        if not math.isfinite(yaw_deg):
            logger.warn(f"비정상 yaw 값이라 J6 회전을 생략합니다: {yaw_deg}")
            return False

        yaw_deg = normalize_angle_deg(yaw_deg)
        if abs(yaw_deg) < 0.1:
            logger.info("VLM yaw가 매우 작아 J6 회전을 생략합니다.")
            return True

        robot_model = self.robot.get_robot_model()
        jmg = robot_model.get_joint_model_group(GROUP_NAME)
        joint_names = list(jmg.active_joint_model_names)
        if WRIST_JOINT_NAME not in joint_names:
            logger.error(
                f"{GROUP_NAME} 그룹에 {WRIST_JOINT_NAME}이 없어 J6 회전을 수행할 수 없습니다. "
                f"(active joints={joint_names})"
            )
            return False

        j6_idx = joint_names.index(WRIST_JOINT_NAME)
        psm = self.robot.get_planning_scene_monitor()
        with psm.read_only() as scene:
            current_positions = np.array(
                scene.current_state.get_joint_group_positions(GROUP_NAME),
                dtype=float,
            )

        if j6_idx >= len(current_positions):
            logger.error(
                f"J6 인덱스({j6_idx})가 현재 조인트 벡터 길이"
                f"({len(current_positions)})를 초과합니다."
            )
            return False

        current_j6 = float(current_positions[j6_idx])
        target_j6 = current_j6 + math.radians(yaw_deg)
        target_j6 = math.atan2(math.sin(target_j6), math.cos(target_j6))

        target_positions = np.array(current_positions, copy=True)
        target_positions[j6_idx] = target_j6

        logger.info(
            "2-1단계: VLM yaw를 J6에 적용 "
            f"(yaw_offset={yaw_deg:.2f}deg, "
            f"j6={math.degrees(current_j6):.2f}deg -> "
            f"{math.degrees(target_j6):.2f}deg)"
        )

        state_goal = RobotState(robot_model)
        state_goal.set_joint_group_positions(GROUP_NAME, target_positions)
        state_goal.update()

        return self.plan_and_execute(logger, state_goal=state_goal)
