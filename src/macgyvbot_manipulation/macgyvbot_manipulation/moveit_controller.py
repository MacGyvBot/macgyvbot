"""MoveIt planning helpers and controller-level motion operations."""

import math

import numpy as np
from moveit.core.robot_state import RobotState

from macgyvbot_manipulation.robot_safezone import (
    clamp_to_safe_workspace,
)

from macgyvbot_config.robot import (
    DRAWER_CLOSED_JOINTS,
    DRAWER_FLOOR_STEP_CLOSED_JOINT_DELTAS,
    DRAWER_FLOOR_STEP_OBSERVATION_JOINT_DELTAS,
    DRAWER_FLOOR_STEP_OPEN_JOINT_DELTAS,
    DRAWER_INSIDE_OBSERVATION_JOINTS,
    DRAWER_OPEN_JOINTS,
    DRAWER_OPEN_JOINT_SEQUENCE,
    apply_joint_deltas,
    EE_LINK,
    GROUP_NAME,
    HOME_JOINTS,
    WRIST_JOINT_NAME,
)
from macgyvbot_manipulation.robot_pose import normalize_angle_deg


def plan_and_execute(
    robot,
    arm,
    logger,
    pose_goal=None,
    state_goal=None,
    params=None,
    clamp_z=True,
):
    arm.set_start_state_to_current_state()

    if pose_goal:
        x = pose_goal.pose.position.x
        y = pose_goal.pose.position.y
        z = pose_goal.pose.position.z
        sx, sy, sz = clamp_to_safe_workspace(x, y, z, logger)
        if clamp_z:
            pose_goal.pose.position.z = sz
        else:
            pose_goal.pose.position.z = z
        pose_goal.pose.position.x = sx
        pose_goal.pose.position.y = sy

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

    def __init__(self, robot, arm, params, fallback_params=None):
        self.robot = robot
        self.arm = arm
        self.params = params
        self.fallback_params = fallback_params

    def plan_and_execute(self, logger, pose_goal=None, state_goal=None, clamp_z=True):
        ok = plan_and_execute(
            self.robot,
            self.arm,
            logger,
            pose_goal=pose_goal,
            state_goal=state_goal,
            params=self.params,
            clamp_z=clamp_z,
        )
        if not ok and self.fallback_params is not None:
            logger.warn("기본 플래너 실패, OMPL 폴백 시도")
            ok = plan_and_execute(
                self.robot,
                self.arm,
                logger,
                pose_goal=pose_goal,
                state_goal=state_goal,
                params=self.fallback_params,
                clamp_z=clamp_z,
            )
        return ok

    def move_to_home_joints(self, logger):
        """Move to the configured Home joint pose."""
        logger.info("Home joint pose로 복귀합니다.")
        return self._move_to_joint_positions(HOME_JOINTS, logger)

    def move_to_drawer_closed_joints(self, logger):
        """Move to the configured drawer closed-handle joint pose."""
        logger.info("서랍 손잡이(닫힌) joint pose로 이동합니다.")
        return self._move_to_joint_positions(DRAWER_CLOSED_JOINTS, logger)

    def move_to_drawer_open_joints(self, logger):
        """Move to the configured drawer open-handle joint pose."""
        logger.info("서랍 손잡이(열린) joint pose로 이동합니다.")
        return self._move_to_joint_positions(DRAWER_OPEN_JOINTS, logger)

    def move_to_drawer_open_joint_sequence(self, logger, after_waypoint=None):
        """Move through taught drawer-open waypoints in order."""
        for name, joint_positions in DRAWER_OPEN_JOINT_SEQUENCE:
            logger.info(f"서랍 열기 waypoint 이동: {name}")
            if not self._move_to_joint_positions(joint_positions, logger):
                logger.error(f"서랍 열기 waypoint 실패: {name}")
                return False
            if after_waypoint is not None:
                after_waypoint(name)
        return True

    def move_to_drawer_inside_observation_joints(self, logger):
        """Move to the taught post-open drawer observation pose, if configured.

        Returns:
            True: configured joint pose reached.
            False: configured joint pose failed.
            None: no taught joint pose configured; caller may use a fallback.
        """
        if DRAWER_INSIDE_OBSERVATION_JOINTS is None:
            logger.warn(
                "DRAWER_INSIDE_OBSERVATION_JOINTS가 아직 설정되지 않아 "
                "FK+offset 관찰 pose fallback을 사용합니다."
            )
            return None

        logger.info("서랍 내부 관찰 joint pose로 이동합니다.")
        return self._move_to_joint_positions(
            DRAWER_INSIDE_OBSERVATION_JOINTS,
            logger,
        )

    def move_to_drawer_floor_closed_joints(self, logger, floor):
        scale = floor - 1
        try:
            joints = apply_joint_deltas(DRAWER_CLOSED_JOINTS, DRAWER_FLOOR_STEP_CLOSED_JOINT_DELTAS, scale)
        except ValueError as exc:
            logger.error(f"서랍 {floor}층 닫힌 joint delta 설정 오류: {exc}")
            return False
        logger.info(f"서랍 {floor}층 손잡이(닫힌) joint pose로 이동합니다.")
        return self._move_to_joint_positions(joints, logger)

    def move_to_drawer_floor_open_joints(self, logger, floor):
        scale = floor - 1
        try:
            joints = apply_joint_deltas(DRAWER_OPEN_JOINTS, DRAWER_FLOOR_STEP_OPEN_JOINT_DELTAS, scale)
        except ValueError as exc:
            logger.error(f"서랍 {floor}층 열린 joint delta 설정 오류: {exc}")
            return False
        logger.info(f"서랍 {floor}층 손잡이(열린) joint pose로 이동합니다.")
        return self._move_to_joint_positions(joints, logger)

    def move_to_drawer_floor_open_joint_sequence(self, logger, floor, after_waypoint=None):
        scale = floor - 1
        for base_name, base_joints in DRAWER_OPEN_JOINT_SEQUENCE:
            name = base_name if scale == 0 else f"{base_name}_FLOOR{floor}"
            try:
                joints = apply_joint_deltas(base_joints, DRAWER_FLOOR_STEP_OPEN_JOINT_DELTAS, scale)
            except ValueError as exc:
                logger.error(f"서랍 열기 waypoint '{name}' joint delta 설정 오류: {exc}")
                return False
            logger.info(f"서랍 열기 waypoint 이동: {name}")
            if not self._move_to_joint_positions(joints, logger):
                logger.error(f"서랍 열기 waypoint 실패: {name}")
                return False
            if after_waypoint is not None:
                after_waypoint(name)
        return True

    def move_to_drawer_floor_inside_observation_joints(self, logger, floor):
        if DRAWER_INSIDE_OBSERVATION_JOINTS is None:
            return None  # caller uses FK fallback
        scale = floor - 1
        try:
            joints = apply_joint_deltas(
                DRAWER_INSIDE_OBSERVATION_JOINTS,
                DRAWER_FLOOR_STEP_OBSERVATION_JOINT_DELTAS,
                scale,
            )
        except ValueError as exc:
            logger.error(f"서랍 {floor}층 관찰 joint delta 설정 오류: {exc}")
            return False
        logger.info(f"서랍 {floor}층 내부 관찰 joint pose로 이동합니다.")
        return self._move_to_joint_positions(joints, logger)

    def _move_to_joint_positions(self, joint_positions, logger):
        state_goal = RobotState(self.robot.get_robot_model())
        state_goal.joint_positions = joint_positions
        state_goal.update()
        return self.plan_and_execute(logger, state_goal=state_goal)

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
