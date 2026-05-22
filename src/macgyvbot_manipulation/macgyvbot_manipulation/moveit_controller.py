"""MoveIt planning helpers and controller-level motion operations."""

import math
import threading
import time

import numpy as np
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from moveit.core.robot_state import RobotState
from rclpy.action import ActionClient

from macgyvbot_manipulation.robot_safezone import (
    clamp_to_safe_workspace,
)

from macgyvbot_config.robot import (
    EE_LINK,
    GROUP_NAME,
    HOME_JOINTS,
    WRIST_JOINT_NAME,
)
from macgyvbot_manipulation.robot_pose import normalize_angle_deg


DEFAULT_TRAJECTORY_ACTION_NAME = "/dsr_moveit_controller/follow_joint_trajectory"


def plan_and_execute(
    robot,
    arm,
    logger,
    pose_goal=None,
    state_goal=None,
    params=None,
    should_interrupt=None,
    execute_trajectory=None,
):
    if should_interrupt is not None and should_interrupt():
        logger.info("중단 요청으로 planning/execution을 시작하지 않습니다.")
        return False

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
        if should_interrupt is not None and should_interrupt():
            logger.info("중단 요청으로 trajectory execution을 시작하지 않습니다.")
            return False

        if execute_trajectory is not None:
            return execute_trajectory(plan_result.trajectory, logger)

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

    def __init__(
        self,
        robot,
        arm,
        params,
        should_interrupt=None,
        node=None,
        trajectory_action_name=DEFAULT_TRAJECTORY_ACTION_NAME,
        poll_interval_sec=0.02,
    ):
        self.robot = robot
        self.arm = arm
        self.params = params
        self.should_interrupt = should_interrupt
        self.node = node
        self.trajectory_action_name = trajectory_action_name
        self.poll_interval_sec = poll_interval_sec
        self._current_goal_handle = None
        self._goal_lock = threading.Lock()
        self._trajectory_client = None
        if self.node is not None:
            self._trajectory_client = ActionClient(
                self.node,
                FollowJointTrajectory,
                self.trajectory_action_name,
            )

    def plan_and_execute(self, logger, pose_goal=None, state_goal=None):
        return plan_and_execute(
            self.robot,
            self.arm,
            logger,
            pose_goal=pose_goal,
            state_goal=state_goal,
            params=self.params,
            should_interrupt=self.should_interrupt,
            execute_trajectory=self._execute_trajectory_action
            if self._trajectory_client is not None
            else None,
        )

    def cancel_current_goal(self, logger=None, reason=""):
        """Cancel the active controller goal if this controller owns one."""
        with self._goal_lock:
            goal_handle = self._current_goal_handle

        if goal_handle is None:
            return False

        log = logger or (self.node.get_logger() if self.node is not None else None)
        if log is not None:
            suffix = f": {reason}" if reason else ""
            log.info(
                "현재 FollowJointTrajectory goal cancel 요청 전송"
                f"{suffix}"
            )

        try:
            goal_handle.cancel_goal_async()
        except Exception as exc:
            if log is not None:
                log.warn(f"현재 trajectory goal cancel 요청 실패: {exc}")
            return False

        return True

    def _execute_trajectory_action(self, robot_trajectory, logger):
        joint_trajectory = self._extract_joint_trajectory(
            robot_trajectory,
            logger,
        )
        if joint_trajectory is None:
            logger.error(
                "MoveIt plan 결과에서 joint_trajectory를 추출하지 못해 "
                "trajectory action 실행을 중단합니다."
            )
            return False

        if not joint_trajectory.joint_names or not joint_trajectory.points:
            logger.error("비어있는 joint trajectory라 실행할 수 없습니다.")
            return False

        if self.should_interrupt is not None and self.should_interrupt():
            logger.info("중단 요청으로 trajectory action goal을 전송하지 않습니다.")
            return False

        if not self._trajectory_client.wait_for_server(timeout_sec=1.0):
            logger.error(
                "FollowJointTrajectory action server를 찾지 못했습니다: "
                f"{self.trajectory_action_name}"
            )
            return False

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = joint_trajectory

        logger.info(
            "FollowJointTrajectory goal 전송: "
            f"action={self.trajectory_action_name}, "
            f"points={len(joint_trajectory.points)}"
        )
        send_future = self._trajectory_client.send_goal_async(goal_msg)

        goal_handle = self._wait_for_goal_acceptance(send_future, logger)
        if goal_handle is None:
            return False

        if not goal_handle.accepted:
            logger.error("FollowJointTrajectory goal이 거부되었습니다.")
            return False

        with self._goal_lock:
            self._current_goal_handle = goal_handle

        try:
            if self.should_interrupt is not None and self.should_interrupt():
                logger.info("goal accept 직후 중단 요청을 확인해 cancel을 전송합니다.")
                self.cancel_current_goal(logger, reason="interrupted_after_accept")

            result_future = goal_handle.get_result_async()
            while not result_future.done():
                if self.should_interrupt is not None and self.should_interrupt():
                    self.cancel_current_goal(logger, reason="exit/pause")
                    self._wait_for_future(
                        result_future,
                        timeout_sec=1.0,
                    )
                    break
                time.sleep(self.poll_interval_sec)

            if not result_future.done():
                logger.warn("trajectory cancel result를 기다리는 중 timeout 발생")
                return False

            try:
                result = result_future.result()
            except Exception as exc:
                logger.error(f"FollowJointTrajectory result 수신 실패: {exc}")
                return False

            status = result.status
            if status == GoalStatus.STATUS_SUCCEEDED:
                return True
            if status == GoalStatus.STATUS_CANCELED:
                logger.info("FollowJointTrajectory goal이 취소되었습니다.")
                return False
            if status == GoalStatus.STATUS_ABORTED:
                logger.error("FollowJointTrajectory goal이 abort되었습니다.")
                return False

            logger.warn(f"FollowJointTrajectory goal 종료 상태: status={status}")
            return False
        finally:
            with self._goal_lock:
                if self._current_goal_handle is goal_handle:
                    self._current_goal_handle = None

    def _wait_for_goal_acceptance(self, send_future, logger, timeout_sec=5.0):
        deadline = time.monotonic() + timeout_sec
        interrupted = False
        while not send_future.done():
            if self.should_interrupt is not None and self.should_interrupt():
                interrupted = True
            if time.monotonic() >= deadline:
                logger.error("FollowJointTrajectory goal accept 대기 timeout")
                return None
            time.sleep(self.poll_interval_sec)

        try:
            goal_handle = send_future.result()
        except Exception as exc:
            logger.error(f"FollowJointTrajectory goal 전송 실패: {exc}")
            return None

        if interrupted and goal_handle is not None and goal_handle.accepted:
            logger.info("goal accept 대기 중 중단 요청을 확인했습니다.")
        return goal_handle

    def _wait_for_future(self, future, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            time.sleep(self.poll_interval_sec)
        return future.done()

    @staticmethod
    def _extract_joint_trajectory(robot_trajectory, logger):
        if hasattr(robot_trajectory, "joint_trajectory"):
            return robot_trajectory.joint_trajectory

        for method_name in (
            "get_robot_trajectory_msg",
            "get_robot_trajectory_message",
            "get_trajectory_msg",
        ):
            method = getattr(robot_trajectory, method_name, None)
            if method is None:
                continue
            try:
                trajectory_msg = method()
            except Exception as exc:
                logger.debug(
                    f"{method_name}()로 trajectory msg 변환 실패: {exc}"
                )
                continue
            if hasattr(trajectory_msg, "joint_trajectory"):
                return trajectory_msg.joint_trajectory

        logger.error(
            "지원하지 않는 MoveIt trajectory 타입입니다: "
            f"{type(robot_trajectory).__name__}"
        )
        return None

    def move_to_home_joints(self, logger):
        """Move to the configured Home joint pose."""
        state_goal = RobotState(self.robot.get_robot_model())
        state_goal.joint_positions = HOME_JOINTS
        state_goal.update()

        logger.info("Home joint pose로 복귀합니다.")
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
