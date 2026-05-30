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
POSE_GOAL_IK_TIMEOUT_SEC = 0.1
POSE_GOAL_IK_MAX_SEEDS = 10
POSE_GOAL_IK_SEED_PERTURB_RAD = math.radians(10.0)
POSE_GOAL_MAX_JOINT_DELTA_RAD = math.radians(120.0)
_TWO_PI = 2.0 * math.pi


def _nearest_equivalent_value(current, target):
    nearest_k = int(round((float(current) - float(target)) / _TWO_PI))
    candidates = [
        float(target) + _TWO_PI * k
        for k in range(nearest_k - 1, nearest_k + 2)
    ]
    return min(candidates, key=lambda value: abs(value - float(current)))


def _nearest_equivalent_positions(current_positions, target_positions):
    return np.array(
        [
            _nearest_equivalent_value(current, target)
            for current, target in zip(current_positions, target_positions)
        ],
        dtype=float,
    )


def _principal_joint_positions(positions):
    return np.array(
        [math.atan2(math.sin(value), math.cos(value)) for value in positions],
        dtype=float,
    )


def _ik_seed_positions(current_positions):
    seeds = [np.array(current_positions, copy=True)]
    principal_positions = _principal_joint_positions(current_positions)

    if not np.allclose(principal_positions, current_positions, atol=1e-6):
        seeds.append(principal_positions)

    for index in range(len(current_positions)):
        if np.isclose(
            principal_positions[index],
            current_positions[index],
            atol=1e-6,
        ):
            continue
        seed = np.array(current_positions, copy=True)
        seed[index] = principal_positions[index]
        seeds.append(seed)

    for index in range(len(current_positions)):
        for direction in (1.0, -1.0):
            seed = np.array(current_positions, copy=True)
            seed[index] += direction * POSE_GOAL_IK_SEED_PERTURB_RAD
            seeds.append(seed)

    unique_seeds = []
    seen = set()
    for seed in seeds:
        key = tuple(round(float(value), 6) for value in seed)
        if key in seen:
            continue
        seen.add(key)
        unique_seeds.append(seed)
        if len(unique_seeds) >= POSE_GOAL_IK_MAX_SEEDS:
            break

    return unique_seeds


def _state_satisfies_bounds(state, joint_model_group):
    satisfies_bounds = getattr(state, "satisfies_bounds", None)
    if satisfies_bounds is None:
        return True

    for args in ((joint_model_group,), (GROUP_NAME,), ()):
        try:
            return bool(satisfies_bounds(*args))
        except TypeError:
            continue
    return True


def _format_joint_deltas(joint_names, current_positions, raw_positions, goal_positions):
    parts = []
    for name, current, raw, goal in zip(
        joint_names,
        current_positions,
        raw_positions,
        goal_positions,
    ):
        raw_delta = float(raw) - float(current)
        goal_delta = float(goal) - float(current)
        parts.append(
            f"{name}: curr={math.degrees(current):.1f}deg, "
            f"ik={math.degrees(raw):.1f}deg, "
            f"raw={math.degrees(raw_delta):.1f}deg, "
            f"short={math.degrees(goal_delta):.1f}deg"
        )
    return "; ".join(parts)


def _pose_goal_to_near_current_state_goal(robot, pose_goal, logger):
    robot_model = robot.get_robot_model()
    jmg = robot_model.get_joint_model_group(GROUP_NAME)
    joint_names = list(jmg.active_joint_model_names)

    try:
        with robot.get_planning_scene_monitor().read_only() as scene:
            current_positions = np.array(
                scene.current_state.get_joint_group_positions(GROUP_NAME),
                dtype=float,
            )
    except Exception as exc:
        logger.warn(
            "pose_goal IK seed용 현재 joint state를 읽지 못했습니다. "
            f"pose_goal planning을 중단합니다: {exc}"
        )
        return None

    if len(current_positions) != len(joint_names):
        logger.warn(
            "pose_goal IK seed joint 수가 맞지 않습니다. "
            "pose_goal planning을 중단합니다: "
            f"positions={len(current_positions)}, joints={len(joint_names)}"
        )
        return None

    candidates = []
    candidate_keys = set()
    ik_failures = 0

    for seed_index, seed_positions in enumerate(
        _ik_seed_positions(current_positions),
        start=1,
    ):
        ik_state = RobotState(robot_model)
        ik_state.set_joint_group_positions(GROUP_NAME, seed_positions)
        ik_state.update()

        try:
            found_ik = ik_state.set_from_ik(
                GROUP_NAME,
                pose_goal.pose,
                EE_LINK,
                POSE_GOAL_IK_TIMEOUT_SEC,
            )
        except Exception as exc:
            ik_failures += 1
            logger.warn(
                "pose_goal IK 계산 실패: "
                f"seed={seed_index}, error={exc}"
            )
            continue

        if not found_ik:
            ik_failures += 1
            continue

        ik_state.update()
        raw_positions = np.array(
            ik_state.get_joint_group_positions(GROUP_NAME),
            dtype=float,
        )
        goal_positions = _nearest_equivalent_positions(
            current_positions,
            raw_positions,
        )
        candidate_key = tuple(round(float(value), 6) for value in goal_positions)
        if candidate_key in candidate_keys:
            continue
        candidate_keys.add(candidate_key)

        goal_state = RobotState(robot_model)
        goal_state.set_joint_group_positions(GROUP_NAME, goal_positions)
        goal_state.update()
        if not _state_satisfies_bounds(goal_state, jmg):
            logger.warn(
                "pose_goal IK 후보가 joint bounds를 벗어나 제외합니다: "
                + _format_joint_deltas(
                    joint_names,
                    current_positions,
                    raw_positions,
                    goal_positions,
                )
            )
            continue

        deltas = goal_positions - current_positions
        abs_deltas = np.abs(deltas)
        max_delta = float(np.max(abs_deltas))
        total_delta = float(np.sum(abs_deltas))
        candidates.append(
            (
                max_delta,
                total_delta,
                seed_index,
                goal_state,
                raw_positions,
                goal_positions,
            )
        )

    if not candidates:
        logger.warn(
            "pose_goal IK 후보를 찾지 못했습니다. "
            f"seeds={POSE_GOAL_IK_MAX_SEEDS}, failures={ik_failures}"
        )
        return None

    candidates.sort(key=lambda candidate: (candidate[0], candidate[1]))
    (
        max_delta,
        total_delta,
        seed_index,
        goal_state,
        raw_positions,
        goal_positions,
    ) = candidates[0]

    logger.info(
        "pose_goal IK best candidate: "
        f"seed={seed_index}, candidates={len(candidates)}, "
        f"max_delta={math.degrees(max_delta):.1f}deg, "
        f"total_delta={math.degrees(total_delta):.1f}deg"
    )
    logger.info(
        "pose_goal IK selected delta: "
        + _format_joint_deltas(
            joint_names,
            current_positions,
            raw_positions,
            goal_positions,
        )
    )

    if max_delta > POSE_GOAL_MAX_JOINT_DELTA_RAD:
        logger.warn(
            "pose_goal IK 후보의 최대 joint delta가 안전 한계를 초과해 "
            "planning을 중단합니다: "
            f"max_delta={math.degrees(max_delta):.1f}deg, "
            f"limit={math.degrees(POSE_GOAL_MAX_JOINT_DELTA_RAD):.1f}deg"
        )
        return None

    return goal_state


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

        ik_state_goal = _pose_goal_to_near_current_state_goal(
            robot,
            pose_goal,
            logger,
        )

        if ik_state_goal is not None:
            arm.set_goal_state(robot_state=ik_state_goal)
        else:
            logger.error(
                "pose_goal을 현재 joint 기준 가까운 IK joint goal로 "
                "변환하지 못해 planning을 중단합니다."
            )
            return False

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

        node_log = getattr(self.node, "service_log", None) if self.node is not None else None
        log = logger or (node_log.bind("motion") if node_log is not None else None)
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
        if self._is_at_joint_goal(HOME_JOINTS, logger):
            logger.info("현재 joint pose가 이미 Home 허용 오차 안에 있어 이동을 생략합니다.")
            return True

        state_goal = RobotState(self.robot.get_robot_model())
        state_goal.joint_positions = HOME_JOINTS
        state_goal.update()

        logger.info("Home joint pose로 복귀합니다.")
        ok = self.plan_and_execute(logger, state_goal=state_goal)
        if ok:
            return True

        if self._wait_until_at_joint_goal(HOME_JOINTS, logger):
            logger.warn(
                "Home trajectory 결과는 실패로 보고되었지만 현재 joint pose가 "
                "Home 허용 오차 안에 있어 Home 복귀 성공으로 처리합니다."
            )
            return True

        return False

    def _wait_until_at_joint_goal(
        self,
        goal_joints,
        logger,
        timeout_sec=0.5,
        tolerance_rad=0.02,
    ):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() <= deadline:
            if self._is_at_joint_goal(
                goal_joints,
                logger,
                tolerance_rad=tolerance_rad,
            ):
                return True
            time.sleep(self.poll_interval_sec)

        return False

    def _is_at_joint_goal(self, goal_joints, logger, tolerance_rad=0.02):
        try:
            robot_model = self.robot.get_robot_model()
            jmg = robot_model.get_joint_model_group(GROUP_NAME)
            joint_names = list(jmg.active_joint_model_names)
            psm = self.robot.get_planning_scene_monitor()
            with psm.read_only() as scene:
                current_positions = np.array(
                    scene.current_state.get_joint_group_positions(GROUP_NAME),
                    dtype=float,
                )
        except Exception as exc:
            log_debug = getattr(logger, "debug", logger.info)
            log_debug(f"현재 joint pose 확인 실패, Home 이동을 계속합니다: {exc}")
            return False

        if len(current_positions) < len(joint_names):
            return False

        for index, joint_name in enumerate(joint_names):
            if joint_name not in goal_joints:
                return False
            diff = current_positions[index] - float(goal_joints[joint_name])
            wrapped_diff = math.atan2(math.sin(diff), math.cos(diff))
            if abs(wrapped_diff) > tolerance_rad:
                return False

        return True

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
