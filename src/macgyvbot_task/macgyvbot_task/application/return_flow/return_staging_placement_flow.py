"""Place a returned tool at the drawer-store staging point."""

from __future__ import annotations

import numpy as np
from moveit.core.robot_state import RobotState

from macgyvbot_config.drawer import (
    DRAWER_STORE_FORCE_DESCENT_START_Z_OFFSET_M,
    DRAWER_STORE_TOOL_OBSERVE_POINT,
    TOOL_OBSERVE_X_BACKOFF_M,
)
from macgyvbot_config.return_flow import RETURN_TOOL_RELEASE_WAIT_SEC
from macgyvbot_config.robot import EE_LINK
from macgyvbot_manipulation.force_detection import ForceReactionDetector
from macgyvbot_manipulation.robot_pose import (
    current_ee_orientation,
    get_ee_matrix,
    make_safe_pose,
    orientation_from_transform,
)
from macgyvbot_manipulation.robot_safezone import SAFE_Z_MIN


class ReturnStagingPlacementFlow:
    """Move to the staging point, release the returned tool, and retreat."""

    def __init__(
        self,
        robot,
        motion_controller,
        gripper,
        state,
        reporter,
        wait_fn,
        tool_hold_monitor=None,
        interrupted=None,
    ):
        self.robot = robot
        self.motion = motion_controller
        self.gripper = gripper
        self.state = state
        self.reporter = reporter
        self.wait_fn = wait_fn
        self.tool_hold_monitor = tool_hold_monitor
        self.interrupted = interrupted or (lambda: False)
        self.force_detector = ForceReactionDetector(
            motion_controller,
            state,
            wait_fn,
            interrupted=self.interrupted,
        )

    def place_at_store_observe_point(self, tool_name, command, logger):
        if self.interrupted():
            logger.info("반납 임시 배치 시작 전 stop/pause 요청으로 중단합니다.")
            return False

        self.reporter.publish(
            "placing_return_tool",
            tool_name,
            f"{tool_name} 임시 관찰 위치로 이동합니다.",
            command,
        )

        logger.info(f"반납 2단계: {tool_name} 임시 관찰 joint pose 이동")
        ok = self.move_to_store_observe_point(logger)
        if not ok:
            if self.interrupted():
                logger.info("반납 임시 관찰 위치 이동 중 stop/pause 요청으로 중단합니다.")
                return False

            self.reporter.fail(
                tool_name,
                f"{tool_name} 임시 관찰 위치 이동에 실패했습니다.",
                "return_store_observe_move_failed",
                command,
                logger,
            )
            return False

        current_pose = get_ee_matrix(self.robot)
        target_x = float(current_pose[0, 3])
        target_y = float(current_pose[1, 3])
        ori = current_ee_orientation(self.robot)
        descent_start_z = (
            SAFE_Z_MIN + DRAWER_STORE_FORCE_DESCENT_START_Z_OFFSET_M
        )

        self.reporter.publish(
            "lowering_return_tool",
            tool_name,
            "임시 관찰 위치에서 Z를 낮추며 반력을 확인합니다.",
            command,
        )
        if not self._move_to_force_descent_start(
            target_x,
            target_y,
            descent_start_z,
            ori,
            tool_name,
            command,
            logger,
        ):
            return False

        stop_z = self.force_detector.descend_until_z_reaction(
            target_x,
            target_y,
            descent_start_z,
            ori,
            logger,
        )
        if stop_z is None:
            if self.interrupted():
                logger.info("반납 임시 관찰 위치 Z 하강 중 stop/pause 요청으로 중단합니다.")
                return False

            self.reporter.fail(
                tool_name,
                "반납 임시 관찰 위치 Z 하강에 실패했습니다.",
                "return_store_observe_descent_failed",
                command,
                logger,
            )
            return False

        logger.info(f"반납 4단계: {tool_name} 임시 관찰 위치에 놓기")
        if self.interrupted():
            logger.info("반납 공구 놓기 전 stop/pause 요청으로 중단합니다.")
            return False

        if self.tool_hold_monitor is not None:
            self.tool_hold_monitor.stop("return_store_observe_release")
        self.gripper.open_gripper()
        self.wait_fn(RETURN_TOOL_RELEASE_WAIT_SEC)
        if self.interrupted():
            logger.info("반납 공구 놓기 후 stop/pause 요청으로 중단합니다.")
            return False

        return self.move_to_store_observe_viewpoint(tool_name, command, logger)

    def move_to_store_observe_point(self, logger):
        if self.interrupted():
            logger.info("임시 관찰 위치 이동 전 stop/pause 요청으로 중단합니다.")
            return False

        state_goal = RobotState(self.robot.get_robot_model())
        state_goal.joint_positions = dict(DRAWER_STORE_TOOL_OBSERVE_POINT)
        state_goal.update()
        return self.motion.plan_and_execute(logger, state_goal=state_goal)

    def _move_to_force_descent_start(
        self,
        target_x,
        target_y,
        descent_start_z,
        ori,
        tool_name,
        command,
        logger,
    ):
        if self.interrupted():
            logger.info("반력 하강 시작 위치 이동 전 stop/pause 요청으로 중단합니다.")
            return False

        logger.info(
            "반력 확인 하강 시작 위치 이동: "
            f"z={descent_start_z:.3f} "
            f"(SAFE_Z_MIN + {DRAWER_STORE_FORCE_DESCENT_START_Z_OFFSET_M:.3f})"
        )
        ok = self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(
                target_x,
                target_y,
                descent_start_z,
                ori,
                logger,
            ),
        )
        if ok:
            return True

        if self.interrupted():
            return False

        self.reporter.fail(
            tool_name,
            "반납 임시 관찰 위치 반력 하강 시작점 이동에 실패했습니다.",
            "return_store_observe_descent_failed",
            command,
            logger,
        )
        return False

    def move_to_store_observe_viewpoint(self, tool_name, command, logger):
        if self.interrupted():
            logger.info("반납 후 임시 관찰 viewpoint 이동 전 stop/pause 요청으로 중단합니다.")
            return False

        target_x, target_y, target_z, ori = self._store_observe_view_pose()
        logger.info(
            "반납 5단계: 공구를 놓은 뒤 임시 관찰 viewpoint로 이동 "
            f"(observe_x_backoff={TOOL_OBSERVE_X_BACKOFF_M:.3f}, "
            f"target=({target_x:.3f}, {target_y:.3f}, {target_z:.3f}))"
        )
        ok = self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(target_x, target_y, target_z, ori, logger),
        )
        if ok:
            return True

        if self.interrupted():
            return False

        self.reporter.fail(
            tool_name,
            "반납 공구를 놓은 뒤 임시 관찰 viewpoint 이동에 실패했습니다.",
            "return_store_observe_viewpoint_failed",
            command,
            logger,
        )
        return False

    def _store_observe_view_pose(self):
        state = RobotState(self.robot.get_robot_model())
        state.joint_positions = dict(DRAWER_STORE_TOOL_OBSERVE_POINT)
        state.update()
        transform = np.asarray(state.get_global_link_transform(EE_LINK), dtype=float)
        ori = orientation_from_transform(transform)

        return (
            float(transform[0, 3]) - TOOL_OBSERVE_X_BACKOFF_M,
            float(transform[1, 3]),
            float(transform[2, 3]),
            ori,
        )
