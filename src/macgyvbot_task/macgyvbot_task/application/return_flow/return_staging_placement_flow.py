"""Place a returned tool at the drawer-store staging point."""

from __future__ import annotations

from moveit.core.robot_state import RobotState

from macgyvbot_config.drawer import DRAWER_STORE_TOOL_OBSERVE_POINT
from macgyvbot_manipulation.force_detection import ForceReactionDetector
from macgyvbot_manipulation.robot_pose import (
    current_ee_orientation,
    get_ee_matrix,
    make_safe_pose,
)


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
        approach_z = float(current_pose[2, 3])
        ori = current_ee_orientation(self.robot)

        self.reporter.publish(
            "lowering_return_tool",
            tool_name,
            "임시 관찰 위치에서 Z를 낮추며 반력을 확인합니다.",
            command,
        )
        stop_z = self.force_detector.descend_until_z_reaction(
            target_x,
            target_y,
            approach_z,
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
        self.wait_fn(0.8)
        if self.interrupted():
            logger.info("반납 공구 놓기 후 stop/pause 요청으로 중단합니다.")
            return False

        return self.retreat_after_staging_release(
            target_x,
            target_y,
            approach_z,
            ori,
            tool_name,
            command,
            logger,
        )

    def move_to_store_observe_point(self, logger):
        if self.interrupted():
            logger.info("임시 관찰 위치 이동 전 stop/pause 요청으로 중단합니다.")
            return False

        state_goal = RobotState(self.robot.get_robot_model())
        state_goal.joint_positions = dict(DRAWER_STORE_TOOL_OBSERVE_POINT)
        state_goal.update()
        return self.motion.plan_and_execute(logger, state_goal=state_goal)

    def retreat_after_staging_release(
        self,
        target_x,
        target_y,
        approach_z,
        ori,
        tool_name,
        command,
        logger,
    ):
        if self.interrupted():
            logger.info("반납 후 복귀 시작 전 stop/pause 요청으로 중단합니다.")
            return False

        logger.info("반납 5단계: 공구를 놓은 뒤 임시 관찰 안전 높이로 복귀")
        ok = self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(target_x, target_y, approach_z, ori, logger),
        )
        if not ok:
            if self.interrupted():
                logger.info("반납 후 안전 높이 복귀 중 stop/pause 요청으로 중단합니다.")
                return False

            self.reporter.fail(
                tool_name,
                "반납 공구를 놓은 뒤 임시 관찰 안전 높이 복귀에 실패했습니다.",
                "return_store_observe_retreat_failed",
                command,
                logger,
            )
            return False

        logger.info("반납 6단계: 임시 관찰 joint pose로 복귀")
        ok = self.move_to_store_observe_point(logger)
        if not ok:
            if self.interrupted():
                logger.info("반납 후 임시 관찰 위치 복귀 중 stop/pause 요청으로 중단합니다.")
                return False

            self.reporter.fail(
                tool_name,
                "반납 공구를 놓은 뒤 임시 관찰 위치 복귀에 실패했습니다.",
                "return_store_observe_after_release_failed",
                command,
                logger,
            )
            return False

        return True
