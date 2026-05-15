"""Place a returned tool at the robot Home position."""

from __future__ import annotations

from macgyvbot.control.force_detection import ForceReactionDetector
from macgyvbot.control.robot_pose import get_ee_matrix, make_safe_pose


class ReturnHomePlacementFlow:
    """Move to Home, descend with force feedback, release, and retreat."""

    def __init__(self, robot, motion_controller, gripper, state, reporter, wait_fn):
        self.robot = robot
        self.motion = motion_controller
        self.gripper = gripper
        self.state = state
        self.reporter = reporter
        self.wait_fn = wait_fn
        self.force_detector = ForceReactionDetector(
            motion_controller,
            state,
            wait_fn,
        )

    def place_at_robot_home(self, tool_name, ori, command, logger):
        self.reporter.publish(
            "placing_return_tool",
            tool_name,
            f"{tool_name} 반납 위치인 Home joint pose로 이동합니다.",
            command,
        )

        logger.info(f"반납 2단계: {tool_name} 반납 Home joint pose 이동")
        ok = self.motion.move_to_home_joints(logger)
        if not ok:
            self.reporter.fail(
                tool_name,
                f"{tool_name} 반납 Home joint pose 이동에 실패했습니다.",
                "return_home_move_failed",
                command,
                logger,
            )
            return False

        current_pose = get_ee_matrix(self.robot)
        target_x = float(current_pose[0, 3])
        target_y = float(current_pose[1, 3])
        approach_z = float(current_pose[2, 3])

        self.reporter.publish(
            "lowering_return_tool",
            tool_name,
            "Home joint pose에서 Z를 낮추며 반력을 확인합니다.",
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
            self.reporter.fail(
                tool_name,
                "반납 Home Z 하강에 실패했습니다.",
                "return_home_descent_failed",
                command,
                logger,
            )
            return False

        logger.info(f"반납 4단계: {tool_name} Home 위치에 놓기")
        self.gripper.open_gripper()
        self.wait_fn(0.8)

        return self.move_home_after_return(
            target_x,
            target_y,
            approach_z,
            ori,
            tool_name,
            command,
            logger,
        )

    def move_home_after_return(
        self,
        target_x,
        target_y,
        approach_z,
        ori,
        tool_name,
        command,
        logger,
    ):
        logger.info("반납 5단계: 공구를 놓은 뒤 Home 안전 높이로 복귀")
        ok = self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(target_x, target_y, approach_z, ori, logger),
        )
        if not ok:
            self.reporter.fail(
                tool_name,
                "반납 공구를 놓은 뒤 Home 안전 높이 복귀에 실패했습니다.",
                "return_home_retreat_failed",
                command,
                logger,
            )
            return False

        logger.info("반납 6단계: Home joint pose로 복귀")
        ok = self.motion.move_to_home_joints(logger)
        if not ok:
            self.reporter.fail(
                tool_name,
                "반납 공구를 놓은 뒤 Home 복귀에 실패했습니다.",
                "return_home_after_release_failed",
                command,
                logger,
            )
            return False

        return True
