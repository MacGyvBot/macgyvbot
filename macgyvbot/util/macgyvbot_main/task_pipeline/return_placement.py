"""Home placement flow for returned tools."""

import rclpy

from macgyvbot.config.config import (
    RETURN_HOME_DESCENT_START_Z,
    RETURN_HOME_DESCENT_STEP_M,
    RETURN_HOME_FORCE_THRESHOLD_N,
    SEQUENCE_WAIT_POLL_SEC,
)
from macgyvbot.util.macgyvbot_main.model_control.robot_pose import make_safe_pose
from macgyvbot.util.macgyvbot_main.model_control.robot_safezone import SAFE_Z_MIN


class ReturnPlacementRunner:
    """Place a returned tool at Home using force-guided Z descent."""

    def __init__(
        self,
        motion_controller,
        gripper,
        state,
        publish_status_cb,
        fail_cb,
        wait_fn,
    ):
        self.motion = motion_controller
        self.gripper = gripper
        self.state = state
        self.publish_status = publish_status_cb
        self.fail = fail_cb
        self.wait = wait_fn

    def place_at_robot_home(self, tool_name, ori, command, logger):
        target_x, target_y, _ = self.state.home_xyz
        approach_z = max(RETURN_HOME_DESCENT_START_Z, SAFE_Z_MIN)

        self._publish_status(
            "placing_return_tool",
            tool_name,
            f"{tool_name} 반납 위치인 Home으로 이동합니다.",
            command,
        )

        logger.info(
            f"반납 2단계: {tool_name} 반납 Home 위치 이동 "
            f"x={target_x:.3f}, y={target_y:.3f}, z={approach_z:.3f}"
        )
        ok = self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(target_x, target_y, approach_z, ori, logger),
        )
        if not ok:
            self.fail(
                tool_name,
                f"{tool_name} 반납 Home 위치 이동에 실패했습니다.",
                "return_home_move_failed",
                command,
                logger,
            )
            return False

        stop_z = self._descend_until_z_reaction(
            target_x,
            target_y,
            approach_z,
            ori,
            tool_name,
            command,
            logger,
        )
        if stop_z is None:
            return False

        logger.info(f"반납 4단계: {tool_name} Home 위치에 놓기")
        self.gripper.open_gripper()
        self.wait(0.8)

        return self._move_home_after_return(
            target_x,
            target_y,
            approach_z,
            ori,
            tool_name,
            command,
            logger,
        )

    def _descend_until_z_reaction(
        self,
        target_x,
        target_y,
        start_z,
        ori,
        tool_name,
        command,
        logger,
    ):
        self.state.latest_wrench = None
        current_z = float(start_z)

        self._publish_status(
            "lowering_return_tool",
            tool_name,
            "Home에서 Z를 낮추며 반력을 확인합니다.",
            command,
        )
        logger.info(
            "반납 3단계: Home에서 Z 하강 시작 "
            f"threshold={RETURN_HOME_FORCE_THRESHOLD_N:.1f}N, "
            f"min_z={SAFE_Z_MIN:.3f}"
        )

        while rclpy.ok():
            force_z = self._latest_force_z()
            if force_z is not None and force_z >= RETURN_HOME_FORCE_THRESHOLD_N:
                logger.info(
                    "Z 반대방향 힘 감지로 하강을 중단합니다: "
                    f"force_z={force_z:.2f}N, z={current_z:.3f}"
                )
                return current_z

            if current_z <= SAFE_Z_MIN:
                logger.warn(
                    "Z 반력이 임계값에 도달하지 않았지만 안전 최소 Z까지 하강했습니다: "
                    f"last_force_z={force_z}"
                )
                return current_z

            next_z = max(SAFE_Z_MIN, current_z - RETURN_HOME_DESCENT_STEP_M)
            ok = self.motion.plan_and_execute(
                logger,
                pose_goal=make_safe_pose(target_x, target_y, next_z, ori, logger),
            )
            if not ok:
                self.fail(
                    tool_name,
                    "반납 Home Z 하강에 실패했습니다.",
                    "return_home_descent_failed",
                    command,
                    logger,
                )
                return None

            current_z = next_z
            self.wait(SEQUENCE_WAIT_POLL_SEC)

        return None

    def _move_home_after_return(
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
            self.fail(
                tool_name,
                "반납 공구를 놓은 뒤 Home 안전 높이 복귀에 실패했습니다.",
                "return_home_retreat_failed",
                command,
                logger,
            )
            return False

        final_x, final_y, final_z = self.state.home_xyz
        final_z = max(final_z, approach_z)
        ok = self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(final_x, final_y, final_z, ori, logger),
        )
        if not ok:
            self.fail(
                tool_name,
                "반납 공구를 놓은 뒤 Home 복귀에 실패했습니다.",
                "return_home_after_release_failed",
                command,
                logger,
            )
            return False

        return True

    def _latest_force_z(self):
        wrench = getattr(self.state, "latest_wrench", None)
        if wrench is None:
            return None
        try:
            return float(wrench.force.z)
        except (AttributeError, TypeError, ValueError):
            return None

    def _publish_status(self, status, tool_name, message, command):
        self.publish_status(
            status,
            tool_name,
            message=message,
            command=command,
        )
