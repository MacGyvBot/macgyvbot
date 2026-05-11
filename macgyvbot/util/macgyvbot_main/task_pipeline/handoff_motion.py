"""Motion helpers for robot-to-human handoff."""

from macgyvbot.config.config import GRASP_ADVANCE_DISTANCE_M, SAFE_Z
from macgyvbot.util.macgyvbot_main.model_control.robot_pose import make_safe_pose
from macgyvbot.util.macgyvbot_main.model_control.robot_safezone import (
    clamp_to_safe_workspace,
)


class HandoffMotion:
    """Move to the handoff pose and return home after release."""

    def __init__(self, motion_controller, state):
        self.motion = motion_controller
        self.state = state

    def move_to_handoff_pose(self, travel_z, ori, logger):
        target_x = self.state.home_xyz[0] + GRASP_ADVANCE_DISTANCE_M
        target_y = self.state.home_xyz[1]
        target_z = max(travel_z, self.state.home_xyz[2], SAFE_Z)
        safe_x, safe_y, safe_z = clamp_to_safe_workspace(
            target_x,
            target_y,
            target_z,
            logger,
        )
        logger.info(
            "7단계: 사용자 전달 위치 이동 "
            f"Home 기준 전방 20cm x={self.state.home_xyz[0]:.3f}->{safe_x:.3f}, "
            f"y={safe_y:.3f}, z={safe_z:.3f}"
        )
        ok = self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(safe_x, safe_y, safe_z, ori, logger),
        )
        if not ok:
            logger.error("사용자 전달 위치 이동 실패. Pick 시퀀스 중단")
            self.state._publish_robot_status(
                "failed",
                message="사용자 전달 위치 이동에 실패했습니다.",
                reason="handoff_pose_move_failed",
                command=self.state.current_command,
            )
            return None, None, None

        return safe_x, safe_y, safe_z

    def move_home_after_handoff(self, travel_z, ori, logger):
        ok = self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(
                self.state.home_xyz[0],
                self.state.home_xyz[1],
                max(travel_z, self.state.home_xyz[2], SAFE_Z),
                ori,
                logger,
            ),
        )
        if not ok:
            logger.error("전달 후 Home 복귀 실패")
            self.state._publish_robot_status(
                "failed",
                message="공구 전달 후 Home 복귀에 실패했습니다.",
                reason="home_after_handoff_failed",
                command=self.state.current_command,
            )
            return False

        return True
