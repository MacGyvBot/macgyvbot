"""Force-based robot motion helpers."""

import rclpy

from macgyvbot.config.config import (
    RETURN_HOME_DESCENT_STEP_M,
    RETURN_HOME_FORCE_THRESHOLD_N,
    SEQUENCE_WAIT_POLL_SEC,
)
from macgyvbot.util.macgyvbot_main.model_control.robot_pose import make_safe_pose
from macgyvbot.util.macgyvbot_main.model_control.robot_safezone import SAFE_Z_MIN


class ForceReactionDetector:
    """Descend along Z until an upward reaction force is detected."""

    def __init__(self, motion_controller, state, wait_fn):
        self.motion = motion_controller
        self.state = state
        self.wait = wait_fn

    def descend_until_z_reaction(self, target_x, target_y, start_z, ori, logger):
        self.state.latest_wrench = None
        current_z = float(start_z)

        logger.info(
            "Z force 하강 시작 "
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
                logger.error("Z force 하강 motion 계획/실행 실패")
                return None

            current_z = next_z
            self.wait(SEQUENCE_WAIT_POLL_SEC)

        return None

    def _latest_force_z(self):
        wrench = getattr(self.state, "latest_wrench", None)
        if wrench is None:
            return None
        try:
            return float(wrench.force.z)
        except (AttributeError, TypeError, ValueError):
            return None
