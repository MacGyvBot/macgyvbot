"""Force-based robot motion helpers."""
from macgyvbot_domain.logging import emit_structured_log

import rclpy

from macgyvbot_config.return_flow import (
    RETURN_HOME_DESCENT_STEP_M,
    RETURN_HOME_FORCE_THRESHOLD_N,
)
from macgyvbot_config.timing import SEQUENCE_WAIT_POLL_SEC
from macgyvbot_manipulation.robot_pose import make_safe_pose
from macgyvbot_manipulation.robot_safezone import SAFE_Z_MIN


class ForceReactionDetector:
    """Descend along Z until an upward reaction force is detected."""

    def __init__(self, motion_controller, state, wait_fn, interrupted=None):
        self.motion = motion_controller
        self.state = state
        self.wait = wait_fn
        self.interrupted = interrupted or (lambda: False)

    def descend_until_z_reaction(self, target_x, target_y, start_z, ori, logger):
        self.state.latest_wrench = None
        current_z = float(start_z)

        logger.info(
            "force_descent",
            "start",
            pipe="manipulation",
            threshold_n=f"{RETURN_HOME_FORCE_THRESHOLD_N:.1f}",
            min_z=f"{SAFE_Z_MIN:.3f}",
            msg="Z force descent started",
        )
        emit_structured_log(logger, 'info', "log", "status", svc='manipulation', pipe='force', msg="Z force 하강 시작 "
            f"threshold={RETURN_HOME_FORCE_THRESHOLD_N:.1f}N, "
            f"min_z={SAFE_Z_MIN:.3f}")

        while rclpy.ok():
            if self.interrupted():
                logger.info(
                    "force_descent",
                    "cancel",
                    pipe="manipulation",
                    reason="interrupted",
                    msg="Z force descent interrupted",
                )
                emit_structured_log(logger, 'info', "log", "status", svc='manipulation', pipe='force', msg="Z force 하강을 stop/pause 요청으로 중단합니다.")
                return None

            force_z = self._latest_force_z()
            if force_z is not None and force_z >= RETURN_HOME_FORCE_THRESHOLD_N:
                logger.info(
                    "force_descent",
                    "done",
                    pipe="manipulation",
                    reason="threshold_reached",
                    force_z=f"{force_z:.2f}",
                    z=f"{current_z:.3f}",
                    msg="Z force threshold reached",
                )
                emit_structured_log(logger, 'info', "log", "status", svc='manipulation', pipe='force', msg="Z 반대방향 힘 감지로 하강을 중단합니다: "
                    f"force_z={force_z:.2f}N, z={current_z:.3f}")
                return current_z

            if current_z <= SAFE_Z_MIN:
                logger.warn(
                    "force_descent",
                    "done",
                    pipe="manipulation",
                    reason="safe_z_min_reached",
                    last_force_z=force_z,
                    z=f"{current_z:.3f}",
                    msg="safe minimum Z reached before force threshold",
                )
                emit_structured_log(logger, 'warn', "log", "status", svc='manipulation', pipe='force', msg="Z 반력이 임계값에 도달하지 않았지만 안전 최소 Z까지 하강했습니다: "
                    f"last_force_z={force_z}")
                return current_z

            next_z = max(SAFE_Z_MIN, current_z - RETURN_HOME_DESCENT_STEP_M)
            ok = self.motion.plan_and_execute(
                logger,
                pose_goal=make_safe_pose(target_x, target_y, next_z, ori, logger),
            )
            if not ok:
                if self.interrupted():
                    emit_structured_log(logger, 'info', "log", "status", svc='manipulation', pipe='force', msg="Z force 하강 motion 중 stop/pause 요청을 확인했습니다.")
                    return None

                logger.error(
                    "force_descent",
                    "fail",
                    pipe="manipulation",
                    reason="motion_failed",
                    z=f"{next_z:.3f}",
                    msg="Z force descent motion failed",
                )
                emit_structured_log(logger, 'error', "log", "status", svc='manipulation', pipe='force', msg="Z force 하강 motion 계획/실행 실패")
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
