"""Initial gripper grasp verification workflow."""

from __future__ import annotations

import time

from macgyvbot_config.grasp import (
    GRASP_VERIFY_POLL_SEC,
    PREGRASP_MAX_EXTRA_DESCENT_M,
    PREGRASP_MEASUREMENT_SETTLE_TIMEOUT_SEC,
)
from macgyvbot_manipulation.grasp_verifier import GraspVerifier


class PickGraspFlow:
    """Handle initial robot grasp verification for pick sequences."""

    def __init__(
        self,
        gripper,
        state,
        wait_fn,
        interrupted=None,
    ):
        self.state = state
        self.grasp_verifier = GraspVerifier(
            gripper,
            wait_fn,
            interrupted=interrupted,
        )

    def try_robot_grasp(self, logger):
        def publish_attempt(attempt, retry_limit):
            self.state._publish_robot_status(
                "grasping",
                message=f"공구 grasp 시도 {attempt}/{retry_limit}",
                command=self.state.current_command,
            )

        return self.grasp_verifier.try_grasp(
            logger,
            publish_attempt=publish_attempt,
            failure_prefix="그리퍼 grasp",
        )

    def measure_pregrasp_depth(self, logger):
        """Close once and return settled width/depth in millimeters."""
        gripper = self.grasp_verifier.gripper
        gripper.close_gripper()

        start_time = time.monotonic()
        last_width_mm = None
        last_depth_mm = None
        while time.monotonic() - start_time < PREGRASP_MEASUREMENT_SETTLE_TIMEOUT_SEC:
            if self.grasp_verifier.interrupted():
                logger.info("pre-grasp depth 측정을 stop/pause 요청으로 중단합니다.")
                return None

            try:
                status = gripper.get_status()
                busy = bool(status[0]) if status else False
                last_width_mm = float(gripper.get_width())
                last_depth_mm = float(gripper.get_depth())
            except Exception as exc:
                logger.warn(f"pre-grasp 그리퍼 depth 읽기 실패: {exc}")
                return None

            if not busy:
                logger.info(
                    "pre-grasp gripper measurement: "
                    f"width={last_width_mm:.1f}mm, "
                    f"depth={last_depth_mm:.1f}mm"
                )
                return {
                    "width_mm": last_width_mm,
                    "depth_mm": last_depth_mm,
                }

            self.grasp_verifier.wait_fn(GRASP_VERIFY_POLL_SEC)

        logger.warn(
            "pre-grasp depth 측정 timeout: "
            f"last_width={last_width_mm}mm, "
            f"last_depth={last_depth_mm}mm"
        )
        if last_depth_mm is None:
            return None
        return {
            "width_mm": last_width_mm,
            "depth_mm": last_depth_mm,
        }


def calculate_pregrasp_extra_descent(depth_mm):
    """Convert RG2 actual depth in millimeters to extra Z descent in meters."""
    if depth_mm is None:
        return None

    descent_m = abs(float(depth_mm)) / 1000.0
    return min(descent_m, PREGRASP_MAX_EXTRA_DESCENT_M)
