"""Initial gripper grasp verification workflow."""

from __future__ import annotations

import time

from macgyvbot_config.grasp import (
    GRASP_WIDTH_MIN_MM,
    GRASP_WIDTH_OFFSET_PER_SIDE_MM,
    GRASP_VERIFY_POLL_SEC,
    PREGRASP_MAX_EXTRA_DESCENT_M,
    PREGRASP_MEASUREMENT_SETTLE_TIMEOUT_SEC,
)
from macgyvbot_manipulation.grasp_verifier import GraspVerifier
from macgyvbot_task.application.logging_utils import log_info, log_warn


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
                message=f"robot grasp attempt {attempt}/{retry_limit}",
                command=self.state.current_command,
            )

        return self.grasp_verifier.try_grasp(
            logger,
            publish_attempt=publish_attempt,
            failure_prefix="robot grasp",
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
                log_info(
                    logger,
                    "pregrasp measurement interrupted",
                    step="pregrasp",
                    event="interrupt",
                )
                return None

            try:
                status = gripper.get_status()
                busy = bool(status[0]) if status else False
                last_width_mm = float(gripper.get_width())
                last_depth_mm = float(gripper.get_depth())
            except Exception as exc:
                log_warn(
                    logger,
                    "pregrasp gripper depth read failed",
                    step="pregrasp",
                    event="fail",
                    reason=str(exc) or type(exc).__name__,
                )
                return None

            if not busy:
                log_info(
                    logger,
                    "pregrasp gripper measurement",
                    step="pregrasp",
                    event="measured",
                    width_mm=last_width_mm,
                    depth_mm=last_depth_mm,
                )
                return {
                    "width_mm": last_width_mm,
                    "depth_mm": last_depth_mm,
                }

            self.grasp_verifier.wait_fn(GRASP_VERIFY_POLL_SEC)

        log_warn(
            logger,
            "pregrasp measurement timed out",
            step="pregrasp",
            event="timeout",
            last_width_mm=last_width_mm,
            last_depth_mm=last_depth_mm,
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


def calculate_limited_grasp_width_mm(mask_width_mm, max_width_mm=None):
    """Add per-side clearance to a measured mask width and clamp to RG limits."""
    if mask_width_mm is None:
        return None

    width_mm = (
        float(mask_width_mm)
        + (2.0 * float(GRASP_WIDTH_OFFSET_PER_SIDE_MM))
    )
    width_mm = max(float(GRASP_WIDTH_MIN_MM), width_mm)
    if max_width_mm is not None and max_width_mm > 0.0:
        width_mm = min(width_mm, float(max_width_mm))
    return width_mm
