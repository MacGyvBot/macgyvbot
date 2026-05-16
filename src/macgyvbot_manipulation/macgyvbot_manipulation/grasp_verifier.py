"""Shared robot gripper grasp verification policy."""

import time

from macgyvbot_config.grasp import (
    GRASP_RETRY_LIMIT,
    GRASP_VERIFY_POLL_SEC,
    GRASP_VERIFY_STABLE_COUNT,
    GRASP_VERIFY_TIMEOUT_SEC,
    GRIPPER_CLOSED_WIDTH_THRESHOLD_MM,
)
from macgyvbot_config.timing import SEQUENCE_WAIT_POLL_SEC


class GraspVerifier:
    """Confirm that the gripper is stably holding an object."""

    def __init__(self, gripper, wait_fn=None):
        self.gripper = gripper
        self.wait_fn = wait_fn or cooperative_wait

    def try_grasp(self, logger, publish_attempt=None, failure_prefix="공구 grasp"):
        for attempt in range(1, GRASP_RETRY_LIMIT + 1):
            logger.info(f"{failure_prefix} 시도 {attempt}/{GRASP_RETRY_LIMIT}")
            if publish_attempt is not None:
                publish_attempt(attempt, GRASP_RETRY_LIMIT)

            self.gripper.close_gripper()
            if self.verify(logger):
                return True

            logger.warn(
                f"{failure_prefix} 실패. 재시도 준비 {attempt}/{GRASP_RETRY_LIMIT}"
            )
            self.gripper.open_gripper()
            self.wait_fn(0.5)

        return False

    def verify(self, logger):
        start_time = time.monotonic()
        last_status = None
        stable_count = 0

        while time.monotonic() - start_time < GRASP_VERIFY_TIMEOUT_SEC:
            try:
                confirmed, busy, status, width_mm = read_grasp_confirmation(
                    self.gripper,
                    logger,
                )
            except Exception as exc:
                logger.warn(f"그리퍼 상태 읽기 실패: {exc}")
                return False

            last_status = {
                "status": status,
                "width_mm": width_mm,
            }

            if confirmed:
                stable_count += 1
                if stable_count >= GRASP_VERIFY_STABLE_COUNT:
                    return True
            else:
                stable_count = 0

            if not busy and stable_count == 0:
                break

            self.wait_fn(GRASP_VERIFY_POLL_SEC)

        logger.warn(f"그리퍼 grasp 확인 실패: status={last_status}")
        return False


def read_grasp_confirmation(gripper, logger):
    """Return whether the current gripper state is a valid object grasp."""
    status = gripper.get_status()
    grip_detected = len(status) > 1 and bool(status[1])
    busy = bool(status[0]) if status else False
    width_mm = _read_width_mm(gripper, logger)

    if busy:
        return False, busy, status, width_mm

    if not grip_detected:
        return False, busy, status, width_mm

    if width_mm is None:
        logger.warn(
            "그리퍼 grip detected 신호가 있으나 폭을 확인하지 못해 "
            "grasp 성공으로 처리하지 않습니다."
        )
        return False, busy, status, width_mm

    if width_mm <= GRIPPER_CLOSED_WIDTH_THRESHOLD_MM:
        logger.warn(
            "그리퍼가 완전히 닫힌 상태로 판단되어 grasp 실패로 처리합니다: "
            f"width={width_mm:.1f}mm, "
            f"threshold={GRIPPER_CLOSED_WIDTH_THRESHOLD_MM:.1f}mm"
        )
        return False, busy, status, width_mm

    logger.info(
        "그리퍼 grip detected 신호와 폭으로 grasp 성공을 확인했습니다: "
        f"width={width_mm:.1f}mm"
    )
    return True, busy, status, width_mm


def _read_width_mm(gripper, logger):
    try:
        return float(gripper.get_width())
    except Exception as exc:
        logger.warn(f"그리퍼 폭 읽기 실패: {exc}")
        return None


def cooperative_wait(duration_sec):
    import rclpy

    end_time = time.monotonic() + max(0.0, float(duration_sec))
    while rclpy.ok() and time.monotonic() < end_time:
        remaining = end_time - time.monotonic()
        time.sleep(min(SEQUENCE_WAIT_POLL_SEC, max(0.0, remaining)))
