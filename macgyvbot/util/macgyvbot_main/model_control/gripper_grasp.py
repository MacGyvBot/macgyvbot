"""Helpers for confirming an OnRobot RG gripper actually holds an object."""

from macgyvbot.config.config import GRIPPER_CLOSED_WIDTH_THRESHOLD_MM


def read_grasp_confirmation(gripper, logger):
    """Return whether the current gripper state is a valid object grasp."""
    status = gripper.get_status()
    grip_detected = len(status) > 1 and bool(status[1])
    busy = bool(status[0]) if status else False
    width_mm = _read_width_mm(gripper, logger)

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
