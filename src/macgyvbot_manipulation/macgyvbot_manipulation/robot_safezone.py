"""Safety helpers for robot workspace limits."""

from macgyvbot_config.drawer import DRAWER_1_SAFE_Z_OFFSET_M

SAFE_X_MIN = 0.0
SAFE_Y_MIN = -0.3
SAFE_Y_MAX = 0.3
SAFE_Z_MIN = 0.24


def safe_z_min_for_drawer(drawer_id):
    """Return the minimum safe Z for a drawer-specific motion context."""
    if drawer_id == 1:
        return SAFE_Z_MIN + DRAWER_1_SAFE_Z_OFFSET_M
    return SAFE_Z_MIN


def clamp_to_safe_workspace(x: float, y: float, z: float, logger, min_z=None):
    """Clamp a target position to the safe workspace."""
    safe_x = x
    safe_y = y
    safe_z = z
    z_min = SAFE_Z_MIN if min_z is None else float(min_z)

    if safe_x < SAFE_X_MIN:
        logger.warning(
            f"요청 x ({safe_x:.3f} m)가 안전 하한 "
            f"({SAFE_X_MIN:.3f} m)보다 낮아 SAFE_X_MIN으로 제한합니다."
        )
        safe_x = SAFE_X_MIN

    if safe_y < SAFE_Y_MIN:
        logger.warning(
            f"요청 y ({safe_y:.3f} m)가 안전 하한 "
            f"({SAFE_Y_MIN:.3f} m)보다 낮아 SAFE_Y_MIN으로 제한합니다."
        )
        safe_y = SAFE_Y_MIN
    elif safe_y > SAFE_Y_MAX:
        logger.warning(
            f"요청 y ({safe_y:.3f} m)가 안전 상한 "
            f"({SAFE_Y_MAX:.3f} m)보다 높아 SAFE_Y_MAX로 제한합니다."
        )
        safe_y = SAFE_Y_MAX

    if safe_z < z_min:
        logger.warning(
            f"요청 z ({safe_z:.3f} m)가 안전 하한 "
            f"({z_min:.3f} m)보다 낮아 최소 안전 z로 제한합니다."
        )
        safe_z = z_min

    return safe_x, safe_y, safe_z
