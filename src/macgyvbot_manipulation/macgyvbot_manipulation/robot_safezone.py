"""Safety helpers for robot workspace limits."""

SAFE_X_MIN = 0.0
SAFE_Y_MIN = -0.3
SAFE_Y_MAX = 0.3
SAFE_Z_MIN = 0.24


def clamp_to_safe_workspace(x: float, y: float, z: float, logger):
    """Clamp a target position to the safe workspace."""
    safe_x = x
    safe_y = y
    safe_z = z

    if safe_x < SAFE_X_MIN:
        logger.warning(
            f"Requested x ({safe_x:.3f} m) is below safety limit "
            f"({SAFE_X_MIN:.3f} m). Clamping to SAFE_X_MIN."
        )
        safe_x = SAFE_X_MIN

    if safe_y < SAFE_Y_MIN:
        logger.warning(
            f"Requested y ({safe_y:.3f} m) is below safety limit "
            f"({SAFE_Y_MIN:.3f} m). Clamping to SAFE_Y_MIN."
        )
        safe_y = SAFE_Y_MIN
    elif safe_y > SAFE_Y_MAX:
        logger.warning(
            f"Requested y ({safe_y:.3f} m) is above safety limit "
            f"({SAFE_Y_MAX:.3f} m). Clamping to SAFE_Y_MAX."
        )
        safe_y = SAFE_Y_MAX

    if safe_z < SAFE_Z_MIN:
        logger.warning(
            f"Requested z ({safe_z:.3f} m) is below safety limit "
            f"({SAFE_Z_MIN:.3f} m). Clamping to SAFE_Z_MIN."
        )
        safe_z = SAFE_Z_MIN

    return safe_x, safe_y, safe_z
