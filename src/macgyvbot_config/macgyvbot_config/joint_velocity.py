"""User-tunable robot joint velocity configuration."""

from __future__ import annotations

import math

JOINT_VELOCITY_LIMITS_RAD_PER_SEC = {
    "joint_1": math.radians(30.0),
    "joint_2": math.radians(30.0),
    "joint_3": math.radians(36.0),
    "joint_4": math.radians(45.0),
    "joint_5": math.radians(45.0),
    "joint_6": math.radians(90.0),
}

MOTION_VELOCITY_SCALING_FACTOR = 1.0
