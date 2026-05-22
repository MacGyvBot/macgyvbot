"""Robot frame, link, and joint-pose configuration."""

import math

GROUP_NAME = "manipulator"

SAFE_X_MIN = 0.0
SAFE_Y_MIN = -0.3
SAFE_Y_MAX = 0.3
SAFE_Z_MIN = 0.24
BASE_FRAME = "base_link"
EE_LINK = "link_6"
WRIST_JOINT_NAME = "joint_6"

HOME_JOINTS = {
    "joint_1": math.radians(0.0),
    "joint_2": math.radians(0.0),
    "joint_3": math.radians(90.0),
    "joint_4": math.radians(0.0),
    "joint_5": math.radians(90.0),
    "joint_6": math.radians(90.0),
}

OBSERVATION_JOINTS = {
    "joint_1": math.radians(0.0),
    "joint_2": math.radians(-40.0),
    "joint_3": math.radians(55.0),
    "joint_4": math.radians(0.0),
    "joint_5": math.radians(120.0),
    "joint_6": math.radians(90.0),
}

# TODO: DART Studio에서 서랍 손잡이(닫힌 상태) 위치로 조그 후 관절값 입력
DRAWER_CLOSED_JOINTS = {
    "joint_1": math.radians(4.02),
    "joint_2": math.radians(47.76),
    "joint_3": math.radians(47.17),
    "joint_4": math.radians(-3.79),
    "joint_5": math.radians(87.74),
    "joint_6": math.radians(1.34),
}

DRAWER_OPEN_JOINTS = {
    "joint_1": math.radians(-8.39),
    "joint_2": math.radians(50.33),
    "joint_3": math.radians(44.20),
    "joint_4": math.radians(-5.51),
    "joint_5": math.radians(88.91),
    "joint_6": math.radians(0.12),
}

# Optional taught waypoint sequence for opening the drawer without twisting the
# handle. Add intermediate joint dictionaries here after hardware teaching if a
# direct DRAWER_CLOSED_JOINTS -> DRAWER_OPEN_JOINTS move rotates the gripper.
DRAWER_OPEN_JOINT_SEQUENCE = [
    ("DRAWER_OPEN_JOINTS", DRAWER_OPEN_JOINTS),
]

# Optional hardware-taught view pose for looking into the opened drawer.
# Leave as None until measured; drawer flow falls back to a conservative
# current-open-handle FK + Z offset observation pose for software validation.
DRAWER_INSIDE_OBSERVATION_JOINTS = {
    "joint_1": math.radians(-1.62),
    "joint_2": math.radians(31.59),
    "joint_3": math.radians(52.82),
    "joint_4": math.radians(-1.21),
    "joint_5": math.radians(97.25),
    "joint_6": math.radians(87.13),
}


# Per-floor-step joint deltas (degrees, 0-indexed).
# floor N joints = base + (N-1) * step_delta.  Measure on hardware and fill in.
DRAWER_FLOOR_STEP_CLOSED_JOINT_DELTAS: dict[int, float] = {}
DRAWER_FLOOR_STEP_OPEN_JOINT_DELTAS: dict[int, float] = {}
DRAWER_FLOOR_STEP_OBSERVATION_JOINT_DELTAS: dict[int, float] = {}


def apply_joint_deltas(base_joints: dict, step_deltas: dict, scale: int = 1) -> dict:
    """Return base_joints + scale * step_deltas (keys are 0-indexed, values in degrees)."""
    if scale == 0 or not step_deltas:
        return dict(base_joints)
    n = len(base_joints)
    invalid = [idx for idx in step_deltas if not isinstance(idx, int) or idx < 0 or idx >= n]
    if invalid:
        raise ValueError(
            f"joint delta 인덱스가 유효 범위(0–{n - 1})를 벗어납니다: {invalid}"
        )
    sorted_keys = sorted(base_joints.keys(), key=lambda k: int(k.rsplit("_", 1)[-1]) if k.rsplit("_", 1)[-1].isdigit() else k)
    joints = dict(base_joints)
    for idx, delta_deg in step_deltas.items():
        key = sorted_keys[idx]
        joints[key] = joints[key] + math.radians(delta_deg * scale)
    return joints
