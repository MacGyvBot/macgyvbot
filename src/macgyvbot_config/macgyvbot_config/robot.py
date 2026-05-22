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
    "joint_1": math.radians(0.86),
    "joint_2": math.radians(21.67),
    "joint_3": math.radians(90.70),
    "joint_4": math.radians(21.2),
    "joint_5": math.radians(67.65),
    "joint_6": math.radians(-6.05),
}

# 층간 이동 delta (도 단위, 0-indexed). closed 기준으로 모든 pose에 동일하게 적용.
# floor-N closed      = DRAWER_CLOSED_JOINTS + (N-1) * DRAWER_FLOOR_STEP_JOINT_DELTAS
# floor-N open        = floor-N-closed + DRAWER_CLOSED_TO_OPEN_JOINT_DELTAS
# floor-N observation = floor-N-closed + DRAWER_CLOSED_TO_OBSERVATION_JOINT_DELTAS
DRAWER_FLOOR_STEP_JOINT_DELTAS: dict[int, float] = {
    0: 0.0,
    1: 2.92,
    2: -7.5,
    3: 0.0,
    4: 10.0,
    5: 4.0,
}

# closed → open 전환 delta (도 단위, 0-indexed). 하드웨어 티칭 후 채울 것.
DRAWER_CLOSED_TO_OPEN_JOINT_DELTAS: dict[int, float] = {
    0: 0.0,
    1: 0.0,
    2: 0.0,
    3: 21.4,
    4: 0.0,
    5: 0.0,
}

# closed → 서랍 내부 관찰 pose 전환 delta (도 단위, 0-indexed). 하드웨어 티칭 후 채울 것.
DRAWER_CLOSED_TO_OBSERVATION_JOINT_DELTAS: dict[int, float] = {
    0: 6.77,
    1: -18.74,
    2: 8.62,
    3: -16.9,
    4: 8.34,
    5: 87.0,
}


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
