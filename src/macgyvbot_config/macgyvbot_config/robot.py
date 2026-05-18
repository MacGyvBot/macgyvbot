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
