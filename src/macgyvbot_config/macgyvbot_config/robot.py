"""Robot frame, link, and joint-pose configuration."""

import math

GROUP_NAME = "manipulator"
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
