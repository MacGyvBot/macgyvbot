"""Robot frame, link, and joint-pose configuration."""

import math

GROUP_NAME = "manipulator"
BASE_FRAME = "base_link"
WORLD_FRAME = "world"
EE_LINK = "link_6"
WRIST_JOINT_NAME = "joint_6"
WRIST_JOINT_LOWER_LIMIT_RAD = -2.0 * math.pi
WRIST_JOINT_UPPER_LIMIT_RAD = 2.0 * math.pi

SAFE_X_MIN = 0.0
SAFE_Y_MIN = -0.3
SAFE_Y_MAX = 0.3
SAFE_Z_MIN = 0.24

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

DEFAULT_TRAJECTORY_ACTION_NAME = "/dsr_moveit_controller/follow_joint_trajectory"
POSE_GOAL_IK_TIMEOUT_SEC = 0.1
POSE_GOAL_IK_MAX_SEEDS = 10
POSE_GOAL_IK_SEED_PERTURB_RAD = math.radians(10.0)
POSE_GOAL_MAX_JOINT_DELTA_RAD = math.radians(120.0)
HOME_JOINT_GOAL_TOLERANCE_RAD = math.radians(2.0)
HOME_JOINT_SETTLE_TIMEOUT_SEC = 1.5
HOME_JOINT_POSITION_CONFIRM_TIMEOUT_SEC = 30.0
MOVEIT_EXECUTION_POLL_INTERVAL_SEC = 0.02
MOVEIT_ACTION_SERVER_WAIT_TIMEOUT_SEC = 1.0
MOVEIT_SEND_GOAL_TIMEOUT_SEC = 1.0
MOVEIT_GOAL_ACCEPTANCE_TIMEOUT_SEC = 5.0
MOVEIT_JOINT_GOAL_SETTLE_TIMEOUT_SEC = 0.5
MOVEIT_JOINT_GOAL_TOLERANCE_RAD = 0.02
