"""Shared MacGyvBot runtime configuration."""

import math

GROUP_NAME = "manipulator"
BASE_FRAME = "base_link"
EE_LINK = "link_6"

HOME_JOINTS = {
    "joint_1": math.radians(0.0),
    "joint_2": math.radians(0.0),
    "joint_3": math.radians(90.0),
    "joint_4": math.radians(0.0),
    "joint_5": math.radians(90.0),
    "joint_6": math.radians(90.0),
}
WRIST_JOINT_NAME = "joint_6"

SAFE_Z = 0.40
APPROACH_Z_OFFSET = 0.18
GRASP_Z_OFFSET = 0.3
COLLISION_MARGIN = 0.02
MIN_GRASP_CLEARANCE = 0.02
MIN_TRAVEL_Z = 0.06
MIN_PICK_Z = 0.30
MAX_DESCENT_FROM_APPROACH = 0.08

YOLO_MODEL_NAME = "yolov11_best.pt"
TOOL_COMMAND_TOPIC = "/tool_command"
ROBOT_STATUS_TOPIC = "/robot_task_status"
HAND_GRASP_TOPIC = "/human_grasped_tool"
HAND_GRASP_IMAGE_TOPIC = "/hand_grasp_detection/annotated_image"
HAND_GRASP_TIMEOUT_SEC = 20.0
GRASP_VERIFY_TIMEOUT_SEC = 3.0
GRASP_VERIFY_POLL_SEC = 0.2
GRASP_RETRY_LIMIT = 5
SEQUENCE_WAIT_POLL_SEC = 0.03
ROBOT_WINDOW_NAME = "YOLO Robot Pick"
HAND_GRASP_WINDOW_NAME = "Hand Grasp Detection"

RETURN_STAGING_POSE = {
    "x": 0.35,
    "y": 0.00,
    "z": SAFE_Z,
}
RETURN_PLACE_Z = 0.32
RETURN_APPROACH_Z_OFFSET = 0.12

TOOL_HOME_POSES = {
    "drill": {"x": 0.45, "y": -0.20, "z": RETURN_PLACE_Z},
    "hammer": {"x": 0.45, "y": -0.10, "z": RETURN_PLACE_Z},
    "pliers": {"x": 0.45, "y": 0.00, "z": RETURN_PLACE_Z},
    "screwdriver": {"x": 0.45, "y": 0.10, "z": RETURN_PLACE_Z},
    "tape_measure": {"x": 0.45, "y": 0.20, "z": RETURN_PLACE_Z},
    "wrench": {"x": 0.45, "y": 0.30, "z": RETURN_PLACE_Z},
}

GRASP_POINT_MODE_CENTER = "center"
GRASP_POINT_MODE_VLM = "vlm"
DEFAULT_GRASP_POINT_MODE = GRASP_POINT_MODE_CENTER
VLM_GRASP_GRID_SIZES = ((3, 3), (4, 4))
