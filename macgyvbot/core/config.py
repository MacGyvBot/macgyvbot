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
HAND_GRASP_TOPIC = "/human_grasped_tool"
HAND_GRASP_IMAGE_TOPIC = "/hand_grasp_detection/annotated_image"
HAND_GRASP_TIMEOUT_SEC = 20.0
GRASPNET_POSE_TOPIC = "/graspnet/target_pose"
GRASPNET_POSE_TIMEOUT_SEC = 1.0
GRASPNET_WAIT_TIMEOUT_SEC = 2.0
GRASPNET_TARGET_DISTANCE_TOLERANCE_M = 0.12
ROBOT_WINDOW_NAME = "YOLO Robot Pick"
HAND_GRASP_WINDOW_NAME = "Hand Grasp Detection"

GRASP_POINT_MODE_CENTER = "center"
GRASP_POINT_MODE_VLM = "vlm"
GRASP_POINT_MODE_GRASPNET = "graspnet"
DEFAULT_GRASP_POINT_MODE = GRASP_POINT_MODE_CENTER
VLM_GRASP_GRID_SIZES = ((3, 3), (4, 4))
