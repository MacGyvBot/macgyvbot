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
GRASP_VERIFY_STABLE_COUNT = 3
GRASP_RETRY_LIMIT = 5
GRASP_ADVANCE_DISTANCE_M = 0.20
GRIPPER_CLOSED_WIDTH_THRESHOLD_MM = 5.0
FORCE_TORQUE_TOPIC = "/force_torque_sensor_broadcaster/wrench"
FORCE_THRES_2_PAUSE = 5.0 #추가됨 - 충돌 판정 임계 힘 크기
RETURN_HOME_DESCENT_START_Z = 0.30
RETURN_HOME_DESCENT_STEP_M = 0.01
RETURN_HOME_FORCE_THRESHOLD_N = 8.0
SEQUENCE_WAIT_POLL_SEC = 0.03
ROBOT_WINDOW_NAME = "YOLO Robot Pick"
HAND_GRASP_WINDOW_NAME = "Hand Grasp Detection"

GRASP_POINT_MODE_CENTER = "center"
GRASP_POINT_MODE_VLM = "vlm"
DEFAULT_GRASP_POINT_MODE = GRASP_POINT_MODE_CENTER
VLM_GRASP_GRID_SIZES = ((3, 3), (4, 4))
