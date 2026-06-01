"""Drawer handle poses, offsets, marker targets, and tool mapping."""

import math

DRAWER_JOINT_NAMES = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
]

DRAWER_HANDLE_JOINT_DEGREES = {
    1: [0.86, 18.77, 83.32, 20.08, 77.24, -2.24],
    0: [0.86, 21.67, 90.68, 21.22, 67.64, -6.05],
}

TOOL_DRAWER_IDS = {
    "screwdriver": 1,
    "pliers": 0,
}

DRAWER_ARUCO_MARKER_IDS = {
    1: 11,
    0: 10,
}

DRAWER_STORE_TOOL_OBSERVE_POINT = {
    "joint_1": math.radians(-22.82),
    "joint_2": math.radians(3.95),
    "joint_3": math.radians(85.9),
    "joint_4": math.radians(-0.02),
    "joint_5": math.radians(90.16),
    "joint_6": math.radians(67.19),
}
DRAWER_STORE_FORCE_DESCENT_START_Z_OFFSET_M = 0.03

DRAWER_OPEN_OFFSET_XYZ_M = [0.0, -0.14, 0.0]
DRAWER_OBSERVE_OFFSET_XYZ_M = [0.0, 0.06, 0.20]
DRAWER_GRIPPER_SETTLE_SEC = 0.8
DRAWER_1_SAFE_Z_OFFSET_M = 0.082
DRAWER_STORE_MARKER_CLEARANCE_Z_OFFSET_M = 0.10
DRAWER_STORE_MARKER_EXIT_OFFSET_XYZ_M = [0.0, -0.15, 0.0]
DRAWER_STORE_MARKER_APPROACH_Z_OFFSET_M = 0.08
DRAWER_STORE_MARKER_RELEASE_Z_OFFSET_M = 0.02
DRAWER_CLOSE_LIFT_OFFSET_M = 0.01
DRAWER_OBSERVATION_J6_DEG = 0.0

# Initial drawer collision boundaries in the robot base frame. These are
# intentionally conservative placeholders and should be calibrated in RViz.
DRAWER_COLLISION_FRAME_ID = "base_link"
DRAWER_COLLISION_APPLY_SERVICE = "/apply_planning_scene"
DRAWER_COLLISION_SCENE_TOPICS = [
    "/planning_scene",
    "/moveit_cpp/monitored_planning_scene",
]
DRAWER_COLLISION_BOXES = [
    {
        "id": "drawer_cabinet_boundary",
        "frame_id": DRAWER_COLLISION_FRAME_ID,
        "center_xyz": [0.55, -0.30, 0.36],
        "size_xyz": [0.58, 0.34, 0.42],
        "color_rgba": [0.12, 0.45, 0.85, 0.35],
    },
    {
        "id": "drawer_front_boundary",
        "frame_id": DRAWER_COLLISION_FRAME_ID,
        "center_xyz": [0.55, -0.48, 0.35],
        "size_xyz": [0.58, 0.05, 0.30],
        "color_rgba": [0.95, 0.55, 0.15, 0.45],
    },
    {
        "id": "drawer_inner_keepout",
        "frame_id": DRAWER_COLLISION_FRAME_ID,
        "center_xyz": [0.55, -0.24, 0.35],
        "size_xyz": [0.46, 0.20, 0.22],
        "color_rgba": [0.85, 0.15, 0.12, 0.25],
    },
]
