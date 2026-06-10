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
    2: [24.84, 15.75, 74.14, -12.17, 65.79, 119.95],
    1: [24.68, 17.01, 85.48, -12.82, 53.29, 123.09],
    0: [24.86, 20.51, 94.78, -17.16, 40.44, 126.99],
}

TOOL_DRAWER_IDS = {
    "wrench": 2,
    "screwdriver": 1,
    "pliers": 0,
}

ENABLE_DRAWER_COLLISION_SCENE_DEFAULT = True

DRAWER_ARUCO_MARKER_IDS = {
    2: 12,
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

# Back off from the detected or staged tool along base X for camera observation.
TOOL_OBSERVE_X_BACKOFF_M = 0.08
# Start force-based staging descent slightly above the global minimum safe Z.
DRAWER_STORE_FORCE_DESCENT_START_Z_OFFSET_M = 0.03

DRAWER_OPEN_OFFSET_XYZ_M = [-0.185, 0.0, 0.0]
DRAWER_OBSERVE_OFFSET_XYZ_M = [0.10, 0.0, 0.15]
DRAWER_HANDLE_PREAPPROACH_X_OFFSET_M = -0.02
DRAWER_GRIPPER_SETTLE_SEC = 0.8
# Drawer floor heights are modeled as offsets above the global minimum safe Z.
DRAWER_1_SAFE_Z_OFFSET_M = 0.087
DRAWER_2_SAFE_Z_OFFSET_M = 0.169
# Clearance above each drawer floor used to cross drawer walls or lips.
DRAWER_WALL_CLEARANCE_Z_OFFSET_M = 0.10
DRAWER_STORE_MARKER_EXIT_OFFSET_XYZ_M = [-0.15, 0.0, 0.0]
# Marker placement offsets are relative to the detected drawer marker Z.
DRAWER_STORE_MARKER_APPROACH_Z_OFFSET_M = 0.09
DRAWER_STORE_MARKER_RELEASE_Z_OFFSET_M = 0.02
DRAWER_CLOSE_LIFT_OFFSET_M = 0.01
DRAWER_OBSERVATION_J6_DEG = 90.0

# Drawer collision boundaries in the robot base frame. Each measured bottom
# rectangle is in meters and attached to the floor at z=0.
DRAWER_COLLISION_FRAME_ID = "base_link"
DRAWER_COLLISION_APPLY_SERVICE = "/apply_planning_scene"
DRAWER_COLLISION_SCENE_TOPICS = [
    "/planning_scene",
    "/moveit_cpp/monitored_planning_scene",
]
DRAWER_COLLISION_SERVICE_WAIT_TIMEOUT_SEC = 0.2
DRAWER_COLLISION_PROFILE_DRAWER_ONLY = "drawer_only"
DRAWER_COLLISION_PROFILE_DRAWER_OPENED = "drawer_opened"
DRAWER_COLLISION_DEFAULT_PROFILE = DRAWER_COLLISION_PROFILE_DRAWER_ONLY

DRAWER_BODY_COLLISION_BOX = {
    "id": "drawer_body_boundary",
    "frame_id": DRAWER_COLLISION_FRAME_ID,
    "center_xyz": [0.7300, 0.2150, 0.1265],
    "size_xyz": [0.2400, 0.1700, 0.2530],
    "color_rgba": [0.12, 0.45, 0.85, 0.35],
}
DRAWER_OPENED_COLLISION_BOX = {
    "id": "drawer_opened_boundary",
    "frame_id": DRAWER_COLLISION_FRAME_ID,
    "center_xyz": [0.5200, 0.2150, 0.1265],
    "size_xyz": [0.1800, 0.1700, 0.2530],
    "color_rgba": [0.12, 0.45, 0.85, 0.35],
}
DRAWER_COLLISION_BOX_PROFILES = {
    DRAWER_COLLISION_PROFILE_DRAWER_ONLY: [
        DRAWER_BODY_COLLISION_BOX,
    ],
    DRAWER_COLLISION_PROFILE_DRAWER_OPENED: [
        DRAWER_BODY_COLLISION_BOX,
        DRAWER_OPENED_COLLISION_BOX,
    ],
}
DRAWER_COLLISION_BOXES = DRAWER_COLLISION_BOX_PROFILES[
    DRAWER_COLLISION_DEFAULT_PROFILE
]
DRAWER_COLLISION_SCENE_KEY_PROFILES = {
    # on pick
    #"handoff/move_to_user": DRAWER_COLLISION_PROFILE_DRAWER_OPENED,
    #"drawer/approach_to_close": DRAWER_COLLISION_PROFILE_DRAWER_OPENED,

    # on return
    # considered not needed. but it is ready to be applied.
}
