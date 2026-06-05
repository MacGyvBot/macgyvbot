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
DRAWER_2_SAFE_Z_OFFSET_M = DRAWER_1_SAFE_Z_OFFSET_M * 2.0
# Clearance above each drawer floor used to cross drawer walls or lips.
DRAWER_WALL_CLEARANCE_Z_OFFSET_M = 0.10
DRAWER_STORE_MARKER_EXIT_OFFSET_XYZ_M = [-0.15, 0.0, 0.0]
# Marker placement offsets are relative to the detected drawer marker Z.
DRAWER_STORE_MARKER_APPROACH_Z_OFFSET_M = 0.09
DRAWER_STORE_MARKER_RELEASE_Z_OFFSET_M = 0.02
DRAWER_CLOSE_LIFT_OFFSET_M = 0.01
DRAWER_OBSERVATION_J6_DEG = 90.0
