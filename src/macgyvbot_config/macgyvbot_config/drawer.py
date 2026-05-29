"""Drawer handle poses, offsets, and tool mapping."""

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

DRAWER_OPEN_OFFSET_XYZ_M = [0.0, -0.14, 0.0]
DRAWER_OBSERVE_OFFSET_XYZ_M = [0.0, 0.06, 0.20]
DRAWER_GRIPPER_SETTLE_SEC = 0.8
DRAWER_1_SAFE_Z_OFFSET_M = 0.08
DRAWER_CLOSE_LIFT_OFFSET_M = 0.01
DRAWER_OBSERVATION_J6_DEG = 0.0
