"""Drawer interaction configuration constants."""

import math

DRAWER_DETECTION_POLL_SEC = 0.2

DRAWER_YOLO_MODEL_NAME = "yolov11n_drawer.pt"
DRAWER_LABEL = "drawer"
DRAWER_HANDLE_LABEL = "drawer_handle"
DRAWER_DETECTION_TIMEOUT_SEC = 8.0
DRAWER_APPROACH_Z_OFFSET = 0.18
DRAWER_HANDLE_APPROACH_Z_OFFSET = 0.08
DRAWER_HANDLE_GRASP_Z_OFFSET = 0.02
USE_DRAWER_HANDLE_OFFSET_FALLBACK = True
DRAWER_HANDLE_OFFSET_X = 0.0
DRAWER_HANDLE_OFFSET_Y = 0.0
DRAWER_HANDLE_OFFSET_Z = -0.08
DRAWER_TOOL_PLACE_APPROACH_Z_OFFSET = 0.16
DRAWER_TOOL_PLACE_Z_OFFSET = 0.04
DRAWER_PICK_GRASP_FROM_HANDLE_Z_M = -0.013
DRAWER_PICK_APPROACH_LIFT_M = 0.030
DRAWER_ENTRY_GRIPPER_WIDTH_M = 0.08
DRAWER_SAFE_Z_MIN_BASE: float = 0.245
DRAWER_FLOOR_STEP_Z_M: float = 0.033

DRAWER_JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

DRAWER_HANDLE_JOINT_DEGREES: dict[int, list[float]] = {
    1: [0.86, 18.77, 83.32, 20.08, 77.24, -2.24],
    2: [0.86, 21.67, 90.68, 21.22, 67.64, -6.05],
}

TOOL_DRAWER_IDS: dict[str, int] = {
    "screwdriver": 1,
    "pliers": 2,
}

DRAWER_OPEN_OFFSET_XYZ_M: list[float] = [0.0, -0.12, 0.0]
DRAWER_OBSERVE_OFFSET_XYZ_M: list[float] = [0.0, -0.06, 0.20]


def get_drawer_safe_z_min(drawer_id: int) -> float:
    return DRAWER_SAFE_Z_MIN_BASE + (drawer_id - 1) * DRAWER_FLOOR_STEP_Z_M


def get_tool_drawer_id(tool_name: str) -> int:
    drawer_id = TOOL_DRAWER_IDS.get(tool_name)
    if drawer_id is None:
        raise ValueError(
            f"TOOL_DRAWER_IDS에 '{tool_name}'이 없습니다. "
            f"등록된 공구: {list(TOOL_DRAWER_IDS.keys())}"
        )
    return drawer_id


def get_drawer_handle_joints(drawer_id: int) -> dict:
    degrees = DRAWER_HANDLE_JOINT_DEGREES.get(drawer_id)
    if degrees is None:
        raise ValueError(
            f"DRAWER_HANDLE_JOINT_DEGREES에 서랍 ID {drawer_id}가 없습니다. "
            f"등록된 ID: {list(DRAWER_HANDLE_JOINT_DEGREES.keys())}"
        )
    expected = len(DRAWER_JOINT_NAMES)
    if len(degrees) != expected:
        raise ValueError(
            f"서랍 ID {drawer_id}의 joint 각도가 {len(degrees)}개입니다 (기대값: {expected})."
        )
    return {
        name: math.radians(deg)
        for name, deg in zip(DRAWER_JOINT_NAMES, degrees)
    }
