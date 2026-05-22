"""Drawer interaction configuration constants."""

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
DRAWER_PULL_DISTANCE_M = 0.18
DRAWER_OPEN_DIRECTION_X = 1.0
DRAWER_TOOL_PLACE_APPROACH_Z_OFFSET = 0.16
DRAWER_TOOL_PLACE_Z_OFFSET = 0.04
DRAWER_INSIDE_OBSERVATION_Z_OFFSET = 0.05
DRAWER_ORIENTATION_DELTA_WARN_DEG = 10.0
DRAWER_PICK_GRASP_FROM_HANDLE_Z_M = -0.013
DRAWER_PICK_APPROACH_LIFT_M = 0.030
DRAWER_ENTRY_GRIPPER_WIDTH_M = 0.095

DRAWER_PICK_FLOOR1_MIN_GRASP_Z_M: float = 0.245
DRAWER_FLOOR_STEP_Z_M: float = 0.033

TOOL_DRAWER_FLOOR: dict[str, int] = {
    # "screwdriver": 2,  # placeholder — fill in actual assignments
}


def get_drawer_floor_min_grasp_z(floor: int) -> float:
    return DRAWER_PICK_FLOOR1_MIN_GRASP_Z_M + (floor - 1) * DRAWER_FLOOR_STEP_Z_M


def get_tool_drawer_floor(tool_name: str) -> int:
    floor = TOOL_DRAWER_FLOOR.get(tool_name, 1)
    if not isinstance(floor, int) or floor < 1:
        raise ValueError(
            f"TOOL_DRAWER_FLOOR['{tool_name}'] = {floor!r}은 1 이상의 정수여야 합니다."
        )
    return floor