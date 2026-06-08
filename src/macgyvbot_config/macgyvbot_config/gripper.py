from itertools import combinations

ENABLE_GRIPPER_SELF_COLLISION_ACM_DEFAULT = True

RG2_INTERNAL_COLLISION_LINKS = (
    "quick_changer",
    "angle_bracket",
    "gripper_body",
    "left_inner_knuckle",
    "right_outer_knuckle",
    "left_outer_knuckle",
    "left_inner_finger",
    "right_inner_finger",
    "right_inner_knuckle",
)

RG2_MOUNT_ALLOWED_COLLISION_PAIRS = (
    ("link_6", "quick_changer"),
)

RG2_ALLOWED_COLLISION_PAIRS = tuple(
    combinations(RG2_INTERNAL_COLLISION_LINKS, 2)
) + RG2_MOUNT_ALLOWED_COLLISION_PAIRS

DEFAULT_GET_PLANNING_SCENE_SERVICE = "/get_planning_scene"
DEFAULT_APPLY_PLANNING_SCENE_SERVICE = "/apply_planning_scene"
DEFAULT_PLANNING_SCENE_TOPICS = (
    "/planning_scene",
    "/moveit_cpp/monitored_planning_scene",
)

DEFAULT_SCENE_SETTLE_SEC = 0.1
DEFAULT_PLANNING_SCENE_SERVICE_TIMEOUT_SEC = 3.0
