"""Backward-compatible configuration facade.

New code should prefer importing from role-specific modules in
``macgyvbot.config``. This module re-exports the existing public constants so
older imports keep working during the refactor.
"""

from macgyvbot.config.grasp import (
    GRASP_ADVANCE_DISTANCE_M,
    GRASP_RETRY_LIMIT,
    GRASP_VERIFY_POLL_SEC,
    GRASP_VERIFY_STABLE_COUNT,
    GRASP_VERIFY_TIMEOUT_SEC,
    GRIPPER_CLOSED_WIDTH_THRESHOLD_MM,
)
from macgyvbot.config.hand_grasp import (
    HAND_GRASP_LOCK_ON_STATUS,
    HAND_GRASP_MASK_LOCK_TIMEOUT_SEC,
    HAND_GRASP_ML_CONFIDENCE,
    HAND_GRASP_TIMEOUT_SEC,
)
from macgyvbot.config.handoff import (
    ALLOW_SEARCH_VERTEX_FALLBACK_FOR_HANDOVER,
    HANDOVER_HAND_X_OFFSET_M,
    HANDOVER_HAND_Z_OFFSET_M,
    HANDOVER_REPLAN_MAX_ATTEMPTS,
    HANDOVER_REPLAN_X_STEP_M,
    HANDOVER_SEARCH_TIMEOUT_SEC,
    HAND_POSE_STABLE_TOLERANCE_M,
    HAND_POSE_WAIT_AFTER_DETECTION_SEC,
    OBSERVATION_TIMEOUT_SEC,
)
from macgyvbot.config.models import (
    HAND_GRASP_MODEL_NAME,
    HAND_GRASP_SAM_CHECKPOINT_NAME,
    YOLO_MODEL_NAME,
)
from macgyvbot.config.pick import (
    APPROACH_Z_OFFSET,
    GRASP_Z_OFFSET,
    OBJECT_Z_HEIGHT_BIAS_M,
    SAFE_Z,
)
from macgyvbot.config.return_flow import (
    RETURN_HOME_DESCENT_STEP_M,
    RETURN_HOME_FORCE_THRESHOLD_N,
)
from macgyvbot.config.robot import (
    BASE_FRAME,
    EE_LINK,
    GROUP_NAME,
    HOME_JOINTS,
    OBSERVATION_JOINTS,
    WRIST_JOINT_NAME,
)
from macgyvbot.config.timing import SEQUENCE_WAIT_POLL_SEC
from macgyvbot.config.topics import (
    FORCE_TORQUE_TOPIC,
    HAND_GRASP_IMAGE_TOPIC,
    HAND_GRASP_MASK_LOCK_TOPIC,
    HAND_GRASP_TOPIC,
    ROBOT_STATUS_TOPIC,
    TOOL_COMMAND_TOPIC,
)
from macgyvbot.config.ui import (
    HAND_GRASP_WINDOW_NAME,
    ROBOT_WINDOW_NAME,
)
from macgyvbot.config.vlm import (
    DEFAULT_GRASP_POINT_MODE,
    GRASP_POINT_MODE_CENTER,
    GRASP_POINT_MODE_VLM,
    VLM_GRASP_GRID_SIZES,
)
