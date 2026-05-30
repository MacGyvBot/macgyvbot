import numpy as np

from macgyvbot_domain.mask_models import LockedToolMask
from macgyvbot_perception.hand_tool_grasp.calculations import build_depth_grasp_info
from macgyvbot_perception.hand_tool_grasp.ml_grasp_classifier import (
    FEATURE_COUNT,
    extract_ml_features,
)
from macgyvbot_perception.hand_tool_grasp.sam_tool_mask import (
    compute_mask_contact,
    validate_tracked_mask,
)
from macgyvbot_perception.hand_tool_grasp.visualization import (
    close_roi_depth_status,
)


def test_extract_ml_features_returns_normalized_xyz_triplets():
    landmarks = {index: (index * 2, index) for index in range(21)}
    features = extract_ml_features({"landmarks": landmarks})

    assert len(features) == FEATURE_COUNT
    assert features[0:3] == [0.0, 0.0, 0.0]
    assert features[3] > 0.0
    assert features[4] > 0.0


def test_compute_mask_contact_confirms_landmark_inside_locked_mask():
    mask = np.zeros((20, 20), dtype=bool)
    mask[5:10, 5:10] = True
    locked_tool = LockedToolMask(
        roi=(5, 5, 10, 10),
        mask=mask,
        source="SAM_LOCKED",
    )
    hand_info = {
        "landmarks": {
            0: (6, 6),
            1: (30, 30),
        }
    }

    result = compute_mask_contact(
        hand_info=hand_info,
        locked_tool=locked_tool,
        contact_radius=0,
        min_contact_landmarks=1,
        proximity_threshold=5.0,
    )

    assert result.mask_contact_count == 1
    assert result.mask_contact_confirmed is True
    assert result.near_or_contact is True


def test_build_depth_grasp_info_confirms_similar_hand_tool_depth():
    depth_mm = np.full((20, 20), 500.0, dtype=np.float32)
    depth_mm[6, 6] = 512.0
    depth_mm[7, 7] = 515.0
    hand_info = {
        "landmarks": {
            0: (6, 6),
            1: (7, 7),
            2: (18, 18),
        }
    }

    result = build_depth_grasp_info(
        hand_info=hand_info,
        tool_roi=(5, 5, 10, 10),
        depth_mm=depth_mm,
        depth_diff_threshold_mm=35.0,
        min_depth_contact_landmarks=2,
        roi_margin=0,
    )

    assert result["depth_available"] is True
    assert result["depth_contact_count"] == 2
    assert result["depth_grasp_confirmed"] is True


def test_validate_tracked_mask_accepts_stable_mask():
    previous_mask = np.zeros((40, 40), dtype=bool)
    previous_mask[10:20, 10:20] = True
    tracked_mask = np.zeros((40, 40), dtype=bool)
    tracked_mask[11:21, 12:22] = True

    result = validate_tracked_mask(
        previous=LockedToolMask((10, 10, 20, 20), previous_mask, "SAM_TRACKED"),
        tracked=LockedToolMask((12, 11, 22, 21), tracked_mask, "SAM_TRACKED"),
        max_center_shift_px=8.0,
        min_area_ratio=0.5,
        max_area_ratio=2.0,
    )

    assert result.accepted is True


def test_validate_tracked_mask_rejects_center_jump():
    previous_mask = np.zeros((60, 60), dtype=bool)
    previous_mask[10:20, 10:20] = True
    tracked_mask = np.zeros((60, 60), dtype=bool)
    tracked_mask[40:50, 40:50] = True

    result = validate_tracked_mask(
        previous=LockedToolMask((10, 10, 20, 20), previous_mask, "SAM_TRACKED"),
        tracked=LockedToolMask((40, 40, 50, 50), tracked_mask, "SAM_TRACKED"),
        max_center_shift_px=8.0,
        min_area_ratio=0.5,
        max_area_ratio=2.0,
    )

    assert result.accepted is False
    assert result.reason == "tracked_mask_center_jump"


def test_validate_tracked_mask_rejects_area_jump():
    previous_mask = np.zeros((60, 60), dtype=bool)
    previous_mask[10:20, 10:20] = True
    tracked_mask = np.zeros((60, 60), dtype=bool)
    tracked_mask[8:28, 8:28] = True

    result = validate_tracked_mask(
        previous=LockedToolMask((10, 10, 20, 20), previous_mask, "SAM_TRACKED"),
        tracked=LockedToolMask((8, 8, 28, 28), tracked_mask, "SAM_TRACKED"),
        max_center_shift_px=20.0,
        min_area_ratio=0.5,
        max_area_ratio=2.0,
    )

    assert result.accepted is False
    assert result.reason == "tracked_mask_too_large"


def test_close_roi_depth_status_reports_depth_range():
    close_roi = (10, 10, 30, 30)
    tool_roi = (15, 15, 25, 25)
    depth_mm = np.full((40, 40), 200.0, dtype=np.float32)

    result = close_roi_depth_status(close_roi, tool_roi, depth_mm)

    assert result["color"] == (0, 255, 0)
    assert "depth ok" in result["message"]


def test_close_roi_depth_status_reports_tool_too_far():
    close_roi = (10, 10, 30, 30)
    tool_roi = (15, 15, 25, 25)
    depth_mm = np.full((40, 40), 320.0, dtype=np.float32)

    result = close_roi_depth_status(close_roi, tool_roi, depth_mm)

    assert result["color"] == (0, 0, 255)
    assert "too far" in result["message"]


def test_close_roi_depth_status_waits_for_tool_inside_roi():
    close_roi = (10, 10, 30, 30)
    tool_roi = (31, 15, 39, 25)
    depth_mm = np.full((40, 40), 200.0, dtype=np.float32)

    result = close_roi_depth_status(close_roi, tool_roi, depth_mm)

    assert result["color"] == (0, 220, 255)
    assert "move tool into ROI" in result["message"]
