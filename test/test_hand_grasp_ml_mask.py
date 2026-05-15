import numpy as np

from macgyvbot.perception.hand_grasp.calculations import (
    build_depth_grasp_info,
)
from macgyvbot.perception.hand_grasp.ml_grasp_classifier import (
    FEATURE_COUNT,
    extract_ml_features,
)
from macgyvbot.perception.hand_grasp.sam_tool_mask import (
    LockedToolMask,
    compute_mask_contact,
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
