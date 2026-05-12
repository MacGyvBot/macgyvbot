import numpy as np

from macgyvbot.util.hand_grasp_detection.hand_grasp.ml_grasp_classifier import (
    FEATURE_COUNT,
    extract_ml_features,
)
from macgyvbot.util.hand_grasp_detection.hand_grasp.sam_tool_mask import (
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
