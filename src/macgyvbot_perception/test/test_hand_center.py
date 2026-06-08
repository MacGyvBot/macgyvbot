import numpy as np

from macgyvbot_perception.hand_tool_grasp.hand_center import (
    extract_hand_center_pixel,
)


def test_extract_hand_center_pixel_uses_palm_center_without_tool_occlusion():
    hand = {
        "palm_center": (10, 12),
        "landmarks": {
            0: (8, 10),
            5: (10, 12),
            17: (12, 14),
        },
    }

    result = extract_hand_center_pixel(hand, [hand])

    assert result == {
        "u": 10,
        "v": 12,
        "source": "palm_center",
    }


def test_extract_hand_center_pixel_excludes_tool_occluded_landmarks():
    tool_mask = np.zeros((50, 50), dtype=bool)
    tool_mask[0:21, 0:21] = True
    hand = {
        "palm_center": (10, 10),
        "landmarks": {
            0: (8, 10),
            5: (10, 10),
            17: (12, 10),
            8: (40, 20),
            12: (42, 22),
            16: (44, 24),
        },
    }

    result = extract_hand_center_pixel(
        hand,
        [hand],
        tool_mask=tool_mask,
    )

    assert result == {
        "u": 42,
        "v": 22,
        "source": "visible_landmark_centroid",
    }


def test_extract_hand_center_pixel_keeps_palm_when_tool_mask_does_not_occlude_hand():
    tool_mask = np.zeros((50, 50), dtype=bool)
    tool_mask[45:50, 45:50] = True
    hand = {
        "palm_center": (10, 10),
        "landmarks": {
            0: (8, 10),
            5: (10, 10),
            17: (12, 10),
            8: (40, 20),
            12: (42, 22),
            16: (44, 24),
        },
    }

    result = extract_hand_center_pixel(
        hand,
        [hand],
        tool_mask=tool_mask,
    )

    assert result == {
        "u": 10,
        "v": 10,
        "source": "palm_center",
    }
