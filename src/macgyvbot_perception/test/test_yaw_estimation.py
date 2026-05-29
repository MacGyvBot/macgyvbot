import math

import cv2
import numpy as np

from macgyvbot_perception.grasp_point.center_method.selector import (
    CenterGraspPointSelector,
)
from macgyvbot_perception.grasp_point.grasp_method.yaw_estimation import (
    aggregate_masks_across_frames,
    apply_pca_yaw_to_grasp_result,
    estimate_sam_mask_for_crop,
    estimate_yaw_from_mask,
    normalize_parallel_gripper_yaw,
)


class FakeSamPredictor:
    def __init__(self, masks, scores=None):
        self._masks = masks
        self._scores = scores or [1.0] * len(masks)
        self.last_image = None

    def set_image(self, image):
        self.last_image = image

    def predict(self, point_coords=None, point_labels=None, box=None, multimask_output=True):
        return np.asarray(self._masks), np.asarray(self._scores), None


class FakeLogger:
    def info(self, message):
        return None

    def warn(self, message):
        return None


def _rotated_rect_mask(size, center, rect_size, angle_deg):
    mask = np.zeros(size, dtype=np.uint8)
    rect = (center, rect_size, angle_deg)
    box = cv2.boxPoints(rect).astype(np.int32)
    cv2.fillConvexPoly(mask, box, 1)
    return mask


def test_normalize_parallel_gripper_yaw_examples():
    assert normalize_parallel_gripper_yaw(0) == 0.0
    assert normalize_parallel_gripper_yaw(30) == 30.0
    assert normalize_parallel_gripper_yaw(100) == -80.0
    assert normalize_parallel_gripper_yaw(135) == -45.0
    assert normalize_parallel_gripper_yaw(-135) == 45.0
    assert normalize_parallel_gripper_yaw(-100) == 80.0
    assert normalize_parallel_gripper_yaw(180) == 0.0


def test_estimate_yaw_from_mask_horizontal_rectangle():
    mask = np.zeros((120, 160), dtype=np.uint8)
    mask[50:70, 30:130] = 1

    yaw_deg, debug = estimate_yaw_from_mask(mask)

    assert debug["success"] is True
    assert abs(yaw_deg) < 1.0


def test_estimate_yaw_from_mask_vertical_rectangle():
    mask = np.zeros((160, 120), dtype=np.uint8)
    mask[30:130, 50:70] = 1

    yaw_deg, debug = estimate_yaw_from_mask(mask)

    assert debug["success"] is True
    assert math.isclose(abs(yaw_deg), 90.0, abs_tol=1.0)


def test_estimate_yaw_from_mask_positive_diagonal():
    mask = _rotated_rect_mask((180, 180), (90, 90), (110, 20), 45.0)

    yaw_deg, debug = estimate_yaw_from_mask(mask)

    assert debug["success"] is True
    assert 35.0 <= yaw_deg <= 55.0


def test_estimate_yaw_from_mask_negative_diagonal():
    mask = _rotated_rect_mask((180, 180), (90, 90), (110, 20), -45.0)

    yaw_deg, debug = estimate_yaw_from_mask(mask)

    assert debug["success"] is True
    assert -55.0 <= yaw_deg <= -35.0


def test_estimate_yaw_from_mask_rejects_small_mask():
    mask = np.zeros((20, 20), dtype=np.uint8)
    mask[5:7, 5:7] = 1

    yaw_deg, debug = estimate_yaw_from_mask(mask)

    assert yaw_deg == 0.0
    assert debug["success"] is False
    assert debug["reason"] == "mask_has_too_few_pixels"


def test_estimate_sam_mask_for_crop_prefers_mask_containing_grasp_point():
    crop = np.zeros((60, 80, 3), dtype=np.uint8)
    mask_far = np.zeros((60, 80), dtype=np.uint8)
    mask_far[5:15, 5:15] = 1
    mask_good = np.zeros((60, 80), dtype=np.uint8)
    mask_good[20:45, 25:55] = 1
    predictor = FakeSamPredictor([mask_far, mask_good], scores=[0.95, 0.9])

    mask, debug = estimate_sam_mask_for_crop(
        crop,
        sam_predictor=predictor,
        grasp_point=(40, 30),
    )

    assert debug["success"] is True
    assert debug["grasp_point_inside_mask"] is True
    assert mask is not None
    assert mask[30, 40] == 1


def test_aggregate_masks_majority_stabilizes_jitter():
    base = np.zeros((40, 40), dtype=np.uint8)
    base[10:30, 12:28] = 1
    shifted1 = np.zeros((40, 40), dtype=np.uint8)
    shifted1[10:30, 11:27] = 1
    shifted2 = np.zeros((40, 40), dtype=np.uint8)
    shifted2[11:31, 12:28] = 1

    mask, debug = aggregate_masks_across_frames([base, shifted1, shifted2])

    assert debug["success"] is True
    assert mask is not None
    assert mask[20, 20] == 1
    assert debug["num_valid_masks"] == 3


def test_aggregate_masks_ignores_invalid_masks_and_resizes():
    mask_a = np.zeros((30, 30), dtype=np.uint8)
    mask_a[8:22, 10:20] = 1
    mask_b = np.zeros((20, 20), dtype=np.uint8)
    mask_b[5:15, 6:14] = 1

    mask, debug = aggregate_masks_across_frames([mask_a, None, mask_b])

    assert debug["success"] is True
    assert debug["num_valid_masks"] == 2
    assert mask.shape == mask_a.shape


def test_aggregate_masks_fails_when_valid_masks_too_few():
    mask = np.zeros((30, 30), dtype=np.uint8)
    mask[10:20, 10:20] = 1

    aggregated, debug = aggregate_masks_across_frames([mask], config={"min_valid_masks": 2})

    assert aggregated is None
    assert debug["success"] is False
    assert debug["reason"] == "insufficient_valid_masks"


def test_apply_pca_yaw_to_grasp_result_overrides_yaw_with_multi_frame_pca():
    crop = np.zeros((100, 100, 3), dtype=np.uint8)
    mask1 = _rotated_rect_mask((100, 100), (50, 50), (60, 18), 40.0)

    result, debug = apply_pca_yaw_to_grasp_result(
        {
            "x_px": 50,
            "y_px": 50,
            "yaw_deg": -45.0,
            "confidence": 0.82,
            "reason": "sturdy visible handle",
        },
        [crop, crop],
        sam_predictor=FakeSamPredictor([mask1]),
        config={"min_valid_masks": 2},
    )

    assert debug["multi_frame_debug"]["success"] is True
    assert result["confidence"] == 0.82
    assert result["reason"] == "sturdy visible handle"
    assert 30.0 <= result["yaw_deg"] <= 50.0


def test_apply_pca_yaw_to_grasp_result_single_frame_fallback():
    crop = np.zeros((100, 100, 3), dtype=np.uint8)
    good_mask = _rotated_rect_mask((100, 100), (50, 50), (60, 18), -35.0)

    class SequencePredictor:
        def __init__(self):
            self.calls = 0

        def set_image(self, image):
            self.calls += 1

        def predict(self, point_coords=None, point_labels=None, box=None, multimask_output=True):
            if self.calls <= 2:
                return np.asarray([np.zeros((100, 100), dtype=np.uint8)]), np.asarray([0.9]), None
            return np.asarray([good_mask]), np.asarray([0.9]), None

    result, debug = apply_pca_yaw_to_grasp_result(
        {"x_px": 50, "y_px": 50, "yaw_deg": 20.0, "confidence": 0.7, "reason": "ok"},
        [crop, crop],
        sam_predictor=SequencePredictor(),
    )

    assert debug["fallback_source"] == "single_frame_pca"
    assert -45.0 <= result["yaw_deg"] <= -25.0


def test_apply_pca_yaw_to_grasp_result_falls_back_to_vlm_yaw():
    crop = np.zeros((80, 80, 3), dtype=np.uint8)
    predictor = FakeSamPredictor([np.zeros((80, 80), dtype=np.uint8)])

    result, debug = apply_pca_yaw_to_grasp_result(
        {"x_px": 20, "y_px": 30, "yaw_deg": 33.0, "confidence": 0.5, "reason": "vlm"},
        crop,
        sam_predictor=predictor,
    )

    assert result["yaw_deg"] == 33.0
    assert debug["fallback_source"] == "input_yaw"


def test_apply_pca_yaw_to_grasp_result_falls_back_to_zero_without_vlm_yaw():
    crop = np.zeros((80, 80, 3), dtype=np.uint8)
    predictor = FakeSamPredictor([np.zeros((80, 80), dtype=np.uint8)])

    result, debug = apply_pca_yaw_to_grasp_result(
        {"x_px": 20, "y_px": 30, "confidence": 0.5, "reason": "vlm"},
        crop,
        sam_predictor=predictor,
    )

    assert result["yaw_deg"] == 0.0
    assert debug["fallback_source"] == "default_zero"


def test_apply_pca_yaw_to_grasp_result_inverts_sign():
    crop = np.zeros((100, 100, 3), dtype=np.uint8)
    mask = _rotated_rect_mask((100, 100), (50, 50), (60, 18), 30.0)

    result, debug = apply_pca_yaw_to_grasp_result(
        {"x_px": 50, "y_px": 50, "yaw_deg": 0.0, "confidence": 0.8, "reason": "ok"},
        [crop, crop],
        sam_predictor=FakeSamPredictor([mask]),
        config={"invert_yaw_sign": True, "min_valid_masks": 2},
    )

    assert debug["invert_yaw_sign_applied"] is True
    assert -40.0 <= result["yaw_deg"] <= -20.0


def test_center_selector_returns_refined_orientation():
    class FakeRefiner:
        def refine(self, grasp_result, crop_frames_bgr):
            return (
                {**grasp_result, "yaw_deg": 12.5},
                {
                    "sam_success": True,
                    "aggregation_success": False,
                    "pca_success": True,
                    "fallback_used": True,
                    "fallback_source": "single_frame_pca",
                    "invert_yaw_sign_applied": False,
                },
            )

    selector = CenterGraspPointSelector(FakeLogger())
    selector.yaw_refiner = FakeRefiner()
    image = np.zeros((100, 120, 3), dtype=np.uint8)

    result = selector.select_grasp_pixel((10, 20, 70, 80), color_image=image)

    assert result == (40, 50, "center", (0.0, 0.0, 12.5))
