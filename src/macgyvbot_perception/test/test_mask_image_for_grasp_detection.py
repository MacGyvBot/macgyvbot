import cv2
import numpy as np

from macgyvbot_perception.grasp_point.mask_image_for_grasp_detection import (
    generate_mask_image_for_grasp_detection,
    generate_sam_depth_mask_image_for_grasp_detection,
)


class FakeSegmenter:
    def segment(self, _color_image, _bbox):
        mask = np.zeros((20, 30), dtype=bool)
        mask[5:15, 8:18] = True
        mask[5:15, 13:18] = True
        return mask


def test_generate_mask_image_for_grasp_detection_saves_triplet(tmp_path):
    color = np.zeros((20, 30, 3), dtype=np.uint8)
    color[:, :] = (10, 20, 30)
    mask = np.zeros((20, 30), dtype=np.uint8)
    mask[5:15, 8:18] = 255

    cropped_binary, cropped_rgb, frame_rgb = generate_mask_image_for_grasp_detection(
        color,
        mask,
        (8, 5, 18, 15),
        data_root=tmp_path,
        filename_prefix="test",
        timestamp="20260101_000000_000000",
    )

    assert cropped_binary.shape == (10, 10, 3)
    assert cropped_rgb.shape == (10, 10, 3)
    assert frame_rgb.shape == (20, 30, 3)
    assert int(cropped_binary.min()) == 255
    assert np.array_equal(cropped_rgb, color[5:15, 8:18])
    assert np.array_equal(frame_rgb, color)

    yaw_pca_path = tmp_path / "yaw_pca" / "20260101_000000_000000_test.jpg"
    crop_path = (
        tmp_path
        / "inference_history"
        / "crop_image"
        / "20260101_000000_000000_test.jpg"
    )
    frame_path = (
        tmp_path
        / "inference_history"
        / "frame_image"
        / "20260101_000000_000000_test_frame.jpg"
    )
    assert yaw_pca_path.exists()
    assert crop_path.exists()
    assert frame_path.exists()
    assert cv2.imread(str(yaw_pca_path)) is not None


def test_generate_sam_depth_mask_image_for_grasp_detection_refines_and_crops(
    tmp_path,
):
    color = np.zeros((20, 30, 3), dtype=np.uint8)
    color[:, :] = (10, 20, 30)
    depth = np.zeros((20, 30), dtype=np.float32)
    depth[5:15, 8:13] = 1000.0
    depth[5:15, 13:18] = 1500.0

    cropped_binary, cropped_rgb, frame_rgb = (
        generate_sam_depth_mask_image_for_grasp_detection(
            color_image=color,
            depth_mm=depth,
            bbox_xyxy=(8, 5, 18, 15),
            sam_segmenter=FakeSegmenter(),
            data_root=tmp_path,
            filename_prefix="sam_depth",
            timestamp="20260101_000000_000001",
            sam_depth_tolerance_mm=25.0,
            sam_depth_min_valid_ratio=0.03,
            sam_depth_expand_iterations=0,
        )
    )

    assert cropped_binary.shape == (10, 10, 3)
    assert cropped_rgb.shape == (10, 10, 3)
    assert frame_rgb.shape == (20, 30, 3)
    assert np.array_equal(cropped_rgb, color[5:15, 8:18])
    assert int(cropped_binary[:, :5].max()) == 255
    assert int(cropped_binary[:, 5:].max()) == 0

    yaw_pca_path = tmp_path / "yaw_pca" / "20260101_000000_000001_sam_depth.jpg"
    assert yaw_pca_path.exists()
