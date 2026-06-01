import cv2
import numpy as np

from macgyvbot_perception.grasp_point.mask_image_for_grasp_detection import (
    generate_mask_image_for_grasp_detection,
)


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
