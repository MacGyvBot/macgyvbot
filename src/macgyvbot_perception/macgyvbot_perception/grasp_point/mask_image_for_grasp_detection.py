"""Generate and save image triplets for gripper grasp detection."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
from typing import Optional

import cv2
import numpy as np


DEFAULT_DATA_ROOT = Path(__file__).resolve().parents[2] / "data"


def generate_mask_image_for_grasp_detection(
    color_image: np.ndarray,
    binary_mask_or_image: np.ndarray,
    bbox_xyxy,
    data_root: str | Path = DEFAULT_DATA_ROOT,
    filename_prefix: str = "grasp_detection",
    timestamp: Optional[str] = None,
) -> list[np.ndarray]:
    """Return and save [cropped_binary, cropped_rgb, frame_rgb] images.

    The returned arrays are OpenCV images. They are saved to:
    - data/yaw_pca/ for the cropped binary image
    - data/inference_history/crop_image/ for the cropped RGB image
    - data/inference_history/frame_image/ for the full RGB frame
    """
    x1, y1, x2, y2 = _clamp_bbox_to_image(bbox_xyxy, color_image)
    binary_image = _binary_to_bgr(binary_mask_or_image, color_image.shape[:2])

    cropped_binary = binary_image[y1:y2, x1:x2].copy()
    cropped_rgb = color_image[y1:y2, x1:x2].copy()
    frame_rgb = color_image.copy()

    _save_triplet(
        cropped_binary=cropped_binary,
        cropped_rgb=cropped_rgb,
        frame_rgb=frame_rgb,
        data_root=Path(data_root),
        filename_prefix=filename_prefix,
        timestamp=timestamp,
    )
    return [cropped_binary, cropped_rgb, frame_rgb]


def _save_triplet(
    *,
    cropped_binary: np.ndarray,
    cropped_rgb: np.ndarray,
    frame_rgb: np.ndarray,
    data_root: Path,
    filename_prefix: str,
    timestamp: Optional[str],
) -> None:
    safe_prefix = _safe_name(filename_prefix)
    stamp = timestamp or datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    filename = f"{stamp}_{safe_prefix}.jpg"

    yaw_pca_dir = data_root / "yaw_pca"
    crop_dir = data_root / "inference_history" / "crop_image"
    frame_dir = data_root / "inference_history" / "frame_image"
    yaw_pca_dir.mkdir(parents=True, exist_ok=True)
    crop_dir.mkdir(parents=True, exist_ok=True)
    frame_dir.mkdir(parents=True, exist_ok=True)

    cv2.imwrite(str(yaw_pca_dir / filename), cropped_binary)
    cv2.imwrite(str(crop_dir / filename), cropped_rgb)
    cv2.imwrite(str(frame_dir / f"{stamp}_{safe_prefix}_frame.jpg"), frame_rgb)


def _binary_to_bgr(binary_mask_or_image: np.ndarray, frame_shape) -> np.ndarray:
    binary = np.asarray(binary_mask_or_image)
    height, width = frame_shape
    if binary.shape[:2] != (height, width):
        raise ValueError(
            "binary_mask_or_image shape must match color_image height and width"
        )

    if binary.ndim == 3:
        gray = cv2.cvtColor(binary, cv2.COLOR_BGR2GRAY)
    else:
        gray = binary
    mask = gray > 0
    output = np.zeros((height, width, 3), dtype=np.uint8)
    output[mask] = (255, 255, 255)
    return output


def _clamp_bbox_to_image(bbox_xyxy, image: np.ndarray) -> tuple[int, int, int, int]:
    if len(bbox_xyxy) < 4:
        raise ValueError("bbox_xyxy must contain four values")
    height, width = image.shape[:2]
    x1 = max(0, min(width - 1, int(np.floor(float(bbox_xyxy[0])))))
    y1 = max(0, min(height - 1, int(np.floor(float(bbox_xyxy[1])))))
    x2 = max(0, min(width, int(np.ceil(float(bbox_xyxy[2])))))
    y2 = max(0, min(height, int(np.ceil(float(bbox_xyxy[3])))))
    if x2 <= x1 or y2 <= y1:
        raise ValueError("bbox_xyxy is empty after clipping")
    return x1, y1, x2, y2


def _safe_name(value: str) -> str:
    return "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in value)
