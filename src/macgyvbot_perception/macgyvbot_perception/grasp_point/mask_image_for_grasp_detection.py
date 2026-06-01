"""Generate and save image triplets for gripper grasp detection."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
from typing import Optional

import cv2
import numpy as np


DEFAULT_DATA_ROOT = Path("src/macgyvbot_perception/data")


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


def generate_sam_depth_mask_image_for_grasp_detection(
    *,
    color_image: np.ndarray,
    depth_mm: np.ndarray | None,
    bbox_xyxy,
    sam_segmenter,
    data_root: str | Path = DEFAULT_DATA_ROOT,
    filename_prefix: str = "grasp_detection",
    timestamp: Optional[str] = None,
    sam_depth_tolerance_mm: float = 30.0,
    sam_depth_min_valid_ratio: float = 0.03,
    sam_depth_expand_iterations: int = 1,
) -> list[np.ndarray] | None:
    """Generate/save a YOLO-bbox crop using a SAM mask refined by depth.

    The crop area is always the YOLO bbox. Inside that crop, mask pixels are
    white and non-mask pixels are black.
    """
    if sam_segmenter is None:
        return None

    bbox = _clamp_bbox_to_image(bbox_xyxy, color_image)
    sam_mask = sam_segmenter.segment(color_image, bbox)
    if sam_mask is None or int(np.asarray(sam_mask).sum()) <= 0:
        return None

    refined_mask = _refine_sam_mask_with_depth(
        sam_mask,
        depth_mm,
        tolerance_mm=sam_depth_tolerance_mm,
        min_valid_ratio=sam_depth_min_valid_ratio,
        expand_iterations=sam_depth_expand_iterations,
    )
    if int(refined_mask.sum()) <= 0:
        return None

    return generate_mask_image_for_grasp_detection(
        color_image=color_image,
        binary_mask_or_image=refined_mask,
        bbox_xyxy=bbox,
        data_root=data_root,
        filename_prefix=filename_prefix,
        timestamp=timestamp,
    )


def _refine_sam_mask_with_depth(
    sam_mask: np.ndarray,
    depth_mm: np.ndarray | None,
    *,
    tolerance_mm: float,
    min_valid_ratio: float,
    expand_iterations: int,
) -> np.ndarray:
    sam_bool = np.asarray(sam_mask).astype(bool)
    if depth_mm is None or not sam_bool.any():
        return _expand_binary_mask(
            sam_bool.astype(np.uint8),
            iterations=expand_iterations,
        ).astype(bool)

    depth = np.asarray(depth_mm)
    valid = (depth > 0) & sam_bool
    sam_area = max(1, int(sam_bool.sum()))
    valid_ratio = int(valid.sum()) / sam_area
    if valid_ratio < min_valid_ratio:
        return _expand_binary_mask(
            sam_bool.astype(np.uint8),
            iterations=expand_iterations,
        ).astype(bool)

    target_depth = float(np.median(depth[valid]))
    close_depth = (depth > 0) & (np.abs(depth - target_depth) <= tolerance_mm)
    refined = sam_bool & (close_depth | ~valid)
    if not refined.any():
        refined = sam_bool
    return _expand_binary_mask(
        refined.astype(np.uint8),
        iterations=expand_iterations,
    ).astype(bool)


def _expand_binary_mask(mask: np.ndarray, iterations: int = 1) -> np.ndarray:
    kernel = np.ones((3, 3), np.uint8)
    expanded = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_CLOSE, kernel)
    return cv2.dilate(expanded, kernel, iterations=max(0, int(iterations)))


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

    _write_image(yaw_pca_dir / filename, cropped_binary)
    _write_image(crop_dir / filename, cropped_rgb)
    _write_image(frame_dir / f"{stamp}_{safe_prefix}_frame.jpg", frame_rgb)


def _write_image(path: Path, image: np.ndarray) -> None:
    if not cv2.imwrite(str(path), image):
        raise RuntimeError(f"failed to write image: {path}")


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
