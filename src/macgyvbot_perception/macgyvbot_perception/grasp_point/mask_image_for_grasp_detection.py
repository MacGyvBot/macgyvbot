"""Generate, save, and record grasp detection images."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Optional

import cv2
import numpy as np
from PIL import Image


DEFAULT_DATA_ROOT = Path("src/macgyvbot_perception/data")


@dataclass(frozen=True)
class GraspDetectionRecordConfig:
    """Configuration for optional grasp detection recording."""

    enabled: bool = True
    root_dir: str = "src/macgyvbot_perception/data/inference_history"
    csv_name: str = "inference_history.csv"


class GraspDetectionRecorder:
    """Build image triplets, save debug images, and record inference history."""

    FIELD_NAMES = (
        "timestamp",
        "mode",
        "model_id",
        "target_label",
        "detected_label",
        "bbox_xyxy",
        "image_path",
        "frame_image_path",
        "raw_response",
        "parsed_point",
        "yaw_deg",
        "orientation_rpy_deg",
        "success",
        "error",
    )

    def __init__(self, config: GraspDetectionRecordConfig | None = None, logger=None):
        self.config = config or GraspDetectionRecordConfig()
        self.logger = logger

    def build_images(
        self,
        color_image: np.ndarray,
        binary_mask_or_image: np.ndarray,
        bbox_xyxy,
    ) -> list[np.ndarray]:
        """Return [cropped_binary, cropped_rgb, frame_rgb] OpenCV images."""
        x1, y1, x2, y2 = _clamp_bbox_to_image(bbox_xyxy, color_image)
        binary_image = _binary_to_bgr(binary_mask_or_image, color_image.shape[:2])
        return [
            binary_image[y1:y2, x1:x2].copy(),
            color_image[y1:y2, x1:x2].copy(),
            color_image.copy(),
        ]

    def save_images(
        self,
        *,
        cropped_binary: np.ndarray | None = None,
        cropped_rgb: np.ndarray | Image.Image | None = None,
        frame_rgb: np.ndarray | Image.Image | None = None,
        data_root: str | Path = DEFAULT_DATA_ROOT,
        filename_prefix: str = "grasp_detection",
        timestamp: Optional[str] = None,
    ) -> dict[str, str]:
        """Save any provided grasp images and return their paths."""
        safe_prefix = _safe_name(filename_prefix)
        stamp = timestamp or datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        filename = f"{stamp}_{safe_prefix}.jpg"
        root = Path(data_root)
        paths = {"yaw_pca_image_path": "", "image_path": "", "frame_image_path": ""}

        if cropped_binary is not None:
            yaw_pca_dir = root / "yaw_pca"
            yaw_pca_dir.mkdir(parents=True, exist_ok=True)
            path = yaw_pca_dir / filename
            _write_image(path, cropped_binary)
            paths["yaw_pca_image_path"] = str(path)

        if cropped_rgb is not None:
            crop_dir = root / "inference_history" / "crop_image"
            crop_dir.mkdir(parents=True, exist_ok=True)
            path = crop_dir / filename
            _write_image(path, cropped_rgb)
            paths["image_path"] = str(path)

        if frame_rgb is not None:
            frame_dir = root / "inference_history" / "frame_image"
            frame_dir.mkdir(parents=True, exist_ok=True)
            path = frame_dir / f"{stamp}_{safe_prefix}_frame.jpg"
            _write_image(path, frame_rgb)
            paths["frame_image_path"] = str(path)

        return paths

    def record_inference(
        self,
        *,
        image: Image.Image | np.ndarray | None,
        mode: str,
        model_id: str,
        target_label: str,
        detected_label: str,
        bbox_xyxy,
        raw_response: str = "",
        parsed_point=None,
        yaw_deg=None,
        orientation_rpy_deg=None,
        success: bool = True,
        error: str = "",
        frame_image: Image.Image | np.ndarray | None = None,
        image_path: str = "",
        frame_image_path: str = "",
    ) -> None:
        """Append one CSV row when inference-history recording is enabled."""
        if not self.config.enabled:
            return

        try:
            root = Path(self.config.root_dir)
            root.mkdir(parents=True, exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            self._append_csv_row(
                root=root,
                timestamp=timestamp,
                mode=mode,
                model_id=model_id,
                target_label=target_label,
                detected_label=detected_label,
                bbox_xyxy=bbox_xyxy,
                image_path=image_path,
                frame_image_path=frame_image_path,
                raw_response=raw_response,
                parsed_point=parsed_point,
                yaw_deg=yaw_deg,
                orientation_rpy_deg=orientation_rpy_deg,
                success=success,
                error=error,
            )
        except Exception as exc:
            if self.logger is not None:
                self.logger.warn(f"grasp detection recording failed: {exc}")

    def record(self, **kwargs) -> None:
        """Compatibility wrapper for previous inference-history callers."""
        self.record_inference(**kwargs)

    def _append_csv_row(
        self,
        *,
        root: Path,
        timestamp: str,
        mode: str,
        model_id: str,
        target_label: str,
        detected_label: str,
        bbox_xyxy,
        image_path: str,
        frame_image_path: str,
        raw_response: str,
        parsed_point,
        yaw_deg,
        orientation_rpy_deg,
        success: bool,
        error: str,
    ) -> None:
        csv_path = root / self.config.csv_name
        is_new = not csv_path.exists()
        with csv_path.open("a", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=self.FIELD_NAMES)
            if is_new:
                writer.writeheader()
            writer.writerow(
                {
                    "timestamp": timestamp,
                    "mode": mode,
                    "model_id": model_id,
                    "target_label": target_label,
                    "detected_label": detected_label,
                    "bbox_xyxy": self._stringify(bbox_xyxy),
                    "image_path": image_path,
                    "frame_image_path": frame_image_path,
                    "raw_response": raw_response,
                    "parsed_point": self._stringify(parsed_point),
                    "yaw_deg": self._stringify(yaw_deg),
                    "orientation_rpy_deg": self._stringify(orientation_rpy_deg),
                    "success": str(bool(success)),
                    "error": error,
                }
            )

    @staticmethod
    def _stringify(value: Any) -> str:
        if value is None:
            return ""
        return str(value)


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
    recorder = GraspDetectionRecorder()
    cropped_binary, cropped_rgb, frame_rgb = recorder.build_images(
        color_image,
        binary_mask_or_image,
        bbox_xyxy,
    )
    recorder.save_images(
        cropped_binary=cropped_binary,
        cropped_rgb=cropped_rgb,
        frame_rgb=frame_rgb,
        data_root=data_root,
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

    target_depth = _foreground_depth_mm(depth[valid])
    close_depth = (depth > 0) & (np.abs(depth - target_depth) <= tolerance_mm)
    refined = sam_bool & (close_depth | ~valid)
    if not refined.any():
        refined = sam_bool
    return _expand_binary_mask(
        refined.astype(np.uint8),
        iterations=expand_iterations,
    ).astype(bool)


def _foreground_depth_mm(valid_depth_mm: np.ndarray) -> float:
    """Pick the near-depth cluster from SAM pixels for foreground tool masking."""
    values = np.asarray(valid_depth_mm, dtype=np.float32)
    values = values[np.isfinite(values) & (values > 0)]
    if values.size == 0:
        return 0.0

    near_cutoff = float(np.percentile(values, 40.0))
    near_values = values[values <= near_cutoff]
    if near_values.size == 0:
        near_values = values
    return float(np.median(near_values))


def _expand_binary_mask(mask: np.ndarray, iterations: int = 1) -> np.ndarray:
    kernel = np.ones((3, 3), np.uint8)
    expanded = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_CLOSE, kernel)
    return cv2.dilate(expanded, kernel, iterations=max(0, int(iterations)))


def _write_image(path: Path, image: np.ndarray | Image.Image) -> None:
    if isinstance(image, Image.Image):
        image.convert("RGB").save(str(path), format="JPEG", quality=90)
        return
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
