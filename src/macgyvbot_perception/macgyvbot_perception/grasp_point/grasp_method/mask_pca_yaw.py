"""Estimate gripper yaw from a cropped binary tool-mask image."""

from __future__ import annotations

import math
from typing import Any

import cv2
import numpy as np


def normalize_parallel_gripper_yaw(angle_deg: float) -> float:
    """Normalize yaw to the nearest equivalent parallel-gripper angle."""
    angle = ((float(angle_deg) + 180.0) % 360.0) - 180.0
    normalized = ((angle + 90.0) % 180.0) - 90.0
    if math.isclose(normalized, -90.0, abs_tol=1e-6) and angle > 0.0:
        normalized = 90.0
    if math.isclose(abs(normalized), 0.0, abs_tol=1e-6):
        normalized = 0.0
    return float(normalized)


def estimate_yaw_from_binary_crop(
    cropped_binary_image: np.ndarray | None,
    *,
    min_pixels: int = 30,
) -> tuple[float | None, dict[str, Any]]:
    """Return yaw from the white pixels in a cropped binary mask image."""
    debug: dict[str, Any] = {
        "success": False,
        "num_pixels": 0,
        "raw_angle_deg": None,
        "normalized_yaw_deg": None,
        "reason": "",
    }
    mask = _binary_mask(cropped_binary_image)
    if mask is None:
        debug["reason"] = "invalid_binary_crop"
        return None, debug

    ys, xs = np.where(mask > 0)
    debug["num_pixels"] = int(xs.size)
    if xs.size < int(min_pixels):
        debug["reason"] = "too_few_mask_pixels"
        return None, debug

    coords = np.column_stack((xs, ys)).astype(np.float32)
    centered = coords - coords.mean(axis=0)
    covariance = np.cov(centered, rowvar=False)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    principal = eigenvectors[:, int(np.argmax(eigenvalues))]
    raw_angle_deg = math.degrees(
        math.atan2(-float(principal[0]), float(principal[1]))
    )
    yaw_deg = normalize_parallel_gripper_yaw(raw_angle_deg)
    debug.update(
        {
            "success": True,
            "raw_angle_deg": float(raw_angle_deg),
            "normalized_yaw_deg": float(yaw_deg),
            "reason": "ok",
        }
    )
    return float(yaw_deg), debug


def _binary_mask(image: np.ndarray | None) -> np.ndarray | None:
    if image is None or not isinstance(image, np.ndarray) or image.ndim < 2:
        return None
    if image.ndim == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image
    return (gray > 0).astype(np.uint8)
