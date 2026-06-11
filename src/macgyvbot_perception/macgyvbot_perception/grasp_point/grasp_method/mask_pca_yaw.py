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


def estimate_yaw_and_cross_section_width_from_binary_crop(
    cropped_binary_image: np.ndarray | None,
    *,
    grasp_point_xy: tuple[float, float] | None = None,
    depth_mm: np.ndarray | None = None,
    crop_origin_xy: tuple[float, float] = (0.0, 0.0),
    camera_fx: float | None = None,
    camera_fy: float | None = None,
    min_pixels: int = 30,
    cross_section_half_band_px: float = 2.0,
) -> tuple[float | None, float | None, dict[str, Any]]:
    """Return PCA yaw and the mask width crossing the grasp point in millimeters."""
    yaw_deg, debug = estimate_yaw_from_binary_crop(
        cropped_binary_image,
        min_pixels=min_pixels,
    )
    debug.update(
        {
            "has_width": False,
            "width_px": None,
            "width_mm": None,
            "width_reason": "",
        }
    )
    if yaw_deg is None:
        debug["width_reason"] = str(debug.get("reason") or "pca_yaw_failed")
        return None, None, debug

    width_px, width_debug = estimate_cross_section_width_px_from_binary_crop(
        cropped_binary_image,
        grasp_point_xy=grasp_point_xy,
        min_pixels=min_pixels,
        cross_section_half_band_px=cross_section_half_band_px,
    )
    debug.update(width_debug)
    if width_px is None:
        return yaw_deg, None, debug

    width_mm = _width_px_to_mm(
        width_px,
        yaw_deg=yaw_deg,
        depth_mm=depth_mm,
        grasp_point_xy=grasp_point_xy,
        crop_origin_xy=crop_origin_xy,
        camera_fx=camera_fx,
        camera_fy=camera_fy,
    )
    if width_mm is None:
        debug["width_reason"] = "metric_width_unavailable"
        return yaw_deg, None, debug

    debug.update(
        {
            "has_width": True,
            "width_mm": float(width_mm),
            "width_reason": "ok",
        }
    )
    return yaw_deg, float(width_mm), debug


def estimate_cross_section_width_px_from_binary_crop(
    cropped_binary_image: np.ndarray | None,
    *,
    grasp_point_xy: tuple[float, float] | None = None,
    min_pixels: int = 30,
    cross_section_half_band_px: float = 2.0,
) -> tuple[float | None, dict[str, Any]]:
    """Measure mask width along the minor PCA axis at a grasp point."""
    debug: dict[str, Any] = {
        "width_px": None,
        "width_reason": "",
    }
    mask = _binary_mask(cropped_binary_image)
    if mask is None:
        debug["width_reason"] = "invalid_binary_crop"
        return None, debug

    ys, xs = np.where(mask > 0)
    if xs.size < int(min_pixels):
        debug["width_reason"] = "too_few_mask_pixels"
        return None, debug

    coords = np.column_stack((xs, ys)).astype(np.float32)
    centered = coords - coords.mean(axis=0)
    covariance = np.cov(centered, rowvar=False)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    principal = eigenvectors[:, int(np.argmax(eigenvalues))].astype(np.float32)
    principal /= float(np.linalg.norm(principal)) or 1.0
    minor = np.array([-principal[1], principal[0]], dtype=np.float32)

    if grasp_point_xy is None:
        point = coords.mean(axis=0)
    else:
        point = np.array(grasp_point_xy, dtype=np.float32)

    along_principal = (coords - point) @ principal
    cross_section = coords[
        np.abs(along_principal) <= float(cross_section_half_band_px)
    ]
    if cross_section.shape[0] < 2:
        debug["width_reason"] = "too_few_cross_section_pixels"
        return None, debug

    along_minor = (cross_section - point) @ minor
    width_px = float(np.max(along_minor) - np.min(along_minor) + 1.0)
    debug.update(
        {
            "width_px": width_px,
            "width_reason": "ok",
        }
    )
    return width_px, debug


def _binary_mask(image: np.ndarray | None) -> np.ndarray | None:
    if image is None or not isinstance(image, np.ndarray) or image.ndim < 2:
        return None
    if image.ndim == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image
    return (gray > 0).astype(np.uint8)


def _width_px_to_mm(
    width_px: float,
    *,
    yaw_deg: float,
    depth_mm: np.ndarray | None,
    grasp_point_xy: tuple[float, float] | None,
    crop_origin_xy: tuple[float, float],
    camera_fx: float | None,
    camera_fy: float | None,
) -> float | None:
    if depth_mm is None or grasp_point_xy is None:
        return None
    if camera_fx is None or camera_fy is None or camera_fx <= 0.0 or camera_fy <= 0.0:
        return None

    crop_x, crop_y = crop_origin_xy
    local_x, local_y = grasp_point_xy
    u = int(round(float(crop_x) + float(local_x)))
    v = int(round(float(crop_y) + float(local_y)))
    if v < 0 or u < 0 or v >= depth_mm.shape[0] or u >= depth_mm.shape[1]:
        return None

    z_mm = _local_depth_mm(depth_mm, u, v)
    if z_mm is None or z_mm <= 0.0:
        return None

    angle_rad = math.radians(float(yaw_deg))
    minor_dx = math.cos(angle_rad)
    minor_dy = math.sin(angle_rad)
    mm_per_px = float(z_mm) * math.sqrt(
        (minor_dx / float(camera_fx)) ** 2 + (minor_dy / float(camera_fy)) ** 2
    )
    return float(width_px) * mm_per_px


def _local_depth_mm(depth_mm: np.ndarray, u: int, v: int, radius: int = 2) -> float | None:
    y1 = max(0, v - radius)
    y2 = min(depth_mm.shape[0], v + radius + 1)
    x1 = max(0, u - radius)
    x2 = min(depth_mm.shape[1], u + radius + 1)
    values = np.asarray(depth_mm[y1:y2, x1:x2], dtype=np.float32)
    valid = values[np.isfinite(values) & (values > 0.0)]
    if valid.size == 0:
        return None
    return float(np.median(valid))
