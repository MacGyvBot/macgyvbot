"""Depth image projection helpers."""

from __future__ import annotations

import time

import numpy as np


def pixel_to_camera_point(
    u,
    v,
    depth_image,
    intrinsics,
    depth_scale=0.001,
    logger=None,
    source="pixel",
    warn_fn=None,
):
    """Project an image pixel with depth into the camera frame."""
    height, width = depth_image.shape[:2]

    if not (0 <= u < width and 0 <= v < height):
        _warn(
            logger,
            warn_fn,
            (
                f"{source} is outside depth image: u={u}, v={v}, "
                f"size=({width}, {height})"
            ),
        )
        return None

    z_raw = float(depth_image[v, u])
    if not np.isfinite(z_raw) or z_raw <= 0.0:
        _warn(
            logger,
            warn_fn,
            f"{source} depth is invalid: u={u}, v={v}, depth={z_raw}",
        )
        return None

    z_m = z_raw * depth_scale
    cam_x = (u - intrinsics["ppx"]) * z_m / intrinsics["fx"]
    cam_y = (v - intrinsics["ppy"]) * z_m / intrinsics["fy"]
    return float(cam_x), float(cam_y), float(z_m)


def transform_point_to_base(cam_xyz, base_to_camera):
    """Transform a camera-frame point into the robot base frame."""
    coord = np.append(np.array(cam_xyz, dtype=float), 1.0)
    return (base_to_camera @ coord)[:3]


class DepthProjector:
    """Project image-space grasp pixels into the robot base frame."""

    def __init__(self, base_to_camera_provider):
        self.base_to_camera_provider = base_to_camera_provider
        self._warn_history = {}

    def pixel_to_base_target(
        self,
        u,
        v,
        label,
        source,
        depth_image,
        intrinsics,
        logger,
        vlm_rpy_deg=None,
    ):
        camera_point = pixel_to_camera_point(
            u,
            v,
            depth_image,
            intrinsics,
            logger=logger,
            source=source,
            warn_fn=lambda message: self._warn_throttled(
                logger,
                source,
                message,
            ),
        )
        if camera_point is None:
            return None

        cam_x, cam_y, z_m = camera_point
        bx, by, bz = self.camera_to_base(camera_point)

        logger.info(
            f"'{label}' detected: source={source}, "
            f"pixel=({u}, {v}), "
            f"camera=({cam_x:.3f}, {cam_y:.3f}, {z_m:.3f}), "
            f"base=({bx:.3f}, {by:.3f}, {bz:.3f})"
        )

        return bx, by, bz, z_m, vlm_rpy_deg

    def camera_to_base(self, cam_xyz):
        return transform_point_to_base(cam_xyz, self.base_to_camera_provider())

    def pixel_to_camera_point(
        self,
        u,
        v,
        depth_image,
        intrinsics,
        logger=None,
        source="pixel",
        depth_scale=0.001,
    ):
        return pixel_to_camera_point(
            u,
            v,
            depth_image,
            intrinsics,
            depth_scale=depth_scale,
            logger=logger,
            source=source,
            warn_fn=lambda message: self._warn_throttled(
                logger,
                source,
                message,
            ),
        )

    def _warn_throttled(self, logger, source, message, interval_sec=1.0):
        if logger is None:
            return

        now = time.monotonic()
        last_time = self._warn_history.get(source)
        if last_time is not None and now - last_time < interval_sec:
            return

        self._warn_history[source] = now
        logger.warn(message)


def _warn(logger, warn_fn, message):
    if warn_fn is not None:
        warn_fn(message)
        return
    if logger is not None:
        logger.warn(message)
