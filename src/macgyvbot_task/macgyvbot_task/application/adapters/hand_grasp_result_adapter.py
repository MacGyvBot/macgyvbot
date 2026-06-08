"""Adapt hand-grasp result payloads for task workflow consumption."""

from __future__ import annotations

import time

import numpy as np

from macgyvbot_config.robot import BASE_FRAME
from macgyvbot_task.application.logging_utils import log_warn


class HandGraspResultAdapter:
    """Attach base-frame hand position to hand-grasp JSON payloads when needed."""

    DEPTH_WINDOW_RADIUS_PX = 3

    def __init__(self, state, depth_projector, logger):
        self.state = state
        self.depth_projector = depth_projector
        self.logger = logger

    def attach_base_position(self, result):
        position = result.get("position")
        if isinstance(position, dict) and all(
            k in position for k in ("x", "y", "z")
        ):
            return

        hand_pixel = result.get("hand_pixel")
        if not isinstance(hand_pixel, dict):
            return
        if self.state.depth_image is None or self.state.intrinsics is None:
            return

        try:
            u = int(hand_pixel["u"])
            v = int(hand_pixel["v"])
        except (KeyError, TypeError, ValueError):
            return

        height, width = self.state.depth_image.shape[:2]
        clamped_u = min(max(u, 0), max(width - 1, 0))
        clamped_v = min(max(v, 0), max(height - 1, 0))
        if clamped_u != u or clamped_v != v:
            log_warn(
                self.logger,
                "handover hand pixel clamped",
                step="hand_position",
                event="clamped",
                raw_u=u,
                raw_v=v,
                clamped_u=clamped_u,
                clamped_v=clamped_v,
                width=width,
                height=height,
            )
        u, v = clamped_u, clamped_v

        camera_point = self._pixel_to_camera_point_with_median_depth(
            u,
            v,
            self.state.depth_image,
            self.state.intrinsics,
            depth_encoding=getattr(self.state, "depth_encoding", None),
        )
        if camera_point is None:
            return

        bx, by, bz = self.depth_projector.camera_to_base(camera_point)
        result["position"] = {
            "x": float(bx),
            "y": float(by),
            "z": float(bz),
            "frame_id": BASE_FRAME,
        }
        result["position_observed_monotonic_sec"] = result.get(
            "_received_monotonic_sec",
            time.monotonic(),
        )

    def _pixel_to_camera_point_with_median_depth(
        self,
        u,
        v,
        depth_image,
        intrinsics,
        depth_encoding=None,
    ):
        depth_m = self._median_depth_meters(
            depth_image,
            u,
            v,
            depth_encoding=depth_encoding,
        )
        if depth_m is None:
            log_warn(
                self.logger,
                "handover hand depth unavailable",
                step="hand_position",
                event="unavailable",
                reason="invalid_depth",
                u=u,
                v=v,
                radius_px=self.DEPTH_WINDOW_RADIUS_PX,
            )
            return None

        cam_x = (u - intrinsics["ppx"]) * depth_m / intrinsics["fx"]
        cam_y = (v - intrinsics["ppy"]) * depth_m / intrinsics["fy"]
        return float(cam_x), float(cam_y), float(depth_m)

    def _median_depth_meters(
        self,
        depth_image,
        u,
        v,
        depth_encoding=None,
    ):
        height, width = depth_image.shape[:2]
        radius = int(self.DEPTH_WINDOW_RADIUS_PX)
        x1 = max(0, min(width, int(u) - radius))
        y1 = max(0, min(height, int(v) - radius))
        x2 = max(0, min(width, int(u) + radius + 1))
        y2 = max(0, min(height, int(v) + radius + 1))
        if x2 <= x1 or y2 <= y1:
            return None

        values = np.asarray(depth_image[y1:y2, x1:x2], dtype=np.float32)
        values = values[np.isfinite(values) & (values > 0.0)]
        if values.size == 0:
            return None

        scale = self._depth_scale(depth_image, depth_encoding)
        return float(np.median(values) * scale)

    @staticmethod
    def _depth_scale(depth_image, depth_encoding=None):
        if str(depth_encoding or "").upper() == "32FC1":
            return 1.0

        depth = np.asarray(depth_image)
        if np.issubdtype(depth.dtype, np.floating):
            finite = depth[np.isfinite(depth) & (depth > 0.0)]
            if finite.size and float(np.nanmax(finite)) < 20.0:
                return 1.0

        return 0.001
