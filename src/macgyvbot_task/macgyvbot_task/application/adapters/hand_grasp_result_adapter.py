"""Adapt hand-grasp result payloads for task workflow consumption."""

from __future__ import annotations

import time

from macgyvbot_config.robot import BASE_FRAME
from macgyvbot_perception.depth_projection import pixel_to_camera_point


class HandGraspResultAdapter:
    """Attach base-frame hand position to hand-grasp JSON payloads when needed."""

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
            self.logger.warn(
                "handover_hand_pixel 경계 클램프 적용: "
                f"raw=({u}, {v}), clamped=({clamped_u}, {clamped_v}), "
                f"size=({width}, {height})"
            )
        u, v = clamped_u, clamped_v

        camera_point = pixel_to_camera_point(
            u,
            v,
            self.state.depth_image,
            self.state.intrinsics,
            logger=self.logger,
            source="handover_hand_pixel",
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
