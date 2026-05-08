"""Bounding-box center grasp point selection."""

from __future__ import annotations

from macgyvbot.config.config import GRASP_POINT_MODE_CENTER


def select_bbox_center_pixel(bbox):
    """Return the center pixel of an object bounding box."""
    u = int((bbox[0] + bbox[2]) / 2)
    v = int((bbox[1] + bbox[3]) / 2)
    return u, v, GRASP_POINT_MODE_CENTER, None
