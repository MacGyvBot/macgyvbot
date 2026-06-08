"""Helpers for deriving a stable hand-center pixel from hand landmarks."""

from __future__ import annotations

from typing import Optional

Point = tuple[int, int]

MIN_VISIBLE_LANDMARKS = 3


def extract_hand_center_pixel(
    active_hand: Optional[dict],
    hand_infos: list[dict],
    tool_mask=None,
):
    """Return a robust hand-center pixel; fallback to landmark centroid."""
    hand = active_hand if active_hand is not None else (hand_infos[0] if hand_infos else None)
    if hand is None:
        return None

    landmarks = hand.get("landmarks", {})
    if isinstance(landmarks, dict) and landmarks:
        visible_center = _visible_landmark_center(
            landmarks,
            tool_mask,
            hand.get("palm_center"),
        )
        if visible_center is not None:
            return {
                "u": int(visible_center[0]),
                "v": int(visible_center[1]),
                "source": "visible_landmark_centroid",
            }

    palm_center = hand.get("palm_center")
    if isinstance(palm_center, tuple) and len(palm_center) == 2:
        return {
            "u": int(palm_center[0]),
            "v": int(palm_center[1]),
            "source": "palm_center",
        }

    if not isinstance(landmarks, dict) or not landmarks:
        return None

    points = [point for point in landmarks.values() if isinstance(point, tuple) and len(point) == 2]
    if not points:
        return None

    u = int(sum(point[0] for point in points) / len(points))
    v = int(sum(point[1] for point in points) / len(points))
    return {
        "u": u,
        "v": v,
        "source": "landmark_centroid",
    }


def _visible_landmark_center(
    landmarks: dict,
    tool_mask,
    palm_center: Optional[Point],
):
    if tool_mask is None:
        return None

    points = [
        point
        for point in landmarks.values()
        if isinstance(point, tuple) and len(point) == 2
    ]
    if not points:
        return None

    occluded_points = [
        point
        for point in points
        if _point_in_mask(point, tool_mask)
    ]
    palm_occluded = (
        isinstance(palm_center, tuple)
        and len(palm_center) == 2
        and _point_in_mask(palm_center, tool_mask)
    )
    if not occluded_points and not palm_occluded:
        return None

    visible_points = [
        point
        for point in points
        if not _point_in_mask(point, tool_mask)
    ]
    if len(visible_points) < MIN_VISIBLE_LANDMARKS:
        return None

    u = int(sum(point[0] for point in visible_points) / len(visible_points))
    v = int(sum(point[1] for point in visible_points) / len(visible_points))
    return u, v


def _point_in_mask(point: Point, mask) -> bool:
    x, y = point
    try:
        height, width = mask.shape[:2]
    except (AttributeError, TypeError, ValueError):
        return False
    if not (0 <= x < width and 0 <= y < height):
        return False
    return bool(mask[y, x])
