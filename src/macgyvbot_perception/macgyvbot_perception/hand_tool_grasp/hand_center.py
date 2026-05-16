"""Helpers for deriving a stable hand-center pixel from hand landmarks."""

from __future__ import annotations

from typing import Optional


def extract_hand_center_pixel(active_hand: Optional[dict], hand_infos: list[dict]):
    """Return a robust hand-center pixel; fallback to landmark centroid."""
    hand = active_hand if active_hand is not None else (hand_infos[0] if hand_infos else None)
    if hand is None:
        return None

    palm_center = hand.get("palm_center")
    if isinstance(palm_center, tuple) and len(palm_center) == 2:
        return {
            "u": int(palm_center[0]),
            "v": int(palm_center[1]),
            "source": "palm_center",
        }

    landmarks = hand.get("landmarks", {})
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
