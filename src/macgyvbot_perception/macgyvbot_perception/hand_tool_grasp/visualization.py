"""Visualization helpers for hand grasp detection frames."""

from __future__ import annotations

from typing import Optional

import cv2

from macgyvbot_perception.hand_tool_grasp.tool_detector import (
    ToolDetection,
)
from macgyvbot_perception.hand_tool_grasp.calculations import draw_text
from macgyvbot_perception.hand_tool_grasp.sam_tool_mask import (
    LockedToolMask,
    overlay_locked_mask,
)


def draw_grasp_overlay(
    frame,
    hand_infos: list[dict],
    active_hand: Optional[dict],
    tool_detection: Optional[ToolDetection],
    result: dict,
    locked_tool: Optional[LockedToolMask] = None,
) -> None:
    """Draw tool, hand, and grasp-state overlays on a camera frame."""
    overlay_locked_mask(frame, locked_tool)

    if tool_detection is not None:
        x1, y1, x2, y2 = tool_detection.roi
        color = (0, 255, 0) if result["human_grasped_tool"] else (255, 120, 0)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        draw_text(
            frame,
            f"{tool_detection.label} {tool_detection.confidence:.2f}",
            (x1, max(24, y1 - 10)),
            color,
            scale=0.55,
        )

    active_hand_index = active_hand["hand_index"] if active_hand else None
    for hand_info in hand_infos:
        is_active = hand_info["hand_index"] == active_hand_index
        point_color = (0, 255, 255) if is_active else (255, 200, 0)
        for point in hand_info["landmarks"].values():
            cv2.circle(frame, point, 4 if is_active else 3, point_color, -1)

    draw_text(frame, f"State: {result['state']}", (20, 32))
    draw_text(
        frame,
        f"human_grasped_tool: {result['human_grasped_tool']}",
        (20, 62),
        scale=0.6,
    )
    draw_text(
        frame,
        f"depth_contact: {result['depth_contact_count']}",
        (20, 92),
        scale=0.6,
    )
    draw_text(
        frame,
        f"ml: {result.get('ml_stable_state', 'n/a')} raw={result.get('ml_raw_state', 'n/a')}",
        (20, 122),
        scale=0.6,
    )
    draw_text(
        frame,
        f"mask: {result.get('mask_source', 'NONE')} contact={result.get('mask_contact_count', 0)}",
        (20, 152),
        scale=0.6,
    )
    draw_text(
        frame,
        f"depth_ok: {result.get('depth_grasp_ok', False)}",
        (20, 182),
        scale=0.6,
    )
