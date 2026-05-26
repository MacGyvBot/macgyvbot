"""Visualization helpers for hand grasp detection frames."""

from __future__ import annotations

from typing import Optional

import cv2

from macgyvbot_config.return_flow import (
    RETURN_HAND_CLOSE_DEPTH_MAX_MM,
    RETURN_HAND_CLOSE_DEPTH_MIN_MM,
    RETURN_HAND_CLOSE_ROI_CENTER_X,
    RETURN_HAND_CLOSE_ROI_CENTER_Y,
    RETURN_HAND_CLOSE_ROI_HEIGHT_RATIO,
    RETURN_HAND_CLOSE_ROI_WIDTH_RATIO,
)
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
    draw_close_roi(frame)

    active_tool_roi = locked_tool.roi if locked_tool is not None else (
        tool_detection.roi if tool_detection is not None else None
    )
    if active_tool_roi is not None:
        label = (
            "locked tool ROI"
            if locked_tool is not None
            else f"{tool_detection.label} {tool_detection.confidence:.2f}"
        )
        color = (0, 255, 0) if result["human_grasped_tool"] else (255, 120, 0)
        draw_tool_roi(frame, active_tool_roi, label, color)

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


def draw_tool_roi(frame, roi, label, color):
    x1, y1, x2, y2 = [int(value) for value in roi]
    center = (int((x1 + x2) * 0.5), int((y1 + y2) * 0.5))
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
    cv2.drawMarker(
        frame,
        center,
        color,
        markerType=cv2.MARKER_CROSS,
        markerSize=16,
        thickness=2,
    )
    draw_text(
        frame,
        label,
        (x1, max(24, y1 - 10)),
        color,
        scale=0.55,
    )
    draw_text(
        frame,
        f"roi center=({center[0]},{center[1]})",
        (x1, min(frame.shape[0] - 10, y2 + 20)),
        color,
        scale=0.45,
    )


def draw_close_roi(frame):
    height, width = frame.shape[:2]
    center_u = int(width * RETURN_HAND_CLOSE_ROI_CENTER_X)
    center_v = int(height * RETURN_HAND_CLOSE_ROI_CENTER_Y)
    half_width = int(width * RETURN_HAND_CLOSE_ROI_WIDTH_RATIO * 0.5)
    half_height = int(height * RETURN_HAND_CLOSE_ROI_HEIGHT_RATIO * 0.5)
    x1 = max(0, center_u - half_width)
    y1 = max(0, center_v - half_height)
    x2 = min(width - 1, center_u + half_width)
    y2 = min(height - 1, center_v + half_height)
    color = (0, 220, 255)
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 1)
    cv2.drawMarker(
        frame,
        (center_u, center_v),
        color,
        markerType=cv2.MARKER_TILTED_CROSS,
        markerSize=14,
        thickness=1,
    )
    draw_text(
        frame,
        (
            "close ROI "
            f"{RETURN_HAND_CLOSE_DEPTH_MIN_MM / 10:.0f}-"
            f"{RETURN_HAND_CLOSE_DEPTH_MAX_MM / 10:.0f}cm"
        ),
        (x1, max(24, y1 - 8)),
        color,
        scale=0.45,
    )
