"""Visualization helpers for hand grasp detection frames."""

from __future__ import annotations

from typing import Optional, TypedDict

import cv2
import numpy as np

from macgyvbot_config.return_flow import (
    RETURN_HAND_CLOSE_DEPTH_MAX_MM,
    RETURN_HAND_CLOSE_DEPTH_MIN_MM,
    RETURN_HAND_CLOSE_ROI_CENTER_X,
    RETURN_HAND_CLOSE_ROI_CENTER_Y,
    RETURN_HAND_CLOSE_ROI_HEIGHT_RATIO,
    RETURN_HAND_CLOSE_ROI_WIDTH_RATIO,
)
from macgyvbot_domain.mask_models import LockedToolMask
from macgyvbot_perception.hand_tool_grasp.tool_detector import (
    ToolDetection,
)
from macgyvbot_perception.hand_tool_grasp.calculations import (
    draw_text,
    median_depth_in_rect,
)
from macgyvbot_perception.hand_tool_grasp.sam_tool_mask import (
    overlay_locked_mask,
)

Point = tuple[int, int]
Rect = tuple[int, int, int, int]


class ReturnCloseDepthStatus(TypedDict):
    center: Point
    color: tuple[int, int, int]
    message: str


def draw_grasp_overlay(
    frame: np.ndarray,
    hand_infos: list[dict],
    active_hand: Optional[dict],
    tool_detection: Optional[ToolDetection],
    result: dict,
    locked_tool: Optional[LockedToolMask] = None,
    candidate_tool_mask: Optional[LockedToolMask] = None,
    show_close_roi: bool = False,
    depth_mm: Optional[np.ndarray] = None,
) -> None:
    """Backward-compatible wrapper for pick handoff visualization."""
    draw_pick_overlay(
        frame=frame,
        hand_infos=hand_infos,
        active_hand=active_hand,
        tool_detection=tool_detection,
        result=result,
        locked_tool=locked_tool,
        candidate_tool_mask=candidate_tool_mask,
    )


def draw_pick_overlay(
    frame: np.ndarray,
    hand_infos: list[dict],
    active_hand: Optional[dict],
    tool_detection: Optional[ToolDetection],
    result: dict,
    locked_tool: Optional[LockedToolMask] = None,
    candidate_tool_mask: Optional[LockedToolMask] = None,
) -> None:
    """Draw pick handoff state with SAM mask tracking/lock emphasis."""
    active_mask = locked_tool or candidate_tool_mask
    active_tool_roi = active_mask.roi if active_mask is not None else (
        tool_detection.roi if tool_detection is not None else None
    )

    if candidate_tool_mask is not None and locked_tool is None:
        overlay_locked_mask(frame, candidate_tool_mask, color=(255, 220, 0), alpha=0.28)
    if locked_tool is not None:
        overlay_locked_mask(frame, locked_tool, color=(0, 255, 0), alpha=0.42)

    if active_tool_roi is not None:
        label = (
            f"locked mask {locked_tool.source}"
            if locked_tool is not None
            else f"tracking mask {candidate_tool_mask.source}"
            if candidate_tool_mask is not None
            else f"{tool_detection.label} {tool_detection.confidence:.2f}"
        )
        color = mask_roi_color(locked_tool, candidate_tool_mask, result)
        draw_tool_roi(frame, active_tool_roi, label, color)

    draw_hands(frame, hand_infos, active_hand)
    draw_grasp_status_text(frame, result, locked_tool, candidate_tool_mask)


def draw_return_overlay(
    frame: np.ndarray,
    hand_infos: list[dict],
    active_hand: Optional[dict],
    tool_detection: Optional[ToolDetection],
    result: dict,
    depth_mm: Optional[np.ndarray] = None,
) -> None:
    """Draw return handoff state with close ROI/depth emphasis."""
    active_roi = (
        tool_detection.roi
        if tool_detection is not None
        else active_hand.get("hand_rect") if active_hand is not None else None
    )
    draw_close_roi(frame, active_roi, depth_mm)

    if active_roi is not None:
        label = (
            f"{tool_detection.label} {tool_detection.confidence:.2f}"
            if tool_detection is not None
            else "active hand ROI"
        )
        color = (255, 120, 0) if tool_detection is not None else (0, 220, 255)
        draw_tool_roi(frame, active_roi, label, color)

    draw_hands(frame, hand_infos, active_hand)
    draw_return_status_text(frame, result)


def draw_hands(
    frame: np.ndarray,
    hand_infos: list[dict],
    active_hand: Optional[dict],
) -> None:
    active_hand_index = active_hand["hand_index"] if active_hand else None
    for hand_info in hand_infos:
        is_active = hand_info["hand_index"] == active_hand_index
        point_color = (0, 255, 255) if is_active else (255, 200, 0)
        for point in hand_info["landmarks"].values():
            cv2.circle(frame, point, 4 if is_active else 3, point_color, -1)


def draw_grasp_status_text(
    frame: np.ndarray,
    result: dict,
    locked_tool: Optional[LockedToolMask],
    candidate_tool_mask: Optional[LockedToolMask],
) -> None:
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
        (
            f"mask: {mask_status_text(locked_tool, candidate_tool_mask)} "
            f"contact={result.get('mask_contact_count', 0)}"
        ),
        (20, 152),
        scale=0.6,
    )
    draw_text(
        frame,
        f"depth_ok: {result.get('depth_grasp_ok', False)}",
        (20, 182),
        scale=0.6,
    )


def draw_return_status_text(frame: np.ndarray, result: dict) -> None:
    draw_text(frame, f"Return State: {result['state']}", (20, 32))
    draw_text(
        frame,
        f"hand_present: {result.get('hand_present', False)}",
        (20, 62),
        scale=0.6,
    )
    draw_text(
        frame,
        f"roi_depth: {format_depth_mm(result.get('tool_depth_mm'))}",
        (20, 92),
        scale=0.6,
    )
    draw_text(
        frame,
        f"depth_available: {result.get('depth_available', False)}",
        (20, 122),
        scale=0.6,
    )


def format_depth_mm(depth_mm: Optional[float]) -> str:
    if depth_mm is None:
        return "n/a"
    return f"{depth_mm:.0f}mm"


def mask_roi_color(
    locked_tool: Optional[LockedToolMask],
    candidate_tool_mask: Optional[LockedToolMask],
    result: dict,
    active_hand: Optional[dict] = None,
) -> tuple[int, int, int]:
    if locked_tool is not None:
        return (0, 255, 0) if result["human_grasped_tool"] else (0, 180, 255)
    if candidate_tool_mask is not None:
        return (255, 220, 0)
    if active_hand is not None:
        return (0, 220, 255)
    return (255, 120, 0)


def mask_status_text(
    locked_tool: Optional[LockedToolMask],
    candidate_tool_mask: Optional[LockedToolMask],
) -> str:
    if locked_tool is not None:
        return f"LOCKED/{locked_tool.source}"
    if candidate_tool_mask is not None:
        return f"TRACKING/{candidate_tool_mask.source}"
    return "NONE"


def draw_tool_roi(
    frame: np.ndarray,
    roi: Rect,
    label: str,
    color: tuple[int, int, int],
) -> None:
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


def draw_close_roi(
    frame: np.ndarray,
    tool_roi: Optional[Rect] = None,
    depth_mm: Optional[np.ndarray] = None,
) -> None:
    height, width = frame.shape[:2]
    close_roi = close_roi_rect(width, height)
    x1, y1, x2, y2 = close_roi
    status = close_roi_depth_status(close_roi, tool_roi, depth_mm)
    color = status["color"]
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 1)
    cv2.drawMarker(
        frame,
        status["center"],
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
    draw_text(
        frame,
        status["message"],
        (x1, min(frame.shape[0] - 12, y1 + 22)),
        color,
        scale=0.45,
    )


def close_roi_rect(width: int, height: int) -> Rect:
    center_u = int(width * RETURN_HAND_CLOSE_ROI_CENTER_X)
    center_v = int(height * RETURN_HAND_CLOSE_ROI_CENTER_Y)
    half_width = int(width * RETURN_HAND_CLOSE_ROI_WIDTH_RATIO * 0.5)
    half_height = int(height * RETURN_HAND_CLOSE_ROI_HEIGHT_RATIO * 0.5)
    return (
        max(0, center_u - half_width),
        max(0, center_v - half_height),
        min(width - 1, center_u + half_width),
        min(height - 1, center_v + half_height),
    )


def close_roi_depth_status(
    close_roi: Rect,
    tool_roi: Optional[Rect],
    depth_mm: Optional[np.ndarray],
) -> ReturnCloseDepthStatus:
    x1, y1, x2, y2 = close_roi
    center = (int((x1 + x2) * 0.5), int((y1 + y2) * 0.5))
    if tool_roi is None:
        return {
            "center": center,
            "color": (0, 220, 255),
            "message": "return close: no tool ROI",
        }

    tool_center = rect_center(tool_roi)
    if not point_in_rect(tool_center, close_roi):
        return {
            "center": center,
            "color": (0, 220, 255),
            "message": "return close: move tool into ROI",
        }

    if depth_mm is None:
        return {
            "center": center,
            "color": (80, 180, 255),
            "message": "return close: depth unavailable",
        }

    tool_depth = median_depth_in_rect(depth_mm, tool_roi)
    if tool_depth is None:
        return {
            "center": center,
            "color": (80, 180, 255),
            "message": "return close: no valid depth",
        }

    if tool_depth < RETURN_HAND_CLOSE_DEPTH_MIN_MM:
        return {
            "center": center,
            "color": (0, 140, 255),
            "message": f"return close: too close {tool_depth:.0f}mm",
        }
    if tool_depth > RETURN_HAND_CLOSE_DEPTH_MAX_MM:
        return {
            "center": center,
            "color": (0, 0, 255),
            "message": f"return close: too far {tool_depth:.0f}mm",
        }

    return {
        "center": center,
        "color": (0, 255, 0),
        "message": f"return close: depth ok {tool_depth:.0f}mm",
    }


def rect_center(rect: Rect) -> Point:
    x1, y1, x2, y2 = [int(value) for value in rect]
    return (int((x1 + x2) * 0.5), int((y1 + y2) * 0.5))


def point_in_rect(point: Point, rect: Rect) -> bool:
    x, y = point
    x1, y1, x2, y2 = rect
    return x1 <= x <= x2 and y1 <= y <= y2
