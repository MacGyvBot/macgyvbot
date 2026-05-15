from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)

from macgyvbot.util.hand_grasp_detection.hand_grasp.calculations import (
    point_to_rect_distance,
)

Rect = Tuple[int, int, int, int]


@dataclass(frozen=True)
class LockedToolMask:
    roi: Rect
    mask: np.ndarray
    source: str


@dataclass(frozen=True)
class MaskContactResult:
    mask_contact_count: int
    mask_contact_confirmed: bool
    mask_proximity_ok: bool
    min_landmark_to_tool_distance: Optional[float]

    @property
    def near_or_contact(self) -> bool:
        return bool(
            self.mask_contact_confirmed
            or self.mask_proximity_ok
            or self.mask_contact_count > 0
        )


class BBoxPromptSegmenter:
    """SAM-family bbox-prompt segmenter used only for tool-mask lock."""

    def __init__(
        self,
        backend: str,
        checkpoint_path: str,
        model_type: str,
        device: str,
        clip_margin: int = 8,
        open_kernel: int = 3,
        close_kernel: int = 5,
        keep_largest_component: bool = True,
    ) -> None:
        checkpoint = resolve_checkpoint_path(checkpoint_path)
        if not checkpoint.exists():
            raise RuntimeError(f"SAM checkpoint not found: {checkpoint}")

        if backend == "mobile_sam":
            from mobile_sam import SamPredictor, sam_model_registry
        elif backend == "sam":
            from segment_anything import SamPredictor, sam_model_registry
        else:
            raise RuntimeError(f"unsupported SAM backend: {backend}")

        sam = sam_model_registry[model_type](checkpoint=str(checkpoint))
        sam.to(device=device)
        self.predictor = SamPredictor(sam)
        self.clip_margin = clip_margin
        self.open_kernel = open_kernel
        self.close_kernel = close_kernel
        self.keep_largest_component = keep_largest_component

    def segment(self, frame: np.ndarray, roi: Rect) -> Optional[np.ndarray]:
        x1, y1, x2, y2 = roi
        if x2 <= x1 or y2 <= y1:
            return None

        self.predictor.set_image(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        box = np.array([x1, y1, x2, y2], dtype=np.float32)
        center = np.array([[(x1 + x2) / 2.0, (y1 + y2) / 2.0]], dtype=np.float32)
        labels = np.array([1], dtype=np.int32)
        masks, scores, _ = self.predictor.predict(
            point_coords=center,
            point_labels=labels,
            box=box,
            multimask_output=True,
        )
        if masks is None or len(masks) == 0:
            return None

        mask = self._select_mask(masks, scores, roi)
        return self._postprocess_mask(mask, roi, frame.shape[:2])

    def track_from_mask(
        self,
        frame: np.ndarray,
        previous_mask: LockedToolMask,
        margin: int = 12,
    ) -> Optional[LockedToolMask]:
        roi = mask_to_roi(previous_mask.mask)
        if roi is None:
            roi = previous_mask.roi

        tracked_roi = _expand_rect(roi, margin)
        mask = self.segment(frame, tracked_roi)
        if mask is None or int(mask.sum()) <= 0:
            return None

        tracked_mask_roi = mask_to_roi(mask) or tracked_roi
        return LockedToolMask(
            roi=tracked_mask_roi,
            mask=mask,
            source="SAM_TRACKED",
        )

    @staticmethod
    def _select_mask(masks, scores, roi: Rect) -> np.ndarray:
        x1, y1, x2, y2 = roi
        roi_area = max(1, (x2 - x1) * (y2 - y1))
        best_index = 0
        best_score = -1.0
        for index, mask in enumerate(masks):
            area = int(mask.sum())
            fill_ratio = area / roi_area
            score = float(scores[index]) - abs(fill_ratio - 0.35)
            if score > best_score:
                best_score = score
                best_index = index
        return masks[best_index].astype(np.uint8)

    def _postprocess_mask(
        self,
        mask: np.ndarray,
        roi: Rect,
        frame_shape: tuple[int, int],
    ) -> np.ndarray:
        height, width = frame_shape
        x1, y1, x2, y2 = _clip_rect(_expand_rect(roi, self.clip_margin), width, height)
        clipped = np.zeros((height, width), dtype=np.uint8)
        clipped[y1:y2, x1:x2] = mask[y1:y2, x1:x2]

        if self.open_kernel > 1:
            kernel = np.ones((self.open_kernel, self.open_kernel), np.uint8)
            clipped = cv2.morphologyEx(clipped, cv2.MORPH_OPEN, kernel)
        if self.close_kernel > 1:
            kernel = np.ones((self.close_kernel, self.close_kernel), np.uint8)
            clipped = cv2.morphologyEx(clipped, cv2.MORPH_CLOSE, kernel)
        if self.keep_largest_component:
            clipped = _largest_component(clipped)
        return clipped.astype(bool)


def create_bbox_locked_mask(roi: Rect, frame_shape: tuple[int, int]) -> LockedToolMask:
    height, width = frame_shape
    x1, y1, x2, y2 = _clip_rect(roi, width, height)
    mask = np.zeros((height, width), dtype=bool)
    mask[y1:y2, x1:x2] = True
    return LockedToolMask(roi=(x1, y1, x2, y2), mask=mask, source="BBOX_LOCKED")


def compute_mask_contact(
    hand_info: Optional[dict],
    locked_tool: Optional[LockedToolMask],
    contact_radius: int,
    min_contact_landmarks: int,
    proximity_threshold: float,
) -> MaskContactResult:
    if hand_info is None or locked_tool is None:
        return MaskContactResult(0, False, False, None)

    landmarks = hand_info["landmarks"]
    mask = locked_tool.mask
    height, width = mask.shape[:2]
    contact_count = 0
    min_distance = min(
        point_to_rect_distance(point, locked_tool.roi)
        for point in landmarks.values()
    )

    if contact_radius > 0:
        kernel = np.ones((contact_radius * 2 + 1, contact_radius * 2 + 1), np.uint8)
        contact_mask = cv2.dilate(mask.astype(np.uint8), kernel).astype(bool)
    else:
        contact_mask = mask

    for x, y in landmarks.values():
        if 0 <= x < width and 0 <= y < height and contact_mask[y, x]:
            contact_count += 1

    return MaskContactResult(
        mask_contact_count=contact_count,
        mask_contact_confirmed=contact_count >= min_contact_landmarks,
        mask_proximity_ok=min_distance <= proximity_threshold,
        min_landmark_to_tool_distance=min_distance,
    )


def overlay_locked_mask(frame: np.ndarray, locked_tool: Optional[LockedToolMask]) -> None:
    if locked_tool is None:
        return
    green = np.zeros_like(frame)
    green[:, :] = (0, 255, 0)
    mask = locked_tool.mask.astype(bool)
    frame[mask] = cv2.addWeighted(frame, 0.65, green, 0.35, 0)[mask]


def resolve_checkpoint_path(checkpoint_path: str) -> Path:
    path = Path(checkpoint_path).expanduser()
    if path.exists() or path.is_absolute():
        return path

    package_root = Path(__file__).resolve().parents[3]
    project_root = Path(__file__).resolve().parents[4]
    try:
        package_share = Path(get_package_share_directory("macgyvbot"))
    except PackageNotFoundError:
        package_share = None

    candidates = [
        *(
            (package_share / path, package_share / "weights" / path)
            if package_share is not None
            else ()
        ),
        Path.cwd() / path,
        Path.cwd() / "weights" / path,
        project_root / path,
        project_root / "weights" / path,
        package_root / path,
        package_root / "weights" / path,
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return path


def mask_to_roi(mask: np.ndarray) -> Optional[Rect]:
    ys, xs = np.where(mask.astype(bool))
    if xs.size == 0 or ys.size == 0:
        return None
    return (
        int(xs.min()),
        int(ys.min()),
        int(xs.max()) + 1,
        int(ys.max()) + 1,
    )


def _expand_rect(rect: Rect, margin: int) -> Rect:
    x1, y1, x2, y2 = rect
    return (x1 - margin, y1 - margin, x2 + margin, y2 + margin)


def _clip_rect(rect: Rect, width: int, height: int) -> Rect:
    x1, y1, x2, y2 = rect
    return (
        max(0, min(width, x1)),
        max(0, min(height, y1)),
        max(0, min(width, x2)),
        max(0, min(height, y2)),
    )


def _largest_component(mask: np.ndarray) -> np.ndarray:
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), 8)
    if count <= 1:
        return mask
    largest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return labels == largest
