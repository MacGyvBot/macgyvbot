"""Domain types for tool mask tracking and grasp contact checks."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional, Tuple

Rect = Tuple[int, int, int, int]


@dataclass(frozen=True)
class LockedToolMask:
    roi: Rect
    mask: Any
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


@dataclass(frozen=True)
class MaskTrackValidation:
    accepted: bool
    reason: str = ""
