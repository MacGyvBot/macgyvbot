"""Domain types for object target acquisition."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class PickTarget:
    """Result of locating a target object and projecting a pick target."""

    found: bool
    label: str
    pixel: Optional[tuple[int, int]]
    base_xyz: Optional[tuple[float, float, float]]
    depth_m: Optional[float]
    yaw_deg: Optional[float]
    reason: str = ""
    source: str = ""
