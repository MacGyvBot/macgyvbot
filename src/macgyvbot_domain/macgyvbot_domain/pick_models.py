"""Domain types for pick motion planning."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class PickMotionPlan:
    """Resolved pick motion waypoints and derived heights."""

    target_x: float
    target_y: float
    travel_z: float
    approach_z: float
    grasp_z: float
    current_x: float
    current_y: float
    corrected_bz: float
    should_descend_to_grasp: bool
