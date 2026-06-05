"""Domain types for pick motion planning."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class PickMotionPlan:
    """Resolved pick motion waypoints and derived heights."""

    target_x: float
    target_y: float
    drawer_wall_clearance_z: float
    grasp_z: float
    should_descend_to_grasp: bool
