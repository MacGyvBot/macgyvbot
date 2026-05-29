"""Compatibility imports for the grid-based VLM grasp selector."""

from macgyvbot_perception.grasp_point.vlm.models import (
    GridChoice,
    GraspRegionResult,
    VLM,
    VLMOnly,
    VLMResult,
)
from macgyvbot_perception.grasp_point.vlm_method.selector import VLMGraspPointSelector

VLMModel = VLMOnly

__all__ = [
    "GridChoice",
    "GraspRegionResult",
    "VLM",
    "VLMOnly",
    "VLMModel",
    "VLMResult",
    "VLMGraspPointSelector",
]
