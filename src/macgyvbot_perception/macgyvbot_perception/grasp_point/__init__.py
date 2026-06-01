"""Grasp point selection mechanisms."""

from macgyvbot_perception.grasp_point.mask_image_for_grasp_detection import (
    generate_mask_image_for_grasp_detection,
    generate_sam_depth_mask_image_for_grasp_detection,
)

__all__ = [
    "generate_mask_image_for_grasp_detection",
    "generate_sam_depth_mask_image_for_grasp_detection",
]
