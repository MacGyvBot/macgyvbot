"""Close-range ROI/depth policy for receiving returned tools."""

from __future__ import annotations

from dataclasses import dataclass

from macgyvbot_config.return_flow import (
    RETURN_HAND_CLOSE_DEPTH_MAX_MM,
    RETURN_HAND_CLOSE_DEPTH_MIN_MM,
    RETURN_HAND_CLOSE_ROI_CENTER_X,
    RETURN_HAND_CLOSE_ROI_CENTER_Y,
    RETURN_HAND_CLOSE_ROI_HEIGHT_RATIO,
    RETURN_HAND_CLOSE_ROI_WIDTH_RATIO,
)


@dataclass(frozen=True)
class ReturnClosePolicy:
    """Check whether a detected tool is close enough to grasp."""

    center_x_ratio: float = RETURN_HAND_CLOSE_ROI_CENTER_X
    center_y_ratio: float = RETURN_HAND_CLOSE_ROI_CENTER_Y
    width_ratio: float = RETURN_HAND_CLOSE_ROI_WIDTH_RATIO
    height_ratio: float = RETURN_HAND_CLOSE_ROI_HEIGHT_RATIO
    min_depth_mm: float = RETURN_HAND_CLOSE_DEPTH_MIN_MM
    max_depth_mm: float = RETURN_HAND_CLOSE_DEPTH_MAX_MM

    def matches(self, image_shape, tool_roi, tool_depth_mm) -> bool:
        return self.roi_in_close_region(
            image_shape,
            tool_roi,
        ) and self.depth_in_close_range(tool_depth_mm)

    def roi_in_close_region(self, image_shape, tool_roi) -> bool:
        if image_shape is None:
            return False
        if not isinstance(tool_roi, (list, tuple)) or len(tool_roi) != 4:
            return False

        try:
            height, width = image_shape[:2]
        except (TypeError, ValueError):
            return False

        if width <= 0 or height <= 0:
            return False

        try:
            x1, y1, x2, y2 = [float(value) for value in tool_roi]
        except (TypeError, ValueError):
            return False

        center_u = (x1 + x2) * 0.5
        center_v = (y1 + y2) * 0.5
        roi_center_u = width * self.center_x_ratio
        roi_center_v = height * self.center_y_ratio
        half_width = width * self.width_ratio * 0.5
        half_height = height * self.height_ratio * 0.5

        return (
            abs(center_u - roi_center_u) <= half_width
            and abs(center_v - roi_center_v) <= half_height
        )

    def depth_in_close_range(self, tool_depth_mm) -> bool:
        try:
            depth_mm = float(tool_depth_mm)
        except (TypeError, ValueError):
            return False

        return self.min_depth_mm <= depth_mm <= self.max_depth_mm
