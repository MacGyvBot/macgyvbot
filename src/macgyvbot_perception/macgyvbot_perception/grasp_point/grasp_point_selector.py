"""Grasp point selection orchestration."""

from __future__ import annotations

from macgyvbot_config.vlm import (
    GRASP_POINT_MODE_CENTER,
    GRASP_POINT_MODE_VLM,
)


class GraspPointSelector:
    """Select an image-space grasp point from detector output."""

    def __init__(self, mode, logger):
        self.mode = mode
        self.logger = logger
        self.vlm_grasp_point_selector = None

    def select(
        self,
        box,
        label,
        color_image,
        depth_image,
        intrinsics,
        target_label,
    ):
        bbox = box.xyxy[0].cpu().numpy()

        if self.mode == GRASP_POINT_MODE_VLM:
            vlm_pixel = self._select_vlm_grasp_pixel(
                bbox,
                label,
                color_image,
                depth_image,
                intrinsics,
                target_label,
            )
            if vlm_pixel is not None:
                return vlm_pixel

            self.logger.warn("VLM grasp point 실패. box 중심점으로 대체합니다.")

        return self._select_bbox_center_pixel(bbox)

    @staticmethod
    def _select_bbox_center_pixel(bbox):
        """Return the center pixel of an object bounding box."""
        u = int((bbox[0] + bbox[2]) / 2)
        v = int((bbox[1] + bbox[3]) / 2)
        return u, v, GRASP_POINT_MODE_CENTER, None

    def _select_vlm_grasp_pixel(
        self,
        bbox,
        label,
        color_image,
        depth_image,
        intrinsics,
        target_label,
    ):
        try:
            from macgyvbot_perception.grasp_point.vlm_grasp_point_selector import (
                VLMGraspPointSelector,
            )
        except ImportError as exc:
            self.logger.warn(f"VLM grasp 모듈 import 실패: {exc}")
            return None

        if self.vlm_grasp_point_selector is None:
            self.vlm_grasp_point_selector = VLMGraspPointSelector(self.logger)

        return self.vlm_grasp_point_selector.select_grasp_pixel(
            bbox,
            label,
            color_image,
            depth_image,
            intrinsics,
            target_label,
        )


def normalize_grasp_point_mode(mode, logger):
    """Return a supported grasp point mode, falling back to center."""
    if mode in (GRASP_POINT_MODE_CENTER, GRASP_POINT_MODE_VLM):
        return mode

    logger.warn(
        f"알 수 없는 grasp_point_mode '{mode}'. "
        f"'{GRASP_POINT_MODE_CENTER}'로 대체합니다."
    )
    return GRASP_POINT_MODE_CENTER
