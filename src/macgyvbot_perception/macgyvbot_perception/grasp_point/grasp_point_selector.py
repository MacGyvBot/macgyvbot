"""Grasp point selection orchestration."""

from __future__ import annotations

from macgyvbot_config.vlm import (
    GRASP_POINT_MODE_API,
    GRASP_POINT_MODE_CENTER,
    GRASP_POINT_MODE_VLM,
)


class GraspPointSelector:
    """Select an image-space grasp point from detector output."""

    def __init__(
        self,
        mode,
        logger,
        api_model="",
        api_env_file="",
        api_base_url="",
        api_timeout_sec=30.0,
    ):
        self.mode = mode
        self.logger = logger
        self.api_model = api_model
        self.api_env_file = api_env_file
        self.api_base_url = api_base_url
        self.api_timeout_sec = api_timeout_sec
        self.vlm_grasp_point_selector = None
        self.api_grasp_point_selector = None

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

            self.logger.warn(
                "VLM grasp point failed. Falling back to bbox center."
            )

        if self.mode == GRASP_POINT_MODE_API:
            api_pixel = self._select_api_grasp_pixel(
                bbox,
                label,
                color_image,
                depth_image,
                intrinsics,
                target_label,
            )
            if api_pixel is not None:
                return api_pixel

            self.logger.warn(
                "API VLM grasp point failed. Falling back to bbox center."
            )

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
            self.logger.warn(f"VLM grasp module import failed: {exc}")
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

    def _select_api_grasp_pixel(
        self,
        bbox,
        label,
        color_image,
        depth_image,
        intrinsics,
        target_label,
    ):
        try:
            from macgyvbot_perception.grasp_point.api_vlm_grasp_point_selector import (
                APIVLMGraspPointSelector,
            )
        except ImportError as exc:
            self.logger.warn(f"API VLM grasp module import failed: {exc}")
            return None

        if self.api_grasp_point_selector is None:
            self.api_grasp_point_selector = APIVLMGraspPointSelector(
                self.logger,
                model=self.api_model or None,
                env_file=self.api_env_file or None,
                base_url=self.api_base_url or None,
                timeout_sec=self.api_timeout_sec,
            )

        return self.api_grasp_point_selector.select_grasp_pixel(
            bbox,
            label,
            color_image,
            depth_image,
            intrinsics,
            target_label,
        )


def normalize_grasp_point_mode(mode, logger):
    """Return a supported grasp point mode, falling back to center."""
    if mode in (
        GRASP_POINT_MODE_CENTER,
        GRASP_POINT_MODE_VLM,
        GRASP_POINT_MODE_API,
    ):
        return mode

    logger.warn(
        f"Unknown grasp_point_mode '{mode}'. "
        f"Falling back to '{GRASP_POINT_MODE_CENTER}'."
    )
    return GRASP_POINT_MODE_CENTER
