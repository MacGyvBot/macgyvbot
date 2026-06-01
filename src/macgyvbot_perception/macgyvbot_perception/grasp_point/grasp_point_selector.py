"""Grasp point selection orchestration."""

from __future__ import annotations

from macgyvbot_config.vlm import (
    DEFAULT_GRASP_POINT_MODE,
    GRASP_POINT_MODE_API,
    GRASP_POINT_MODE_CENTER,
    GRASP_POINT_MODE_VLM,
    GRASP_POINT_MODE_VLM_ONLY_QWEN3B,
    GRASP_POINT_MODE_VLM_ONLY_QWEN7B,
    GRASP_POINT_MODE_VLM_ONLY_SMOL,
    VLM_ONLY_MODEL_BY_MODE,
    VLM_ONLY_MODES,
)
from macgyvbot_perception.grasp_point.center_method.selector import (
    CenterGraspPointSelector,
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
        sam_enabled=True,
        sam_checkpoint="",
        sam_backend="mobile_sam",
        sam_model_type="vit_t",
        sam_device="cuda",
    ):
        self.mode = mode
        self.logger = logger
        self.api_model = api_model
        self.api_env_file = api_env_file
        self.api_base_url = api_base_url
        self.api_timeout_sec = api_timeout_sec
        self.sam_enabled = sam_enabled
        self.sam_checkpoint = sam_checkpoint
        self.sam_backend = sam_backend
        self.sam_model_type = sam_model_type
        self.sam_device = sam_device
        self.vlm_grasp_point_selector = None
        self.vlm_only_grasp_point_selector = None
        self.api_grasp_point_selector = None
        self.center_grasp_point_selector = CenterGraspPointSelector(logger)

    def preload_vlm_if_needed(self):
        if not self._uses_vlm():
            return

        if (
            self.mode in VLM_ONLY_MODES
            or (
                self.mode == GRASP_POINT_MODE_API
                and DEFAULT_GRASP_POINT_MODE in VLM_ONLY_MODES
            )
        ):
            self._preload_vlm_only()
            return

        try:
            from macgyvbot_perception.grasp_point.vlm_grasp_point_selector import (
                VLMGraspPointSelector,
            )
        except ImportError as exc:
            self.logger.warn(f"VLM grasp module import failed: {exc}")
            return

        if self.vlm_grasp_point_selector is None:
            self.vlm_grasp_point_selector = VLMGraspPointSelector(
                self.logger,
                sam_enabled=self.sam_enabled,
                sam_checkpoint=self.sam_checkpoint,
                sam_backend=self.sam_backend,
                sam_model_type=self.sam_model_type,
                sam_device=self.sam_device,
            )

        try:
            self.vlm_grasp_point_selector.preload()
        except Exception as exc:
            self.logger.warn(f"VLM preload failed: {exc}")

    def _uses_vlm(self):
        return (
            self.mode == GRASP_POINT_MODE_VLM
            or self.mode in VLM_ONLY_MODES
            or (
                self.mode == GRASP_POINT_MODE_API
                and DEFAULT_GRASP_POINT_MODE
                in (GRASP_POINT_MODE_VLM, *VLM_ONLY_MODES)
            )
        )

    def should_defer_vlm_until_top_view(self):
        return (
            self.mode == GRASP_POINT_MODE_CENTER
            or self.mode == GRASP_POINT_MODE_VLM
            or self.mode in VLM_ONLY_MODES
        )

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

        if self.mode in VLM_ONLY_MODES:
            vlm_only_pixel = self._select_vlm_only_grasp_pixel(
                bbox,
                label,
                color_image,
                depth_image,
                intrinsics,
                target_label,
            )
            if vlm_only_pixel is not None:
                return vlm_only_pixel

            self.logger.warn(
                "VLM-only grasp point failed. Falling back to bbox center."
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
                "API grasp point 선택 실패. "
                f"기본 모드({DEFAULT_GRASP_POINT_MODE})로 대체합니다."
            )
            default_pixel = self._select_default_grasp_pixel(
                bbox,
                label,
                color_image,
                depth_image,
                intrinsics,
                target_label,
            )
            if default_pixel is not None:
                return default_pixel

            self.logger.warn(
                "기본 grasp point 모드도 실패했습니다. bbox center로 대체합니다."
            )

        return self._select_bbox_center_pixel(
            bbox,
            label,
            color_image,
            target_label,
        )

    def _select_bbox_center_pixel(
        self,
        bbox,
        label=None,
        color_image=None,
        target_label=None,
    ):
        """Return the center pixel of an object bounding box."""
        return self.center_grasp_point_selector.select_grasp_pixel(
            bbox,
            label=label,
            color_image=color_image,
            target_label=target_label,
        )

    def select_bbox_center(self, box):
        bbox = box.xyxy[0].cpu().numpy()
        return self._select_bbox_center_pixel(bbox)

    def _select_default_grasp_pixel(
        self,
        bbox,
        label,
        color_image,
        depth_image,
        intrinsics,
        target_label,
    ):
        if DEFAULT_GRASP_POINT_MODE == GRASP_POINT_MODE_VLM:
            return self._select_vlm_grasp_pixel(
                bbox,
                label,
                color_image,
                depth_image,
                intrinsics,
                target_label,
            )

        if DEFAULT_GRASP_POINT_MODE in VLM_ONLY_MODES:
            return self._select_vlm_only_grasp_pixel(
                bbox,
                label,
                color_image,
                depth_image,
                intrinsics,
                target_label,
            )

        if DEFAULT_GRASP_POINT_MODE == GRASP_POINT_MODE_CENTER:
            return self._select_bbox_center_pixel(
                bbox,
                label,
                color_image,
                target_label,
            )

        return None

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
            self.vlm_grasp_point_selector = VLMGraspPointSelector(
                self.logger,
                sam_enabled=self.sam_enabled,
                sam_checkpoint=self.sam_checkpoint,
                sam_backend=self.sam_backend,
                sam_model_type=self.sam_model_type,
                sam_device=self.sam_device,
            )

        return self.vlm_grasp_point_selector.select_grasp_pixel(
            bbox,
            label,
            color_image,
            depth_image,
            intrinsics,
            target_label,
        )

    def _preload_vlm_only(self):
        try:
            from macgyvbot_perception.grasp_point.vlm_only_grasp_point_selector import (
                VLMOnlyGraspPointSelector,
            )
        except ImportError as exc:
            self.logger.warn(f"VLM-only grasp module import failed: {exc}")
            return

        if self.vlm_only_grasp_point_selector is None:
            self.vlm_only_grasp_point_selector = VLMOnlyGraspPointSelector(
                self.logger,
                sam_enabled=self.sam_enabled,
                sam_checkpoint=self.sam_checkpoint,
                sam_backend=self.sam_backend,
                sam_model_type=self.sam_model_type,
                sam_device=self.sam_device,
                model_id=self._vlm_only_model_id(),
                mode=self._vlm_only_mode(),
            )

        try:
            self.vlm_only_grasp_point_selector.preload()
        except Exception as exc:
            self.logger.warn(f"VLM-only preload failed: {exc}")

    def _select_vlm_only_grasp_pixel(
        self,
        bbox,
        label,
        color_image,
        depth_image,
        intrinsics,
        target_label,
    ):
        try:
            from macgyvbot_perception.grasp_point.vlm_only_grasp_point_selector import (
                VLMOnlyGraspPointSelector,
            )
        except ImportError as exc:
            self.logger.warn(f"VLM-only grasp module import failed: {exc}")
            return None

        if self.vlm_only_grasp_point_selector is None:
            self.vlm_only_grasp_point_selector = VLMOnlyGraspPointSelector(
                self.logger,
                sam_enabled=self.sam_enabled,
                sam_checkpoint=self.sam_checkpoint,
                sam_backend=self.sam_backend,
                sam_model_type=self.sam_model_type,
                sam_device=self.sam_device,
                model_id=self._vlm_only_model_id(),
                mode=self._vlm_only_mode(),
            )

        return self.vlm_only_grasp_point_selector.select_grasp_pixel(
            bbox,
            label,
            color_image,
            depth_image,
            intrinsics,
            target_label,
        )

    def _vlm_only_model_id(self):
        mode = self._vlm_only_mode()
        return VLM_ONLY_MODEL_BY_MODE.get(
            mode,
            VLM_ONLY_MODEL_BY_MODE[GRASP_POINT_MODE_VLM_ONLY_QWEN3B],
        )

    def _vlm_only_mode(self):
        if self.mode == GRASP_POINT_MODE_API:
            return DEFAULT_GRASP_POINT_MODE
        return self.mode

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
            from macgyvbot_perception.grasp_point.api_grasp_point_selector import (
                APIGraspPointSelector,
            )
        except ImportError as exc:
            self.logger.warn(f"API grasp 모듈 import 실패: {exc}")
            return None

        if self.api_grasp_point_selector is None:
            self.api_grasp_point_selector = APIGraspPointSelector(
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
        *VLM_ONLY_MODES,
        GRASP_POINT_MODE_API,
    ):
        return mode

    logger.warn(
        f"Unknown grasp_point_mode '{mode}'. "
        f"Falling back to '{GRASP_POINT_MODE_CENTER}'."
    )
    return GRASP_POINT_MODE_CENTER
