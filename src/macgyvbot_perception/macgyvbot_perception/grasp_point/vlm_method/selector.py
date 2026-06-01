"""Grid-based VLM grasp point selector."""

from __future__ import annotations

import math

from PIL import Image

from macgyvbot_config.vlm import (
    GRASP_POINT_MODE_VLM,
    VLM_GRASP_GRID_SIZES,
    VLM_INFERENCE_HISTORY_DIR,
    VLM_INFERENCE_HISTORY_ENABLED,
)
from macgyvbot_domain.logging import exception_log_fields
from macgyvbot_perception.grasp_point.vlm.inference_history_recode import (
    InferenceHistoryConfig,
    InferenceHistoryRecode,
)
from macgyvbot_perception.grasp_point.vlm.models import VLM
from macgyvbot_perception.grasp_point.vlm.parser import Parser
from macgyvbot_perception.grasp_point.vlm_method import prompts
from macgyvbot_perception.grasp_point.vlm_method.grid import GridPolicy


class VLMGraspPointSelector:
    """Select grasp pixels using grid-based VLM reasoning."""

    def __init__(
        self,
        logger,
        sam_enabled=True,
        sam_checkpoint="",
        sam_backend="mobile_sam",
        sam_model_type="vit_t",
        sam_device="cuda",
        history_enabled=VLM_INFERENCE_HISTORY_ENABLED,
        history_dir=VLM_INFERENCE_HISTORY_DIR,
    ):
        self.logger = logger
        self.model = None
        self.parser = Parser()
        self.grid_policy = GridPolicy()
        self.history = InferenceHistoryRecode(
            InferenceHistoryConfig(enabled=history_enabled, root_dir=history_dir),
            logger=logger,
        )

    def preload(self):
        self._ensure_model_loaded()

    def select_grasp_pixel(
        self,
        bbox,
        label,
        color_image,
        depth_image,
        intrinsics,
        target_label,
    ):
        x1, y1, x2, y2 = self.clamp_bbox_to_image(bbox, color_image)
        if x2 <= x1 or y2 <= y1:
            self.logger.warn(
                "crop",
                "fail",
                pipe="vlm",
                target=target_label,
                detected_label=label,
                reason="empty_bbox",
                mode=GRASP_POINT_MODE_VLM,
            )
            return None

        self._ensure_model_loaded()
        import cv2

        crop_bgr = color_image[y1:y2, x1:x2]
        crop_rgb = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)
        crop_image = Image.fromarray(crop_rgb).copy()
        frame_image = Image.fromarray(cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)).copy()
        result = None

        try:
            result = self.model.inference(
                crop_image,
                prompts=prompts,
                parser=self.parser,
                grid_policy=self.grid_policy,
                grid_sizes=VLM_GRASP_GRID_SIZES,
                object_label=label,
                user_request=target_label,
            )
        except Exception as exc:
            self.history.record(
                image=crop_image,
                frame_image=frame_image,
                mode=GRASP_POINT_MODE_VLM,
                model_id=self.model.model_id if self.model is not None else "",
                target_label=target_label,
                detected_label=label,
                bbox_xyxy=(x1, y1, x2, y2),
                raw_response=result.raw_text if result is not None else "",
                success=False,
                error=str(exc),
            )
            self.logger.warn(
                "inference",
                "fail",
                pipe="vlm",
                target=target_label,
                detected_label=label,
                mode=GRASP_POINT_MODE_VLM,
                **exception_log_fields(exc),
            )
            return None

        u = x1 + int(round(result.point[0]))
        v = y1 + int(round(result.point[1]))
        self.history.record(
            image=crop_image,
            frame_image=frame_image,
            mode=GRASP_POINT_MODE_VLM,
            model_id=self.model.model_id,
            target_label=target_label,
            detected_label=label,
            bbox_xyxy=(x1, y1, x2, y2),
            raw_response=result.raw_text,
            parsed_point=(u, v),
            yaw_deg=result.angle_deg,
            orientation_rpy_deg=result.orientation_rpy_deg,
            success=True,
        )

        self.logger.info(
            "selection",
            "done",
            pipe="vlm",
            target=target_label,
            detected_label=label,
            mode=GRASP_POINT_MODE_VLM,
            u=u,
            v=v,
            angle_deg=f"{result.angle_deg:.1f}",
            rpy_deg=result.orientation_rpy_deg,
        )
        return u, v, GRASP_POINT_MODE_VLM, result.orientation_rpy_deg

    def _ensure_model_loaded(self):
        if self.model is not None:
            return

        self.logger.info(
            "model_load",
            "start",
            pipe="vlm",
            mode=GRASP_POINT_MODE_VLM,
        )
        self.model = VLM(logger=self.logger)
        runtime = self.model.get_runtime_info()
        self.logger.info(
            "runtime",
            "status",
            pipe="vlm",
            mode=GRASP_POINT_MODE_VLM,
            device=runtime["device"],
            dtype=runtime["dtype"],
            local_weights=runtime["using_local_weights"],
            source=runtime["model_source"],
        )
        if runtime["device"] != "cuda":
            self.logger.warn(
                "runtime",
                "fallback",
                pipe="vlm",
                mode=GRASP_POINT_MODE_VLM,
                reason="cuda_unavailable",
                device=runtime["device"],
            )
        self.model.load()
        self.logger.info(
            "model_load",
            "done",
            pipe="vlm",
            mode=GRASP_POINT_MODE_VLM,
        )

    @staticmethod
    def clamp_bbox_to_image(bbox, image):
        height, width = image.shape[:2]
        x1 = max(0, min(width - 1, int(math.floor(bbox[0]))))
        y1 = max(0, min(height - 1, int(math.floor(bbox[1]))))
        x2 = max(0, min(width, int(math.ceil(bbox[2]))))
        y2 = max(0, min(height, int(math.ceil(bbox[3]))))
        return x1, y1, x2, y2
