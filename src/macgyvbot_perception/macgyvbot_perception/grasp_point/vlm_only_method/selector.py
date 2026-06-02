"""Single-call VLM grasp point selector."""

from __future__ import annotations

import math

from PIL import Image

from macgyvbot_config.vlm import (
    GRASP_POINT_MODE_VLM_ONLY_QWEN3B,
    VLM_INFERENCE_HISTORY_DIR,
    VLM_INFERENCE_HISTORY_ENABLED,
    VLM_MODEL_QWEN3B,
)
from macgyvbot_perception.grasp_point.mask_image_for_grasp_detection import (
    GraspDetectionRecordConfig,
    GraspDetectionRecorder,
)
from macgyvbot_perception.grasp_point.vlm.models import VLMOnly
from macgyvbot_perception.grasp_point.vlm.parser import Parser
from macgyvbot_perception.grasp_point.vlm_only_method.prompts import build_prompt


class VLMOnlyGraspPointSelector:
    """Select grasp pixel and yaw with exactly one VLM generation call."""

    def __init__(
        self,
        logger,
        sam_enabled=True,
        sam_checkpoint="",
        sam_backend="mobile_sam",
        sam_model_type="vit_t",
        sam_device="cuda",
        model_id=VLM_MODEL_QWEN3B,
        mode=GRASP_POINT_MODE_VLM_ONLY_QWEN3B,
        history_enabled=VLM_INFERENCE_HISTORY_ENABLED,
        history_dir=VLM_INFERENCE_HISTORY_DIR,
    ):
        self.logger = logger
        self.model_id = model_id
        self.mode = mode
        self.model = None
        self.parser = Parser()
        self.history = GraspDetectionRecorder(
            GraspDetectionRecordConfig(enabled=history_enabled, root_dir=history_dir),
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
            self.logger.warn("VLM-only crop bbox is empty.")
            return None

        self._ensure_model_loaded()
        import cv2

        crop_bgr = color_image[y1:y2, x1:x2]
        crop_rgb = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)
        crop_image = Image.fromarray(crop_rgb).copy()
        frame_image = Image.fromarray(cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)).copy()

        result = None
        try:
            prompt = self._build_prompt(
                label=label,
                target_label=target_label,
                image_size=crop_image.size,
                sam_source=None,
            )
            result = self.model.inference(crop_image, prompt)
            parsed = self.parser.parse_vlm_only_result(
                result,
                crop_image.size[0],
                crop_image.size[1],
            )
        except Exception as exc:
            self.history.record(
                image=crop_image,
                frame_image=frame_image,
                mode=self.mode,
                model_id=self.model_id,
                target_label=target_label,
                detected_label=label,
                bbox_xyxy=(x1, y1, x2, y2),
                raw_response=result.text if result is not None else "",
                success=False,
                error=str(exc),
            )
            self.logger.warn(f"VLM-only grasp point inference failed: {exc}")
            return None

        if parsed is None:
            raw_text = result.text if result is not None else ""
            self.history.record(
                image=crop_image,
                frame_image=frame_image,
                mode=self.mode,
                model_id=self.model_id,
                target_label=target_label,
                detected_label=label,
                bbox_xyxy=(x1, y1, x2, y2),
                raw_response=raw_text,
                success=False,
                error="invalid VLM-only response",
            )
            self.logger.warn(
                "VLM-only response did not contain valid x_px, y_px, yaw_deg: "
                f"{raw_text}"
            )
            return None

        point, yaw = parsed
        u = x1 + int(round(point[0]))
        v = y1 + int(round(point[1]))
        orientation = (0.0, 0.0, yaw)
        self.history.record(
            image=crop_image,
            frame_image=frame_image,
            mode=self.mode,
            model_id=self.model_id,
            target_label=target_label,
            detected_label=label,
            bbox_xyxy=(x1, y1, x2, y2),
            raw_response=result.text,
            parsed_point=(u, v),
            yaw_deg=yaw,
            orientation_rpy_deg=orientation,
            success=True,
        )

        self.logger.info(
            f"VLM-only grasp point selected: pixel=({u}, {v}), "
            f"yaw={yaw:.1f}deg, source={self.mode}"
        )
        return u, v, self.mode, orientation

    @staticmethod
    def _build_prompt(label, target_label, image_size, sam_source=None):
        return build_prompt(label=label, target_label=target_label, image_size=image_size)

    def _ensure_model_loaded(self):
        if self.model is not None:
            return

        self.logger.info(
            f"VLM-only grasp model lazy load preparing: model_id={self.model_id}"
        )
        self.model = VLMOnly(
            model_id=self.model_id,
            max_new_tokens=96,
            logger=self.logger,
        )
        runtime = self.model.get_runtime_info()
        self.logger.info(
            "VLM-only runtime: "
            f"device={runtime['device']}, "
            f"dtype={runtime['dtype']}, "
            f"local_weights={runtime['using_local_weights']}, "
            f"source={runtime['model_source']}"
        )
        self.model.load()
        self.logger.info("VLM-only weights loaded.")

    @staticmethod
    def clamp_bbox_to_image(bbox, image):
        height, width = image.shape[:2]
        x1 = max(0, min(width - 1, int(math.floor(bbox[0]))))
        y1 = max(0, min(height - 1, int(math.floor(bbox[1]))))
        x2 = max(0, min(width, int(math.ceil(bbox[2]))))
        y2 = max(0, min(height, int(math.ceil(bbox[3]))))
        return x1, y1, x2, y2
