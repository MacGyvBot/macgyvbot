"""API-based grasp point selector."""

from __future__ import annotations

import math

from PIL import Image

from macgyvbot_config.vlm import (
    GRASP_POINT_MODE_API,
    VLM_INFERENCE_HISTORY_DIR,
    VLM_INFERENCE_HISTORY_ENABLED,
)
from macgyvbot_perception.grasp_point.api_method.client import GeminiGraspAPIClient
from macgyvbot_perception.grasp_point.mask_image_for_grasp_detection import (
    GraspDetectionRecordConfig,
    GraspDetectionRecorder,
)


class APIGraspPointSelector:
    """Select grasp pixels using one Gemini API prompt."""

    def __init__(
        self,
        logger,
        model: str | None = None,
        env_file: str | None = None,
        base_url: str | None = None,
        timeout_sec: float = 30.0,
        history_enabled=VLM_INFERENCE_HISTORY_ENABLED,
        history_dir=VLM_INFERENCE_HISTORY_DIR,
    ):
        self.logger = logger
        self.client = GeminiGraspAPIClient(
            model_id=model,
            env_file=env_file,
            base_url=base_url,
            timeout_sec=timeout_sec,
        )
        self.history = GraspDetectionRecorder(
            GraspDetectionRecordConfig(enabled=history_enabled, root_dir=history_dir),
            logger=logger,
        )

    def select_grasp_pixel(
        self,
        bbox,
        label,
        color_image,
        depth_image,
        intrinsics,
        target_label,
    ):
        x1, y1, x2, y2 = self._clamp_bbox_to_image(bbox, color_image)
        if x2 <= x1 or y2 <= y1:
            self.logger.warn("API grasp crop bbox is empty.")
            return None

        import cv2

        crop_bgr = color_image[y1:y2, x1:x2]
        crop_rgb = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)
        crop_image = Image.fromarray(crop_rgb).copy()
        frame_image = Image.fromarray(cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)).copy()

        try:
            result = self.client.select_grasp_pose(
                crop_image,
                object_label=label,
                user_request=target_label,
                bbox_xyxy=(x1, y1, x2, y2),
            )
        except Exception as exc:
            self.history.record(
                image=crop_image,
                frame_image=frame_image,
                mode=GRASP_POINT_MODE_API,
                model_id=self.client.model_id,
                target_label=target_label,
                detected_label=label,
                bbox_xyxy=(x1, y1, x2, y2),
                success=False,
                error=str(exc),
            )
            self.logger.warn(f"API grasp point inference failed: {exc}")
            return None

        u = x1 + int(round(result.point[0]))
        v = y1 + int(round(result.point[1]))
        self.history.record(
            image=crop_image,
            frame_image=frame_image,
            mode=GRASP_POINT_MODE_API,
            model_id=self.client.model_id,
            target_label=target_label,
            detected_label=label,
            bbox_xyxy=(x1, y1, x2, y2),
            raw_response=result.raw_text,
            parsed_point=(u, v),
            yaw_deg=result.orientation_rpy_deg[2],
            orientation_rpy_deg=result.orientation_rpy_deg,
            success=True,
        )
        self.logger.info(
            f"API grasp point selected: pixel=({u}, {v}), "
            f"rpy_deg={result.orientation_rpy_deg}, "
            f"confidence={result.confidence}, source={GRASP_POINT_MODE_API}"
        )
        return u, v, GRASP_POINT_MODE_API, result.orientation_rpy_deg

    @staticmethod
    def _clamp_bbox_to_image(bbox, image):
        height, width = image.shape[:2]
        x1 = max(0, min(width - 1, int(math.floor(bbox[0]))))
        y1 = max(0, min(height - 1, int(math.floor(bbox[1]))))
        x2 = max(0, min(width, int(math.ceil(bbox[2]))))
        y2 = max(0, min(height, int(math.ceil(bbox[3]))))
        return x1, y1, x2, y2
