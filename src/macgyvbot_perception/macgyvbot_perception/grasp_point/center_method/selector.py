"""Bounding-box center grasp point selector."""

from __future__ import annotations

import math

from PIL import Image

from macgyvbot_config.vlm import (
    GRASP_POINT_MODE_CENTER,
    PCA_YAW_SAM_DEFAULT_CONFIG,
    VLM_INFERENCE_HISTORY_DIR,
    VLM_INFERENCE_HISTORY_ENABLED,
)
from macgyvbot_perception.grasp_point.grasp_method.yaw_estimation import (
    SamPcaYawRefiner,
)
from macgyvbot_perception.grasp_point.vlm.inference_history_recode import (
    InferenceHistoryConfig,
    InferenceHistoryRecode,
)


class CenterGraspPointSelector:
    """Select the grasp point at the center of a detector bounding box."""

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
        self.yaw_refiner = SamPcaYawRefiner(
            logger,
            sam_enabled=sam_enabled,
            sam_checkpoint=sam_checkpoint,
            sam_backend=sam_backend,
            sam_model_type=sam_model_type,
            sam_device=sam_device,
            config=dict(PCA_YAW_SAM_DEFAULT_CONFIG),
        )
        self.history = InferenceHistoryRecode(
            InferenceHistoryConfig(enabled=history_enabled, root_dir=history_dir),
            logger=logger,
        )

    def select_grasp_pixel(
        self,
        bbox,
        label=None,
        color_image=None,
        target_label=None,
    ):
        """Return the center pixel and optionally record the selection."""
        u = int((bbox[0] + bbox[2]) / 2)
        v = int((bbox[1] + bbox[3]) / 2)
        yaw_deg = None
        orientation = None
        if color_image is not None:
            x1, y1, x2, y2 = self._clamp_bbox_to_image(bbox, color_image)
            if x2 > x1 and y2 > y1:
                crop_bgr = color_image[y1:y2, x1:x2]
                grasp_result = {
                    "x_px": u - x1,
                    "y_px": v - y1,
                    "yaw_deg": None,
                    "confidence": None,
                    "reason": "bbox_center",
                }
                final_grasp_result, yaw_debug = self.yaw_refiner.refine(
                    grasp_result,
                    crop_bgr,
                )
                yaw_deg = float(final_grasp_result["yaw_deg"])
                orientation = (0.0, 0.0, yaw_deg)
                self.logger.info(
                    "Center yaw refinement: "
                    f"final_yaw={yaw_deg:.1f}deg, "
                    f"sam_success={yaw_debug.get('sam_success')}, "
                    f"aggregation_success={yaw_debug.get('aggregation_success')}, "
                    f"pca_success={yaw_debug.get('pca_success')}, "
                    f"fallback_used={yaw_debug.get('fallback_used')}, "
                    f"fallback_source={yaw_debug.get('fallback_source')}, "
                    f"invert_yaw_sign={yaw_debug.get('invert_yaw_sign_applied')}"
                )
        self._record_selection(
            bbox,
            label,
            color_image,
            target_label,
            parsed_point=(u, v),
            yaw_deg=yaw_deg,
            orientation_rpy_deg=orientation,
        )
        return u, v, GRASP_POINT_MODE_CENTER, orientation

    def _record_selection(
        self,
        bbox,
        label,
        color_image,
        target_label,
        parsed_point,
        yaw_deg,
        orientation_rpy_deg,
    ):
        if color_image is None:
            return

        try:
            x1, y1, x2, y2 = self._clamp_bbox_to_image(bbox, color_image)
            if x2 <= x1 or y2 <= y1:
                return

            crop_bgr = color_image[y1:y2, x1:x2]
            crop_image = Image.fromarray(self._bgr_to_rgb(crop_bgr)).copy()
            frame_image = Image.fromarray(self._bgr_to_rgb(color_image)).copy()
            self.history.record(
                image=crop_image,
                frame_image=frame_image,
                mode=GRASP_POINT_MODE_CENTER,
                model_id="",
                target_label=target_label or label or "",
                detected_label=label or "",
                bbox_xyxy=(x1, y1, x2, y2),
                raw_response="bbox_center",
                parsed_point=parsed_point,
                yaw_deg=yaw_deg,
                orientation_rpy_deg=orientation_rpy_deg,
                success=True,
            )
        except Exception as exc:
            self.logger.warn(f"bbox center history recording failed: {exc}")

    @staticmethod
    def _clamp_bbox_to_image(bbox, image):
        height, width = image.shape[:2]
        x1 = max(0, min(width - 1, int(math.floor(bbox[0]))))
        y1 = max(0, min(height - 1, int(math.floor(bbox[1]))))
        x2 = max(0, min(width, int(math.ceil(bbox[2]))))
        y2 = max(0, min(height, int(math.ceil(bbox[3]))))
        return x1, y1, x2, y2

    @staticmethod
    def _bgr_to_rgb(image):
        if len(image.shape) < 3 or image.shape[2] < 3:
            return image.copy()
        return image[:, :, ::-1].copy()
