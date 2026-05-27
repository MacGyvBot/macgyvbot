"""Single-call VLM grasp point selection.

This selector keeps the fast path intentionally simple: it sends one
SAM-augmented crop to the local VLM and asks for x, y, and yaw only.
"""

from __future__ import annotations

import math
from pathlib import Path

import cv2
from PIL import Image

from macgyvbot_config.models import HAND_GRASP_SAM_CHECKPOINT_NAME
from macgyvbot_config.vlm import GRASP_POINT_MODE_VLM_ONLY, VLM_MODEL_SMOL
from macgyvbot_perception.grasp_point.vlm_grasp_point_selector import (
    VLMModel,
)
from macgyvbot_perception.hand_tool_grasp.sam_tool_mask import (
    BBoxPromptSegmenter,
    LockedToolMask,
    overlay_locked_mask,
)


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
        model_id=VLM_MODEL_SMOL,
    ):
        self.logger = logger
        self.model = None
        self.model_id = model_id
        self.sam_enabled = bool(sam_enabled)
        self.sam_checkpoint = sam_checkpoint or str(
            Path("weights") / HAND_GRASP_SAM_CHECKPOINT_NAME
        )
        self.sam_backend = sam_backend
        self.sam_model_type = sam_model_type
        self.sam_device = sam_device
        self.sam_segmenter = None
        self.sam_unavailable = False

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

        vlm_image, sam_source = self._build_vlm_input_image(
            color_image,
            (x1, y1, x2, y2),
        )
        crop_bgr = vlm_image[y1:y2, x1:x2]
        crop_rgb = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)
        crop_image = Image.fromarray(crop_rgb)
        self.logger.info(
            "VLM_ONLY stage=input_ready "
            f"bbox=({x1},{y1},{x2},{y2}) "
            f"crop_size={crop_image.size} "
            f"sam_input={sam_source or 'none'}"
        )

        self.logger.info("VLM_ONLY stage=generate_start event=inference start")
        try:
            result = self.model.ask(
                crop_image,
                self._build_prompt(
                    label=label,
                    target_label=target_label,
                    image_size=crop_image.size,
                    sam_source=sam_source,
                ),
            )
        except Exception as exc:
            self.logger.warn(f"VLM-only grasp point inference failed: {exc}")
            return None
        finally:
            self.logger.info("VLM_ONLY stage=generate_done event=inference end")

        data = self.model._as_mapping(result.data)
        point = self.model._extract_point_px(
            data,
            result.text,
            crop_image.size[0],
            crop_image.size[1],
        )
        yaw = self.model._extract_yaw_deg(data, result.text)

        if point is None or yaw is None:
            self.logger.warn(
                "VLM-only response did not contain valid x_px, y_px, yaw_deg: "
                f"{result.text}"
            )
            return None

        u = x1 + int(round(point[0]))
        v = y1 + int(round(point[1]))
        yaw = self.model._normalize_angle_deg(float(yaw))
        source = GRASP_POINT_MODE_VLM_ONLY

        refined = VLMModel.refine_grasp_point_with_depth(
            depth_image,
            (u, v),
            focal_px=(intrinsics["fx"] + intrinsics["fy"]) / 2.0,
        )
        if refined is not None:
            u, v = refined.point
            source = f"{GRASP_POINT_MODE_VLM_ONLY}+depth"

        self.logger.info(
            "VLM_ONLY stage=result "
            f"pixel=({u},{v}) "
            f"yaw_deg={yaw:.1f} "
            f"source={source} "
            f"sam_input={sam_source or 'none'}"
        )
        return u, v, source, (0.0, 0.0, yaw)

    @staticmethod
    def _build_prompt(label, target_label, image_size, sam_source):
        width, height = image_size
        mask_instruction = ""
        if sam_source is not None:
            mask_instruction = (
                "- The green translucent overlay is the SAM-segmented tool mask. "
                "Choose a grasp point inside that mask.\n"
            )

        return (
            "Choose one grasp center and wrist yaw for a two-finger parallel "
            "robot gripper.\n"
            "Return strict compact JSON only with keys: x_px, y_px, yaw_deg, confidence, reason.\n"
            f"Image size: width={width}, height={height}.\n"
            "Yaw definition:\n"
            "- yaw_deg is the additional robot wrist rotation from the current pose.\n"
            "- Positive yaw_deg means rotate the wrist left/counterclockwise in the image.\n"
            "- Negative yaw_deg means rotate the wrist right/clockwise in the image.\n"
            "- For long tools, rotate until the two gripper fingers close across the tool width.\n"
            "- If a long tool axis is about 45 degrees in the image, yaw_deg should be about 45 or -135.\n"
            "- If a long tool axis is about -45 degrees in the image, yaw_deg should be about -45 or 135.\n"
            "Constraints:\n"
            f"- x_px must be an integer in [0, {width - 1}]\n"
            f"- y_px must be an integer in [0, {height - 1}]\n"
            "- yaw_deg must be in [-180, 180]\n"
            "- choose a sturdy graspable region, not a sharp tip, blade, hole, or edge\n"
            "- use the visible object only; do not choose background\n"
            "- reason must be 8 words or fewer\n"
            f"{mask_instruction}"
            "- no markdown, no code fence, no explanation outside JSON\n\n"
            f"Detected object label: {label}\n"
            f"Robot task target: {target_label}\n"
        )

    def _build_vlm_input_image(self, color_image, roi):
        segmenter = self._ensure_sam_segmenter()
        if segmenter is None:
            return color_image, None

        try:
            mask = segmenter.segment(color_image, roi)
        except Exception as exc:
            self.logger.warn(f"SAM mask for VLM-only input failed: {exc}")
            return color_image, None

        if mask is None or int(mask.sum()) <= 0:
            self.logger.warn("SAM mask for VLM-only input is empty. Using plain RGB crop.")
            return color_image, None

        masked_image = color_image.copy()
        overlay_locked_mask(
            masked_image,
            LockedToolMask(roi=roi, mask=mask, source="SAM_VLM_ONLY_INPUT"),
        )
        return masked_image, "SAM_VLM_ONLY_INPUT"

    def _ensure_sam_segmenter(self):
        if not self.sam_enabled or self.sam_unavailable:
            return None
        if self.sam_segmenter is not None:
            return self.sam_segmenter

        try:
            self.sam_segmenter = BBoxPromptSegmenter(
                backend=self.sam_backend,
                checkpoint_path=self.sam_checkpoint,
                model_type=self.sam_model_type,
                device=self.sam_device,
            )
        except Exception as exc:
            self.sam_unavailable = True
            self.logger.warn(
                f"SAM VLM-only input disabled; segmenter init failed: {exc}"
            )
            return None

        self.logger.info(
            "SAM VLM-only input enabled: "
            f"backend={self.sam_backend}, checkpoint={self.sam_checkpoint}"
        )
        return self.sam_segmenter

    def _ensure_model_loaded(self):
        if self.model is not None:
            return

        self.logger.info(
            "VLM_ONLY stage=model_init "
            f"model_id={self.model_id} "
            "max_new_tokens=48"
        )
        self.model = VLMModel(model_id=self.model_id, max_new_tokens=48)
        runtime = self.model.get_runtime_info()
        self.logger.info(
            "VLM_ONLY stage=model_config "
            f"device={runtime['device']}, "
            f"dtype={runtime['dtype']}, "
            f"local_weights={runtime['using_local_weights']}, "
            f"source={runtime['model_source']}, "
            f"device_map={runtime['device_map']}"
        )
        self.logger.info("VLM_ONLY stage=model_load_start")
        self.model.load()
        runtime = self.model.get_runtime_info()
        self.logger.info(
            "VLM_ONLY stage=model_load_done "
            f"device_map={runtime['device_map']}"
        )

    @staticmethod
    def clamp_bbox_to_image(bbox, image):
        height, width = image.shape[:2]
        x1 = max(0, min(width - 1, int(math.floor(bbox[0]))))
        y1 = max(0, min(height - 1, int(math.floor(bbox[1]))))
        x2 = max(0, min(width, int(math.ceil(bbox[2]))))
        y2 = max(0, min(height, int(math.ceil(bbox[3]))))
        return x1, y1, x2, y2
