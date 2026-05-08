"""Grasp point selection from object detections."""

import math

import cv2

from macgyvbot.core.config import (
    GRASP_POINT_MODE_CENTER,
    GRASP_POINT_MODE_GRASPNET,
    GRASP_POINT_MODE_VLM,
    VLM_GRASP_GRID_SIZES,
)


class GraspPointSelector:
    def __init__(self, mode, logger):
        self.mode = mode
        self.logger = logger
        self.vlm_grasp_model = None

    def select_grasp_pixel(
        self,
        box,
        label,
        color_image,
        depth_image,
        intrinsics,
        target_label,
    ):
        bbox = box.xyxy[0].cpu().numpy()

        if self.mode == GRASP_POINT_MODE_GRASPNET:
            return self.select_box_center_pixel(bbox)

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

        return self.select_box_center_pixel(bbox)

    @staticmethod
    def select_box_center_pixel(bbox):
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
        x1, y1, x2, y2 = self.clamp_bbox_to_image(bbox, color_image)

        if x2 <= x1 or y2 <= y1:
            self.logger.warn("VLM crop bbox가 비어 있습니다.")
            return None

        try:
            from PIL import Image as PILImage

            from macgyvbot.grasp_point_detection import VLMModel
        except ImportError as exc:
            self.logger.warn(f"VLM grasp 모듈 import 실패: {exc}")
            return None

        if self.vlm_grasp_model is None:
            self.logger.info("VLM grasp 모델을 lazy load 준비합니다.")
            self.vlm_grasp_model = VLMModel()
            runtime = self.vlm_grasp_model.get_runtime_info()
            self.logger.info(
                "VLM runtime: "
                f"device={runtime['device']}, "
                f"dtype={runtime['dtype']}, "
                f"local_weights={runtime['using_local_weights']}, "
                f"source={runtime['model_source']}"
            )
            if runtime["device"] == "cuda":
                self.logger.info("VLM은 CUDA를 사용합니다.")
            else:
                self.logger.warn("VLM이 CUDA를 사용하지 않습니다. (CPU 실행)")
            self.logger.info("VLM 가중치 로드 시작...")
            self.vlm_grasp_model.load()
            self.logger.info("VLM 가중치 로드 완료.")

        crop_bgr = color_image[y1:y2, x1:x2]
        crop_rgb = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)
        crop_image = PILImage.fromarray(crop_rgb)

        try:
            result = self.vlm_grasp_model.select_grasp_region(
                crop_image,
                object_label=label,
                user_request=target_label,
                grid_sizes=VLM_GRASP_GRID_SIZES,
            )
        except Exception as exc:
            self.logger.warn(f"VLM grasp point 추론 실패: {exc}")
            return None

        u = x1 + int(round(result.point[0]))
        v = y1 + int(round(result.point[1]))
        source = GRASP_POINT_MODE_VLM

        refined = VLMModel.refine_grasp_point_with_depth(
            depth_image,
            (u, v),
            focal_px=(intrinsics["fx"] + intrinsics["fy"]) / 2.0,
        )
        if refined is not None:
            u, v = refined.point
            source = f"{GRASP_POINT_MODE_VLM}+depth"

        self.logger.info(
            f"VLM grasp point 선택: pixel=({u}, {v}), "
            f"angle={result.angle_deg:.1f}deg, "
            f"rpy_deg={result.orientation_rpy_deg}, source={source}"
        )

        return u, v, source, result.orientation_rpy_deg

    @staticmethod
    def clamp_bbox_to_image(bbox, image):
        height, width = image.shape[:2]
        x1 = max(0, min(width - 1, int(math.floor(bbox[0]))))
        y1 = max(0, min(height - 1, int(math.floor(bbox[1]))))
        x2 = max(0, min(width, int(math.ceil(bbox[2]))))
        y2 = max(0, min(height, int(math.ceil(bbox[3]))))
        return x1, y1, x2, y2
