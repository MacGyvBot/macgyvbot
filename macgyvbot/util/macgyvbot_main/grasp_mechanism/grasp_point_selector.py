"""Grasp point selection orchestration."""

from __future__ import annotations

from macgyvbot.config.config import (
    GRASP_POINT_MODE_CENTER,
    GRASP_POINT_MODE_VLA,
    GRASP_POINT_MODE_VLM,
)


class GraspPointSelector:
    """Select an image-space grasp point from detector output."""

    def __init__(self, mode, logger):
        self.mode = mode
        self.logger = logger
        self.vlm_grasp_mechanism = None
        self.vla_grasp_mechanism = None

    def prepare_runtime_mode(self):
        """Prepare mode-specific runtime dependencies and return active mode."""
        if self.mode != GRASP_POINT_MODE_VLA:
            return self.mode

        self.logger.info(
            "VLA 모드가 선택되어 시작 시점에 VLA 가중치 로드를 확인합니다."
        )
        if self.ensure_vla_grasp_ready() is not None:
            return self.mode

        self.logger.warn(
            "VLA 가중치 로드에 실패하여 grasp_point_mode를 "
            f"'{GRASP_POINT_MODE_CENTER}'로 fallback합니다."
        )
        self.mode = GRASP_POINT_MODE_CENTER
        return self.mode

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

        if self.mode == GRASP_POINT_MODE_VLA:
            return self._select_vla_seed_pixel(bbox)

        return self._select_bbox_center_pixel(bbox)

    @staticmethod
    def _select_bbox_center_pixel(bbox):
        """Return the center pixel of an object bounding box."""
        u = int((bbox[0] + bbox[2]) / 2)
        v = int((bbox[1] + bbox[3]) / 2)
        return u, v, GRASP_POINT_MODE_CENTER, None

    @staticmethod
    def _select_vla_seed_pixel(bbox):
        """Return the seed pixel used before VLA final-pose refinement."""
        u = int((bbox[0] + bbox[2]) / 2)
        v = int((bbox[1] + bbox[3]) / 2)
        return u, v, GRASP_POINT_MODE_VLA, None

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
            from macgyvbot.util.macgyvbot_main.grasp_mechanism.grasp_by_vlm import (
                VLMGraspMechanism,
            )
        except ImportError as exc:
            self.logger.warn(f"VLM grasp 모듈 import 실패: {exc}")
            return None

        if self.vlm_grasp_mechanism is None:
            self.vlm_grasp_mechanism = VLMGraspMechanism(self.logger)

        return self.vlm_grasp_mechanism.select_grasp_pixel(
            bbox,
            label,
            color_image,
            depth_image,
            intrinsics,
            target_label,
        )

    def ensure_vla_grasp_ready(self):
        mechanism = self.get_vla_grasp_mechanism()
        if mechanism is None:
            return None
        return mechanism.ensure_model_loaded()

    def get_vla_grasp_mechanism(self):
        if self.vla_grasp_mechanism is not None:
            return self.vla_grasp_mechanism

        try:
            from macgyvbot.util.macgyvbot_main.grasp_mechanism.grasp_by_vla import (
                VLAGraspMechanism,
            )
        except ImportError as exc:
            self.logger.warn(f"VLA grasp 모듈 import 실패: {exc}")
            return None

        self.vla_grasp_mechanism = VLAGraspMechanism(self.logger)
        return self.vla_grasp_mechanism

    def refine_grasp_pose_with_vla(
        self,
        robot,
        *,
        label,
        bbox,
        object_xyz,
        switch_z,
        color_image,
        task_instruction=None,
    ):
        mechanism = self.get_vla_grasp_mechanism()
        if mechanism is None:
            return None

        return mechanism.propose_grasp_pose(
            robot,
            label=label,
            bbox=bbox,
            object_xyz=object_xyz,
            switch_z=switch_z,
            color_image=color_image,
            task_instruction=task_instruction,
        )


def normalize_grasp_point_mode(mode, logger):
    """Return a supported grasp point mode, falling back to center."""
    if mode in (GRASP_POINT_MODE_CENTER, GRASP_POINT_MODE_VLM, GRASP_POINT_MODE_VLA):
        return mode

    logger.warn(
        f"알 수 없는 grasp_point_mode '{mode}'. "
        f"'{GRASP_POINT_MODE_CENTER}'로 대체합니다."
    )
    return GRASP_POINT_MODE_CENTER
