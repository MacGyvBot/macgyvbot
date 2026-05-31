"""Target acquisition helpers for pick workflow."""

from __future__ import annotations

from macgyvbot_domain.target_models import PickTarget


class PickTargetResolver:
    """Locate a target label using YOLO, grasp point selection, and depth."""

    def __init__(
        self,
        detector,
        grasp_point_selector,
        depth_projector,
        logger,
    ):
        self.detector = detector
        self.grasp_point_selector = grasp_point_selector
        self.depth_projector = depth_projector
        self.logger = logger

    def target_from_boxes(
        self,
        boxes,
        target_label,
        color_image,
        depth_image,
        intrinsics,
        use_bbox_center=False,
    ) -> PickTarget:
        """Build a projected target from already computed YOLO boxes."""
        if boxes is None:
            return self._not_found(target_label, "detector_boxes_unavailable")

        matched_box = self.matching_box(boxes, target_label)
        if matched_box is None:
            return self._not_found(target_label, "target_not_found")

        box, label = matched_box
        if use_bbox_center:
            selected = self.grasp_point_selector.select_bbox_center(box)
        else:
            selected = self.grasp_point_selector.select(
                box,
                label,
                color_image,
                depth_image,
                intrinsics,
                target_label,
            )

        return self.target_from_selected_grasp(
            label,
            target_label,
            selected,
            depth_image,
            intrinsics,
        )

    def matching_box(self, boxes, target_label):
        if boxes is None:
            return None

        for box in boxes:
            label = self._box_label(box)
            if label == target_label:
                return box, label
        return None

    def target_from_selected_grasp(
        self,
        label,
        target_label,
        selected,
        depth_image,
        intrinsics,
    ) -> PickTarget:
        if selected is None:
            return self._not_found(target_label, "grasp_selection_failed")

        u, v, source, vlm_rpy_deg = selected
        target = self.depth_projector.pixel_to_base_target(
            u,
            v,
            label,
            source,
            depth_image,
            intrinsics,
            self.logger,
            vlm_rpy_deg,
        )
        if target is None:
            return self._not_found(target_label, "depth_projection_failed")

        bx, by, bz, z_m, vlm_rpy_deg = target
        return PickTarget(
            found=True,
            label=label,
            pixel=(u, v),
            base_xyz=(bx, by, bz),
            depth_m=z_m,
            yaw_deg=self._extract_yaw(vlm_rpy_deg),
            source=source,
        )

    def should_defer_vlm_until_top_view(self):
        return self.grasp_point_selector.should_defer_vlm_until_top_view()

    @staticmethod
    def _not_found(label, reason) -> PickTarget:
        return PickTarget(
            found=False,
            label=label,
            pixel=None,
            base_xyz=None,
            depth_m=None,
            yaw_deg=None,
            reason=reason,
        )

    def _box_label(self, box):
        try:
            class_id = int(box.cls[0])
        except (TypeError, IndexError):
            class_id = int(box.cls)
        return str(self.detector.names[class_id])

    @staticmethod
    def _extract_yaw(vlm_rpy_deg):
        if vlm_rpy_deg is None or len(vlm_rpy_deg) < 3:
            return None
        try:
            return float(vlm_rpy_deg[2])
        except (TypeError, ValueError):
            return None
