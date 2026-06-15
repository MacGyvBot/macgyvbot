"""Perception callbacks used by the return drawer-store flow."""

from __future__ import annotations

from collections import Counter

from macgyvbot_config.drawer import (
    DRAWER_ARUCO_MARKER_IDS,
    DRAWER_OCCUPANCY_SAMPLE_COUNT,
    DRAWER_OCCUPANCY_SAMPLE_INTERVAL_SEC,
)
from macgyvbot_perception.drawer_marker_resolver import DrawerMarkerResolver
from macgyvbot_task.application.logging_utils import log_error, log_warn


class ReturnPerceptionAdapter:
    """Resolve staged tool labels, bbox targets, and drawer marker targets."""

    def __init__(
        self,
        state,
        detector,
        drawer_flow,
        frame_processor,
        pick_target_resolver,
        depth_projector,
        logger,
        wait_fn=None,
        refine_store_tool_target=None,
    ):
        self.state = state
        self.detector = detector
        self.drawer_flow = drawer_flow
        self.frame_processor = frame_processor
        self.pick_target_resolver = pick_target_resolver
        self.drawer_marker_resolver = DrawerMarkerResolver(
            depth_projector,
            logger,
        )
        self.logger = logger
        self.wait_fn = wait_fn or (lambda _duration: None)
        self.refine_store_tool_target = refine_store_tool_target or (
            lambda target, _tool_name: target
        )

    def detect_store_tool_label(self):
        labels = self.detect_drawer_tool_labels()
        if labels is None:
            return None
        return labels[0] if labels else None

    def detect_drawer_tool_labels(self):
        votes = []
        for sample_index in range(DRAWER_OCCUPANCY_SAMPLE_COUNT):
            labels = self._detect_drawer_tool_labels_once()
            if labels is None:
                return None
            votes.append(tuple(sorted(labels)))
            if sample_index < DRAWER_OCCUPANCY_SAMPLE_COUNT - 1:
                self.wait_fn(DRAWER_OCCUPANCY_SAMPLE_INTERVAL_SEC)

        selected_labels, vote_count = Counter(votes).most_common(1)[0]
        self.logger.info(
            "drawer occupancy yolo vote "
            f"samples={votes}, selected={selected_labels}, votes={vote_count}"
        )
        return list(selected_labels)

    def _detect_drawer_tool_labels_once(self):
        if self.state.color_image is None:
            return None

        results = self.detector.detect(self.state.color_image)
        boxes = results[0].boxes if results else []
        if boxes is None:
            boxes = []

        supported = self.drawer_flow.supported_tool_labels()
        labels = []
        for box in boxes:
            label = self._box_label(box)
            if label in supported and label not in labels:
                labels.append(label)
        return labels

    def resolve_store_tool_target(self, tool_name):
        if not self.frame_processor.has_camera_state():
            log_warn(
                self.logger,
                "store tool target camera state unavailable",
                step="store_tool_target",
                event="fail",
                tool=tool_name,
                reason="camera_state_unavailable",
            )
            return None

        color_image = self.state.color_image.copy()
        depth_image = self.state.depth_image.copy()
        intrinsics = dict(self.state.intrinsics)
        results = self.detector.detect(color_image)
        boxes = results[0].boxes if results else None

        matched_box = self.pick_target_resolver.matching_box(boxes, tool_name)
        if matched_box is None:
            return self._resolve_store_tool_center_target(
                boxes,
                tool_name,
                color_image,
                depth_image,
                intrinsics,
            )

        box, label = matched_box
        selected = (
            self.pick_target_resolver.grasp_point_selector.select_yolo_grasp_point(
                boxes,
                self.detector.names,
                box,
            )
        )
        if selected is None:
            log_warn(
                self.logger,
                "return store YOLO grasp point unavailable; using bbox center",
                step="store_tool_target",
                event="fallback",
                tool=tool_name,
                reason="yolo_grasp_point_unavailable",
            )
            return self._resolve_store_tool_center_target(
                boxes,
                tool_name,
                color_image,
                depth_image,
                intrinsics,
            )

        target = self.pick_target_resolver.target_from_selected_grasp(
            label,
            tool_name,
            selected,
            depth_image,
            intrinsics,
        )
        return self.refine_store_tool_target(target, tool_name)

    def _resolve_store_tool_center_target(
        self,
        boxes,
        tool_name,
        color_image,
        depth_image,
        intrinsics,
    ):
        return self.pick_target_resolver.target_from_boxes(
            boxes,
            tool_name,
            color_image,
            depth_image,
            intrinsics,
            use_bbox_center=True,
        )

    def resolve_drawer_marker_target(self, drawer_id):
        marker_id = DRAWER_ARUCO_MARKER_IDS.get(drawer_id)
        if marker_id is None:
            log_error(
                self.logger,
                "drawer marker id missing",
                step="drawer_marker",
                event="fail",
                drawer_id=drawer_id,
                reason="marker_id_missing",
            )
            return None

        if not self.frame_processor.has_camera_state():
            log_warn(
                self.logger,
                "drawer marker camera state unavailable",
                step="drawer_marker",
                event="fail",
                drawer_id=drawer_id,
                reason="camera_state_unavailable",
            )
            return None

        return self.drawer_marker_resolver.resolve_marker_target(
            self.state.color_image.copy(),
            self.state.depth_image.copy(),
            dict(self.state.intrinsics),
            marker_id,
        )

    def _box_label(self, box):
        try:
            class_id = int(box.cls[0])
        except (TypeError, IndexError):
            class_id = int(box.cls)
        return str(self.detector.names[class_id])
