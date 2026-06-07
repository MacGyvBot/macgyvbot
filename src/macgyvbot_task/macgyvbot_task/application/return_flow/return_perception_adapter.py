"""Perception callbacks used by the return drawer-store flow."""

from __future__ import annotations

from macgyvbot_config.drawer import DRAWER_ARUCO_MARKER_IDS
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

    def detect_store_tool_label(self):
        if self.state.color_image is None:
            return None

        results = self.detector.detect(self.state.color_image)
        boxes = results[0].boxes if results else None
        if boxes is None:
            return None

        supported = self.drawer_flow.supported_tool_labels()
        for box in boxes:
            label = self._box_label(box)
            if label in supported:
                return label
        return None

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
