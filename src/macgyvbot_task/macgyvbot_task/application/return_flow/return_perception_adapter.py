"""Perception callbacks used by the return drawer-store flow."""

from __future__ import annotations
from macgyvbot_domain.logging import emit_structured_log

from macgyvbot_config.drawer import DRAWER_ARUCO_MARKER_IDS
from macgyvbot_perception.drawer_marker_resolver import DrawerMarkerResolver


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
            emit_structured_log(self.logger, 'warn', "log", "status", svc='task', pipe='return', msg="임시 관찰 위치 공구 bbox 확인을 위한 camera state가 없습니다.")
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
            emit_structured_log(self.logger, 'error', "log", "status", svc='task', pipe='return', msg=f"drawer {drawer_id}에 매핑된 ArUco marker id가 없습니다.")
            return None

        if not self.frame_processor.has_camera_state():
            emit_structured_log(self.logger, 'warn', "log", "status", svc='task', pipe='return', msg="서랍 marker 확인을 위한 camera state가 없습니다.")
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
