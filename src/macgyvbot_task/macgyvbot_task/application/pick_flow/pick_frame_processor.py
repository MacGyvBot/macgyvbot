"""Robot camera frame processing for the main pick node."""

from __future__ import annotations

import cv2


class PickFrameProcessor:
    """Run detection on camera frames and start pick sequences for targets."""

    def __init__(
        self,
        state,
        detector,
        pick_target_resolver,
        start_pick_sequence,
        status_publisher,
        logger,
    ):
        self.state = state
        self.detector = detector
        self.pick_target_resolver = pick_target_resolver
        self.start_pick_sequence = start_pick_sequence
        self.status_publisher = status_publisher
        self.logger = logger

    def has_camera_state(self):
        return (
            self.state.color_image is not None
            and self.state.depth_image is not None
            and self.state.intrinsics is not None
        )

    def process_current_frame(self):
        """Detect objects, optionally handle target pick, and return a display frame."""
        if (
            self.state.drawer_observation_validation_only
            and not self.state.drawer_observation_ready
        ):
            if not self.state._drawer_observation_not_ready_logged:
                self.logger.warn(
                    "drawer_observation_validation_only: "
                    "서랍 내부 관찰 pose 도달 전이라 YOLO 탐지를 시작하지 않습니다."
                )
                self.status_publisher(
                    "drawer_observation_pending",
                    action="bring",
                    message="서랍 내부 관찰 pose 도달 전이라 탐지를 대기합니다.",
                    reason="drawer_observation_not_ready",
                    command=self.state.current_command,
                )
                self.state._drawer_observation_not_ready_logged = True
            return self.state.color_image

        results = self.detector.detect(self.state.color_image)
        annotated_frame = results[0].plot()

        if self.state.drawer_observation_validation_only:
            self._process_drawer_observation_validation(
                results[0].boxes,
                annotated_frame,
            )
            return annotated_frame

        if self.state.target_label:
            self.handle_target_detection(
                results[0].boxes,
                annotated_frame,
            )

        return annotated_frame

    def handle_target_detection(self, boxes, annotated_frame):
        target = self.pick_target_resolver.target_from_boxes(
            boxes,
            self.state.target_label,
            self.state.color_image,
            self.state.depth_image,
            self.state.intrinsics,
        )
        if target.found:
            bx, by, bz = target.base_xyz
            u, v = target.pixel
            self.draw_grasp_marker(annotated_frame, u, v, target.source)
            self.start_pick_sequence(bx, by, bz, target.depth_m, target.yaw_deg)
            return

        if target.reason == "target_not_found":
            self.logger.info(
                f"'{self.state.target_label}' 탐색 중... 현재 프레임에서는 미검출"
            )
            if self.state._last_search_status_target != self.state.target_label:
                self.state._last_search_status_target = self.state.target_label
                self.status_publisher(
                    "searching",
                    tool_name=self.state.target_label,
                    action="bring",
                    message=f"{self.state.target_label} 탐색 중입니다.",
                    command=self.state.current_command,
                )

    def _process_drawer_observation_validation(self, boxes, annotated_frame):
        summary = self._box_summary(boxes)
        if summary != self.state._last_drawer_detection_summary:
            self.state._last_drawer_detection_summary = summary
            self.logger.info(
                "drawer_observation_validation_only: "
                f"서랍 내부 YOLO detections: {summary}"
            )
            self.status_publisher(
                "drawer_searching_only",
                tool_name=self.state.target_label,
                action="bring",
                message=f"서랍 내부 detection-only 탐지 중: {summary}",
                command=self.state.current_command,
            )

        if not self.state.target_label:
            return

        target = self.pick_target_resolver.target_from_boxes(
            boxes,
            self.state.target_label,
            self.state.color_image,
            self.state.depth_image,
            self.state.intrinsics,
        )
        if target.found:
            u, v = target.pixel
            self.draw_grasp_marker(annotated_frame, u, v, target.source)
            self.logger.info(
                "drawer_observation_validation_only: "
                f"'{self.state.target_label}' 검출됨. pick은 시작하지 않습니다."
            )
            self.status_publisher(
                "drawer_target_seen_only",
                tool_name=self.state.target_label,
                action="bring",
                message=(
                    f"{self.state.target_label} 검출됨. "
                    "validation-only 모드라 pick은 시작하지 않습니다."
                ),
                command=self.state.current_command,
            )
            return

        self.logger.info(
            "drawer_observation_validation_only: "
            f"'{self.state.target_label}' 미검출 또는 depth projection 실패 "
            f"(reason={target.reason})"
        )

    def _box_summary(self, boxes):
        if boxes is None or len(boxes) == 0:
            return "none"

        counts = {}
        for box in boxes:
            label = self._box_label(box)
            counts[label] = counts.get(label, 0) + 1

        return ", ".join(
            f"{label}:{count}" for label, count in sorted(counts.items())
        )

    def _box_label(self, box):
        try:
            class_id = int(box.cls[0])
        except (TypeError, IndexError):
            class_id = int(box.cls)
        return str(self.detector.names[class_id])

    @staticmethod
    def draw_grasp_marker(frame, u, v, source):
        cv2.circle(frame, (u, v), 6, (0, 255, 255), -1)
        cv2.putText(
            frame,
            source,
            (u + 8, v - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )
