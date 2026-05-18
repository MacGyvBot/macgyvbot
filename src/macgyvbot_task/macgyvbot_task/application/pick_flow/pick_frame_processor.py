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
        results = self.detector.detect(self.state.color_image)
        annotated_frame = results[0].plot()

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
