"""Receive a user-held return tool."""

from __future__ import annotations

import time

import rclpy

from macgyvbot_config.handoff import (
    HANDOVER_HAND_X_OFFSET_M,
    HANDOVER_HAND_Z_OFFSET_M,
)
from macgyvbot_config.return_flow import (
    RETURN_HAND_CLOSE_ROI_POLL_SEC,
    RETURN_HAND_CLOSE_ROI_TIMEOUT_SEC,
)
from macgyvbot_config.robot import BASE_FRAME, WORLD_FRAME
from macgyvbot_manipulation.grasp_verifier import GraspVerifier
from macgyvbot_manipulation.handover_targeting import (
    move_to_candidate_with_offset,
)
from macgyvbot_task.application.return_flow.return_close_policy import (
    ReturnClosePolicy,
)


class ReturnHandoffFlow:
    """Move toward a user-held tool and grasp it from the user."""

    def __init__(
        self,
        robot,
        motion_controller,
        gripper,
        state,
        reporter,
        wait_fn,
        tool_hold_monitor=None,
        interrupted=None,
        close_policy=None,
    ):
        self.motion = motion_controller
        self.gripper = gripper
        self.state = state
        self.reporter = reporter
        self.wait_fn = wait_fn
        self.tool_hold_monitor = tool_hold_monitor
        self.interrupted = interrupted or (lambda: False)
        self.close_policy = close_policy or ReturnClosePolicy()
        self.grasp_verifier = GraspVerifier(
            gripper,
            wait_fn,
            interrupted=self.interrupted,
        )

    def move_to_candidate(self, tool_name, candidate, command, logger):
        if candidate.frame_id not in (WORLD_FRAME, BASE_FRAME):
            self.reporter.fail(
                tool_name,
                (
                    "반납 위치 frame이 planning frame이 아닙니다: "
                    f"{candidate.frame_id}"
                ),
                "return_unsupported_frame",
                command,
                logger,
            )
            return False, "return_unsupported_frame"

        ok, final_pose, reason = move_to_candidate_with_offset(
            self.motion,
            candidate,
            self.state.home_ori,
            logger,
            x_offset_m=HANDOVER_HAND_X_OFFSET_M,
            z_offset_m=HANDOVER_HAND_Z_OFFSET_M,
            should_interrupt=self.interrupted,
        )
        logger.info(
            "감지된 사용자 손 위치로 수령 이동: "
            f"source={candidate.source}, frame={candidate.frame_id}, "
            f"raw=({candidate.x:.3f},{candidate.y:.3f},{candidate.z:.3f}), "
            f"safe=({final_pose.x:.3f},{final_pose.y:.3f},{final_pose.z:.3f})"
        )
        self.reporter.publish(
            "moving_return_detected_pose",
            tool_name,
            "탐색된 사용자 손 위치로 이동합니다.",
            command,
        )
        if not ok:
            failure_reason = (
                "return_detected_pose_move_failed"
                if reason == "target_move_failed"
                else reason
            )
            self.reporter.fail(
                tool_name,
                "탐색된 반납 위치로 이동하지 못했습니다.",
                failure_reason,
                command,
                logger,
            )
            return False, failure_reason

        return True, ""

    def grasp_at_current_position(self, tool_name, command, logger):
        if not self.wait_for_tool_in_close_roi(logger):
            self.reporter.fail(
                tool_name,
                (
                    "그리퍼 전방 영역에서 반납 공구를 "
                    "확인하지 못했습니다."
                ),
                "return_close_roi_timeout",
                command,
                logger,
            )
            return None, "return_close_roi_timeout"

        logger.info(
            "사용자 손 위치에 접근했습니다. "
            "반납 공구 grasp를 시도합니다."
        )
        tool_name = self._record_pregrasp_tool_tracking(tool_name, logger)
        if not self.try_robot_grasp(tool_name, command, logger):
            self.reporter.fail(
                tool_name,
                "반납 공구 grasp에 실패했습니다.",
                "return_grasp_failed",
                command,
                logger,
            )
            return None, "return_grasp_failed"

        self.reporter.publish(
            "grasp_success",
            tool_name,
            "반납 공구 grasp에 성공했습니다.",
            command,
        )
        if self.tool_hold_monitor is not None:
            self.tool_hold_monitor.start(tool_name, "return", command)
        return tool_name, ""

    def _record_pregrasp_tool_tracking(self, fallback_tool_name, logger):
        """Store the currently observed hand-held tool before gripper close."""
        result = self.state.last_grasp_result or {}
        detected_tool = result.get("tool_label")
        detected_tool = str(detected_tool).strip() if detected_tool else ""
        if not detected_tool:
            detected_tool = str(fallback_tool_name or "").strip()
        if not detected_tool:
            detected_tool = "unknown"

        self.state.held_tool = detected_tool
        self.state.last_known_tool_bbox = result.get("tool_roi")
        self.state.last_known_tool_confidence = result.get("tool_confidence")
        logger.info(
            "return pregrasp tool tracking saved: "
            f"tool={detected_tool}, "
            f"confidence={self.state.last_known_tool_confidence}, "
            f"bbox={self.state.last_known_tool_bbox}"
        )
        # TODO: interrupt/resume recovery에서는 여러 frame의 tool_label/confidence를
        # 누적해 가장 안정적인 held_tool을 선택합니다.
        return detected_tool

    def wait_for_tool_in_close_roi(self, logger):
        start_time = time.monotonic()

        while rclpy.ok():
            result = self.state.last_grasp_result or {}
            tool_roi = result.get("tool_roi")
            tool_depth_mm = result.get("tool_depth_mm")
            image_shape = self._color_image_shape()
            if self.close_policy.matches(image_shape, tool_roi, tool_depth_mm):
                logger.info(
                    "그리퍼 close ROI/depth 범위 안에서 공구 확인: "
                    f"tool_roi={tool_roi}, tool_depth_mm={tool_depth_mm}"
                )
                return True

            if time.monotonic() - start_time >= RETURN_HAND_CLOSE_ROI_TIMEOUT_SEC:
                logger.warn(
                    "그리퍼 close ROI/depth 범위 안에서 "
                    "공구를 확인하지 못했습니다: "
                    f"timeout={RETURN_HAND_CLOSE_ROI_TIMEOUT_SEC:.1f}s"
                )
                return False

            self.wait_fn(RETURN_HAND_CLOSE_ROI_POLL_SEC)

        return False

    def _color_image_shape(self):
        color_image = getattr(self.state, "color_image", None)
        if color_image is None:
            return None
        return getattr(color_image, "shape", None)

    def try_robot_grasp(self, tool_name, command, logger):
        def publish_attempt(attempt, retry_limit):
            self.reporter.publish(
                "grasping",
                tool_name,
                f"반납 공구 grasp 시도 {attempt}/{retry_limit}",
                command,
            )

        return self.grasp_verifier.try_grasp(
            logger,
            publish_attempt=publish_attempt,
            failure_prefix="반납 공구 grasp",
        )
