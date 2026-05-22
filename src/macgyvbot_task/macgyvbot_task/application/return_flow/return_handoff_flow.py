"""Receive a user-held return tool."""

from __future__ import annotations

import time

import rclpy

from macgyvbot_config.hand_grasp import HAND_GRASP_TIMEOUT_SEC
from macgyvbot_config.handoff import (
    HANDOVER_HAND_X_OFFSET_M,
    HANDOVER_HAND_Z_OFFSET_M,
    OBSERVATION_TIMEOUT_SEC,
)
from macgyvbot_config.return_flow import (
    RETURN_HAND_CLOSE_DEPTH_MAX_MM,
    RETURN_HAND_CLOSE_DEPTH_MIN_MM,
    RETURN_HAND_CLOSE_ROI_CENTER_X,
    RETURN_HAND_CLOSE_ROI_CENTER_Y,
    RETURN_HAND_CLOSE_ROI_HEIGHT_RATIO,
    RETURN_HAND_CLOSE_ROI_TIMEOUT_SEC,
    RETURN_HAND_CLOSE_ROI_WIDTH_RATIO,
)
from macgyvbot_config.robot import BASE_FRAME
from macgyvbot_manipulation.grasp_verifier import GraspVerifier
from macgyvbot_manipulation.handover_targeting import (
    move_to_candidate_with_offset,
    move_to_observation_pose,
    start_async_observation_search,
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
    ):
        self.robot = robot
        self.motion = motion_controller
        self.gripper = gripper
        self.state = state
        self.reporter = reporter
        self.wait_fn = wait_fn
        self.tool_hold_monitor = tool_hold_monitor
        self.interrupted = interrupted or (lambda: False)
        self.grasp_verifier = GraspVerifier(
            gripper,
            wait_fn,
            interrupted=self.interrupted,
        )

    def receive_from_candidate(self, tool_name, candidate, command, logger):
        ok, reason = self.move_to_candidate(tool_name, candidate, command, logger)
        if not ok:
            return None, reason

        return self.grasp_at_current_position(tool_name, command, logger)

    def move_to_candidate(self, tool_name, candidate, command, logger):
        if candidate.frame_id not in ("world", BASE_FRAME):
            self.reporter.fail(
                tool_name,
                f"반납 위치 frame이 planning frame이 아닙니다: {candidate.frame_id}",
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
                "그리퍼 전방 영역에서 반납 공구를 확인하지 못했습니다.",
                "return_close_roi_timeout",
                command,
                logger,
            )
            return None, "return_close_roi_timeout"

        logger.info("사용자 손 위치에 접근했습니다. 반납 공구 grasp를 시도합니다.")
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

    def wait_for_tool_in_close_roi(self, logger):
        start_time = time.monotonic()

        while rclpy.ok():
            result = self.state.last_grasp_result or {}
            tool_roi = result.get("tool_roi")
            tool_depth_mm = result.get("tool_depth_mm")
            if (
                self._tool_roi_in_close_region(tool_roi)
                and self._tool_depth_in_close_range(tool_depth_mm)
            ):
                logger.info(
                    "그리퍼 close ROI/depth 범위 안에서 공구 확인: "
                    f"tool_roi={tool_roi}, tool_depth_mm={tool_depth_mm}"
                )
                return True

            if time.monotonic() - start_time >= RETURN_HAND_CLOSE_ROI_TIMEOUT_SEC:
                logger.warn(
                    "그리퍼 close ROI/depth 범위 안에서 공구를 확인하지 못했습니다: "
                    f"timeout={RETURN_HAND_CLOSE_ROI_TIMEOUT_SEC:.1f}s"
                )
                return False

            self.wait_fn(0.1)

        return False

    def _tool_roi_in_close_region(self, tool_roi):
        if self.state.color_image is None:
            return False
        if not isinstance(tool_roi, (list, tuple)) or len(tool_roi) != 4:
            return False

        height, width = self.state.color_image.shape[:2]
        if width <= 0 or height <= 0:
            return False

        try:
            x1, y1, x2, y2 = [float(value) for value in tool_roi]
        except (TypeError, ValueError):
            return False

        center_u = (x1 + x2) * 0.5
        center_v = (y1 + y2) * 0.5
        roi_center_u = width * RETURN_HAND_CLOSE_ROI_CENTER_X
        roi_center_v = height * RETURN_HAND_CLOSE_ROI_CENTER_Y
        half_width = width * RETURN_HAND_CLOSE_ROI_WIDTH_RATIO * 0.5
        half_height = height * RETURN_HAND_CLOSE_ROI_HEIGHT_RATIO * 0.5

        return (
            abs(center_u - roi_center_u) <= half_width
            and abs(center_v - roi_center_v) <= half_height
        )

    @staticmethod
    def _tool_depth_in_close_range(tool_depth_mm):
        try:
            depth_mm = float(tool_depth_mm)
        except (TypeError, ValueError):
            return False

        return (
            RETURN_HAND_CLOSE_DEPTH_MIN_MM
            <= depth_mm
            <= RETURN_HAND_CLOSE_DEPTH_MAX_MM
        )

    def receive(self, requested_tool, command, logger):
        advanced, advance_reason = self.advance_for_return_detection(
            requested_tool,
            command,
            logger,
        )
        if not advanced:
            return None, advance_reason

        grasp_result = self.wait_for_user_held_tool(requested_tool, logger)
        if grasp_result is None:
            if self.interrupted():
                logger.info("사용자 반납 공구 대기 중 stop/pause 요청으로 중단합니다.")
                return None, "interrupted"

            self.reporter.fail(
                requested_tool,
                "전방 20cm 위치에서 사용자가 들고 있는 공구를 확인하지 못했습니다.",
                "user_tool_not_detected",
                command,
                logger,
            )
            return None, "user_tool_not_detected"

        detected_tool = grasp_result.get("tool_label") or "unknown"
        tool_name = self.resolve_tool_name(requested_tool, detected_tool, logger)

        logger.info("사용자 hand-tool grasp 확인. 공구 grasp를 시도합니다.")
        if not self.try_robot_grasp(tool_name, command, logger):
            if self.interrupted():
                logger.info("반납 공구 grasp 중 stop/pause 요청으로 중단합니다.")
                return None, "interrupted"

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

    def wait_for_user_held_tool(self, tool_name, logger):
        start_time = time.monotonic()

        while rclpy.ok():
            if self.interrupted():
                logger.info("사용자 반납 공구 인식 대기를 stop/pause 요청으로 중단합니다.")
                return None

            result = self.state.last_grasp_result
            if self.state.human_grasped_tool and result is not None:
                logger.info(
                    "사용자 hand-tool grasp 확인: "
                    f"requested_tool={tool_name}, "
                    f"detected_tool={result.get('tool_label')}, "
                    f"state={result.get('state')}, "
                    f"score={result.get('grasp_score')}"
                )
                return result

            if time.monotonic() - start_time >= HAND_GRASP_TIMEOUT_SEC:
                logger.warn(
                    f"{HAND_GRASP_TIMEOUT_SEC:.1f}초 동안 사용자 반납 공구 "
                    "grasp 인식이 없어 대기 종료"
                )
                return None

            self.wait_fn(0.1)

        return None

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

    @staticmethod
    def resolve_tool_name(requested_tool, detected_tool, logger):
        if detected_tool and detected_tool != "unknown":
            if requested_tool and requested_tool != "unknown" and detected_tool != requested_tool:
                logger.warn(
                    f"반납 명령 공구({requested_tool})와 vision 검출 공구"
                    f"({detected_tool})가 다릅니다. vision 결과를 우선합니다."
                )
            return detected_tool

        if requested_tool and requested_tool != "unknown":
            logger.warn(
                f"vision 공구 라벨이 없어 명령의 공구명({requested_tool})을 사용합니다."
            )
            return requested_tool

        return "unknown"

    def advance_for_return_detection(self, tool_name, command, logger):
        if self.interrupted():
            logger.info("반납 관찰 자세 이동 시작 전 stop/pause 요청으로 중단합니다.")
            return False, "interrupted"

        ok, start_pose = move_to_observation_pose(self.motion, self.robot, logger)

        self.reporter.publish(
            "moving_return_grasp_pose",
            tool_name,
            "반납 공구를 감지하기 위해 관찰 자세로 이동합니다.",
            command,
        )
        logger.info(
            "반납 1단계: 공구 감지 전 관찰 자세 이동 "
            f"pose=({start_pose.x:.3f},{start_pose.y:.3f},{start_pose.z:.3f})"
        )
        if self.interrupted():
            logger.info("반납 관찰 자세 이동 후 stop/pause 요청으로 중단합니다.")
            return False, "interrupted"

        if not ok:
            self.reporter.fail(
                tool_name,
                "반납 공구 감지 전 전방 전진에 실패했습니다.",
                "return_detection_advance_failed",
                command,
                logger,
            )
            return False, "return_detection_advance_failed"

        future = start_async_observation_search(
            self.state,
            logger,
            timeout_sec=OBSERVATION_TIMEOUT_SEC,
            should_interrupt=self.interrupted,
        )
        while not future.done():
            if self.interrupted():
                future.cancel()
                logger.info("반납 위치 관측 중 stop/pause 요청으로 중단합니다.")
                return False, "interrupted"
            self.wait_fn(0.1)

        try:
            candidate = future.result()
        except Exception as exc:
            if self.interrupted():
                logger.info("반납 위치 관측 future를 stop/pause 요청으로 중단합니다.")
                return False, "interrupted"
            logger.error(f"반납 위치 관측 future 실패: {exc}")
            return False, "return_search_failed"

        if self.interrupted():
            logger.info("반납 위치 관측 후 stop/pause 요청으로 중단합니다.")
            return False, "interrupted"

        if not candidate.found:
            self.reporter.fail(
                tool_name,
                "반납받을 사용자 손/공구 위치를 찾지 못했습니다.",
                "return_search_failed",
                command,
                logger,
            )
            return False, "return_search_failed"

        if candidate.frame_id not in ("world", BASE_FRAME):
            self.reporter.fail(
                tool_name,
                f"반납 위치 frame이 planning frame이 아닙니다: {candidate.frame_id}",
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
        if self.interrupted() or reason == "interrupted":
            logger.info("반납 위치 이동 중 stop/pause 요청으로 중단합니다.")
            return False, "interrupted"

        logger.info(
            "반납 손/공구 위치로 수령 이동: "
            f"source={candidate.source}, frame={candidate.frame_id}, "
            f"raw=({candidate.x:.3f},{candidate.y:.3f},{candidate.z:.3f}), "
            f"offset=({HANDOVER_HAND_X_OFFSET_M:.3f},0.000,{HANDOVER_HAND_Z_OFFSET_M:.3f}), "
            f"safe=({final_pose.x:.3f},{final_pose.y:.3f},{final_pose.z:.3f})"
        )
        self.reporter.publish(
            "moving_return_detected_pose",
            tool_name,
            "탐색된 사용자 손/공구 위치로 이동합니다.",
            command,
        )
        if not ok:
            self.reporter.fail(
                tool_name,
                "탐색된 반납 위치로 이동하지 못했습니다.",
                "return_detected_pose_move_failed" if reason == "target_move_failed" else reason,
                command,
                logger,
            )
            return False, (
                "return_detected_pose_move_failed"
                if reason == "target_move_failed"
                else reason
            )

        return True, ""
