"""Receive a user-held return tool."""

from __future__ import annotations

import time

import rclpy

from macgyvbot.config.hand_grasp import HAND_GRASP_TIMEOUT_SEC
from macgyvbot.config.handoff import (
    HANDOVER_HAND_X_OFFSET_M,
    HANDOVER_HAND_Z_OFFSET_M,
    OBSERVATION_TIMEOUT_SEC,
)
from macgyvbot.config.robot import BASE_FRAME
from macgyvbot.control.grasp_verifier import GraspVerifier
from macgyvbot.control.handover_targeting import (
    move_to_candidate_with_offset,
    move_to_observation_pose,
    start_async_observation_search,
)


class ReturnHandoffFlow:
    """Move toward a user-held tool and grasp it from the user."""

    def __init__(self, robot, motion_controller, gripper, state, reporter, wait_fn):
        self.robot = robot
        self.motion = motion_controller
        self.gripper = gripper
        self.state = state
        self.reporter = reporter
        self.wait_fn = wait_fn
        self.grasp_verifier = GraspVerifier(gripper, wait_fn)

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
        return tool_name, ""

    def wait_for_user_held_tool(self, tool_name, logger):
        start_time = time.monotonic()

        while rclpy.ok():
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
        )
        candidate = future.result()
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
        )
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
