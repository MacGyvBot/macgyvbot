"""Handoff-specific workflow helpers for pick sequences."""

from __future__ import annotations

import time

import rclpy

from macgyvbot_config.hand_grasp import (
    HAND_GRASP_MASK_LOCK_TIMEOUT_SEC,
    HAND_GRASP_TIMEOUT_SEC,
)
from macgyvbot_config.handoff import (
    HANDOFF_RELEASE_WAIT_SEC,
    HANDOFF_WAIT_POLL_SEC,
    HANDOVER_HAND_X_OFFSET_M,
    HANDOVER_HAND_Z_OFFSET_M,
    OBSERVATION_TIMEOUT_SEC,
)
from macgyvbot_config.robot import BASE_FRAME, WORLD_FRAME
from macgyvbot_manipulation.handover_targeting import (
    move_to_candidate_with_offset,
    move_to_observation_pose,
    start_async_observation_search,
)
from macgyvbot_manipulation.robot_pose import make_safe_pose
from macgyvbot_manipulation.robot_safezone import SAFE_Z_MIN


class PickHandoffFlow:
    """Handle user handoff motion, waits, and fallback return motion."""

    def __init__(
        self,
        robot,
        motion_controller,
        gripper,
        state,
        wait_fn,
        tool_hold_monitor=None,
        interrupted=None,
    ):
        self.robot = robot
        self.motion = motion_controller
        self.gripper = gripper
        self.state = state
        self.wait_fn = wait_fn
        self.tool_hold_monitor = tool_hold_monitor
        self.interrupted = interrupted or (lambda: False)

    def return_tool_to_original_position(
        self,
        target_x,
        target_y,
        travel_z,
        grasp_z,
        ori,
        logger,
        safe_z_min=SAFE_Z_MIN,
    ):
        if grasp_z < safe_z_min:
            logger.warn(
                f"원래 위치 반환 grasp_z({grasp_z:.3f})가 "
                f"safe_z_min({safe_z_min:.3f})보다 낮아 "
                "safe_z_min으로 맞춥니다."
            )
            grasp_z = safe_z_min

        logger.info("반환 1단계: 원래 공구 위치 상단으로 이동")
        ok = self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(
                target_x,
                target_y,
                travel_z,
                ori,
                logger,
            ),
        )
        if not ok:
            logger.error(
                "원래 공구 위치 상단 이동 실패. "
                "공구를 잡은 상태로 중단합니다."
            )
            return False

        logger.info("반환 2단계: 원래 공구 위치로 하강")
        ok = self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(target_x, target_y, grasp_z, ori, logger),
        )
        if not ok:
            logger.error(
                "원래 공구 위치 하강 실패. "
                "공구를 잡은 상태로 중단합니다."
            )
            return False

        logger.info("반환 3단계: 원래 위치에 공구 놓기")
        if self.tool_hold_monitor is not None:
            self.tool_hold_monitor.stop("return_to_original_position")
        self.gripper.open_gripper()
        self.wait_fn(HANDOFF_RELEASE_WAIT_SEC)

        logger.info("반환 4단계: 공구를 놓은 뒤 안전 높이로 복귀")
        ok = self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(
                target_x,
                target_y,
                travel_z,
                ori,
                logger,
            ),
        )
        if not ok:
            logger.error("공구를 놓은 뒤 안전 높이 복귀 실패")
            return False

        logger.info("반환 5단계: Home joint pose로 복귀")
        ok = self.motion.move_to_home_joints(logger)
        if not ok:
            logger.error("공구 반환 후 Home 복귀 실패")
            return False

        return True

    def move_to_handoff_pose(self, ori, logger):
        if self.interrupted():
            logger.info(
                "사용자 전달 이동 시작 전 "
                "stop/pause 요청으로 handoff를 중단합니다."
            )
            return None, None, None

        if not self._move_to_observation_pose(logger):
            return None, None, None

        candidate = self._observe_handoff_candidate(logger)
        if candidate is None:
            return None, None, None

        if not self._validate_candidate(candidate, logger):
            return None, None, None

        return self._move_to_candidate(candidate, ori, logger)

    def _move_to_observation_pose(self, logger):
        ok, start_pose = move_to_observation_pose(self.motion, self.robot, logger)
        logger.info(
            "7단계: 사용자 전달 관찰 자세 이동 "
            f"pose=({start_pose.x:.3f},{start_pose.y:.3f},{start_pose.z:.3f})"
        )
        if self.interrupted():
            logger.info(
                "관찰 자세 이동 후 "
                "stop/pause 요청으로 handoff를 중단합니다."
            )
            return False

        if ok:
            return True

        logger.error("사용자 전달 위치 이동 실패. Pick 시퀀스 중단")
        self.state._publish_robot_status(
            "failed",
            message="사용자 전달 위치 이동에 실패했습니다.",
            reason="handoff_pose_move_failed",
            command=self.state.current_command,
        )
        return False

    def _observe_handoff_candidate(self, logger):
        future = start_async_observation_search(
            self.state,
            logger,
            timeout_sec=OBSERVATION_TIMEOUT_SEC,
            should_interrupt=self.interrupted,
        )
        while not future.done():
            if self.interrupted():
                future.cancel()
                logger.info(
                    "사용자 손 위치 관측 중 "
                    "stop/pause 요청으로 handoff를 중단합니다."
                )
                return None
            self.wait_fn(HANDOFF_WAIT_POLL_SEC)

        candidate = future.result()

        if self.interrupted():
            logger.info(
                "사용자 손 위치 관측 후 stop/pause 요청으로 "
                "handoff를 중단합니다."
            )
            return None

        return candidate

    def _validate_candidate(self, candidate, logger):
        if not candidate.found:
            logger.error("사용자 손 위치를 찾지 못했습니다.")
            self.state._publish_robot_status(
                "failed",
                message="사용자 손 위치를 찾지 못했습니다.",
                reason="handoff_search_failed",
                command=self.state.current_command,
            )
            return False

        if candidate.frame_id in (WORLD_FRAME, BASE_FRAME):
            return True

        logger.error(
            "사용자 손 위치 frame을 planning에 사용할 수 없습니다: "
            f"frame={candidate.frame_id}, source={candidate.source}"
        )
        self.state._publish_robot_status(
            "failed",
            message="사용자 손 위치 frame이 planning frame이 아닙니다.",
            reason="handoff_unsupported_frame",
            command=self.state.current_command,
        )
        return False

    def _move_to_candidate(self, candidate, ori, logger):
        ok, final_pose, reason = move_to_candidate_with_offset(
            self.motion,
            candidate,
            ori,
            logger,
            x_offset_m=HANDOVER_HAND_X_OFFSET_M,
            z_offset_m=HANDOVER_HAND_Z_OFFSET_M,
            should_interrupt=self.interrupted,
        )
        if self.interrupted():
            logger.info(
                "사용자 손 위치 이동 후 "
                "stop/pause 요청으로 handoff를 중단합니다."
            )
            return None, None, None

        logger.info(
            "사용자 손 위치로 전달 이동: "
            f"source={candidate.source}, frame={candidate.frame_id}, "
            f"raw=({candidate.x:.3f},{candidate.y:.3f},{candidate.z:.3f}), "
            f"offset=({HANDOVER_HAND_X_OFFSET_M:.3f},0.000,"
            f"{HANDOVER_HAND_Z_OFFSET_M:.3f}), "
            f"safe=({final_pose.x:.3f},{final_pose.y:.3f},{final_pose.z:.3f})"
        )
        if not ok:
            logger.error("사용자 손 위치로 전달 이동 실패")
            self.state._publish_robot_status(
                "failed",
                message="사용자 손 위치로 이동하지 못했습니다.",
                reason=(
                    "handoff_hand_pose_move_failed"
                    if reason == "target_move_failed"
                    else reason
                ),
                command=self.state.current_command,
            )
            return None, None, None

        return final_pose.x, final_pose.y, final_pose.z

    def move_home_after_handoff(self, logger, publish_on_failure=True):
        ok = self.motion.move_to_home_joints(logger)
        if not ok:
            logger.error("전달 후 Home 복귀 실패")
            if publish_on_failure:
                self.state._publish_robot_status(
                    "failed",
                    message="공구 전달 후 Home 복귀에 실패했습니다.",
                    reason="home_after_handoff_failed",
                    command=self.state.current_command,
                )
            return False

        return True

    def wait_for_human_grasp(self, logger):
        start_time = time.monotonic()

        while rclpy.ok():
            if self.interrupted():
                logger.info(
                    "사용자 잡기 인식 대기를 "
                    "stop/pause 요청으로 중단합니다."
                )
                return False

            if self.state.human_grasped_tool:
                logger.info("사용자가 공구를 잡은 것으로 확인됨")
                return True

            if time.monotonic() - start_time >= HAND_GRASP_TIMEOUT_SEC:
                logger.warn(
                    f"{HAND_GRASP_TIMEOUT_SEC:.1f}초 동안 "
                    "사용자 잡기 인식이 없어 대기 종료"
                )
                return False

            self.wait_fn(HANDOFF_WAIT_POLL_SEC)

        return False

    def wait_for_tool_mask_lock(self, logger):
        start_time = time.monotonic()

        while rclpy.ok():
            if self.interrupted():
                logger.info(
                    "공구 mask lock 대기를 "
                    "stop/pause 요청으로 중단합니다."
                )
                return False

            result = self.state.last_tool_mask_lock_result
            if self.state.tool_mask_locked and result is not None:
                logger.info(
                    "공구 mask lock 확인: "
                    f"source={result.get('mask_source')}, roi={result.get('tool_roi')}"
                )
                return True

            if result is not None and result.get("locked") is False:
                logger.warn(
                    "공구 mask lock 실패 응답: "
                    f"reason={result.get('reason', 'unknown')}"
                )
                return False

            if time.monotonic() - start_time >= HAND_GRASP_MASK_LOCK_TIMEOUT_SEC:
                logger.warn(
                    f"{HAND_GRASP_MASK_LOCK_TIMEOUT_SEC:.1f}초 동안 "
                    "공구 mask lock 응답이 없어 대기 종료"
                )
                return False

            self.wait_fn(HANDOFF_WAIT_POLL_SEC)

        return False
