"""Handoff-specific workflow helpers for pick sequences."""

from __future__ import annotations

import time

import rclpy

from macgyvbot.config.hand_grasp import (
    HAND_GRASP_MASK_LOCK_TIMEOUT_SEC,
    HAND_GRASP_TIMEOUT_SEC,
)
from macgyvbot.config.handoff import (
    HANDOVER_HAND_X_OFFSET_M,
    HANDOVER_HAND_Z_OFFSET_M,
    OBSERVATION_TIMEOUT_SEC,
)
from macgyvbot.config.robot import BASE_FRAME
from macgyvbot.control.handover_targeting import (
    move_to_candidate_with_offset,
    move_to_observation_pose,
    start_async_observation_search,
)
from macgyvbot.control.robot_pose import make_safe_pose


class PickHandoffFlow:
    """Handle user handoff motion, waits, and fallback return motion."""

    def __init__(self, robot, motion_controller, gripper, state, wait_fn):
        self.robot = robot
        self.motion = motion_controller
        self.gripper = gripper
        self.state = state
        self.wait_fn = wait_fn

    def return_tool_to_original_position(
        self,
        target_x,
        target_y,
        travel_z,
        grasp_z,
        ori,
        logger,
    ):
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
            logger.error("원래 공구 위치 상단 이동 실패. 공구를 잡은 상태로 중단합니다.")
            return False

        logger.info("반환 2단계: 원래 공구 위치로 하강")
        ok = self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(target_x, target_y, grasp_z, ori, logger),
        )
        if not ok:
            logger.error("원래 공구 위치 하강 실패. 공구를 잡은 상태로 중단합니다.")
            return False

        logger.info("반환 3단계: 원래 위치에 공구 놓기")
        self.gripper.open_gripper()
        self.wait_fn(0.8)

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
        ok, start_pose = move_to_observation_pose(self.motion, self.robot, logger)
        logger.info(
            "7단계: 사용자 전달 관찰 자세 이동 "
            f"pose=({start_pose.x:.3f},{start_pose.y:.3f},{start_pose.z:.3f})"
        )
        if not ok:
            logger.error("사용자 전달 위치 이동 실패. Pick 시퀀스 중단")
            self.state._publish_robot_status(
                "failed",
                message="사용자 전달 위치 이동에 실패했습니다.",
                reason="handoff_pose_move_failed",
                command=self.state.current_command,
            )
            return None, None, None

        future = start_async_observation_search(
            self.state,
            logger,
            timeout_sec=OBSERVATION_TIMEOUT_SEC,
        )
        candidate = future.result()

        if not candidate.found:
            logger.error("사용자 손 위치를 찾지 못했습니다.")
            self.state._publish_robot_status(
                "failed",
                message="사용자 손 위치를 찾지 못했습니다.",
                reason="handoff_search_failed",
                command=self.state.current_command,
            )
            return None, None, None

        if candidate.frame_id not in ("world", BASE_FRAME):
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
            return None, None, None

        ok, final_pose, reason = move_to_candidate_with_offset(
            self.motion,
            candidate,
            ori,
            logger,
            x_offset_m=HANDOVER_HAND_X_OFFSET_M,
            z_offset_m=HANDOVER_HAND_Z_OFFSET_M,
        )
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
            if self.state.human_grasped_tool:
                logger.info("사용자가 공구를 잡은 것으로 확인됨")
                return True

            if time.monotonic() - start_time >= HAND_GRASP_TIMEOUT_SEC:
                logger.warn(
                    f"{HAND_GRASP_TIMEOUT_SEC:.1f}초 동안 사용자 잡기 인식이 없어 대기 종료"
                )
                return False

            self.wait_fn(0.1)

        return False

    def wait_for_tool_mask_lock(self, logger):
        start_time = time.monotonic()

        while rclpy.ok():
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

            self.wait_fn(0.1)

        return False
