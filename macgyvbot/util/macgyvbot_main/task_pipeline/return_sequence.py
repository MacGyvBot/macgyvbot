"""Return sequence orchestration for receiving and storing a user-held tool."""

import json
import time

import rclpy
from std_msgs.msg import String

from macgyvbot.config.config import (
    BASE_FRAME,
    HANDOVER_HAND_X_OFFSET_M,
    HANDOVER_HAND_Z_OFFSET_M,
    HAND_GRASP_TIMEOUT_SEC,
    OBSERVATION_TIMEOUT_SEC,
    RETURN_HOME_DESCENT_START_Z,
    SAFE_Z,
    SEQUENCE_WAIT_POLL_SEC,
)
from macgyvbot.util.macgyvbot_main.model_control.force_detection import (
    ForceReactionDetector,
)
from macgyvbot.util.macgyvbot_main.model_control.grasp_verifier import (
    GraspVerifier,
)
from macgyvbot.util.macgyvbot_main.model_control.handover_targeting import (
    move_to_observation_pose,
    move_to_candidate_with_offset,
    start_async_observation_search,
)
from macgyvbot.util.macgyvbot_main.model_control.robot_pose import make_safe_pose
from macgyvbot.util.macgyvbot_main.model_control.robot_safezone import (
    SAFE_Z_MIN,
    clamp_to_safe_workspace,
)


class ReturnSequenceRunner:
    """Receive a user-held tool and place it in its configured home pose."""

    def __init__(self, robot, motion_controller, gripper, state):
        self.robot = robot
        self.motion = motion_controller
        self.gripper = gripper
        self.state = state
        self.grasp_verifier = GraspVerifier(gripper, self._cooperative_wait)
        self.force_detector = ForceReactionDetector(
            motion_controller,
            state,
            self._cooperative_wait,
        )

    def run(self, command):
        log = self.state.logger()
        requested_tool = command.get("tool_name", "unknown")
        raw_text = command.get("raw_text", "")

        try:
            self.state.human_grasped_tool = False
            self.state.last_grasp_result = None

            self._publish_status(
                "waiting_return_handoff",
                requested_tool,
                "사용자 반납 공구를 받을 준비를 시작합니다.",
                command,
            )
            log.info(
                f"반납 명령 수신: tool={requested_tool}, raw_text='{raw_text}'. "
                "그리퍼를 열고 사용자 hand-tool grasp 인식을 기다립니다."
            )

            ori = self.state.home_ori
            self.gripper.open_gripper()
            self._cooperative_wait(0.5)

            if not self._advance_for_return_detection(requested_tool, command, log):
                return

            grasp_result = self._wait_for_user_held_tool(requested_tool, log)
            if grasp_result is None:
                self._fail_and_recover(
                    requested_tool,
                    "전방 20cm 위치에서 사용자가 들고 있는 공구를 확인하지 못했습니다.",
                    "user_tool_not_detected",
                    command,
                    ori,
                    log,
                )
                return

            detected_tool = grasp_result.get("tool_label") or "unknown"
            tool_name = self._resolve_tool_name(requested_tool, detected_tool, log)

            log.info("사용자 hand-tool grasp 확인. 공구 grasp를 시도합니다.")
            if not self._try_robot_grasp(tool_name, command, log):
                self._fail(
                    tool_name,
                    "반납 공구 grasp에 실패했습니다.",
                    "return_grasp_failed",
                    command,
                    log,
                )
                return

            self._publish_status(
                "grasp_success",
                tool_name,
                "반납 공구 grasp에 성공했습니다.",
                command,
            )

            if not self._place_tool_at_robot_home(
                tool_name,
                ori,
                command,
                log,
            ):
                return

            self._publish_status(
                "done",
                tool_name,
                f"{tool_name} 반납 공구를 Home에 배치했습니다.",
                command,
            )

        finally:
            self._clear_state()

    def _wait_for_user_held_tool(self, tool_name, logger):
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

            self._cooperative_wait(0.1)

        return None

    def _try_robot_grasp(self, tool_name, command, logger):
        def publish_attempt(attempt, retry_limit):
            self._publish_status(
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

    def _place_tool_at_robot_home(self, tool_name, ori, command, logger):
        target_x, target_y, _ = self.state.home_xyz
        approach_z = max(RETURN_HOME_DESCENT_START_Z, SAFE_Z_MIN)

        self._publish_status(
            "placing_return_tool",
            tool_name,
            f"{tool_name} 반납 위치인 Home으로 이동합니다.",
            command,
        )

        logger.info(
            f"반납 2단계: {tool_name} 반납 Home 위치 이동 "
            f"x={target_x:.3f}, y={target_y:.3f}, z={approach_z:.3f}"
        )
        ok = self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(target_x, target_y, approach_z, ori, logger),
        )
        if not ok:
            self._fail(
                tool_name,
                f"{tool_name} 반납 Home 위치 이동에 실패했습니다.",
                "return_home_move_failed",
                command,
                logger,
            )
            return False

        self._publish_status(
            "lowering_return_tool",
            tool_name,
            "Home에서 Z를 낮추며 반력을 확인합니다.",
            command,
        )
        stop_z = self.force_detector.descend_until_z_reaction(
            target_x,
            target_y,
            approach_z,
            ori,
            logger,
        )
        if stop_z is None:
            self._fail(
                tool_name,
                "반납 Home Z 하강에 실패했습니다.",
                "return_home_descent_failed",
                command,
                logger,
            )
            return False

        logger.info(f"반납 4단계: {tool_name} Home 위치에 놓기")
        self.gripper.open_gripper()
        self._cooperative_wait(0.8)

        return self._move_home_after_return(
            target_x,
            target_y,
            approach_z,
            ori,
            tool_name,
            command,
            logger,
        )

    def _move_home_after_return(
        self,
        target_x,
        target_y,
        approach_z,
        ori,
        tool_name,
        command,
        logger,
    ):
        logger.info("반납 5단계: 공구를 놓은 뒤 Home 안전 높이로 복귀")
        ok = self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(target_x, target_y, approach_z, ori, logger),
        )
        if not ok:
            self._fail(
                tool_name,
                "반납 공구를 놓은 뒤 Home 안전 높이 복귀에 실패했습니다.",
                "return_home_retreat_failed",
                command,
                logger,
            )
            return False

        final_x, final_y, final_z = self.state.home_xyz
        final_z = max(final_z, approach_z)
        ok = self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(final_x, final_y, final_z, ori, logger),
        )
        if not ok:
            self._fail(
                tool_name,
                "반납 공구를 놓은 뒤 Home 복귀에 실패했습니다.",
                "return_home_after_release_failed",
                command,
                logger,
            )
            return False

        return True

    @staticmethod
    def _resolve_tool_name(requested_tool, detected_tool, logger):
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

    def _advance_for_return_detection(self, tool_name, command, logger):
        ok, start_pose = move_to_observation_pose(self.motion, self.robot, logger)

        self._publish_status(
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
            self._fail_and_recover(
                tool_name,
                "반납 공구 감지 전 전방 전진에 실패했습니다.",
                "return_detection_advance_failed",
                command,
                self.state.home_ori,
                logger,
            )
            return False

        future = start_async_observation_search(
            self.state,
            logger,
            timeout_sec=OBSERVATION_TIMEOUT_SEC,
        )
        candidate = future.result()
        if not candidate.found:
            self._fail_and_recover(
                tool_name,
                "반납받을 사용자 손/공구 위치를 찾지 못했습니다.",
                "return_search_failed",
                command,
                self.state.home_ori,
                logger,
            )
            return False

        if candidate.frame_id not in ("world", BASE_FRAME):
            self._fail_and_recover(
                tool_name,
                f"반납 위치 frame이 planning frame이 아닙니다: {candidate.frame_id}",
                "return_unsupported_frame",
                command,
                self.state.home_ori,
                logger,
            )
            return False

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
        self._publish_status(
            "moving_return_detected_pose",
            tool_name,
            "탐색된 사용자 손/공구 위치로 이동합니다.",
            command,
        )
        if not ok:
            self._fail_and_recover(
                tool_name,
                "탐색된 반납 위치로 이동하지 못했습니다.",
                "return_detected_pose_move_failed" if reason == "target_move_failed" else reason,
                command,
                self.state.home_ori,
                logger,
            )
            return False

        return True

    def _recover_to_home(self, tool_name, command, ori, logger, reason):
        self._publish_status(
            "recovering",
            tool_name,
            "실패 후 Home 복귀를 시도합니다.",
            command,
            reason=reason,
        )
        x, y, z = self.state.home_xyz
        safe_z = max(z, SAFE_Z)
        ok = self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(x, y, safe_z, ori, logger),
        )
        if ok:
            self._publish_status(
                "returned",
                tool_name,
                "실패 후 Home으로 복귀했습니다.",
                command,
                reason=reason,
            )
            return True

        self._publish_status(
            "failed",
            tool_name,
            "실패 후 Home 복귀에도 실패했습니다.",
            command,
            reason=f"{reason}_recovery_failed",
        )
        return False

    def _fail_and_recover(self, tool_name, message, reason, command, ori, logger):
        self._fail(tool_name, message, reason, command, logger)
        self._recover_to_home(tool_name, command, ori, logger, reason=reason)

    def _fail(self, tool_name, message, reason, command, logger):
        logger.error(message)
        self._publish_status(
            "failed",
            tool_name,
            message,
            command,
            reason=reason,
        )

    def _publish_status(self, status, tool_name, message, command, reason=""):
        publisher = getattr(self.state, "robot_status_pub", None)
        if publisher is None:
            return

        payload = {
            "status": status,
            "task": "return_tool",
            "tool_name": tool_name,
            "action": "return",
            "message": message,
            "command": command,
        }
        if reason:
            payload["reason"] = reason
        publisher.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _clear_state(self):
        self.state.picking = False
        self.state.target_label = None
        self.state.human_grasped_tool = False
        self.state.current_command = None

    @staticmethod
    def _cooperative_wait(duration_sec):
        end_time = time.monotonic() + max(0.0, float(duration_sec))
        while rclpy.ok() and time.monotonic() < end_time:
            remaining = end_time - time.monotonic()
            time.sleep(min(SEQUENCE_WAIT_POLL_SEC, max(0.0, remaining)))
