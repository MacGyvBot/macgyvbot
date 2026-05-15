"""Return sequence orchestration for receiving and storing a user-held tool."""

import json
import time

import rclpy
from std_msgs.msg import String

from macgyvbot.config.config import (
    DRAWER_HANDLE_LABEL,
    DRAWER_LABEL,
    GRASP_ADVANCE_DISTANCE_M,
    HAND_GRASP_TIMEOUT_SEC,
    SAFE_Z,
    SEQUENCE_WAIT_POLL_SEC,
    USE_DRAWER_HANDLE_OFFSET_FALLBACK,
)
from macgyvbot.util.macgyvbot_main.model_control.grasp_verifier import (
    GraspVerifier,
)
from macgyvbot.util.macgyvbot_main.model_control.robot_pose import make_safe_pose
from macgyvbot.util.macgyvbot_main.model_control.robot_safezone import (
    clamp_to_safe_workspace,
)
from macgyvbot.util.macgyvbot_main.task_pipeline.drawer_sequence import (
    DrawerInteraction,
)


class ReturnSequenceRunner:
    """Receive a user-held tool and place it in the drawer."""

    def __init__(self, robot, motion_controller, gripper, state):
        self.robot = robot
        self.motion = motion_controller
        self.gripper = gripper
        self.state = state
        self.grasp_verifier = GraspVerifier(gripper, self._cooperative_wait)
        self.drawer = DrawerInteraction(
            robot,
            motion_controller,
            gripper,
            state,
            self._cooperative_wait,
        )

    def run(self, command):
        log = self.state.logger()
        requested_tool = command.get("tool_name", "unknown")
        raw_text = command.get("raw_text", "")
        drawer_motion = None

        try:
            self.state.human_grasped_tool = False
            self.state.last_grasp_result = None

            self._publish_status(
                "searching_drawer",
                requested_tool,
                "반납 공구를 넣을 공구함 찾기 중입니다.",
                command,
            )
            log.info(
                f"반납 명령 수신: tool={requested_tool}, raw_text='{raw_text}'. "
                "서랍을 먼저 연 뒤 사용자 hand-tool grasp 인식을 기다립니다."
            )

            ori = self.state.home_ori
            self.gripper.open_gripper()
            self._cooperative_wait(0.5)

            drawer_target = self.drawer.wait_for_target(DRAWER_LABEL, log)
            if drawer_target is None:
                self._fail(
                    requested_tool,
                    "반납 공구를 넣을 공구함을 찾지 못했습니다.",
                    "drawer_not_found",
                    command,
                    log,
                )
                return

            self._publish_status(
                "moving_to_drawer",
                requested_tool,
                "공구함으로 이동 중입니다.",
                command,
            )
            if not self.drawer.move_to_drawer_view(drawer_target, log):
                self._fail(
                    requested_tool,
                    "공구함으로 이동하지 못했습니다.",
                    "drawer_move_failed",
                    command,
                    log,
                )
                return

            self._publish_status(
                "searching_drawer_handle",
                requested_tool,
                "서랍 손잡이를 찾는 중입니다.",
                command,
            )
            if USE_DRAWER_HANDLE_OFFSET_FALLBACK:
                handle_target = self.drawer.handle_target_from_drawer_offset(
                    drawer_target,
                    log,
                )
            else:
                handle_target = self.drawer.wait_for_target(DRAWER_HANDLE_LABEL, log)
            if handle_target is None:
                self._fail(
                    requested_tool,
                    "서랍 손잡이를 찾지 못했습니다.",
                    "drawer_handle_not_found",
                    command,
                    log,
                )
                return

            self._publish_status(
                "opening_drawer",
                requested_tool,
                "서랍 손잡이를 당겨 여는 중입니다.",
                command,
            )
            drawer_motion = self.drawer.open_drawer(handle_target, log)
            if drawer_motion is None:
                self._fail(
                    requested_tool,
                    "서랍을 열지 못했습니다.",
                    "drawer_open_failed",
                    command,
                    log,
                )
                return

            self._publish_status(
                "waiting_return_handoff",
                requested_tool,
                "사용자 반납 공구를 받을 위치로 이동합니다.",
                command,
            )
            if not self._advance_for_return_detection(requested_tool, command, log):
                self._close_drawer_after_empty_failure(
                    drawer_motion,
                    requested_tool,
                    command,
                    log,
                )
                return

            grasp_result = self._wait_for_user_held_tool(requested_tool, log)
            if grasp_result is None:
                self._close_drawer_after_empty_failure(
                    drawer_motion,
                    requested_tool,
                    command,
                    log,
                )
                self._fail(
                    requested_tool,
                    "전방 20cm 위치에서 사용자가 들고 있는 공구를 확인하지 못했습니다.",
                    "user_tool_not_detected",
                    command,
                    log,
                )
                return

            detected_tool = grasp_result.get("tool_label") or "unknown"
            tool_name = self._resolve_tool_name(requested_tool, detected_tool, log)

            log.info("사용자 hand-tool grasp 확인. 공구 grasp를 시도합니다.")
            if not self._try_robot_grasp(tool_name, command, log):
                self._close_drawer_after_empty_failure(
                    drawer_motion,
                    tool_name,
                    command,
                    log,
                )
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

            if not self._place_tool_in_drawer(
                tool_name,
                drawer_target,
                command,
                log,
            ):
                return

            self._publish_status(
                "closing_drawer",
                tool_name,
                "공구 보관 후 서랍 문을 닫는 중입니다.",
                command,
            )
            if not self.drawer.close_drawer(drawer_motion, log):
                self._fail(
                    tool_name,
                    "서랍 문 닫기에 실패했습니다.",
                    "drawer_close_failed",
                    command,
                    log,
                )
                return

            if not self._move_home_after_return(
                drawer_motion.travel_z,
                ori,
                tool_name,
                command,
                log,
            ):
                return

            self._publish_status(
                "done",
                tool_name,
                f"{tool_name} 반납 공구를 서랍에 넣고 서랍을 닫았습니다.",
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

    def _place_tool_in_drawer(self, tool_name, drawer_target, command, logger):
        self._publish_status(
            "placing_return_tool",
            tool_name,
            f"{tool_name} 반납 공구를 열린 서랍 안에 넣는 중입니다.",
            command,
        )
        if self.drawer.place_tool_in_open_drawer(drawer_target, logger):
            return True

        self._fail(
            tool_name,
            "열린 서랍 내부에 반납 공구를 배치하지 못했습니다.",
            "drawer_tool_place_failed",
            command,
            logger,
        )
        return False

    def _move_home_after_return(self, safe_z, ori, tool_name, command, logger):
        final_x, final_y, final_z = self.state.home_xyz
        final_z = max(final_z, safe_z, SAFE_Z)

        logger.info("반납 마무리: 서랍을 닫은 뒤 Home으로 복귀")
        ok = self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(final_x, final_y, final_z, ori, logger),
        )
        if not ok:
            self._fail(
                tool_name,
                "서랍을 닫은 뒤 Home 복귀에 실패했습니다.",
                "return_home_after_drawer_close_failed",
                command,
                logger,
            )
            return False

        return True

    def _close_drawer_after_empty_failure(
        self,
        drawer_motion,
        tool_name,
        command,
        logger,
    ):
        if drawer_motion is None:
            return True

        self._publish_status(
            "closing_drawer",
            tool_name,
            "반납 공구를 잡지 못해 열린 서랍을 닫는 중입니다.",
            command,
        )
        if self.drawer.close_drawer(drawer_motion, logger):
            return True

        self._fail(
            tool_name,
            "실패 처리 중 서랍 문 닫기에 실패했습니다.",
            "drawer_close_after_failure_failed",
            command,
            logger,
        )
        return False

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
        target_x = self.state.home_xyz[0] + GRASP_ADVANCE_DISTANCE_M
        target_y = self.state.home_xyz[1]
        target_z = max(self.state.home_xyz[2], SAFE_Z)
        x, y, z = clamp_to_safe_workspace(target_x, target_y, target_z, logger)

        self._publish_status(
            "moving_return_grasp_pose",
            tool_name,
            "반납 공구를 감지하기 위해 전방 20cm 위치로 이동합니다.",
            command,
        )
        logger.info(
            "반납 1단계: 공구 감지 전 전방 20cm 전진 "
            f"x={self.state.home_xyz[0]:.3f}->{x:.3f}, y={y:.3f}, z={z:.3f}"
        )
        ok = self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(x, y, z, self.state.home_ori, logger),
        )
        if not ok:
            self._fail(
                tool_name,
                "반납 공구 감지 전 전방 전진에 실패했습니다.",
                "return_detection_advance_failed",
                command,
                logger,
            )
            return False

        return True

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
