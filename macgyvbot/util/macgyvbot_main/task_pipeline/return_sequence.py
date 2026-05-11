"""Return sequence orchestration for receiving and storing a user-held tool."""

import json
import time

import rclpy
from std_msgs.msg import String

from macgyvbot.config.config import (
    GRASP_ADVANCE_DISTANCE_M,
    HAND_GRASP_TIMEOUT_SEC,
    RETURN_HOME_DESCENT_START_Z,
    RETURN_HOME_DESCENT_STEP_M,
    RETURN_HOME_FORCE_THRESHOLD_N,
    GRASP_VERIFY_POLL_SEC,
    GRASP_VERIFY_TIMEOUT_SEC,
    GRASP_RETRY_LIMIT,
    SAFE_Z,
    SEQUENCE_WAIT_POLL_SEC,
)
from macgyvbot.util.macgyvbot_main.model_control.gripper_grasp import (
    read_grasp_confirmation,
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

            if not self._place_tool_at_robot_home(tool_name, ori, command, log):
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
        for attempt in range(1, GRASP_RETRY_LIMIT + 1):
            logger.info(f"반납 공구 grasp 시도 {attempt}/{GRASP_RETRY_LIMIT}")
            self._publish_status(
                "grasping",
                tool_name,
                f"반납 공구 grasp 시도 {attempt}/{GRASP_RETRY_LIMIT}",
                command,
            )

            self.gripper.close_gripper()
            if self._verify_robot_grasp(logger):
                return True

            logger.warn(
                f"반납 공구 grasp 실패. 재시도 준비 {attempt}/{GRASP_RETRY_LIMIT}"
            )
            self.gripper.open_gripper()
            self._cooperative_wait(0.5)

        return False

    def _verify_robot_grasp(self, logger):
        start_time = time.monotonic()
        last_status = None

        while time.monotonic() - start_time < GRASP_VERIFY_TIMEOUT_SEC:
            try:
                confirmed, busy, status, width_mm = read_grasp_confirmation(
                    self.gripper,
                    logger,
                )
            except Exception as exc:
                logger.warn(f"그리퍼 상태 읽기 실패: {exc}")
                return False

            last_status = {
                "status": status,
                "width_mm": width_mm,
            }

            if confirmed:
                return True

            if not busy:
                break

            self._cooperative_wait(GRASP_VERIFY_POLL_SEC)

        logger.warn(f"그리퍼 grasp 확인 실패: status={last_status}")
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

        stop_z = self._descend_until_z_reaction(
            target_x,
            target_y,
            approach_z,
            ori,
            tool_name,
            command,
            logger,
        )
        if stop_z is None:
            return False

        logger.info(f"반납 4단계: {tool_name} Home 위치에 놓기")
        self.gripper.open_gripper()
        self._cooperative_wait(0.8)

        return True

    def _descend_until_z_reaction(
        self,
        target_x,
        target_y,
        start_z,
        ori,
        tool_name,
        command,
        logger,
    ):
        self.state.latest_wrench = None
        current_z = float(start_z)

        self._publish_status(
            "lowering_return_tool",
            tool_name,
            "Home에서 Z를 낮추며 반력을 확인합니다.",
            command,
        )
        logger.info(
            "반납 3단계: Home에서 Z 하강 시작 "
            f"threshold={RETURN_HOME_FORCE_THRESHOLD_N:.1f}N, "
            f"min_z={SAFE_Z_MIN:.3f}"
        )

        while rclpy.ok():
            force_z = self._latest_force_z()
            if force_z is not None and force_z >= RETURN_HOME_FORCE_THRESHOLD_N:
                logger.info(
                    "Z 반대방향 힘 감지로 하강을 중단합니다: "
                    f"force_z={force_z:.2f}N, z={current_z:.3f}"
                )
                return current_z

            if current_z <= SAFE_Z_MIN:
                logger.warn(
                    "Z 반력이 임계값에 도달하지 않았지만 안전 최소 Z까지 하강했습니다: "
                    f"last_force_z={force_z}"
                )
                return current_z

            next_z = max(SAFE_Z_MIN, current_z - RETURN_HOME_DESCENT_STEP_M)
            ok = self.motion.plan_and_execute(
                logger,
                pose_goal=make_safe_pose(target_x, target_y, next_z, ori, logger),
            )
            if not ok:
                self._fail(
                    tool_name,
                    "반납 Home Z 하강에 실패했습니다.",
                    "return_home_descent_failed",
                    command,
                    logger,
                )
                return None

            current_z = next_z
            self._cooperative_wait(SEQUENCE_WAIT_POLL_SEC)

        return None

    def _latest_force_z(self):
        wrench = getattr(self.state, "latest_wrench", None)
        if wrench is None:
            return None
        try:
            return float(wrench.force.z)
        except (AttributeError, TypeError, ValueError):
            return None

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
