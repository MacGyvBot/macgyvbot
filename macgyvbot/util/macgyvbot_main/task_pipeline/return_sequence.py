"""Return sequence orchestration for receiving and storing a user-held tool."""

import json
import time

import rclpy
from std_msgs.msg import String

from macgyvbot.config.config import (
    HAND_GRASP_TIMEOUT_SEC,
    GRASP_VERIFY_POLL_SEC,
    GRASP_VERIFY_TIMEOUT_SEC,
    GRASP_RETRY_LIMIT,
    RETURN_APPROACH_Z_OFFSET,
    RETURN_STAGING_POSE,
    SAFE_Z,
    SEQUENCE_WAIT_POLL_SEC,
    TOOL_HOME_POSES,
)
from macgyvbot.util.macgyvbot_main.model_control.robot_pose import make_safe_pose
from macgyvbot.util.macgyvbot_main.model_control.robot_safezone import (
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

            self.gripper.open_gripper()
            self._cooperative_wait(0.5)

            grasp_result = self._wait_for_user_held_tool(requested_tool, log)
            if grasp_result is None:
                self._fail(
                    requested_tool,
                    "사용자가 들고 있는 공구를 확인하지 못했습니다.",
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

            ori = self.state.home_ori
            if not self._move_to_staging_pose(ori, tool_name, command, log):
                return

            if tool_name == "unknown":
                self._fail(
                    tool_name,
                    "반납 공구 종류를 확정하지 못해 임시 위치에서 중단했습니다.",
                    "unknown_return_tool",
                    command,
                    log,
                )
                return

            home_pose = TOOL_HOME_POSES.get(tool_name)
            if home_pose is None:
                self._fail(
                    tool_name,
                    f"{tool_name}의 원위치 pose가 설정되어 있지 않습니다.",
                    "missing_tool_home_pose",
                    command,
                    log,
                )
                return

            if not self._place_tool_at_home(tool_name, home_pose, ori, command, log):
                return

            self._publish_status(
                "done",
                tool_name,
                f"{tool_name} 반납 공구를 원위치에 배치했습니다.",
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
                status = self.gripper.get_status()
            except Exception as exc:
                logger.warn(f"그리퍼 상태 읽기 실패: {exc}")
                return False

            last_status = status
            grip_detected = len(status) > 1 and bool(status[1])
            busy = bool(status[0]) if status else False

            if grip_detected:
                logger.info("그리퍼 grip detected 신호로 grasp 성공을 확인했습니다.")
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

    def _move_to_staging_pose(self, ori, tool_name, command, logger):
        staging = RETURN_STAGING_POSE
        x, y, z = self._pose_xyz(staging)
        _, _, z = clamp_to_safe_workspace(x, y, max(z, SAFE_Z), logger)

        self._publish_status(
            "moving_staging",
            tool_name,
            "공구 인식을 위해 임시 위치로 이동합니다.",
            command,
        )
        logger.info(
            f"반납 1단계: 임시 위치 이동 x={x:.3f}, y={y:.3f}, z={z:.3f}"
        )
        ok = self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(x, y, z, ori, logger),
        )
        if not ok:
            self._fail(
                tool_name,
                "반납 공구 임시 위치 이동에 실패했습니다.",
                "staging_move_failed",
                command,
                logger,
            )
            return False

        return True

    def _place_tool_at_home(self, tool_name, home_pose, ori, command, logger):
        target_x, target_y, place_z = self._pose_xyz(home_pose)
        approach_z = max(place_z + RETURN_APPROACH_Z_OFFSET, SAFE_Z)

        self._publish_status(
            "placing_return_tool",
            tool_name,
            f"{tool_name} 원위치로 이동합니다.",
            command,
        )

        logger.info(
            f"반납 2단계: {tool_name} 원위치 상단 이동 "
            f"x={target_x:.3f}, y={target_y:.3f}, z={approach_z:.3f}"
        )
        ok = self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(target_x, target_y, approach_z, ori, logger),
        )
        if not ok:
            self._fail(
                tool_name,
                f"{tool_name} 원위치 상단 이동에 실패했습니다.",
                "return_home_approach_failed",
                command,
                logger,
            )
            return False

        logger.info(f"반납 3단계: {tool_name} 원위치로 하강")
        ok = self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(target_x, target_y, place_z, ori, logger),
        )
        if not ok:
            self._fail(
                tool_name,
                f"{tool_name} 원위치 하강에 실패했습니다.",
                "return_home_descent_failed",
                command,
                logger,
            )
            return False

        logger.info(f"반납 4단계: {tool_name} 놓기")
        self.gripper.open_gripper()
        self._cooperative_wait(0.8)

        logger.info("반납 5단계: 안전 높이로 후퇴")
        ok = self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(target_x, target_y, approach_z, ori, logger),
        )
        if not ok:
            self._fail(
                tool_name,
                f"{tool_name} 배치 후 후퇴에 실패했습니다.",
                "return_home_retreat_failed",
                command,
                logger,
            )
            return False

        logger.info("반납 6단계: Home 위치로 복귀")
        return self._move_home(approach_z, ori, tool_name, command, logger)

    def _move_home(self, travel_z, ori, tool_name, command, logger):
        ok = self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(
                self.state.home_xyz[0],
                self.state.home_xyz[1],
                travel_z,
                ori,
                logger,
            ),
        )
        if not ok:
            self._fail(
                tool_name,
                "반납 후 Home 복귀에 실패했습니다.",
                "return_home_failed",
                command,
                logger,
            )
            return False
        return True

    @staticmethod
    def _pose_xyz(pose):
        return float(pose["x"]), float(pose["y"]), float(pose["z"])

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
