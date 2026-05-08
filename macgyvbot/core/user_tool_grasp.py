"""Receive a user-held tool by closing the gripper after hand-tool detection."""

import json
import time

import rclpy
from std_msgs.msg import String

from macgyvbot.core.config import HAND_GRASP_TIMEOUT_SEC


class UserToolGraspRunner:
    """Handle return commands by grasping the tool currently held by the user."""

    def __init__(self, gripper, state):
        self.gripper = gripper
        self.state = state

    def run(self, command):
        log = self.state.logger()
        tool_name = command.get("tool_name", "unknown")
        raw_text = command.get("raw_text", "")

        try:
            self.state.human_grasped_tool = False
            self.state.last_grasp_result = None

            self._publish_status(
                "running",
                tool_name,
                "사용자 반납 공구를 받을 준비를 시작합니다.",
                command,
            )
            log.info(
                f"반납 명령 수신: tool={tool_name}, raw_text='{raw_text}'. "
                "그리퍼를 열고 사용자 hand-tool grasp 인식을 기다립니다."
            )

            self.gripper.open_gripper()
            time.sleep(0.5)

            grasp_result = self._wait_for_user_held_tool(tool_name, log)
            if grasp_result is None:
                message = "사용자가 들고 있는 공구를 확인하지 못했습니다."
                log.error(message)
                self._publish_status("failed", tool_name, message, command)
                return

            detected_tool = grasp_result.get("tool_label") or "unknown"
            if (
                tool_name
                and tool_name != "unknown"
                and detected_tool != "unknown"
                and detected_tool != tool_name
            ):
                log.warn(
                    f"반납 명령 공구({tool_name})와 vision 검출 공구"
                    f"({detected_tool})가 다릅니다. hand grasp 확인을 우선해 잡습니다."
                )

            log.info("사용자 hand-tool grasp 확인. 그리퍼를 닫아 공구를 잡습니다.")
            self.gripper.close_gripper()
            time.sleep(1.0)

            message = "사용자 반납 공구를 그리퍼로 잡았습니다."
            log.info(message)
            self._publish_status("completed", tool_name, message, command)

        finally:
            self.state.picking = False
            self.state.human_grasped_tool = False

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

            time.sleep(0.1)

        return None

    def _publish_status(self, status, tool_name, message, command):
        publisher = getattr(self.state, "robot_status_pub", None)
        if publisher is None:
            return

        payload = {
            "status": status,
            "task": "grasp_user_tool",
            "tool_name": tool_name,
            "message": message,
            "command": command,
        }
        publisher.publish(String(data=json.dumps(payload, ensure_ascii=False)))
