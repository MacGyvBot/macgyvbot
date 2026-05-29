#!/usr/bin/env python3
"""Lightweight task request router for MacGyvBot commands."""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from macgyvbot_config.topics import (
    ROBOT_STATUS_TOPIC,
    TASK_REQUEST_TOPIC,
    TOOL_COMMAND_TOPIC,
)
from macgyvbot_interfaces.msg import RobotTaskStatus, TaskRequest, ToolCommand
from macgyvbot_task.application import RobotStatusPublisher, ToolCommandController


class MacGyvBotNode(Node):
    """Route parsed tool commands to the task coordinator node."""

    _ACTIVE_STATUSES = {
        "accepted",
        "opening_drawer",
        "observing_drawer",
        "searching",
        "picking",
        "waiting_handoff",
        "waiting_return_handoff",
        "checking_return_target",
        "returning_home",
        "closing_drawer",
        "paused",
        "resumed",
        "tool_dropped",
    }
    _FINAL_STATUSES = {
        "done",
        "completed",
        "success",
        "failed",
        "error",
        "cancelled",
        "rejected",
        "busy",
    }
    _ROUTED_ACTIONS = {
        "bring",
        "return",
        "exit",
        "cancel",
        "home",
        "release",
    }

    def __init__(self):
        super().__init__("macgyvbot_main_node")

        self._task_active = False
        self._target_label = None
        self._current_command = None

        self.task_request_pub = self.create_publisher(
            TaskRequest,
            TASK_REQUEST_TOPIC,
            10,
        )
        self.robot_status_pub = self.create_publisher(
            RobotTaskStatus,
            ROBOT_STATUS_TOPIC,
            10,
        )
        self.status_publisher = RobotStatusPublisher(
            self._publish_status_payload,
            target_label_provider=lambda: self._target_label,
        )
        self.task_controller = ToolCommandController(
            self.get_logger(),
            self.status_publisher,
            is_busy=lambda: self._task_active,
            set_target=self._request_bring,
            clear_target=self._clear_pending_target,
            reset_search_status=self._reset_search_status,
            start_return=self._request_return,
            release_gripper=self._request_release,
            move_home=self._request_home,
        )

        self.create_subscription(String, "/target_label", self._target_label_cb, 10)
        self.create_subscription(
            ToolCommand,
            TOOL_COMMAND_TOPIC,
            self._tool_command_cb,
            10,
        )
        self.create_subscription(
            RobotTaskStatus,
            ROBOT_STATUS_TOPIC,
            self._robot_status_cb,
            10,
        )

        self.get_logger().info("macgyvbot main router 초기화 완료")
        self.get_logger().info(f"공구 명령 토픽: {TOOL_COMMAND_TOPIC}")
        self.get_logger().info(f"task request 토픽: {TASK_REQUEST_TOPIC}")
        self.get_logger().info(f"로봇 상태 토픽: {ROBOT_STATUS_TOPIC}")

    def _target_label_cb(self, msg):
        val = msg.data.strip()
        if not val:
            return
        self.task_controller.handle_target_label(val, source="/target_label")

    def _tool_command_cb(self, msg):
        command = self._tool_command_payload(msg)

        action = str(command.get("action", "unknown")).strip().lower()
        if action == "release":
            self._request_release(command=command)
            self._publish_robot_status(
                "accepted",
                tool_name=command.get("tool_name", "unknown"),
                action="release",
                message="release 요청을 task coordinator로 전달했습니다.",
                command=command,
            )
            return

        if action == "home":
            self._request_home(command=command)
            self._publish_robot_status(
                "accepted",
                tool_name=command.get("tool_name", "unknown"),
                action="home",
                message="Home 복귀 요청을 task coordinator로 전달했습니다.",
                command=command,
            )
            return

        self.task_controller.handle_command(command)

    def _request_bring(self, tool_name, command=None):
        self._target_label = tool_name
        self._current_command = command
        self._task_active = True
        self._publish_task_request(
            {
                "task": "bring",
                "tool_name": tool_name,
                "command": command or {
                    "tool_name": tool_name,
                    "action": "bring",
                    "target_mode": "named",
                },
            }
        )

    def _request_return(self, command):
        self._target_label = None
        self._current_command = command
        self._task_active = True
        self._publish_task_request(
            {
                "task": "return",
                "tool_name": command.get("tool_name", "unknown"),
                "command": command,
            }
        )

    def _request_release(self, reason="manual_release", command=None):
        command = command or self._current_command or {
            "tool_name": "unknown",
            "action": "release",
            "target_mode": "unknown",
        }
        self._publish_task_request(
            {
                "task": "release",
                "reason": reason,
                "tool_name": command.get("tool_name", "unknown"),
                "command": command,
            }
        )

    def _request_home(self, command=None):
        command = command or self._current_command or {
            "tool_name": "unknown",
            "action": "home",
            "target_mode": "unknown",
        }
        self._publish_task_request(
            {
                "task": "home",
                "tool_name": command.get("tool_name", "unknown"),
                "command": command,
            }
        )
        return True

    def _clear_pending_target(self):
        self._target_label = None
        self._current_command = None
        self._task_active = False

    @staticmethod
    def _reset_search_status():
        return None

    def _publish_task_request(self, payload):
        msg = TaskRequest()
        msg.task = str(payload.get("task", ""))
        msg.tool_name = str(payload.get("tool_name", "unknown"))
        msg.reason = str(payload.get("reason", ""))
        msg.command = self._tool_command_message(payload.get("command") or {})
        if all(key in payload for key in ("bx", "by", "bz")):
            msg.has_base_target = True
            msg.bx = float(payload["bx"])
            msg.by = float(payload["by"])
            msg.bz = float(payload["bz"])
        vlm_yaw_deg = payload.get("vlm_yaw_deg")
        if vlm_yaw_deg is not None:
            msg.has_vlm_yaw_deg = True
            msg.vlm_yaw_deg = float(vlm_yaw_deg)
        self.task_request_pub.publish(msg)
        self.get_logger().info(
            f"{TASK_REQUEST_TOPIC} 발행: task={payload.get('task')}, "
            f"tool={payload.get('tool_name', 'unknown')}"
        )

    def _robot_status_cb(self, msg):
        try:
            payload = self._robot_status_payload(msg)
        except json.JSONDecodeError:
            return

        action = str(payload.get("action", "")).strip().lower()
        status = str(payload.get("status", payload.get("state", ""))).strip().lower()
        if action not in self._ROUTED_ACTIONS:
            return

        if status in self._FINAL_STATUSES:
            self._task_active = False
            if action in ("bring", "return", "exit", "cancel"):
                self._target_label = None
                self._current_command = None
            return

        if status in self._ACTIVE_STATUSES:
            self._task_active = True
            command = payload.get("command")
            if isinstance(command, dict):
                self._current_command = command
            tool_name = payload.get("tool_name")
            if tool_name not in (None, "", "unknown"):
                self._target_label = tool_name

    def _publish_robot_status(
        self,
        status,
        tool_name=None,
        action=None,
        message="",
        reason="",
        command=None,
    ):
        self.status_publisher.publish(
            status,
            tool_name=tool_name,
            action=action,
            message=message,
            reason=reason,
            command=command,
        )

    def _publish_status_payload(self, payload):
        msg = RobotTaskStatus()
        msg.status = str(payload.get("status", "unknown"))
        msg.task = str(payload.get("task", ""))
        msg.tool_name = str(payload.get("tool_name", "unknown"))
        msg.action = str(payload.get("action", "unknown"))
        msg.message = str(payload.get("message", ""))
        msg.reason = str(payload.get("reason", ""))
        command = payload.get("command")
        msg.command_json = (
            json.dumps(command, ensure_ascii=False) if command is not None else ""
        )
        msg.payload_json = json.dumps(payload, ensure_ascii=False)
        self.robot_status_pub.publish(msg)

    @staticmethod
    def _tool_command_payload(msg):
        return {
            "action": msg.action,
            "tool_name": msg.tool_name,
            "target_mode": msg.target_mode,
            "raw_text": msg.raw_text,
            "match_method": msg.match_method,
            "confidence": msg.confidence,
        }

    @staticmethod
    def _tool_command_message(command):
        msg = ToolCommand()
        msg.action = str(command.get("action", "unknown"))
        msg.tool_name = str(command.get("tool_name", "unknown"))
        msg.target_mode = str(command.get("target_mode", "unknown"))
        msg.raw_text = str(command.get("raw_text", ""))
        msg.match_method = str(command.get("match_method", "unknown"))
        msg.confidence = float(command.get("confidence", 0.0))
        return msg

    @staticmethod
    def _robot_status_payload(msg):
        payload = json.loads(msg.payload_json) if msg.payload_json else {}
        payload.update(
            {
                "status": msg.status,
                "task": msg.task,
                "tool_name": msg.tool_name,
                "action": msg.action,
                "message": msg.message,
                "reason": msg.reason,
            }
        )
        if msg.command_json:
            payload["command"] = json.loads(msg.command_json)
        return payload


def main():
    rclpy.init()
    node = MacGyvBotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
