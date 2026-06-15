#!/usr/bin/env python3
"""Unit tests for operator status message formatting helpers."""

import unittest
import sys
import types


def _install_ros_stubs():
    if "rclpy" not in sys.modules:
        rclpy = types.ModuleType("rclpy")
        rclpy.init = lambda *args, **kwargs: None
        rclpy.shutdown = lambda *args, **kwargs: None
        rclpy.spin = lambda *args, **kwargs: None
        sys.modules["rclpy"] = rclpy

    if "rclpy.node" not in sys.modules:
        rclpy_node = types.ModuleType("rclpy.node")
        rclpy_node.Node = object
        sys.modules["rclpy.node"] = rclpy_node

    if "sensor_msgs.msg" not in sys.modules:
        sensor_msgs = types.ModuleType("sensor_msgs")
        sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
        sensor_msgs_msg.Image = type("Image", (), {})
        sys.modules["sensor_msgs"] = sensor_msgs
        sys.modules["sensor_msgs.msg"] = sensor_msgs_msg

    if "macgyvbot_interfaces.msg" not in sys.modules:
        interfaces = types.ModuleType("macgyvbot_interfaces")
        interfaces_msg = types.ModuleType("macgyvbot_interfaces.msg")
        for name in (
            "CommandFeedback",
            "CommandShutdown",
            "CommandText",
            "HumanGraspResult",
            "RobotTaskControl",
            "RobotTaskStatus",
            "ToolCommand",
            "ToolDropEvent",
        ):
            setattr(interfaces_msg, name, type(name, (), {}))
        interfaces_srv = types.ModuleType("macgyvbot_interfaces.srv")
        interfaces_srv.SetGripper = type("SetGripper", (), {})
        sys.modules["macgyvbot_interfaces"] = interfaces
        sys.modules["macgyvbot_interfaces.msg"] = interfaces_msg
        sys.modules["macgyvbot_interfaces.srv"] = interfaces_srv


_install_ros_stubs()
from macgyvbot_ui.operator_ui_node import OperatorUiNode


class FakeClock:
    def __init__(self, now_ns=0):
        self.now_ns = now_ns

    def now(self):
        return types.SimpleNamespace(nanoseconds=self.now_ns)


def _status_view_node(clock):
    node = object.__new__(OperatorUiNode)
    node.get_clock = lambda: clock
    node._last_target_label = ""
    node._task_chat_command_key = None
    node._shown_task_chat_stamps = {}
    node._task_chat_dedupe_ns = 10_000_000_000
    node._task_chat_count = 0
    node._task_chat_limit = 5
    node._last_robot_status_key = None
    node._last_robot_log_key = None
    return node


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class FakeWindow:
    def __init__(self):
        self.messages = []
        self.commands = []
        self.actions = []

    def append_bot(self, text):
        self.messages.append(text)

    def append_command_result(self, command):
        self.commands.append(command)

    def append_control_actions(self, actions):
        self.actions.append(tuple(actions))


def _control_node(clock=None):
    node = object.__new__(OperatorUiNode)
    node.window = FakeWindow()
    node._tts_pub = FakePublisher()
    node._task_control_pub = FakePublisher()
    node.logs = []
    node.statuses = []
    node._append_log = lambda *args, **kwargs: node.logs.append((args, kwargs))
    node._set_status = lambda status: node.statuses.append(status)
    if clock is not None:
        node.get_clock = lambda: clock
    return node


class OperatorUiStatusMessageTest(unittest.TestCase):
    def test_failed_drawer_tool_not_found_chat_is_short(self):
        message = OperatorUiNode._robot_status_message(
            OperatorUiNode,
            "failed",
            "screwdriver",
            "screwdriver 서랍에서 기대 공구를 찾지 못했습니다. 감지된 공구: none",
            "pick_drawer_tool_not_found",
        )

        self.assertEqual(message, "screwdriver를 서랍에서 찾지 못했습니다.")

    def test_failed_message_does_not_append_reason(self):
        message = OperatorUiNode._robot_status_message(
            OperatorUiNode,
            "failed",
            "공구",
            "공구 위치 확인에 실패했습니다.",
            "some_internal_reason",
        )

        self.assertEqual(message, "공구 위치 확인에 실패했습니다.")

    def test_compact_status_limits_by_words_not_characters(self):
        status = OperatorUiNode._compact_status_text(
            "VLM grasp model inference is running now"
        )

        self.assertEqual(status, "모델 grasp model inference is running now")
        self.assertNotIn("…", status)

    def test_append_bot_publishes_same_text_for_tts(self):
        node = object.__new__(OperatorUiNode)
        node.window = FakeWindow()
        node._tts_pub = FakePublisher()

        node._append_bot("공구를 받아주세요.")

        self.assertEqual(node.window.messages, ["공구를 받아주세요."])
        self.assertEqual(len(node._tts_pub.messages), 1)
        self.assertEqual(node._tts_pub.messages[0].text, "공구를 받아주세요.")
        self.assertEqual(node._tts_pub.messages[0].source, "operator_ui_chat")

    def test_pause_control_does_not_chat_delivery_ack(self):
        node = _control_node()

        node.publish_control_action("pause", "멈춰")

        self.assertEqual(node._task_control_pub.messages[0].action, "pause")
        self.assertNotIn("정지 요청을 로봇에 전달했습니다.", node.window.messages)
        self.assertEqual(
            node.window.messages,
            ["작업을 재개할까요, 아니면 이번 작업을 취소할까요?"],
        )
        self.assertEqual(len(node._tts_pub.messages), 1)

    def test_resume_control_waits_for_task_status_chat(self):
        node = _control_node()

        node.publish_control_action("resume", "재개")

        self.assertEqual(node._task_control_pub.messages[0].action, "resume")
        self.assertEqual(node.window.messages, [])
        self.assertEqual(node._tts_pub.messages, [])

    def test_pause_feedback_does_not_chat_delivery_ack(self):
        clock = FakeClock(now_ns=1_000_000_000)
        node = _control_node(clock)
        node._last_feedback_key = None
        node._last_feedback_stamp_ns = 0
        node._feedback_dedupe_ns = 1_000_000_000
        node._feedback_payload = lambda msg: {
            "status": "accepted",
            "reason": "",
            "message": "정지 요청을 로봇에 전달했습니다.",
            "raw_text": "멈춰",
            "command": {"action": "pause", "tool_name": "", "raw_text": "멈춰"},
        }

        node._feedback_cb(object())

        self.assertNotIn("정지 요청을 로봇에 전달했습니다.", node.window.messages)
        self.assertEqual(
            node.window.messages,
            ["작업을 재개할까요, 아니면 이번 작업을 취소할까요?"],
        )

    def test_resume_feedback_does_not_chat_delivery_ack(self):
        clock = FakeClock(now_ns=1_000_000_000)
        node = _control_node(clock)
        node._last_feedback_key = None
        node._last_feedback_stamp_ns = 0
        node._feedback_dedupe_ns = 1_000_000_000
        node._feedback_payload = lambda msg: {
            "status": "accepted",
            "reason": "",
            "message": "재개 요청을 로봇에 전달했습니다.",
            "raw_text": "재개",
            "command": {"action": "resume", "tool_name": "", "raw_text": "재개"},
        }

        node._feedback_cb(object())

        self.assertEqual(node.window.messages, [])
        self.assertEqual(node._tts_pub.messages, [])

    def test_resumed_status_is_user_chat_message(self):
        clock = FakeClock(now_ns=1_000_000_000)
        node = _status_view_node(clock)

        view = node._build_robot_status_view(
            {
                "status": "resumed",
                "action": "bring",
                "tool_name": "screwdriver",
                "command": {
                    "action": "bring",
                    "tool_name": "screwdriver",
                    "raw_text": "드라이버 가져다줘",
                },
            }
        )

        self.assertEqual(view["chat_message"], "작업을 재개합니다.")
        self.assertTrue(view["show_chat"])

    def test_failed_drawer_error_stays_out_of_chat(self):
        clock = FakeClock(now_ns=1_000_000_000)
        node = _status_view_node(clock)

        view = node._build_robot_status_view(
            {
                "status": "failed",
                "action": "bring",
                "tool_name": "screwdriver",
                "reason": "drawer_prepare_failed",
                "message": "서랍 열기 실패",
                "command": {
                    "action": "bring",
                    "tool_name": "screwdriver",
                    "raw_text": "드라이버 가져다줘",
                },
            }
        )

        self.assertEqual(view["message"], "서랍을 여는 중 문제가 발생했습니다.")
        self.assertEqual(view["chat_message"], "")
        self.assertFalse(view["show_chat"])
        self.assertTrue(view["show_log"])

    def test_same_task_chat_reopens_after_dedupe_window(self):
        clock = FakeClock(now_ns=1_000_000_000)
        node = _status_view_node(clock)
        status = {
            "status": "handoff_inspection_pending",
            "action": "bring",
            "tool_name": "screwdriver",
            "reason": "handoff_search_failed",
            "command": {
                "action": "bring",
                "tool_name": "screwdriver",
                "raw_text": "screwdriver bring",
            },
        }

        first = node._build_robot_status_view(status)
        second = node._build_robot_status_view(status)
        clock.now_ns += 10_000_000_000
        third = node._build_robot_status_view(status)

        self.assertTrue(first["show_chat"])
        self.assertFalse(second["show_chat"])
        self.assertTrue(third["show_chat"])

    def test_same_event_chat_reopens_after_dedupe_window(self):
        class FakeWindow:
            def __init__(self):
                self.messages = []

            def append_bot(self, text):
                self.messages.append(text)

        class FakePublisher:
            def __init__(self):
                self.messages = []

            def publish(self, msg):
                self.messages.append(msg)

        clock = FakeClock(now_ns=1_000_000_000)
        node = object.__new__(OperatorUiNode)
        node.get_clock = lambda: clock
        node.window = FakeWindow()
        node._tts_pub = FakePublisher()
        node._last_event_chat_key = None
        node._last_event_chat_stamp_ns = 0
        node._event_chat_dedupe_ns = 10_000_000_000

        self.assertTrue(node._append_event_chat("handoff_inspection_pending", "retry?"))
        self.assertFalse(node._append_event_chat("handoff_inspection_pending", "retry?"))
        clock.now_ns += 10_000_000_000
        self.assertTrue(node._append_event_chat("handoff_inspection_pending", "retry?"))
        self.assertEqual(node.window.messages, ["retry?", "retry?"])


if __name__ == "__main__":
    unittest.main()
