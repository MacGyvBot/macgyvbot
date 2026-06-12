#!/usr/bin/env python3
"""Unit tests for command node TTS routing scope."""

import sys
import types
import unittest


def _module(name, **attrs):
    module = types.ModuleType(name)
    for key, value in attrs.items():
        setattr(module, key, value)
    return module


def _install_import_stubs():
    sys.modules.setdefault(
        "rclpy",
        _module(
            "rclpy",
            ok=lambda: True,
            init=lambda *args, **kwargs: None,
            shutdown=lambda: None,
            spin=lambda *_args, **_kwargs: None,
        ),
    )
    sys.modules.setdefault("rclpy.node", _module("rclpy.node", Node=object))

    class _ToolCommand:
        def __init__(self):
            self.action = ""
            self.tool_name = ""
            self.target_mode = ""
            self.raw_text = ""
            self.match_method = ""
            self.confidence = 0.0

    class _CommandFeedback:
        def __init__(self):
            self.status = ""
            self.reason = ""
            self.message = ""
            self.raw_text = ""
            self.command = _ToolCommand()

    class _RobotTaskStatus:
        def __init__(self):
            self.status = ""
            self.task = ""
            self.tool_name = ""
            self.action = ""
            self.message = ""
            self.reason = ""
            self.command = _ToolCommand()

    class _CommandText:
        def __init__(self):
            self.text = ""
            self.source = ""

    msg_module = _module(
        "macgyvbot_interfaces.msg",
        CommandFeedback=_CommandFeedback,
        CommandShutdown=type("CommandShutdown", (), {}),
        CommandText=_CommandText,
        RobotTaskControl=type("RobotTaskControl", (), {}),
        RobotTaskStatus=_RobotTaskStatus,
        ToolCommand=_ToolCommand,
    )
    sys.modules.setdefault(
        "macgyvbot_interfaces",
        _module("macgyvbot_interfaces", msg=msg_module),
    )
    sys.modules.setdefault("macgyvbot_interfaces.msg", msg_module)


_install_import_stubs()

from macgyvbot_command.command_input_node import CommandInputNode  # noqa: E402
from macgyvbot_interfaces.msg import CommandText, RobotTaskStatus  # noqa: E402


class _FakeParser:
    def __init__(self):
        self.statuses = []

    def update_robot_status(self, status):
        self.statuses.append(status)


class CommandInputTtsScopeTest(unittest.TestCase):
    def _make_node(self):
        node = object.__new__(CommandInputNode)
        node._parser = _FakeParser()
        node._last_robot_state = "unknown"
        node._exit_pending = False
        node.spoken = []
        node._speak_bot = node.spoken.append
        return node

    def test_robot_status_updates_context_without_tts(self):
        node = self._make_node()
        msg = RobotTaskStatus()
        msg.status = "moving_to_handoff"
        msg.action = "bring"
        msg.message = "공구를 받아주세요."

        node._robot_status_cb(msg)

        self.assertEqual(node._last_robot_state, "moving_to_handoff")
        self.assertEqual(node._parser.statuses[-1]["status"], "moving_to_handoff")
        self.assertEqual(node.spoken, [])

    def test_tts_text_drives_tts(self):
        node = self._make_node()
        msg = CommandText()
        msg.text = "드라이버를 가져오라는 뜻으로 이해했습니다."
        msg.source = "operator_ui_chat"

        node._tts_text_cb(msg)

        self.assertEqual(
            node.spoken,
            ["드라이버를 가져오라는 뜻으로 이해했습니다."],
        )


if __name__ == "__main__":
    unittest.main()
