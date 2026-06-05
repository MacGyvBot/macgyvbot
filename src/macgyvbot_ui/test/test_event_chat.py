#!/usr/bin/env python3
"""Unit tests for abnormal-event GUI chat mapping."""

import unittest

from macgyvbot_ui.event_chat import (
    GRASP_RETRY_MESSAGE,
    HAND_NOT_FOUND_MESSAGE,
    PARSE_FAILED_MESSAGE,
    TOOL_DROPPED_MESSAGE,
    command_feedback_chat,
    hand_detection_chat,
    robot_status_chat,
    tool_drop_chat,
)


class EventChatTest(unittest.TestCase):
    def test_command_parse_failure_message(self):
        self.assertEqual(
            command_feedback_chat("rejected", "llm_failed"),
            PARSE_FAILED_MESSAGE,
        )
        self.assertEqual(
            command_feedback_chat("accepted", "command_accepted"),
            "",
        )

    def test_raw_hand_detection_transitions_do_not_chat(self):
        self.assertEqual(hand_detection_chat(False, True), "")
        self.assertEqual(hand_detection_chat(True, False), "")
        self.assertEqual(hand_detection_chat(True, True), "")

    def test_robot_status_abnormal_messages(self):
        self.assertEqual(
            robot_status_chat("failed", "handoff_search_failed"),
            HAND_NOT_FOUND_MESSAGE,
        )
        self.assertEqual(
            robot_status_chat("grasping", "", "공구 grasp 시도 2/5"),
            GRASP_RETRY_MESSAGE,
        )
        self.assertEqual(
            robot_status_chat("grasping", "", "공구 grasp 시도 1/5"),
            "",
        )
        self.assertEqual(
            robot_status_chat("tool_dropped"),
            TOOL_DROPPED_MESSAGE,
        )

    def test_tool_drop_event_message(self):
        self.assertEqual(tool_drop_chat("tool_dropped"), TOOL_DROPPED_MESSAGE)
        self.assertEqual(tool_drop_chat("monitor_started"), "")


if __name__ == "__main__":
    unittest.main()
