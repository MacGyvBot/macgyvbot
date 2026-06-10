#!/usr/bin/env python3
"""Unit tests for abnormal-event GUI chat mapping."""

import unittest

from macgyvbot_ui.event_chat import (
    BRING_DONE_MESSAGE,
    BRING_RETURNED_HOME_MESSAGE,
    BRING_WAIT_HANDOFF_MESSAGE,
    GRASP_RETRY_MESSAGE,
    HAND_NOT_FOUND_MESSAGE,
    PARSE_FAILED_MESSAGE,
    RETURN_DONE_MESSAGE,
    RETURN_SEARCH_HAND_MESSAGE,
    RETURN_STORE_START_MESSAGE,
    RETURN_WAIT_TOOL_MESSAGE,
    TOOL_DROPPED_MESSAGE,
    command_feedback_chat,
    hand_detection_chat,
    normal_robot_status_chat,
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
            robot_status_chat("waiting_handoff"),
            "",
        )
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

    def test_normal_bring_status_messages_are_minimal(self):
        self.assertEqual(
            normal_robot_status_chat("searching_hand", "bring"),
            BRING_WAIT_HANDOFF_MESSAGE,
        )
        self.assertEqual(
            normal_robot_status_chat("waiting_handoff", "bring"),
            "",
        )
        self.assertEqual(
            normal_robot_status_chat("done", "bring"),
            BRING_DONE_MESSAGE,
        )
        self.assertEqual(
            normal_robot_status_chat("returned", "bring"),
            BRING_RETURNED_HOME_MESSAGE,
        )
        self.assertEqual(normal_robot_status_chat("moving_to_drawer", "bring"), "")

    def test_normal_return_status_messages_are_minimal(self):
        self.assertEqual(
            normal_robot_status_chat("moving_return_grasp_pose", "return"),
            RETURN_SEARCH_HAND_MESSAGE,
        )
        self.assertEqual(
            normal_robot_status_chat("checking_return_target", "return"),
            RETURN_WAIT_TOOL_MESSAGE,
        )
        self.assertEqual(
            normal_robot_status_chat("grasp_success", "return"),
            RETURN_STORE_START_MESSAGE,
        )
        self.assertEqual(
            normal_robot_status_chat("done", "return"),
            RETURN_DONE_MESSAGE,
        )
        self.assertEqual(normal_robot_status_chat("opening_drawer", "return"), "")

    def test_tool_drop_event_message(self):
        self.assertEqual(tool_drop_chat("tool_dropped"), TOOL_DROPPED_MESSAGE)
        self.assertEqual(tool_drop_chat("monitor_started"), "")


if __name__ == "__main__":
    unittest.main()
