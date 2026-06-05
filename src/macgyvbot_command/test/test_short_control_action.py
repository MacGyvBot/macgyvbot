#!/usr/bin/env python3
"""Unit tests for immediate short control command matching."""

import unittest

from macgyvbot_command.input_mapping.command_hard_parser import (
    find_short_control_action,
)


class ShortControlActionTest(unittest.TestCase):
    def test_short_pause_commands_match_immediately(self):
        for text in ("멈춰", "멈춰줘", "정지해", "스탑"):
            with self.subTest(text=text):
                self.assertEqual(find_short_control_action(text), "pause")

    def test_short_resume_and_home_commands_match_immediately(self):
        cases = {
            "재개": "resume",
            "재개해": "resume",
            "계속해": "resume",
            "복귀해": "home",
            "홈으로 가": "home",
            "원위치로 가줘": "home",
        }
        for text, action in cases.items():
            with self.subTest(text=text):
                self.assertEqual(find_short_control_action(text), action)

    def test_negated_or_long_control_text_uses_full_parser(self):
        for text in ("멈추지마", "정지하지마", "작업 중단하지 말고 계속 진행해"):
            with self.subTest(text=text):
                self.assertEqual(find_short_control_action(text), "")


if __name__ == "__main__":
    unittest.main()
