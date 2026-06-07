#!/usr/bin/env python3
"""Unit tests for wrench command vocabulary and local parsing."""

import unittest

from macgyvbot_command.input_mapping.command_hard_parser import (
    find_action,
    find_tool,
)
from macgyvbot_command.input_mapping.command_llm_parser import CommandLlmParser


class WrenchCommandVocabularyTest(unittest.TestCase):
    def test_wrench_aliases_resolve_to_wrench(self):
        for text in ("렌치 가져다줘", "스패너 줘", "몽키스패너 가져와", "육각 렌치 줘"):
            with self.subTest(text=text):
                tool_name, method, _score, _keyword = find_tool(text)
                self.assertEqual(tool_name, "wrench")
                self.assertEqual(method, "alias")

    def test_wrench_actions_preserve_bring_and_return(self):
        self.assertEqual(find_action("렌치 가져다줘"), "bring")
        self.assertEqual(find_action("스패너 서랍에 넣어줘"), "return")

    def test_local_parser_infers_wrench_from_function_words(self):
        parser = CommandLlmParser(
            ollama_url="",
            model="",
            timeout_sec=1.0,
            min_confidence=0.7,
            use_local_parser=True,
            use_llm_fallback=False,
        )
        cases = {
            "볼트 조이는 거 가져와": ("wrench", "bring"),
            "너트 푸는 공구 줘": ("wrench", "bring"),
            "육각 조이는 공구 정리해": ("wrench", "return"),
        }
        for text, expected in cases.items():
            with self.subTest(text=text):
                result = parser.interpret(text)
                command = result.get("command") or {}
                self.assertEqual(command.get("tool_name"), expected[0])
                self.assertEqual(command.get("action"), expected[1])

    def test_drill_remains_unsupported(self):
        parser = CommandLlmParser(
            ollama_url="",
            model="",
            timeout_sec=1.0,
            min_confidence=0.7,
            use_local_parser=True,
            use_llm_fallback=False,
        )
        result = parser.interpret("드릴 가져와")
        self.assertIsNone(result.get("command"))
        reasons = [
            feedback.get("reason")
            for feedback in result.get("feedbacks", [])
        ]
        self.assertIn("local_parser_failed", reasons)


if __name__ == "__main__":
    unittest.main()
