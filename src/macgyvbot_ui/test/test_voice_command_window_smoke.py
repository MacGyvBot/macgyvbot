#!/usr/bin/env python3
"""Smoke tests for the PyQt operator window.

These tests run the GUI in Qt offscreen mode so the layout and helper methods
can be checked in CI-like environments without opening a visible window.
"""

import os
import re
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from macgyvbot_ui.voice_command_window import (  # noqa: E402
    QApplication,
    VoiceCommandGuiWindow,
)


@unittest.skipIf(
    QApplication is None or VoiceCommandGuiWindow is None,
    "PyQt5 is not installed",
)
class VoiceCommandWindowSmokeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.published_texts = []
        self.window = VoiceCommandGuiWindow(
            on_user_text=self.published_texts.append,
        )

    def tearDown(self):
        self.window.close()
        self.app.processEvents()

    def test_window_size_is_fixed(self):
        self.assertEqual(self.window.width(), 1420)
        self.assertEqual(self.window.height(), 900)
        self.assertEqual(self.window.minimumWidth(), 1420)
        self.assertEqual(self.window.maximumWidth(), 1420)
        self.assertEqual(self.window.minimumHeight(), 900)
        self.assertEqual(self.window.maximumHeight(), 900)

    def test_task_log_uses_structured_format(self):
        self.window.append_task_log(
            "warn",
            "명령을 이해하지 못했습니다.",
            source="command.stt",
            event="PARSE_FAILED",
            detail='topic=/command_feedback, raw_text="..."',
        )

        log_text = self.window._task_log.text()
        self.assertRegex(
            log_text,
            re.compile(
                r'^\[\d{2}:\d{2}:\d{2}\] '
                r'\[command\.stt\] \[WARN\] PARSE_FAILED - '
                r'명령을 이해하지 못했습니다\. '
                r'\| topic=/command_feedback, raw_text="\.\.\."$'
            ),
        )

    def test_chat_and_quick_reply_paths_publish_user_text(self):
        self.window.append_bot("손이 인식되었습니다!")
        self.window.append_system("Detector 영상: 수신 중")
        self.window.append_control_actions((("재개", "재개"),))
        self.window._send_quick_reply("재개")

        self.assertEqual(self.published_texts, ["재개"])
        self.assertGreaterEqual(self.window._chat_layout.count(), 5)

    def test_chat_timestamp_includes_seconds(self):
        self.assertRegex(self.window._timestamp(), r"^\d{2}:\d{2}:\d{2}$")


if __name__ == "__main__":
    unittest.main()
