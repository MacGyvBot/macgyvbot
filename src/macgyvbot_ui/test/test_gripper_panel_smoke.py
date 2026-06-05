#!/usr/bin/env python3
"""Smoke tests for the operator GUI gripper panel."""

import os
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
class GripperPanelSmokeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.gripper_widths = []
        self.window = VoiceCommandGuiWindow(
            on_gripper_width=self.gripper_widths.append,
        )

    def tearDown(self):
        self.window.close()
        self.app.processEvents()

    def test_gripper_panel_starts_disabled(self):
        self.assertFalse(self.window._gripper_slider.isEnabled())
        self.assertFalse(self.window._gripper_apply_button.isEnabled())
        self.assertIn("비활성화", self.window._gripper_status.text())

    def test_gripper_panel_enable_and_release_callback(self):
        self.window.set_gripper_control_state(True, "활성화: 수동 조작 가능")
        self.window._gripper_slider.setValue(42)
        self.window._request_gripper_width_from_slider()

        self.assertTrue(self.window._gripper_slider.isEnabled())
        self.assertEqual(self.window._gripper_value.text(), "폭: 42 mm")
        self.assertEqual(self.gripper_widths, [42])

    def test_gripper_panel_disable_reason_is_visible(self):
        reason = "비활성화: 작업 실행 중"
        self.window.set_gripper_control_state(False, reason)

        self.assertFalse(self.window._gripper_slider.isEnabled())
        self.assertEqual(self.window._gripper_status.text(), reason)


if __name__ == "__main__":
    unittest.main()
