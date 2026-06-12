#!/usr/bin/env python3
"""Smoke tests for the operator GUI gripper panel."""

import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from macgyvbot_ui.voice_command_window import (  # noqa: E402
    QApplication,
    QFrame,
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
        self.published_texts = []
        self.control_actions = []
        self.window = VoiceCommandGuiWindow(
            on_user_text=self.published_texts.append,
            on_gripper_width=self.gripper_widths.append,
            on_control_action=lambda action, text: self.control_actions.append(
                (action, text)
            ),
        )

    def tearDown(self):
        self.window.close()
        self.app.processEvents()

    def test_gripper_panel_starts_disabled(self):
        self.assertFalse(self.window._gripper_slider.isEnabled())
        self.assertFalse(self.window._gripper_width_input.isEnabled())
        self.assertFalse(self.window._gripper_apply_button.isEnabled())
        self.assertIn("비활성화", self.window._gripper_status.text())

    def test_gripper_panel_slider_only_updates_value(self):
        self.window.set_gripper_control_state(True, "활성화: 수동 조작 가능")
        self.window._gripper_slider.setValue(42)

        self.assertTrue(self.window._gripper_slider.isEnabled())
        self.assertTrue(self.window._gripper_width_input.isEnabled())
        self.assertEqual(self.window._gripper_width_input.value(), 42)
        self.assertEqual(self.gripper_widths, [])

    def test_gripper_panel_apply_button_sends_current_width_once(self):
        self.window.set_gripper_control_state(True, "활성화: 수동 조작 가능")
        self.window._gripper_slider.setValue(42)
        self.window._request_gripper_width_from_slider()

        self.assertEqual(self.gripper_widths, [42])

    def test_gripper_panel_numeric_input_updates_slider(self):
        self.window.set_gripper_control_state(True, "활성화: 수동 조작 가능")
        self.window._gripper_width_input.setValue(17)

        self.assertEqual(self.window._gripper_slider.value(), 17)
        self.assertEqual(self.window._gripper_width_input.value(), 17)
        self.assertEqual(self.gripper_widths, [])

    def test_gripper_panel_disable_reason_is_visible(self):
        reason = "비활성화: 작업 실행 중"
        self.window.set_gripper_control_state(False, reason)

        self.assertFalse(self.window._gripper_slider.isEnabled())
        self.assertFalse(self.window._gripper_width_input.isEnabled())
        self.assertEqual(self.window._gripper_status.text(), reason)

    def test_chat_input_can_be_disabled_without_blocking_control_buttons(self):
        self.window.set_chat_input_enabled(False, "동작 실행 중...")
        self.window._input.setText("드라이버 가져다줘")
        self.window._send_text()

        self.assertFalse(self.window._input.isEnabled())
        self.assertFalse(self.window._send_button.isEnabled())
        self.assertEqual(self.published_texts, [])

        self.window._send_control_text("멈춰")

        self.assertEqual(self.published_texts, ["멈춰"])

    def test_chat_input_reenable_restores_text_send_path(self):
        self.window.set_chat_input_enabled(False, "동작 실행 중...")
        self.window.set_chat_input_enabled(True)
        self.window._input.setText("드라이버 가져다줘")
        self.window._send_text()

        self.assertTrue(self.window._input.isEnabled())
        self.assertTrue(self.window._send_button.isEnabled())
        self.assertEqual(self.published_texts, ["드라이버 가져다줘"])

    def test_enabled_chat_input_can_show_waiting_placeholder(self):
        message = "로봇 노드 실행 대기 중입니다. 실행 후 명령을 입력해주세요."
        self.window.set_chat_input_enabled(True, message)

        self.assertTrue(self.window._input.isEnabled())
        self.assertTrue(self.window._send_button.isEnabled())
        self.assertEqual(self.window._input.placeholderText(), message)

    def test_pause_and_resume_status_buttons_publish_control_actions(self):
        self.window._pause_button.click()
        self.window._resume_button.click()

        self.assertEqual(self.published_texts, [])
        self.assertEqual(
            self.control_actions,
            [("pause", "멈춰"), ("resume", "재개")],
        )

    def test_cancel_status_button_publishes_control_action(self):
        self.window._cancel_button.click()

        self.assertEqual(self.published_texts, [])
        self.assertEqual(self.control_actions, [("cancel", "취소")])

    def test_power_button_publishes_exit_text(self):
        self.window._power_button.click()

        self.assertEqual(self.published_texts, ["종료"])
        self.assertEqual(self.control_actions, [])

    def test_gripper_width_input_has_label(self):
        self.assertEqual(self.window._gripper_width_label.text(), "그리퍼 폭:")

    def test_handoff_fallback_button_publishes_cancel_control_action(self):
        self.window.append_control_actions((("복귀", "복귀", "cancel"),))
        buttons = [
            button
            for button in self.window.findChildren(type(self.window._pause_button))
            if button.text() == "복귀"
        ]
        self.assertTrue(buttons)

        buttons[-1].click()

        self.assertEqual(self.published_texts, [])
        self.assertEqual(self.control_actions, [("cancel", "복귀")])

    def test_gripper_panel_has_room_for_controls(self):
        self.window.show()
        self.app.processEvents()

        gripper_panel = self.window.findChild(QFrame, "gripperPanel")
        self.assertIsNotNone(gripper_panel)
        self.assertGreaterEqual(gripper_panel.height(), 245)


if __name__ == "__main__":
    unittest.main()
