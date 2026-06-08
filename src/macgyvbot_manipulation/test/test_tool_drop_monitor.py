import unittest

from macgyvbot_manipulation.tool_drop_monitor import (
    GripperHoldState,
    classify_tool_drop_sample,
)


class TestToolDropMonitorPolicy(unittest.TestCase):
    def test_detects_drop_when_grip_signal_is_lost(self):
        reason = classify_tool_drop_sample(
            GripperHoldState(
                busy=False,
                grip_detected=False,
                status=[0, 0, 0, 0, 0, 0, 0],
                width_mm=18.0,
            ),
            baseline_width_mm=22.0,
        )

        self.assertEqual(reason, "grip_detected_signal_lost")

    def test_detects_thimble_empty_width_even_if_grip_signal_remains(self):
        reason = classify_tool_drop_sample(
            GripperHoldState(
                busy=False,
                grip_detected=True,
                status=[0, 1, 0, 0, 0, 0, 0],
                width_mm=11.5,
            ),
            baseline_width_mm=22.0,
            empty_closed_width_threshold_mm=12.0,
        )

        self.assertEqual(reason, "width_reached_empty_closed_threshold")

    def test_detects_large_width_release_from_held_baseline(self):
        reason = classify_tool_drop_sample(
            GripperHoldState(
                busy=False,
                grip_detected=True,
                status=[0, 1, 0, 0, 0, 0, 0],
                width_mm=14.0,
            ),
            baseline_width_mm=22.0,
            empty_closed_width_threshold_mm=12.0,
            release_delta_mm=5.0,
        )

        self.assertEqual(reason, "width_reduced_from_held_baseline")

    def test_keeps_hold_when_width_is_stable_and_grip_signal_remains(self):
        reason = classify_tool_drop_sample(
            GripperHoldState(
                busy=False,
                grip_detected=True,
                status=[0, 1, 0, 0, 0, 0, 0],
                width_mm=20.0,
            ),
            baseline_width_mm=22.0,
            empty_closed_width_threshold_mm=12.0,
            release_delta_mm=5.0,
        )

        self.assertIsNone(reason)

    def test_ignores_busy_motion_sample(self):
        reason = classify_tool_drop_sample(
            GripperHoldState(
                busy=True,
                grip_detected=False,
                status=[1, 0, 0, 0, 0, 0, 0],
                width_mm=10.0,
            ),
            baseline_width_mm=22.0,
        )

        self.assertIsNone(reason)


if __name__ == "__main__":
    unittest.main()
