import unittest

from macgyvbot_manipulation.grasp_verifier import read_grasp_confirmation


class FakeGripper:
    def __init__(self, status, width):
        self.status = status
        self.width = width

    def get_status(self):
        return self.status

    def get_width(self):
        if isinstance(self.width, Exception):
            raise self.width
        return self.width


class FakeLogger:
    def info(self, message):
        pass

    def warn(self, message):
        pass


class TestGripperGraspConfirmation(unittest.TestCase):
    def test_rejects_fully_closed_gripper_even_when_grip_detected(self):
        confirmed, busy, status, width = read_grasp_confirmation(
            FakeGripper([0, 1, 0, 0, 0, 0, 0], 0.0),
            FakeLogger(),
        )

        self.assertFalse(confirmed)
        self.assertFalse(busy)
        self.assertEqual(status[1], 1)
        self.assertEqual(width, 0.0)

    def test_accepts_grip_detected_with_open_width(self):
        confirmed, busy, status, width = read_grasp_confirmation(
            FakeGripper([0, 1, 0, 0, 0, 0, 0], 20.0),
            FakeLogger(),
        )

        self.assertTrue(confirmed)
        self.assertFalse(busy)
        self.assertEqual(status[1], 1)
        self.assertEqual(width, 20.0)

    def test_rejects_grip_detected_while_motion_is_busy(self):
        confirmed, busy, status, width = read_grasp_confirmation(
            FakeGripper([1, 1, 0, 0, 0, 0, 0], 20.0),
            FakeLogger(),
        )

        self.assertFalse(confirmed)
        self.assertTrue(busy)
        self.assertEqual(status[1], 1)
        self.assertEqual(width, 20.0)


if __name__ == "__main__":
    unittest.main()
