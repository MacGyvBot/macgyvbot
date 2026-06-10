import unittest

from macgyvbot_config.joint_velocity import (
    JOINT_VELOCITY_LIMITS_RAD_PER_SEC,
    MOTION_VELOCITY_SCALING_FACTOR,
)


class JointVelocityConfigTest(unittest.TestCase):
    def test_joint_velocity_config_covers_arm_joints(self):
        self.assertEqual(
            set(JOINT_VELOCITY_LIMITS_RAD_PER_SEC),
            {
                "joint_1",
                "joint_2",
                "joint_3",
                "joint_4",
                "joint_5",
                "joint_6",
            },
        )

    def test_joint_velocity_values_are_positive(self):
        for velocity in JOINT_VELOCITY_LIMITS_RAD_PER_SEC.values():
            self.assertGreater(velocity, 0.0)

    def test_motion_velocity_scaling_factor_is_positive(self):
        self.assertGreater(MOTION_VELOCITY_SCALING_FACTOR, 0.0)
