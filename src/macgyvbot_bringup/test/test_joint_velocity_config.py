import unittest
import xml.etree.ElementTree as ET

from macgyvbot_bringup.joint_velocity_config import (
    apply_joint_velocity_limits_to_moveit_config,
    apply_joint_velocity_limits_to_urdf,
    validate_joint_velocity_limits,
)


def _robot_description():
    joints = "\n".join(
        f"""
        <joint name="joint_{index}" type="revolute">
          <limit effort="1" lower="-1" upper="1" velocity="1.0" />
        </joint>
        """
        for index in range(1, 7)
    )
    return f"<robot name=\"test_robot\">{joints}</robot>"


class JointVelocityConfigPatchTest(unittest.TestCase):
    def test_joint_velocity_config_rejects_missing_joint(self):
        with self.assertRaisesRegex(ValueError, "Missing joint velocity limits"):
            validate_joint_velocity_limits({"joint_1": 1.0})

    def test_apply_joint_velocity_limits_to_urdf(self):
        limits = {f"joint_{index}": index * 0.5 for index in range(1, 7)}

        updated = apply_joint_velocity_limits_to_urdf(
            _robot_description(),
            limits,
        )
        root = ET.fromstring(updated)

        for joint_name, velocity in limits.items():
            joint = root.find(f".//joint[@name='{joint_name}']")
            self.assertEqual(joint.find("limit").get("velocity"), str(velocity))

    def test_apply_joint_velocity_limits_to_moveit_config(self):
        limits = {f"joint_{index}": index * 0.5 for index in range(1, 7)}
        moveit_config = {
            "robot_description": _robot_description(),
            "robot_description_planning": {
                "joint_limits": {
                    "joint_1": {
                        "has_velocity_limits": True,
                        "max_velocity": 1.0,
                    },
                },
            },
        }

        updated = apply_joint_velocity_limits_to_moveit_config(
            moveit_config,
            limits,
        )

        self.assertNotEqual(
            moveit_config["robot_description"],
            updated["robot_description"],
        )
        joint_limits = updated["robot_description_planning"]["joint_limits"]
        for joint_name, velocity in limits.items():
            self.assertTrue(joint_limits[joint_name]["has_velocity_limits"])
            self.assertEqual(joint_limits[joint_name]["max_velocity"], velocity)
