import sys
import types
import unittest


class DummyPoseStamped:
    def __init__(self):
        self.header = types.SimpleNamespace(frame_id="")
        self.pose = types.SimpleNamespace(
            position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
            orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )


geometry_msgs_module = types.ModuleType("geometry_msgs")
geometry_msgs_msg_module = types.ModuleType("geometry_msgs.msg")
geometry_msgs_msg_module.PoseStamped = DummyPoseStamped
geometry_msgs_module.msg = geometry_msgs_msg_module
sys.modules.setdefault("geometry_msgs", geometry_msgs_module)
sys.modules.setdefault("geometry_msgs.msg", geometry_msgs_msg_module)

moveit_module = types.ModuleType("moveit")
moveit_core_module = types.ModuleType("moveit.core")
moveit_robot_state_module = types.ModuleType("moveit.core.robot_state")
moveit_robot_state_module.RobotState = type("RobotState", (), {})
moveit_module.core = moveit_core_module
moveit_core_module.robot_state = moveit_robot_state_module
sys.modules.setdefault("moveit", moveit_module)
sys.modules.setdefault("moveit.core", moveit_core_module)
sys.modules.setdefault("moveit.core.robot_state", moveit_robot_state_module)

scipy_module = types.ModuleType("scipy")
scipy_spatial_module = types.ModuleType("scipy.spatial")
scipy_transform_module = types.ModuleType("scipy.spatial.transform")
scipy_transform_module.Rotation = types.SimpleNamespace(
    from_matrix=lambda matrix: types.SimpleNamespace(
        as_quat=lambda: (0.0, 0.0, 0.0, 1.0),
    ),
)
scipy_module.spatial = scipy_spatial_module
scipy_spatial_module.transform = scipy_transform_module
sys.modules.setdefault("scipy", scipy_module)
sys.modules.setdefault("scipy.spatial", scipy_spatial_module)
sys.modules.setdefault("scipy.spatial.transform", scipy_transform_module)

from macgyvbot_config.drawer import (  # noqa: E402
    DRAWER_CLOSE_PREPARE_WRIST_YAW_DEG,
)
from macgyvbot_manipulation.drawer_motion import DrawerMotionFlow  # noqa: E402


class FakeLogger:
    def info(self, message):
        pass

    def warn(self, message):
        pass


class FakeMotion:
    def __init__(self):
        self.yaws = []

    def rotate_wrist_by_yaw_deg(self, yaw_deg, logger, collision_scene_key=None):
        self.yaws.append((yaw_deg, collision_scene_key))
        return True


class TestDrawerMotionClose(unittest.TestCase):
    def _flow(self, motion):
        flow = DrawerMotionFlow(
            robot=object(),
            motion_controller=motion,
            gripper=object(),
            wait_fn=lambda _duration: None,
        )
        flow._opened_drawers[1] = {
            "xyz": (0.5, 0.1, 0.3),
            "ori": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        }
        flow._move_to_handle_pose_with_preapproach = lambda *args, **kwargs: True
        flow._close_gripper = lambda *args, **kwargs: True
        flow._move_by_offset = lambda *args, **kwargs: True
        flow._open_gripper = lambda *args, **kwargs: True
        return flow

    def test_close_prepare_rotates_j6_positive_before_close_pose(self):
        motion = FakeMotion()
        flow = self._flow(motion)

        self.assertTrue(flow._prepare_wrist_for_close(1, FakeLogger()))
        self.assertEqual(
            motion.yaws,
            [
                (
                    DRAWER_CLOSE_PREPARE_WRIST_YAW_DEG,
                    "drawer/close_prepare_wrist",
                )
            ],
        )

    def test_close_drawer_does_not_prepare_wrist_by_default(self):
        motion = FakeMotion()
        flow = self._flow(motion)

        self.assertTrue(flow.close_drawer(1, FakeLogger()))
        self.assertEqual(motion.yaws, [])

    def test_close_drawer_prepares_wrist_when_requested(self):
        motion = FakeMotion()
        flow = self._flow(motion)

        self.assertTrue(flow.close_drawer(1, FakeLogger(), prepare_wrist=True))
        self.assertEqual(
            motion.yaws,
            [
                (
                    DRAWER_CLOSE_PREPARE_WRIST_YAW_DEG,
                    "drawer/close_prepare_wrist",
                )
            ],
        )


if __name__ == "__main__":
    unittest.main()
