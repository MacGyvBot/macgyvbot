import sys
import types
import unittest


scipy_module = types.ModuleType("scipy")
scipy_spatial_module = types.ModuleType("scipy.spatial")
scipy_transform_module = types.ModuleType("scipy.spatial.transform")
scipy_transform_module.Rotation = types.SimpleNamespace(
    from_matrix=lambda _matrix: types.SimpleNamespace(
        as_quat=lambda: (0.0, 0.0, 0.0, 1.0),
    ),
)
scipy_module.spatial = scipy_spatial_module
scipy_spatial_module.transform = scipy_transform_module
sys.modules.setdefault("scipy", scipy_module)
sys.modules.setdefault("scipy.spatial", scipy_spatial_module)
sys.modules.setdefault("scipy.spatial.transform", scipy_transform_module)

geometry_module = types.ModuleType("geometry_msgs")
geometry_msg_module = types.ModuleType("geometry_msgs.msg")
geometry_msg_module.PoseStamped = object
geometry_module.msg = geometry_msg_module
sys.modules.setdefault("geometry_msgs", geometry_module)
sys.modules.setdefault("geometry_msgs.msg", geometry_msg_module)

moveit_module = types.ModuleType("moveit")
moveit_core_module = types.ModuleType("moveit.core")
moveit_robot_state_module = types.ModuleType("moveit.core.robot_state")
moveit_robot_state_module.RobotState = object
moveit_module.core = moveit_core_module
moveit_core_module.robot_state = moveit_robot_state_module
sys.modules.setdefault("moveit", moveit_module)
sys.modules.setdefault("moveit.core", moveit_core_module)
sys.modules.setdefault("moveit.core.robot_state", moveit_robot_state_module)

from macgyvbot_task.application.robot import robot_home_initializer
from macgyvbot_task.application.robot.robot_home_initializer import (
    RobotHomeInitializer,
)


class FakeLogger:
    def __init__(self):
        self.errors = []
        self.infos = []

    def info(self, message):
        self.infos.append(message)

    def error(self, message):
        self.errors.append(message)


class FakeState:
    def __init__(self):
        self.logger = FakeLogger()
        self.home_xyz = None
        self.home_ori = None

    def get_logger(self):
        return self.logger


class FakeMotion:
    def __init__(self, result=True, exc=None):
        self.result = result
        self.exc = exc

    def move_to_home_joints(self, _logger):
        if self.exc is not None:
            raise self.exc
        return self.result


class FakeMatrix:
    def __getitem__(self, key):
        if isinstance(key, tuple) and len(key) == 2:
            row, column = key
            if isinstance(row, slice) or isinstance(column, slice):
                return [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ]
            values = {
                (0, 3): 0.1,
                (1, 3): 0.2,
                (2, 3): 0.3,
            }
            return values[(row, column)]
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


class RobotHomeInitializerTest(unittest.TestCase):
    def test_motion_exception_returns_false(self):
        state = FakeState()
        initializer = RobotHomeInitializer(
            robot=object(),
            motion_controller=FakeMotion(exc=RuntimeError("no action server")),
            state=state,
        )

        self.assertFalse(initializer.initialize())
        self.assertIsNone(state.home_xyz)
        self.assertTrue(state.logger.errors)

    def test_pose_exception_returns_false(self):
        state = FakeState()
        original = robot_home_initializer.get_ee_matrix
        robot_home_initializer.get_ee_matrix = (
            lambda _robot: (_ for _ in ()).throw(RuntimeError("no tf"))
        )
        try:
            initializer = RobotHomeInitializer(
                robot=object(),
                motion_controller=FakeMotion(),
                state=state,
            )

            self.assertFalse(initializer.initialize())
            self.assertIsNone(state.home_xyz)
            self.assertTrue(state.logger.errors)
        finally:
            robot_home_initializer.get_ee_matrix = original

    def test_success_stores_home_pose(self):
        state = FakeState()
        original = robot_home_initializer.get_ee_matrix
        robot_home_initializer.get_ee_matrix = lambda _robot: FakeMatrix()
        try:
            initializer = RobotHomeInitializer(
                robot=object(),
                motion_controller=FakeMotion(),
                state=state,
            )

            self.assertTrue(initializer.initialize())
            self.assertEqual(state.home_xyz, (0.1, 0.2, 0.3))
            self.assertEqual(state.home_ori["w"], 1.0)
        finally:
            robot_home_initializer.get_ee_matrix = original


if __name__ == "__main__":
    unittest.main()
