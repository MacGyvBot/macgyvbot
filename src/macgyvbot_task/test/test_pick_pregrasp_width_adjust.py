import math
import sys
import types


class DummyPoseStamped:
    def __init__(self):
        self.header = types.SimpleNamespace(frame_id="")
        self.pose = types.SimpleNamespace(
            position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
            orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )


class DummyRobotState:
    def __init__(self, *_args, **_kwargs):
        self.joint_positions = {}

    def update(self):
        pass

    def get_global_link_transform(self, _link):
        return [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]


rclpy_module = types.ModuleType("rclpy")
rclpy_module.ok = lambda: True
sys.modules.setdefault("rclpy", rclpy_module)

geometry_module = types.ModuleType("geometry_msgs")
geometry_msg_module = types.ModuleType("geometry_msgs.msg")
geometry_msg_module.PoseStamped = DummyPoseStamped
geometry_module.msg = geometry_msg_module
sys.modules.setdefault("geometry_msgs", geometry_module)
sys.modules.setdefault("geometry_msgs.msg", geometry_msg_module)

moveit_module = types.ModuleType("moveit")
moveit_core_module = types.ModuleType("moveit.core")
moveit_robot_state_module = types.ModuleType("moveit.core.robot_state")
moveit_robot_state_module.RobotState = DummyRobotState
moveit_module.core = moveit_core_module
moveit_core_module.robot_state = moveit_robot_state_module
sys.modules.setdefault("moveit", moveit_module)
sys.modules.setdefault("moveit.core", moveit_core_module)
sys.modules.setdefault("moveit.core.robot_state", moveit_robot_state_module)

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

from macgyvbot_config.grasp import PREGRASP_MAX_EXTRA_DESCENT_M
from macgyvbot_task.application.pick_flow.pick_grasp_flow import (
    calculate_limited_grasp_width_mm,
    calculate_pregrasp_extra_descent,
)
from macgyvbot_task.application.pick_flow.pick_sequence import PickSequenceRunner


class FakeLogger:
    def info(self, _message):
        pass

    def warn(self, _message):
        pass

    def error(self, _message):
        pass


class FakeState:
    target_label = "driver"
    current_command = {"action": "bring"}
    grasp_detection_width_mm = 20.0
    grasp_detection_width_target = "driver"

    def logger(self):
        return FakeLogger()

    def _publish_robot_status(self, *_args, **_kwargs):
        pass


class FakeGripper:
    max_width = 800

    def __init__(self):
        self.open_calls = 0
        self.move_calls = []

    def open_gripper(self):
        self.open_calls += 1

    def move_gripper(self, width_raw):
        self.move_calls.append(width_raw)


class FakeGrasp:
    def measure_pregrasp_depth(self, _logger):
        return {
            "width_mm": 11.0,
            "depth_mm": 12.0,
        }


class FakeMotion:
    def __init__(self):
        self.targets = []

    def plan_and_execute(self, _logger, pose_goal=None, **_kwargs):
        self.targets.append(pose_goal)
        return True


def test_pregrasp_extra_descent_uses_actual_depth_mm():
    assert math.isclose(calculate_pregrasp_extra_descent(12.0), 0.012)


def test_pregrasp_extra_descent_uses_absolute_depth():
    assert math.isclose(calculate_pregrasp_extra_descent(-12.0), 0.012)


def test_pregrasp_extra_descent_caps_depth_compensation():
    assert calculate_pregrasp_extra_descent(120.0) == PREGRASP_MAX_EXTRA_DESCENT_M


def test_limited_grasp_width_adds_per_side_offset():
    assert calculate_limited_grasp_width_mm(20.0, max_width_mm=80.0) == 30.0


def test_pregrasp_reopen_uses_limited_width(monkeypatch):
    monkeypatch.setattr(
        "macgyvbot_task.application.pick_flow.pick_sequence.cooperative_wait",
        lambda _duration: None,
    )
    runner = PickSequenceRunner.__new__(PickSequenceRunner)
    runner.state = FakeState()
    runner.gripper = FakeGripper()
    runner.grasp = FakeGrasp()
    runner.motion = FakeMotion()
    runner._interrupted = lambda: False

    plan = type(
        "Plan",
        (),
        {
            "target_x": 0.3,
            "target_y": 0.1,
            "grasp_z": 0.4,
        },
    )()
    context = {
        "plan": plan,
        "ori": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        "safe_z_min": 0.2,
    }

    assert runner._pregrasp_depth_adjust(context)
    assert runner.gripper.open_calls == 0
    assert runner.gripper.move_calls == [300]


def test_grasp_descent_applies_limited_width():
    runner = PickSequenceRunner.__new__(PickSequenceRunner)
    runner.state = FakeState()
    runner.gripper = FakeGripper()

    runner._set_limited_gripper_width_for_descent()

    assert runner.gripper.open_calls == 0
    assert runner.gripper.move_calls == [300]
