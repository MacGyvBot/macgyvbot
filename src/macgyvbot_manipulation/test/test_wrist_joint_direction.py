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


action_msgs_module = types.ModuleType("action_msgs")
action_msgs_msg_module = types.ModuleType("action_msgs.msg")
action_msgs_msg_module.GoalStatus = types.SimpleNamespace()
action_msgs_module.msg = action_msgs_msg_module
sys.modules.setdefault("action_msgs", action_msgs_module)
sys.modules.setdefault("action_msgs.msg", action_msgs_msg_module)

control_msgs_module = types.ModuleType("control_msgs")
control_msgs_action_module = types.ModuleType("control_msgs.action")
control_msgs_action_module.FollowJointTrajectory = types.SimpleNamespace()
control_msgs_module.action = control_msgs_action_module
sys.modules.setdefault("control_msgs", control_msgs_module)
sys.modules.setdefault("control_msgs.action", control_msgs_action_module)

geometry_msgs_module = types.ModuleType("geometry_msgs")
geometry_msgs_msg_module = types.ModuleType("geometry_msgs.msg")
geometry_msgs_msg_module.PoseStamped = DummyPoseStamped
geometry_msgs_module.msg = geometry_msgs_msg_module
sys.modules.setdefault("geometry_msgs", geometry_msgs_module)
sys.modules.setdefault("geometry_msgs.msg", geometry_msgs_msg_module)

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

moveit_module = types.ModuleType("moveit")
moveit_core_module = types.ModuleType("moveit.core")
moveit_robot_state_module = types.ModuleType("moveit.core.robot_state")
moveit_robot_state_module.RobotState = type("RobotState", (), {})
moveit_module.core = moveit_core_module
moveit_core_module.robot_state = moveit_robot_state_module
sys.modules.setdefault("moveit", moveit_module)
sys.modules.setdefault("moveit.core", moveit_core_module)
sys.modules.setdefault("moveit.core.robot_state", moveit_robot_state_module)

rclpy_module = types.ModuleType("rclpy")
rclpy_module.ok = lambda: True
rclpy_action_module = types.ModuleType("rclpy.action")
rclpy_action_module.ActionClient = type("ActionClient", (), {})
rclpy_module.action = rclpy_action_module
sys.modules.setdefault("rclpy", rclpy_module)
sys.modules.setdefault("rclpy.action", rclpy_action_module)

import macgyvbot_manipulation.moveit_controller as moveit_controller  # noqa: E402
from macgyvbot_manipulation.moveit_controller import (  # noqa: E402
    MoveItController,
    _negative_first_equivalent_values,
)


class FakeLogger:
    def __init__(self):
        self.errors = []
        self.infos = []
        self.warnings = []

    def error(self, message):
        self.errors.append(message)

    def info(self, message):
        self.infos.append(message)

    def warn(self, message):
        self.warnings.append(message)


class FakeDrawerCollisionScene:
    def __init__(self, ready=True):
        self.ready = ready
        self.profile_keys = []
        self.ready_profiles = []

    def profile_for_scene_key(self, scene_key):
        self.profile_keys.append(scene_key)
        return "drawer_only"

    def is_ready(self, profile):
        self.ready_profiles.append(profile)
        return self.ready

    def ensure_ready_for_scene_key(self, *_args, **_kwargs):
        raise AssertionError("planning precondition must not refresh drawer scene")


class FakeRobot:
    def get_robot_model(self):
        return object()


class FakeRobotState:
    def __init__(self, _robot_model):
        self.joint_positions = None

    def update(self):
        pass


def test_positive_final_angle_prefers_principal_rotation_candidate():
    current = math.radians(0.0)
    target = math.radians(10.0)

    candidates = _negative_first_equivalent_values(current, target)
    deltas = [math.degrees(candidate - current) for candidate in candidates]

    assert math.isclose(deltas[0], 10.0)
    assert any(math.isclose(delta, -350.0) for delta in deltas)


def test_negative_final_angle_keeps_short_negative_rotation_first():
    current = math.radians(0.0)
    target = math.radians(-10.0)

    candidates = _negative_first_equivalent_values(current, target)
    deltas = [math.degrees(candidate - current) for candidate in candidates]

    assert math.isclose(deltas[0], -10.0)
    assert any(math.isclose(delta, 350.0) for delta in deltas)


def test_equivalent_current_angle_stays_first():
    current = math.radians(30.0)
    target = math.radians(390.0)

    candidates = _negative_first_equivalent_values(current, target)
    deltas = [math.degrees(candidate - current) for candidate in candidates]

    assert math.isclose(deltas[0], 0.0, abs_tol=1e-6)


def test_large_positive_angle_prefers_principal_equivalent_before_wide_rotation():
    current = math.radians(0.0)
    target = current + math.radians(140.0)

    candidates = _negative_first_equivalent_values(current, target)
    deltas = [math.degrees(candidate - current) for candidate in candidates]

    assert math.isclose(deltas[0], 140.0)
    assert any(math.isclose(delta, -220.0) for delta in deltas)
    assert any(math.isclose(delta, 140.0) for delta in deltas)


def test_nonprincipal_target_prefers_equivalent_within_half_turn():
    current = math.radians(0.0)
    target = math.radians(260.834756723043)

    candidates = _negative_first_equivalent_values(current, target)
    deltas = [math.degrees(candidate - current) for candidate in candidates]

    assert math.isclose(deltas[0], -99.16524327695704)
    assert any(math.isclose(delta, 260.834756723043) for delta in deltas)


def test_drawer_scene_precondition_checks_scene_key_without_refresh():
    drawer_scene = FakeDrawerCollisionScene(ready=True)
    controller = MoveItController(
        robot=None,
        arm=None,
        params=None,
        drawer_collision_scene=drawer_scene,
        enable_gripper_self_collision_acm=False,
    )

    assert controller._ensure_drawer_collision_scene_ready(
        FakeLogger(),
        collision_scene_key="handoff/move_to_user",
    )
    assert drawer_scene.profile_keys == ["handoff/move_to_user"]
    assert drawer_scene.ready_profiles == ["drawer_only"]


def test_drawer_scene_precondition_blocks_when_initial_scene_missing():
    drawer_scene = FakeDrawerCollisionScene(ready=False)
    logger = FakeLogger()
    controller = MoveItController(
        robot=None,
        arm=None,
        params=None,
        drawer_collision_scene=drawer_scene,
        enable_gripper_self_collision_acm=False,
    )

    assert not controller._ensure_drawer_collision_scene_ready(
        logger,
        collision_scene_key="drawer/approach_to_close",
    )
    assert drawer_scene.profile_keys == ["drawer/approach_to_close"]
    assert drawer_scene.ready_profiles == ["drawer_only"]
    assert logger.errors


def test_home_move_interrupt_skips_settle_wait(monkeypatch):
    controller = MoveItController(
        robot=FakeRobot(),
        arm=None,
        params=None,
        enable_gripper_self_collision_acm=False,
    )
    logger = FakeLogger()
    settle_calls = []

    monkeypatch.setattr(moveit_controller, "RobotState", FakeRobotState)
    monkeypatch.setattr(controller, "_is_at_joint_goal", lambda *_args, **_kwargs: False)
    monkeypatch.setattr(controller, "plan_and_execute", lambda *_args, **_kwargs: False)
    monkeypatch.setattr(
        controller,
        "_wait_until_at_joint_goal",
        lambda *_args, **_kwargs: settle_calls.append(True) or True,
    )
    controller.should_interrupt = lambda: True

    assert not controller.move_to_home_joints(logger)
    assert settle_calls == []


def test_joint_goal_settle_wait_stops_on_interrupt(monkeypatch):
    controller = MoveItController(
        robot=FakeRobot(),
        arm=None,
        params=None,
        enable_gripper_self_collision_acm=False,
    )
    logger = FakeLogger()
    goal_checks = []

    monkeypatch.setattr(
        controller,
        "_is_at_joint_goal",
        lambda *_args, **_kwargs: goal_checks.append(True) or False,
    )
    controller.should_interrupt = lambda: True

    assert not controller._wait_until_at_joint_goal(
        {},
        logger,
        timeout_sec=1.0,
    )
    assert goal_checks == []
