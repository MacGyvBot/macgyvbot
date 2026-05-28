import sys
import types


class DummyPoseStamped:
    def __init__(self):
        self.header = types.SimpleNamespace(frame_id="")
        self.pose = types.SimpleNamespace(
            position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
            orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )


rclpy_module = types.ModuleType("rclpy")
rclpy_module.ok = lambda: True
sys.modules.setdefault("rclpy", rclpy_module)

geometry_module = types.ModuleType("geometry_msgs")
geometry_msg_module = types.ModuleType("geometry_msgs.msg")
geometry_msg_module.PoseStamped = DummyPoseStamped
geometry_module.msg = geometry_msg_module
sys.modules.setdefault("geometry_msgs", geometry_module)
sys.modules.setdefault("geometry_msgs.msg", geometry_msg_module)

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

from macgyvbot_task.application.return_flow import return_home_placement_flow
from macgyvbot_task.application.return_flow.return_home_placement_flow import (
    ReturnHomePlacementFlow,
)


class FakeLogger:
    def info(self, message):
        pass

    def warn(self, message):
        pass

    def error(self, message):
        pass


class FakeMotion:
    def __init__(self, home_results=None, plan_results=None):
        self.home_results = list(home_results or [True, True])
        self.plan_results = list(plan_results or [True])
        self.home_calls = 0
        self.plan_calls = 0

    def move_to_home_joints(self, logger):
        self.home_calls += 1
        return self.home_results.pop(0)

    def plan_and_execute(self, logger, pose_goal=None, state_goal=None):
        self.plan_calls += 1
        return self.plan_results.pop(0)


class FakeReporter:
    def __init__(self):
        self.published = []
        self.failed = []

    def publish(self, status, tool_name, message, command, reason=""):
        self.published.append((status, tool_name, message, command, reason))

    def fail(self, tool_name, message, reason, command, logger):
        self.failed.append((tool_name, message, reason, command))


class FakeForceDetector:
    def __init__(self, stop_z):
        self.stop_z = stop_z

    def descend_until_z_reaction(self, target_x, target_y, start_z, ori, logger):
        return self.stop_z


class FakeGripper:
    def __init__(self):
        self.open_calls = 0

    def open_gripper(self):
        self.open_calls += 1


class FakeMatrix:
    def __getitem__(self, key):
        row, column = key
        values = {
            (0, 3): 0.4,
            (1, 3): 0.1,
            (2, 3): 0.5,
        }
        return values[(row, column)]


def make_flow(home_results=None, plan_results=None, stop_z=0.3):
    motion = FakeMotion(home_results, plan_results)
    flow = ReturnHomePlacementFlow(
        robot=object(),
        motion_controller=motion,
        gripper=FakeGripper(),
        state=types.SimpleNamespace(latest_wrench=None),
        reporter=FakeReporter(),
        wait_fn=lambda _duration: None,
    )
    flow.force_detector = FakeForceDetector(stop_z)
    return flow


def run_place(flow):
    return flow.place_at_robot_home(
        "hammer",
        {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        {},
        FakeLogger(),
    )


def test_home_move_failure_reports_reason(monkeypatch):
    monkeypatch.setattr(
        return_home_placement_flow,
        "get_ee_matrix",
        lambda _robot: FakeMatrix(),
    )
    flow = make_flow(home_results=[False])

    assert not run_place(flow)
    assert flow.reporter.failed[-1][2] == "return_home_move_failed"


def test_force_descent_failure_reports_reason(monkeypatch):
    monkeypatch.setattr(
        return_home_placement_flow,
        "get_ee_matrix",
        lambda _robot: FakeMatrix(),
    )
    flow = make_flow(stop_z=None)

    assert not run_place(flow)
    assert flow.reporter.failed[-1][2] == "return_home_descent_failed"


def test_retreat_failure_reports_reason(monkeypatch):
    monkeypatch.setattr(
        return_home_placement_flow,
        "get_ee_matrix",
        lambda _robot: FakeMatrix(),
    )
    flow = make_flow(plan_results=[False])

    assert not run_place(flow)
    assert flow.reporter.failed[-1][2] == "return_home_retreat_failed"


def test_final_home_failure_reports_reason(monkeypatch):
    monkeypatch.setattr(
        return_home_placement_flow,
        "get_ee_matrix",
        lambda _robot: FakeMatrix(),
    )
    flow = make_flow(home_results=[True, False])

    assert not run_place(flow)
    assert flow.reporter.failed[-1][2] == "return_home_after_release_failed"


def test_success_opens_gripper_and_returns_home(monkeypatch):
    monkeypatch.setattr(
        return_home_placement_flow,
        "get_ee_matrix",
        lambda _robot: FakeMatrix(),
    )
    flow = make_flow()

    assert run_place(flow)
    assert flow.gripper.open_calls == 1
    assert flow.motion.home_calls == 2
    assert flow.motion.plan_calls == 1
