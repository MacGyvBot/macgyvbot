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


rclpy_module = types.ModuleType("rclpy")
rclpy_module.ok = lambda: True
sys.modules.setdefault("rclpy", rclpy_module)

moveit_module = types.ModuleType("moveit")
moveit_core_module = types.ModuleType("moveit.core")
moveit_robot_state_module = types.ModuleType("moveit.core.robot_state")
moveit_robot_state_module.RobotState = DummyRobotState
moveit_module.core = moveit_core_module
moveit_core_module.robot_state = moveit_robot_state_module
sys.modules.setdefault("moveit", moveit_module)
sys.modules.setdefault("moveit.core", moveit_core_module)
sys.modules.setdefault("moveit.core.robot_state", moveit_robot_state_module)

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

from macgyvbot_task.application.return_flow import return_staging_placement_flow
from macgyvbot_task.application.return_flow.return_staging_placement_flow import (
    ReturnStagingPlacementFlow,
)


class FakeLogger:
    def info(self, message):
        pass

    def warn(self, message):
        pass

    def error(self, message):
        pass


class FakeRobot:
    def get_robot_model(self):
        return object()


class FakeMotion:
    def __init__(self, plan_results=None):
        self.plan_results = list(plan_results or [True, True, True, True])
        self.plan_calls = 0

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


def patch_pose_helpers(monkeypatch):
    monkeypatch.setattr(
        return_staging_placement_flow,
        "get_ee_matrix",
        lambda _robot: FakeMatrix(),
    )
    monkeypatch.setattr(
        return_staging_placement_flow,
        "current_ee_orientation",
        lambda _robot: {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    )


def make_flow(plan_results=None, stop_z=0.3):
    motion = FakeMotion(plan_results)
    flow = ReturnStagingPlacementFlow(
        robot=FakeRobot(),
        motion_controller=motion,
        gripper=FakeGripper(),
        state=types.SimpleNamespace(latest_wrench=None),
        reporter=FakeReporter(),
        wait_fn=lambda _duration: None,
    )
    flow.force_detector = FakeForceDetector(stop_z)
    return flow


def run_place(flow):
    return flow.place_at_store_observe_point(
        "hammer",
        {},
        FakeLogger(),
    )


def test_store_observe_move_failure_reports_reason(monkeypatch):
    patch_pose_helpers(monkeypatch)
    flow = make_flow(plan_results=[False])

    assert not run_place(flow)
    assert flow.reporter.failed[-1][2] == "return_store_observe_move_failed"


def test_force_descent_failure_reports_reason(monkeypatch):
    patch_pose_helpers(monkeypatch)
    flow = make_flow(stop_z=None)

    assert not run_place(flow)
    assert flow.reporter.failed[-1][2] == "return_store_observe_descent_failed"


def test_retreat_failure_reports_reason(monkeypatch):
    patch_pose_helpers(monkeypatch)
    flow = make_flow(plan_results=[True, True, False])

    assert not run_place(flow)
    assert flow.reporter.failed[-1][2] == "return_store_observe_retreat_failed"


def test_final_store_observe_failure_reports_reason(monkeypatch):
    patch_pose_helpers(monkeypatch)
    flow = make_flow(plan_results=[True, True, True, False])

    assert not run_place(flow)
    assert flow.reporter.failed[-1][2] == "return_store_observe_after_release_failed"


def test_success_opens_gripper_and_returns_to_store_observe(monkeypatch):
    patch_pose_helpers(monkeypatch)
    flow = make_flow()

    assert run_place(flow)
    assert flow.gripper.open_calls == 1
    assert flow.motion.plan_calls == 4
