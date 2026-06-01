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
    from_matrix=lambda _matrix: types.SimpleNamespace(
        as_quat=lambda: (0.0, 0.0, 0.0, 1.0),
    ),
)
scipy_module.spatial = scipy_spatial_module
scipy_spatial_module.transform = scipy_transform_module
sys.modules.setdefault("scipy", scipy_module)
sys.modules.setdefault("scipy.spatial", scipy_spatial_module)
sys.modules.setdefault("scipy.spatial.transform", scipy_transform_module)

from macgyvbot_config.drawer import DRAWER_STORE_MARKER_CLEARANCE_Z_OFFSET_M
from macgyvbot_config.drawer import DRAWER_STORE_MARKER_EXIT_OFFSET_XYZ_M
from macgyvbot_manipulation.robot_safezone import safe_z_min_for_drawer
from macgyvbot_task.application.pick_flow import pick_handoff_flow
from macgyvbot_task.application.pick_flow.pick_handoff_flow import PickHandoffFlow
from macgyvbot_task.application.pick_flow.pick_sequence import PickSequenceRunner


class FakeLogger:
    def info(self, _message):
        pass

    def warn(self, _message):
        pass

    def error(self, _message):
        pass


class FakeMotion:
    def __init__(self, results=None):
        self.results = list(results or [True, True, True, True, True, True])
        self.targets = []
        self.home_calls = 0

    def plan_and_execute(self, logger, pose_goal=None, state_goal=None):
        self.targets.append(pose_goal)
        return self.results.pop(0)

    def move_to_home_joints(self, logger):
        self.home_calls += 1
        return True


class FakeGripper:
    def __init__(self):
        self.open_calls = 0

    def open_gripper(self):
        self.open_calls += 1


class FakeState:
    def __init__(self):
        self.current_command = {"action": "bring"}
        self.target_label = "screwdriver"
        self.statuses = []

    def logger(self):
        return FakeLogger()

    def _publish_robot_status(self, status, **kwargs):
        self.statuses.append((status, kwargs))


class FakeMatrix:
    def __getitem__(self, key):
        row, column = key
        values = {
            (0, 3): 0.12,
            (1, 3): -0.05,
            (2, 3): 0.50,
        }
        return values[(row, column)]


class FakeHandoff:
    def __init__(self):
        self.returned_drawer_id = None
        self.events = []

    def wait_for_human_grasp(self, logger):
        return False

    def return_tool_to_original_position(
        self,
        target_x,
        target_y,
        travel_z,
        grasp_z,
        ori,
        logger,
        safe_z_min,
        drawer_id=None,
        move_home=True,
        lift_from_current=True,
    ):
        self.returned_drawer_id = drawer_id
        self.lift_from_current = lift_from_current
        self.events.append("return")
        assert not move_home
        return True

    def move_to_handoff_pose(self, ori, logger):
        return None, None, None

    def move_home_after_handoff(self, logger, publish_on_failure=True):
        self.events.append("home")
        return True


class FakeDrawerFlow:
    def __init__(self, close_result=True, events=None):
        self.close_result = close_result
        self.events = events
        self.closed = []

    def close_drawer(self, drawer_id, logger):
        self.closed.append(drawer_id)
        if self.events is not None:
            self.events.append("close")
        return self.close_result


def test_return_tool_to_original_position_uses_drawer_clearance(monkeypatch):
    monkeypatch.setattr(pick_handoff_flow, "get_ee_matrix", lambda _robot: FakeMatrix())
    motion = FakeMotion()
    gripper = FakeGripper()
    flow = PickHandoffFlow(
        robot=object(),
        motion_controller=motion,
        gripper=gripper,
        state=FakeState(),
        wait_fn=lambda _duration: None,
    )
    ori = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}

    assert flow.return_tool_to_original_position(
        0.30,
        0.10,
        0.40,
        0.35,
        ori,
        FakeLogger(),
        safe_z_min=safe_z_min_for_drawer(1),
        drawer_id=1,
    )

    clearance_z = (
        safe_z_min_for_drawer(1)
        + DRAWER_STORE_MARKER_CLEARANCE_Z_OFFSET_M
    )
    positions = [target.pose.position for target in motion.targets]

    assert math.isclose(positions[0].x, 0.12)
    assert math.isclose(positions[0].y, -0.05)
    assert math.isclose(positions[0].z, clearance_z)
    assert math.isclose(positions[1].x, 0.30)
    assert math.isclose(positions[1].y, 0.10)
    assert math.isclose(positions[1].z, clearance_z)
    assert math.isclose(positions[2].z, 0.35)
    assert math.isclose(positions[3].z, clearance_z)
    assert math.isclose(
        positions[4].x,
        0.30 + DRAWER_STORE_MARKER_EXIT_OFFSET_XYZ_M[0],
    )
    assert math.isclose(
        positions[4].y,
        0.10 + DRAWER_STORE_MARKER_EXIT_OFFSET_XYZ_M[1],
    )
    assert math.isclose(
        positions[4].z,
        clearance_z + DRAWER_STORE_MARKER_EXIT_OFFSET_XYZ_M[2],
    )
    assert gripper.open_calls == 1
    assert motion.home_calls == 1


def test_handoff_timeout_return_closes_drawer():
    runner = PickSequenceRunner.__new__(PickSequenceRunner)
    runner.handoff = FakeHandoff()
    runner.drawer_flow = FakeDrawerFlow(events=runner.handoff.events)
    runner.state = FakeState()
    runner.control_events = {}
    plan = types.SimpleNamespace(
        target_x=0.30,
        target_y=0.10,
        travel_z=0.40,
        grasp_z=0.25,
    )
    context = {
        "drawer_id": 1,
        "safe_z_min": safe_z_min_for_drawer(1),
        "ori": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    }

    assert not runner._wait_human_grasp(plan, context)

    assert runner.handoff.returned_drawer_id == 1
    assert runner.handoff.lift_from_current is True
    assert runner.drawer_flow.closed == [1]
    assert runner.handoff.events == ["return", "close", "home"]
    assert runner.state.statuses[-1][0] == "returned"
    assert runner.state.statuses[-1][1]["reason"] == "handoff_timeout"


def test_handoff_timeout_reports_drawer_close_failure():
    runner = PickSequenceRunner.__new__(PickSequenceRunner)
    runner.handoff = FakeHandoff()
    runner.drawer_flow = FakeDrawerFlow(close_result=False)
    runner.state = FakeState()
    runner.control_events = {}
    plan = types.SimpleNamespace(
        target_x=0.30,
        target_y=0.10,
        travel_z=0.40,
        grasp_z=0.25,
    )
    context = {
        "drawer_id": 1,
        "safe_z_min": safe_z_min_for_drawer(1),
        "ori": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    }

    assert not runner._wait_human_grasp(plan, context)

    assert runner.drawer_flow.closed == [1]
    assert runner.state.statuses[-1][0] == "failed"
    assert runner.state.statuses[-1][1]["reason"] == (
        "handoff_timeout_drawer_close_failed"
    )


def test_handoff_search_failure_returns_directly_to_target_clearance():
    runner = PickSequenceRunner.__new__(PickSequenceRunner)
    runner.handoff = FakeHandoff()
    runner.drawer_flow = FakeDrawerFlow(events=runner.handoff.events)
    runner.state = FakeState()
    runner.control_events = {}
    plan = types.SimpleNamespace(
        target_x=0.30,
        target_y=0.10,
        travel_z=0.40,
        grasp_z=0.25,
    )
    context = {
        "drawer_id": 1,
        "safe_z_min": safe_z_min_for_drawer(1),
        "ori": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    }

    assert not runner._move_to_handoff(plan, context)

    assert runner.handoff.returned_drawer_id == 1
    assert runner.handoff.lift_from_current is False
    assert runner.drawer_flow.closed == [1]
    assert runner.handoff.events == ["return", "close", "home"]
    assert runner.state.statuses[-1][0] == "returned"
    assert runner.state.statuses[-1][1]["reason"] == "handoff_pose_unavailable"
