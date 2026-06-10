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
    from_matrix=lambda _matrix: types.SimpleNamespace(
        as_quat=lambda: (0.0, 0.0, 0.0, 1.0),
    ),
)
scipy_module.spatial = scipy_spatial_module
scipy_spatial_module.transform = scipy_transform_module
sys.modules.setdefault("scipy", scipy_module)
sys.modules.setdefault("scipy.spatial", scipy_spatial_module)
sys.modules.setdefault("scipy.spatial.transform", scipy_transform_module)

from macgyvbot_task.application.recovery.recovery_utils import (  # noqa: E402
    RecoveryConfig,
    normalize_tool_name,
)
from macgyvbot_task.application.recovery import pick_recovery  # noqa: E402
from macgyvbot_task.application.recovery import return_recovery  # noqa: E402


class FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, message, **fields):
        self.messages.append(("info", message, fields))

    def warn(self, message, **fields):
        self.messages.append(("warn", message, fields))

    def error(self, message, **fields):
        self.messages.append(("error", message, fields))


class FakeStatus:
    def __init__(self):
        self.recovery_mode = False
        self.held_tool = "pliers"
        self.target_tool = "wrench"
        self.target_label = "screwdriver"
        self.drawer_open = False
        self.opened_drawer_id = None
        self.gripper_holding = False

    def logger(self):
        return FakeLogger()


class FakeDrawer:
    def __init__(self):
        self.events = []

    def drawer_id_for_tool(self, tool_name):
        return {"pliers": 0, "wrench": 2}.get(tool_name)

    def open_drawer(self, drawer_id, _logger):
        self.events.append(("open", drawer_id))
        return True

    def observe_drawer(self, drawer_id, _logger):
        self.events.append(("observe", drawer_id))
        return True

    def close_drawer(self, drawer_id, _logger):
        self.events.append(("close", drawer_id))
        return True


class FakeMotion:
    def move_to_home_joints(self, _logger):
        return True


class FakeMonitor:
    def __init__(self):
        self.starts = []

    def start(self, tool_name, action, command=None):
        self.starts.append((tool_name, action, command))


def test_normalize_tool_name_uses_unknown_for_missing_values():
    assert normalize_tool_name(None) == "unknown"
    assert normalize_tool_name("  ") == "unknown"
    assert normalize_tool_name(" wrench ") == "wrench"


def test_return_recovery_prefers_held_tool_and_observes_open_drawer(monkeypatch):
    status = FakeStatus()
    drawer = FakeDrawer()
    placed = []
    logger = FakeLogger()
    detection = types.SimpleNamespace(found=True, base_xyz=(0.3, 0.1, 0.2))
    config = RecoveryConfig(
        state=status,
        inspection_pose_fn=lambda *_args: True,
        detect_target_fn=lambda target: detection,
        drawer_marker_target_fn=lambda drawer_id: types.SimpleNamespace(
            found=True,
            base_xyz=(0.4, 0.2, 0.1),
            drawer_id=drawer_id,
        ),
        place_tool_fn=lambda marker, tool, _logger: placed.append(
            (marker.drawer_id, tool)
        )
        or True,
    )

    monkeypatch.setattr(return_recovery, "is_graspable", lambda *_args: True)
    monkeypatch.setattr(return_recovery, "attempt_grasp", lambda *_args, **_kwargs: True)
    monkeypatch.setattr(return_recovery, "return_home", lambda *_args: True)

    ok = return_recovery.run_return_recovery(
        status,
        [],
        None,
        object(),
        None,
        object(),
        drawer,
        None,
        config,
        logger,
    )

    assert ok
    assert placed == [(0, "pliers")]
    assert drawer.events == [("open", 0), ("observe", 0), ("close", 0)]
    assert status.held_tool is None
    assert status.gripper_holding is False
    assert status.recovery_mode is False


def test_pick_recovery_observes_target_then_stores_in_original_drawer(monkeypatch):
    status = FakeStatus()
    status.target_tool = "wrench"
    status.target_label = "wrench"
    status.held_tool = "pliers"
    drawer = FakeDrawer()
    monitor = FakeMonitor()
    logger = FakeLogger()
    first_detection = types.SimpleNamespace(
        found=True,
        base_xyz=(0.3, 0.1, 0.2),
        yaw_deg=None,
    )
    final_detection = types.SimpleNamespace(
        found=True,
        base_xyz=(0.31, 0.11, 0.2),
        yaw_deg=42.0,
    )
    calls = []
    placed = []
    config = RecoveryConfig(
        state=status,
        command={"tool_name": "wrench", "action": "bring"},
        inspection_pose_fn=lambda *_args: True,
        initial_detect_target_fn=lambda target: calls.append(("initial", target))
        or first_detection,
        target_observe_fn=lambda detection, target, _logger: calls.append(
            ("observe", target, detection.base_xyz)
        )
        or True,
        detect_target_fn=lambda target: calls.append(("final", target))
        or final_detection,
        drawer_marker_target_fn=lambda drawer_id: types.SimpleNamespace(
            found=True,
            drawer_id=drawer_id,
            base_xyz=(0.4, 0.2, 0.1),
        ),
        place_tool_fn=lambda marker, tool, _logger: placed.append(
            (marker.drawer_id, tool)
        )
        or True,
        tool_hold_monitor=monitor,
    )

    monkeypatch.setattr(pick_recovery, "is_graspable", lambda *_args: True)

    def fake_attempt_grasp(detection, *_args, **kwargs):
        calls.append(("grasp", kwargs["target_tool"], detection.yaw_deg))
        return True

    monkeypatch.setattr(pick_recovery, "attempt_grasp", fake_attempt_grasp)

    ok = pick_recovery.run_pick_recovery(
        status,
        [],
        None,
        FakeMotion(),
        None,
        object(),
        drawer,
        config,
        logger,
    )

    assert ok
    assert calls == [
        ("initial", "wrench"),
        ("observe", "wrench", (0.3, 0.1, 0.2)),
        ("final", "wrench"),
        ("grasp", "wrench", 42.0),
    ]
    assert monitor.starts == [("wrench", "bring", config.command)]
    assert placed == [(2, "wrench")]
    assert drawer.events == [("open", 2), ("observe", 2), ("close", 2)]
    assert status.held_tool is None
    assert status.gripper_holding is False
    assert status.recovery_mode is False
