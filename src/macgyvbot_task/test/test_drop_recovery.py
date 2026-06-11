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
    attempt_grasp,
    normalize_tool_name,
)
import macgyvbot_task.application.recovery.recovery_utils as recovery_utils  # noqa: E402
from macgyvbot_config.return_flow import RETURN_DRAWER_PLACE_WRIST_YAW_DEG  # noqa: E402
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


class FakeRecoveryGripper:
    def __init__(self):
        self.events = []

    def open_gripper(self):
        self.events.append("open")

    def close_gripper(self):
        self.events.append("close")

    def get_status(self):
        return [False, True]

    def get_width(self):
        return 24.0


class FakeStatus:
    def __init__(self):
        self.recovery_mode = False
        self.held_tool = "pliers"
        self.target_tool = "wrench"
        self.target_label = "screwdriver"
        self.drawer_open = False
        self.opened_drawer_id = None
        self.gripper_holding = False
        self.tool_mask_locked = True
        self.last_tool_mask_lock_result = {"locked": True}
        self.grasp_detection_mask_images = ["locked-image"]
        self.grasp_detection_mask_target = "pliers"
        self.grasp_detection_yaw_deg = 17.0
        self.grasp_detection_yaw_target = "pliers"
        self.status_updates = []

    def logger(self):
        return FakeLogger()

    def _publish_robot_status(self, status, **kwargs):
        self.status_updates.append((status, kwargs))


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
    def __init__(self):
        self.wrist_rotations = []
        self.events = []

    def move_to_home_joints(self, _logger):
        return True

    def rotate_wrist_by_yaw_deg(self, yaw_deg, _logger, **_kwargs):
        self.events.append(("yaw", yaw_deg))
        self.wrist_rotations.append(yaw_deg)
        return True

    def plan_and_execute(self, _logger, pose_goal, collision_scene_key=None, **_kwargs):
        self.events.append(
            (
                "move",
                collision_scene_key,
                pose_goal.pose.position.x,
                pose_goal.pose.position.y,
                pose_goal.pose.position.z,
            )
        )
        return True


class FakeRecoveryPlanner:
    def plan(self, *_args, **_kwargs):
        return types.SimpleNamespace(
            target_x=0.31,
            target_y=0.12,
            drawer_wall_clearance_z=0.42,
            grasp_z=0.25,
            should_descend_to_grasp=True,
        )


class FakeGripper:
    def __init__(self):
        self.open_calls = 0

    def open_gripper(self):
        self.open_calls += 1


class FakeMonitor:
    def __init__(self):
        self.starts = []

    def start(self, tool_name, action, command=None):
        self.starts.append((tool_name, action, command))


def test_normalize_tool_name_uses_unknown_for_missing_values():
    assert normalize_tool_name(None) == "unknown"
    assert normalize_tool_name("  ") == "unknown"
    assert normalize_tool_name(" wrench ") == "wrench"


def test_recovery_grasp_rotates_yaw_before_xy_align_and_z_descent(monkeypatch):
    monkeypatch.setattr(
        recovery_utils,
        "current_ee_orientation",
        lambda _robot: {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    )
    motion = FakeMotion()
    gripper = FakeRecoveryGripper()
    detection = types.SimpleNamespace(
        found=True,
        base_xyz=(0.30, 0.10, 0.20),
        yaw_deg=37.0,
    )

    ok = attempt_grasp(
        detection,
        motion,
        FakeRecoveryPlanner(),
        gripper,
        config=RecoveryConfig(robot=object(), wait_fn=lambda _duration: None),
        target_tool="wrench",
    )

    assert ok
    assert motion.events == [
        ("yaw", 37.0),
        ("move", "recovery/grasp_xy_align", 0.31, 0.12, 0.42),
        ("move", "recovery/grasp_descent", 0.31, 0.12, 0.25),
    ]
    assert gripper.events == ["open", "close"]


def test_return_recovery_prefers_held_tool_and_observes_open_drawer(monkeypatch):
    status = FakeStatus()
    drawer = FakeDrawer()
    gripper = FakeGripper()
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
        gripper,
        drawer,
        None,
        config,
        logger,
    )

    assert ok
    assert placed == [(0, "pliers")]
    assert drawer.events == [("open", 0), ("observe", 0), ("close", 0)]
    assert gripper.open_calls == 1
    assert status.held_tool is None
    assert status.gripper_holding is False
    assert status.recovery_mode is False
    assert status.tool_mask_locked is False
    assert status.last_tool_mask_lock_result is None
    assert status.grasp_detection_yaw_deg is None
    assert [
        update[1].get("reason")
        for update in status.status_updates
    ][:3] == [
        "drop_recovery_started",
        "moving_to_recovery_inspection",
        "detecting_recovery_target",
    ]


def test_return_recovery_updates_label_then_redetects_with_grasp_yaw(monkeypatch):
    status = FakeStatus()
    status.held_tool = "unknown"
    status.target_tool = "wrench"
    status.target_label = None
    drawer = FakeDrawer()
    gripper = FakeGripper()
    motion = FakeMotion()
    monitor = FakeMonitor()
    logger = FakeLogger()
    first_detection = types.SimpleNamespace(
        found=True,
        base_xyz=(0.3, 0.1, 0.2),
        yaw_deg=None,
    )
    final_detection = types.SimpleNamespace(
        found=True,
        base_xyz=(0.32, 0.12, 0.2),
        yaw_deg=31.0,
    )
    calls = []
    placed = []
    config = RecoveryConfig(
        state=status,
        command={"tool_name": "wrench", "action": "return"},
        inspection_pose_fn=lambda *_args: True,
        initial_detect_target_fn=lambda target: calls.append(("initial", target))
        or first_detection,
        target_observe_fn=lambda detection, target, _logger: calls.append(
            ("observe", target, detection.base_xyz)
        )
        or True,
        observed_tool_label_fn=lambda: "pliers",
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

    monkeypatch.setattr(return_recovery, "is_graspable", lambda *_args: True)

    def fake_attempt_grasp(detection, *_args, **kwargs):
        calls.append(("grasp", kwargs["target_tool"], detection.yaw_deg))
        return True

    monkeypatch.setattr(return_recovery, "attempt_grasp", fake_attempt_grasp)
    monkeypatch.setattr(return_recovery, "return_home", lambda *_args: True)

    ok = return_recovery.run_return_recovery(
        status,
        [],
        None,
        motion,
        None,
        gripper,
        drawer,
        None,
        config,
        logger,
    )

    assert ok
    assert calls == [
        ("initial", "wrench"),
        ("observe", "wrench", (0.3, 0.1, 0.2)),
        ("observe", "pliers", (0.3, 0.1, 0.2)),
        ("final", "pliers"),
        ("grasp", "pliers", 31.0),
    ]
    assert placed == [(0, "pliers")]
    assert drawer.events == [("open", 0), ("observe", 0), ("close", 0)]
    assert gripper.open_calls == 1
    assert monitor.starts == [("pliers", "return", config.command)]
    assert status.held_tool is None


def test_return_recovery_skips_drawer_open_when_drawer_already_open(monkeypatch):
    status = FakeStatus()
    status.held_tool = "pliers"
    status.drawer_open = True
    status.opened_drawer_id = 0
    drawer = FakeDrawer()
    gripper = FakeGripper()
    monitor = FakeMonitor()
    logger = FakeLogger()
    detection = types.SimpleNamespace(
        found=True,
        base_xyz=(0.3, 0.1, 0.2),
        yaw_deg=19.0,
    )
    calls = []
    placed = []
    config = RecoveryConfig(
        state=status,
        command={"tool_name": "unknown", "action": "return"},
        inspection_pose_fn=lambda *_args: True,
        initial_detect_target_fn=lambda target: calls.append(("initial", target))
        or detection,
        target_observe_fn=lambda _detection, target, _logger: calls.append(
            ("observe_target", target)
        )
        or True,
        observed_tool_label_fn=lambda: "pliers",
        detect_target_fn=lambda target: calls.append(("final", target)) or detection,
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

    monkeypatch.setattr(return_recovery, "is_graspable", lambda *_args: True)
    monkeypatch.setattr(return_recovery, "attempt_grasp", lambda *_args, **_kwargs: True)
    monkeypatch.setattr(return_recovery, "return_home", lambda *_args: True)

    ok = return_recovery.run_return_recovery(
        status,
        [],
        None,
        FakeMotion(),
        None,
        gripper,
        drawer,
        None,
        config,
        logger,
    )

    assert ok
    assert drawer.events == [("observe", 0), ("close", 0)]
    assert ("open", 0) not in drawer.events
    assert calls == [
        ("initial", "pliers"),
        ("observe_target", "pliers"),
        ("observe_target", "pliers"),
        ("final", "pliers"),
    ]
    assert placed == [(0, "pliers")]
    assert gripper.open_calls == 1


def test_pick_recovery_observes_target_then_stores_in_original_drawer(monkeypatch):
    status = FakeStatus()
    status.target_tool = "wrench"
    status.target_label = "wrench"
    status.held_tool = "pliers"
    drawer = FakeDrawer()
    gripper = FakeGripper()
    motion = FakeMotion()
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
        motion,
        None,
        gripper,
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
    assert gripper.open_calls == 1
    assert motion.wrist_rotations == [RETURN_DRAWER_PLACE_WRIST_YAW_DEG]
    assert status.held_tool is None
    assert status.gripper_holding is False
    assert status.recovery_mode is False
    assert "recovery_target_redetected" in [
        update[1].get("reason")
        for update in status.status_updates
    ]
    assert "recovery_grasp_started" in [
        update[1].get("reason")
        for update in status.status_updates
    ]
