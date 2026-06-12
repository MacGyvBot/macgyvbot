import sys
import types
import math


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
    RECOVERY_INSPECTION_SEARCH_TIMEOUT_SEC,
    attempt_grasp,
    detect_target_tool,
    normalize_tool_name,
    run_recovery_step_with_pause_retry,
)
import macgyvbot_task.application.recovery.recovery_utils as recovery_utils  # noqa: E402
from macgyvbot_task.application.recovery import drop_recovery  # noqa: E402
from macgyvbot_config.robot import RECOVERY_INSPECTION_JOINTS  # noqa: E402


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
        self.move_calls = []
        self.max_width = 800

    def open_gripper(self):
        self.events.append("open")

    def close_gripper(self):
        self.events.append("close")

    def move_gripper(self, width_raw):
        self.move_calls.append(width_raw)
        self.events.append(("move", width_raw))

    def get_status(self):
        return [False, True]

    def get_width(self):
        return 24.0

    def get_depth(self):
        return 12.0


class FakeStatus:
    def __init__(self):
        self.recovery_mode = False
        self.held_tool = "pliers"
        self.target_tool = "wrench"
        self.target_label = "screwdriver"
        self.gripper_holding = False
        self.drawer_open = False
        self.opened_drawer_id = None
        self.tool_mask_locked = True
        self.last_tool_mask_lock_result = {"locked": True}
        self.grasp_detection_mask_images = ["locked-image"]
        self.grasp_detection_mask_target = "pliers"
        self.grasp_detection_yaw_deg = 17.0
        self.grasp_detection_yaw_target = "pliers"
        self.grasp_detection_width_mm = 20.0
        self.grasp_detection_width_target = "pliers"
        self.status_updates = []

    def logger(self):
        return FakeLogger()

    def _publish_robot_status(self, status, **kwargs):
        if status == "grasp_success":
            self.tool_mask_locked = True
            self.last_tool_mask_lock_result = {
                "locked": True,
                "mask_source": "DEPTH_LOCKED_TOOL",
                "tool_roi": {"center_u": 10, "center_v": 20},
                "reason": "drop_recovery_grasp_success",
            }
        self.status_updates.append((status, kwargs))


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

    def plan_and_execute(
        self,
        _logger,
        pose_goal=None,
        state_goal=None,
        collision_scene_key=None,
        **_kwargs,
    ):
        if state_goal is not None:
            self.events.append(
                (
                    "joint_move",
                    collision_scene_key,
                    dict(state_goal.joint_positions),
                )
            )
            return True

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


class FakeGripper:
    def __init__(self):
        self.open_calls = 0

    def open_gripper(self):
        self.open_calls += 1


class FakeJointMotion:
    def __init__(self):
        self.calls = []

    def plan_and_execute(self, _logger, state_goal=None, collision_scene_key=None, **_kwargs):
        self.calls.append((collision_scene_key, dict(state_goal.joint_positions)))
        return True


class FakeRobot:
    def get_robot_model(self):
        return object()


class FakeMonitor:
    def __init__(self):
        self.starts = []

    def start(self, tool_name, action, command=None):
        self.starts.append((tool_name, action, command))


class FakeDrawerCloser:
    def __init__(self, result=True):
        self.result = result
        self.calls = []

    def __call__(self, logger):
        self.calls.append(logger)
        return self.result


class FakeEvent:
    def __init__(self, set_initially=False):
        self._set = bool(set_initially)

    def is_set(self):
        return self._set

    def set(self):
        self._set = True

    def clear(self):
        self._set = False


def test_normalize_tool_name_uses_unknown_for_missing_values():
    assert normalize_tool_name(None) == "unknown"
    assert normalize_tool_name("  ") == "unknown"
    assert normalize_tool_name(" wrench ") == "wrench"


def test_recovery_config_searches_inspection_for_four_seconds():
    assert RecoveryConfig().detection_timeout_sec == RECOVERY_INSPECTION_SEARCH_TIMEOUT_SEC
    assert RECOVERY_INSPECTION_SEARCH_TIMEOUT_SEC == 4.0


def test_move_to_inspection_pose_uses_recovery_joint_pose():
    motion = FakeJointMotion()
    logger = FakeLogger()

    ok = recovery_utils.move_to_inspection_pose(
        motion,
        RecoveryConfig(robot=FakeRobot(), state=types.SimpleNamespace(logger=lambda: logger)),
    )

    assert ok
    assert len(motion.calls) == 1
    collision_scene_key, joint_positions = motion.calls[0]
    assert collision_scene_key == "recovery/inspection_pose"
    assert joint_positions.keys() == RECOVERY_INSPECTION_JOINTS.keys()
    for joint_name, expected_rad in RECOVERY_INSPECTION_JOINTS.items():
        assert math.isclose(joint_positions[joint_name], expected_rad)


def test_recovery_failure_messages_classify_not_found_and_ik_failures():
    assert drop_recovery._recovery_failure_message_for_event(
        "target_detection_failed",
        "TARGET_NOT_FOUND",
        drawer_open=True,
    ) == "공구를 못찾겠습니다. 서랍을 닫고 홈 위치로 복귀합니다."
    assert drop_recovery._recovery_failure_message_for_event(
        "unknown_motion_reason",
        "PLANNING_FAILED",
        drawer_open=True,
    ) == "공구를 못잡겠습니다. 서랍을 닫고 홈 위치로 복귀합니다."


def test_detect_target_tool_retries_until_timeout(monkeypatch):
    now = {"value": 0.0}
    calls = []

    monkeypatch.setattr(
        recovery_utils.time,
        "monotonic",
        lambda: now["value"],
    )

    def detector(_target):
        calls.append(now["value"])
        return None

    def wait(duration):
        now["value"] += duration

    assert detect_target_tool(
        detector,
        "wrench",
        max_retry=1,
        retry_wait_sec=1.0,
        timeout_sec=4.0,
        wait_fn=wait,
    ) is None
    assert calls == [0.0, 1.0, 2.0, 3.0, 4.0]


def test_recovery_step_retries_after_pause_resume():
    status = FakeStatus()
    logger = FakeLogger()
    pause = FakeEvent()
    resume = FakeEvent()
    calls = []

    def wait(_duration):
        pause.clear()
        resume.set()

    def step():
        calls.append("step")
        if len(calls) == 1:
            pause.set()
            return False
        return True

    ok = run_recovery_step_with_pause_retry(
        RecoveryConfig(
            state=status,
            pause_event=pause,
            resume_event=resume,
            wait_fn=wait,
        ),
        status,
        logger,
        "wrench",
        "moving_to_recovery_inspection",
        step,
    )

    assert ok
    assert calls == ["step", "step"]
    assert not pause.is_set()
    assert not resume.is_set()
    assert [update[0] for update in status.status_updates] == [
        "paused",
        "recovering",
    ]


def test_recovery_step_stops_for_restart_even_if_step_returns_success():
    status = FakeStatus()
    logger = FakeLogger()
    drop = FakeEvent()

    def step():
        drop.set()
        return True

    ok = run_recovery_step_with_pause_retry(
        RecoveryConfig(
            state=status,
            drop_event=drop,
            wait_fn=lambda _duration: None,
        ),
        status,
        logger,
        "wrench",
        "recovery_grasp_started",
        step,
    )

    assert not ok
    assert drop.is_set()


def test_recovery_grasp_rotates_yaw_before_xy_align_and_z_descent(monkeypatch):
    monkeypatch.setattr(
        recovery_utils,
        "current_ee_orientation",
        lambda _robot: {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    )
    monkeypatch.setattr(
        recovery_utils,
        "PickTargetPlanner",
        lambda _robot: types.SimpleNamespace(
            plan=lambda *_args, **_kwargs: types.SimpleNamespace(
                target_x=0.31,
                target_y=0.12,
                drawer_wall_clearance_z=0.42,
                grasp_z=0.25,
                should_descend_to_grasp=True,
            )
        ),
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
        gripper,
        config=RecoveryConfig(robot=FakeRobot(), wait_fn=lambda _duration: None),
        target_tool="wrench",
    )

    assert ok
    assert motion.events == [
        ("yaw", 37.0),
        ("move", "recovery/grasp_xy_align", 0.31, 0.12, 0.42),
        ("move", "recovery/grasp_descent", 0.31, 0.12, 0.25),
        ("move", "recovery/pregrasp_depth_adjust", 0.31, 0.12, 0.238),
    ]
    assert gripper.events == ["open", "close", "open", "close"]


def test_recovery_grasp_applies_limited_width_before_each_descent_close(
    monkeypatch,
):
    monkeypatch.setattr(
        recovery_utils,
        "current_ee_orientation",
        lambda _robot: {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    )
    monkeypatch.setattr(
        recovery_utils,
        "PickTargetPlanner",
        lambda _robot: types.SimpleNamespace(
            plan=lambda *_args, **_kwargs: types.SimpleNamespace(
                target_x=0.31,
                target_y=0.12,
                drawer_wall_clearance_z=0.42,
                grasp_z=0.25,
                should_descend_to_grasp=True,
            )
        ),
    )
    motion = FakeMotion()
    gripper = FakeRecoveryGripper()
    status = FakeStatus()
    detection = types.SimpleNamespace(
        found=True,
        base_xyz=(0.30, 0.10, 0.20),
        yaw_deg=37.0,
    )

    ok = attempt_grasp(
        detection,
        motion,
        gripper,
        config=RecoveryConfig(
            robot=FakeRobot(),
            state=status,
            wait_fn=lambda _duration: None,
        ),
        target_tool="pliers",
    )

    assert ok
    assert gripper.move_calls == [300, 300]
    assert gripper.events == [
        ("move", 300),
        "close",
        ("move", 300),
        "close",
    ]


def test_return_recovery_prefers_held_tool_and_returns_home(monkeypatch):
    status = FakeStatus()
    gripper = FakeGripper()
    logger = FakeLogger()
    detection = types.SimpleNamespace(found=True, base_xyz=(0.3, 0.1, 0.2))
    config = RecoveryConfig(
        robot=FakeRobot(),
        state=status,
        detect_target_fn=lambda target: detection,
    )

    monkeypatch.setattr(
        recovery_utils,
        "move_to_observation_pose",
        lambda *_args: (True, None),
    )
    monkeypatch.setattr(drop_recovery, "is_graspable", lambda *_args: True)
    monkeypatch.setattr(drop_recovery, "attempt_grasp", lambda *_args, **_kwargs: True)
    ok = drop_recovery.run_drop_recovery(
        status,
        FakeMotion(),
        gripper,
        config,
        logger,
        task_type="return",
    )

    assert ok
    assert gripper.open_calls == 1
    assert status.held_tool == "pliers"
    assert status.gripper_holding is True
    assert status.recovery_mode is False
    assert status.tool_mask_locked is True
    assert status.last_tool_mask_lock_result == {
        "locked": True,
        "mask_source": "DEPTH_LOCKED_TOOL",
        "tool_roi": {"center_u": 10, "center_v": 20},
        "reason": "drop_recovery_grasp_success",
    }
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
    config = RecoveryConfig(
        robot=FakeRobot(),
        state=status,
        command={"tool_name": "wrench", "action": "return"},
        initial_detect_target_fn=lambda target: calls.append(("initial", target))
        or first_detection,
        target_observe_fn=lambda detection, target, _logger: calls.append(
            ("observe", target, detection.base_xyz)
        )
        or True,
        observed_tool_label_fn=lambda: "pliers",
        detect_target_fn=lambda target: calls.append(("final", target))
        or final_detection,
        tool_hold_monitor=monitor,
    )

    monkeypatch.setattr(
        recovery_utils,
        "move_to_observation_pose",
        lambda *_args: (True, None),
    )
    monkeypatch.setattr(drop_recovery, "is_graspable", lambda *_args: True)

    def fake_attempt_grasp(detection, *_args, **kwargs):
        calls.append(("grasp", kwargs["target_tool"], detection.yaw_deg))
        return True

    monkeypatch.setattr(drop_recovery, "attempt_grasp", fake_attempt_grasp)
    ok = drop_recovery.run_drop_recovery(
        status,
        motion,
        gripper,
        config,
        logger,
        task_type="return",
    )

    assert ok
    assert calls == [
        ("initial", "wrench"),
        ("observe", "wrench", (0.3, 0.1, 0.2)),
        ("final", "pliers"),
        ("grasp", "pliers", 31.0),
    ]
    assert gripper.open_calls == 1
    assert monitor.starts == [("pliers", "return", config.command)]
    assert status.held_tool == "pliers"


def test_return_recovery_uses_recovery_only_state(monkeypatch):
    status = FakeStatus()
    status.held_tool = "pliers"
    gripper = FakeGripper()
    monitor = FakeMonitor()
    logger = FakeLogger()
    detection = types.SimpleNamespace(
        found=True,
        base_xyz=(0.3, 0.1, 0.2),
        yaw_deg=19.0,
    )
    calls = []
    config = RecoveryConfig(
        robot=FakeRobot(),
        state=status,
        command={"tool_name": "unknown", "action": "return"},
        initial_detect_target_fn=lambda target: calls.append(("initial", target))
        or detection,
        target_observe_fn=lambda _detection, target, _logger: calls.append(
            ("observe_target", target)
        )
        or True,
        observed_tool_label_fn=lambda: "pliers",
        detect_target_fn=lambda target: calls.append(("final", target)) or detection,
        tool_hold_monitor=monitor,
    )

    monkeypatch.setattr(
        recovery_utils,
        "move_to_observation_pose",
        lambda *_args: (True, None),
    )
    monkeypatch.setattr(drop_recovery, "is_graspable", lambda *_args: True)
    monkeypatch.setattr(drop_recovery, "attempt_grasp", lambda *_args, **_kwargs: True)
    ok = drop_recovery.run_drop_recovery(
        status,
        FakeMotion(),
        gripper,
        config,
        logger,
        task_type="return",
    )

    assert ok
    assert calls == [
        ("initial", "pliers"),
        ("observe_target", "pliers"),
        ("final", "pliers"),
    ]
    assert gripper.open_calls == 1


def test_recovery_cleanup_returns_home(monkeypatch):
    status = FakeStatus()
    gripper = FakeGripper()
    logger = FakeLogger()
    detection = types.SimpleNamespace(found=True, base_xyz=(0.3, 0.1, 0.2))
    config = RecoveryConfig(
        robot=FakeRobot(),
        state=status,
        detect_target_fn=lambda target: detection,
    )

    monkeypatch.setattr(
        recovery_utils,
        "move_to_observation_pose",
        lambda *_args: (True, None),
    )
    monkeypatch.setattr(drop_recovery, "is_graspable", lambda *_args: True)
    monkeypatch.setattr(drop_recovery, "attempt_grasp", lambda *_args, **_kwargs: True)

    ok = drop_recovery.run_drop_recovery(
        status,
        FakeMotion(),
        gripper,
        config,
        logger,
        task_type="pick",
    )

    assert ok
    assert status.gripper_holding is True
    assert status.recovery_mode is False


def test_recovery_not_found_closes_open_drawer_then_returns_home(monkeypatch):
    status = FakeStatus()
    status.drawer_open = True
    status.opened_drawer_id = 1
    gripper = FakeGripper()
    logger = FakeLogger()
    drawer_closer = FakeDrawerCloser()
    motion = FakeMotion()
    config = RecoveryConfig(
        robot=FakeRobot(),
        state=status,
        detect_target_fn=lambda _target: None,
        close_open_drawer_fn=drawer_closer,
    )

    monkeypatch.setattr(
        recovery_utils,
        "move_to_observation_pose",
        lambda *_args: (True, None),
    )

    ok = drop_recovery.run_drop_recovery(
        status,
        motion,
        gripper,
        config,
        logger,
        task_type="pick",
    )

    assert not ok
    assert len(drawer_closer.calls) == 1
    assert status.recovery_mode is False
    assert ("recovering", {
        "tool_name": "wrench",
        "message": "공구를 못찾겠습니다. 서랍을 닫고 홈 위치로 복귀합니다.",
        "reason": "target_detection_failed",
    }) in status.status_updates


def test_recovery_not_graspable_closes_open_drawer_then_returns_home(monkeypatch):
    status = FakeStatus()
    status.drawer_open = True
    status.opened_drawer_id = 2
    gripper = FakeGripper()
    logger = FakeLogger()
    drawer_closer = FakeDrawerCloser()
    detection = types.SimpleNamespace(found=True, base_xyz=(0.3, 0.1, 0.2))
    config = RecoveryConfig(
        robot=FakeRobot(),
        state=status,
        detect_target_fn=lambda _target: detection,
        close_open_drawer_fn=drawer_closer,
    )

    monkeypatch.setattr(
        recovery_utils,
        "move_to_observation_pose",
        lambda *_args: (True, None),
    )
    monkeypatch.setattr(drop_recovery, "is_graspable", lambda *_args: False)

    ok = drop_recovery.run_drop_recovery(
        status,
        FakeMotion(),
        gripper,
        config,
        logger,
        task_type="pick",
    )

    assert not ok
    assert len(drawer_closer.calls) == 1
    assert status.recovery_mode is False
    assert ("recovering", {
        "tool_name": "wrench",
        "message": "공구를 못잡겠습니다. 서랍을 닫고 홈 위치로 복귀합니다.",
        "reason": "graspability_check_failed",
    }) in status.status_updates


def test_recovery_target_observe_planning_failure_closes_open_drawer(monkeypatch):
    status = FakeStatus()
    status.drawer_open = True
    status.opened_drawer_id = 3
    gripper = FakeGripper()
    logger = FakeLogger()
    drawer_closer = FakeDrawerCloser()
    detection = types.SimpleNamespace(found=True, base_xyz=(0.3, 0.1, 0.2))
    config = RecoveryConfig(
        robot=FakeRobot(),
        state=status,
        initial_detect_target_fn=lambda _target: detection,
        target_observe_fn=lambda *_args: False,
        close_open_drawer_fn=drawer_closer,
    )

    monkeypatch.setattr(
        recovery_utils,
        "move_to_observation_pose",
        lambda *_args: (True, None),
    )

    ok = drop_recovery.run_drop_recovery(
        status,
        FakeMotion(),
        gripper,
        config,
        logger,
        task_type="pick",
    )

    assert not ok
    assert len(drawer_closer.calls) == 1
    assert ("recovering", {
        "tool_name": "wrench",
        "message": "공구를 못잡겠습니다. 서랍을 닫고 홈 위치로 복귀합니다.",
        "reason": "target_observe_move_failed",
    }) in status.status_updates


def test_recovery_failed_event_with_open_drawer_uses_fallback_cleanup(monkeypatch):
    status = FakeStatus()
    status.drawer_open = True
    status.opened_drawer_id = 4
    gripper = FakeGripper()
    logger = FakeLogger()
    drawer_closer = FakeDrawerCloser()
    config = RecoveryConfig(
        robot=FakeRobot(),
        state=status,
        close_open_drawer_fn=drawer_closer,
    )

    monkeypatch.setattr(
        drop_recovery,
        "move_to_inspection_pose",
        lambda *_args: False,
    )

    ok = drop_recovery.run_drop_recovery(
        status,
        FakeMotion(),
        gripper,
        config,
        logger,
        task_type="pick",
    )

    assert not ok
    assert len(drawer_closer.calls) == 1
    assert ("recovering", {
        "tool_name": "wrench",
        "message": "공구를 못잡겠습니다. 서랍을 닫고 홈 위치로 복귀합니다.",
        "reason": "motion_planning_failed",
    }) in status.status_updates


def test_pick_recovery_observes_target_then_returns_home(monkeypatch):
    status = FakeStatus()
    status.target_tool = "wrench"
    status.target_label = "wrench"
    status.held_tool = "pliers"
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
    config = RecoveryConfig(
        robot=FakeRobot(),
        state=status,
        command={"tool_name": "wrench", "action": "bring"},
        initial_detect_target_fn=lambda target: calls.append(("initial", target))
        or first_detection,
        target_observe_fn=lambda detection, target, _logger: calls.append(
            ("observe", target, detection.base_xyz)
        )
        or True,
        detect_target_fn=lambda target: calls.append(("final", target))
        or final_detection,
        tool_hold_monitor=monitor,
    )

    monkeypatch.setattr(
        recovery_utils,
        "move_to_observation_pose",
        lambda *_args: (True, None),
    )
    monkeypatch.setattr(drop_recovery, "is_graspable", lambda *_args: True)

    def fake_attempt_grasp(detection, *_args, **kwargs):
        calls.append(("grasp", kwargs["target_tool"], detection.yaw_deg))
        return True

    monkeypatch.setattr(drop_recovery, "attempt_grasp", fake_attempt_grasp)

    ok = drop_recovery.run_drop_recovery(
        status,
        motion,
        gripper,
        config,
        logger,
        task_type="pick",
    )

    assert ok
    assert calls == [
        ("initial", "wrench"),
        ("observe", "wrench", (0.3, 0.1, 0.2)),
        ("final", "wrench"),
        ("grasp", "wrench", 42.0),
    ]
    assert monitor.starts == [("wrench", "bring", config.command)]
    assert gripper.open_calls == 1
    assert motion.wrist_rotations == []
    assert status.held_tool == "wrench"
    assert status.gripper_holding is True
    assert status.recovery_mode is False
    assert "recovery_target_redetected" in [
        update[1].get("reason")
        for update in status.status_updates
    ]
    assert "recovery_grasp_started" in [
        update[1].get("reason")
        for update in status.status_updates
    ]
