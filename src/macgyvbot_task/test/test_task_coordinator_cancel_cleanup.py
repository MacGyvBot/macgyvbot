import sys
import threading
import types
from collections import deque
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
for package_path in (
    REPO_ROOT / "src" / "macgyvbot_task",
    REPO_ROOT / "src" / "macgyvbot_config",
    REPO_ROOT / "src" / "macgyvbot_domain",
    REPO_ROOT / "src" / "macgyvbot_manipulation",
    REPO_ROOT / "src" / "macgyvbot_perception",
    REPO_ROOT / "src" / "macgyvbot_resources",
):
    sys.path.insert(0, str(package_path))


class FakeLogger:
    def debug(self, *_args, **_kwargs):
        pass

    def info(self, *_args, **_kwargs):
        pass

    def warn(self, *_args, **_kwargs):
        pass

    def error(self, *_args, **_kwargs):
        pass


class FakeMotion:
    def __init__(self):
        self.cancel_reasons = []

    def cancel_current_goal(self, _logger, reason):
        self.cancel_reasons.append(reason)


class FakeToolHoldMonitor:
    def __init__(self):
        self.stop_reasons = []

    def stop(self, reason):
        self.stop_reasons.append(reason)


class FakeDrawerFlow:
    def drawer_id_for_tool(self, tool_name):
        return 1 if tool_name == "screwdriver" else None

    def prepare_open_handle_target(self, drawer_id, _logger):
        return drawer_id == 1

    def move_to_open_handle_preapproach(self, drawer_id, _logger):
        return drawer_id == 1

    def move_to_open_handle_pose(self, drawer_id, _logger):
        return drawer_id == 1

    def grip_open_handle(self, drawer_id, _logger):
        return drawer_id == 1

    def pull_open_drawer(self, drawer_id, _logger):
        return drawer_id == 1

    def release_open_handle(self, drawer_id, _logger):
        return drawer_id == 1

    def observe_drawer(self, drawer_id, _logger):
        return drawer_id == 1


def _module(name, **attrs):
    module = types.ModuleType(name)
    for key, value in attrs.items():
        setattr(module, key, value)
    return module


def _class(name):
    return type(name, (), {})


def _install_task_coordinator_import_stubs(monkeypatch):
    monkeypatch.setitem(
        sys.modules,
        "numpy",
        _module("numpy", load=lambda *_args, **_kwargs: None),
    )
    rclpy_module = _module(
        "rclpy",
        ok=lambda: True,
        init=lambda *args, **kwargs: None,
        shutdown=lambda: None,
    )
    monkeypatch.setitem(sys.modules, "rclpy", rclpy_module)
    monkeypatch.setitem(
        sys.modules,
        "rclpy.node",
        _module("rclpy.node", Node=_class("Node")),
    )
    monkeypatch.setitem(
        sys.modules,
        "rclpy.executors",
        _module(
            "rclpy.executors",
            MultiThreadedExecutor=_class("MultiThreadedExecutor"),
        ),
    )
    monkeypatch.setitem(
        sys.modules,
        "cv_bridge",
        _module("cv_bridge", CvBridge=_class("CvBridge")),
    )
    monkeypatch.setitem(
        sys.modules,
        "geometry_msgs",
        _module("geometry_msgs"),
    )
    monkeypatch.setitem(
        sys.modules,
        "geometry_msgs.msg",
        _module("geometry_msgs.msg", WrenchStamped=_class("WrenchStamped")),
    )
    monkeypatch.setitem(
        sys.modules,
        "sensor_msgs",
        _module("sensor_msgs"),
    )
    monkeypatch.setitem(
        sys.modules,
        "sensor_msgs.msg",
        _module(
            "sensor_msgs.msg",
            CameraInfo=_class("CameraInfo"),
            Image=_class("Image"),
        ),
    )
    monkeypatch.setitem(
        sys.modules,
        "moveit",
        _module("moveit"),
    )
    monkeypatch.setitem(
        sys.modules,
        "moveit.planning",
        _module(
            "moveit.planning",
            MoveItPy=_class("MoveItPy"),
            PlanRequestParameters=_class("PlanRequestParameters"),
        ),
    )

    msg_module = _module(
        "macgyvbot_interfaces.msg",
        HumanGraspResult=_class("HumanGraspResult"),
        RobotTaskControl=_class("RobotTaskControl"),
        RobotTaskStatus=_class("RobotTaskStatus"),
        TaskRequest=_class("TaskRequest"),
        ToolCommand=_class("ToolCommand"),
        ToolDropEvent=_class("ToolDropEvent"),
        ToolMaskLock=_class("ToolMaskLock"),
    )
    srv_module = _module("macgyvbot_interfaces.srv", SetGripper=_class("SetGripper"))
    interfaces_module = _module(
        "macgyvbot_interfaces",
        msg=msg_module,
        srv=srv_module,
    )
    monkeypatch.setitem(sys.modules, "macgyvbot_interfaces", interfaces_module)
    monkeypatch.setitem(sys.modules, "macgyvbot_interfaces.msg", msg_module)
    monkeypatch.setitem(sys.modules, "macgyvbot_interfaces.srv", srv_module)

    stubs = {
        "macgyvbot_manipulation.drawer_collision_scene": {
            "DrawerCollisionSceneManager": _class("DrawerCollisionSceneManager")
        },
        "macgyvbot_manipulation.drawer_motion": {
            "DrawerMotionFlow": _class("DrawerMotionFlow")
        },
        "macgyvbot_manipulation.gripper_collision_scene": {
            "GripperSelfCollisionManager": _class("GripperSelfCollisionManager")
        },
        "macgyvbot_manipulation.moveit_controller": {
            "MoveItController": _class("MoveItController")
        },
        "macgyvbot_manipulation.onrobot_gripper": {"RG": _class("RG")},
        "macgyvbot_manipulation.robot_pose": {
            "get_ee_matrix": lambda *_args, **_kwargs: None,
            "make_safe_pose": lambda *args, **_kwargs: args,
            "normalize_angle_deg": lambda angle: angle,
            "orientation_from_joint_positions": lambda *_args, **_kwargs: None,
        },
        "macgyvbot_perception.depth_projection": {
            "DepthProjector": _class("DepthProjector")
        },
        "macgyvbot_perception.grasp_point.grasp_point_selector": {
            "GraspPointSelector": _class("GraspPointSelector"),
            "normalize_grasp_point_mode": lambda mode, _logger=None: mode,
        },
        "macgyvbot_perception.pick_target_resolver": {
            "PickTargetResolver": _class("PickTargetResolver")
        },
        "macgyvbot_perception.yolo_detector": {"YoloDetector": _class("YoloDetector")},
        "macgyvbot_resources.calibration": {
            "resolve_calibration_file": lambda filename: filename
        },
        "macgyvbot_task.application.adapters.sam_yaw_service_client": {
            "SAMYawServiceClient": _class("SAMYawServiceClient")
        },
        "macgyvbot_task.application.adapters.vlm_grasp_service_client": {
            "VLMGraspServiceClient": _class("VLMGraspServiceClient")
        },
        "macgyvbot_task.application.display.debug_display": {
            "DebugDisplay": _class("DebugDisplay")
        },
        "macgyvbot_task.application.pick_flow.pick_frame_processor": {
            "PickFrameProcessor": _class("PickFrameProcessor")
        },
        "macgyvbot_task.application.pick_flow.pick_sequence": {
            "PickSequenceRunner": _class("PickSequenceRunner")
        },
        "macgyvbot_task.application.return_flow.return_perception_adapter": {
            "ReturnPerceptionAdapter": _class("ReturnPerceptionAdapter")
        },
        "macgyvbot_task.application.return_flow.return_sequence": {
            "ReturnSequenceRunner": _class("ReturnSequenceRunner")
        },
        "macgyvbot_task.application.recovery": {
            "build_drop_recovery_steps": lambda *_args, **_kwargs: [],
            "run_drop_recovery": lambda *_args, **_kwargs: None,
        },
        "macgyvbot_task.application.recovery.recovery_utils": {
            "RecoveryConfig": _class("RecoveryConfig"),
        },
        "macgyvbot_task.application.robot.robot_home_initializer": {
            "RobotHomeInitializer": _class("RobotHomeInitializer")
        },
    }
    for module_name, attrs in stubs.items():
        monkeypatch.setitem(sys.modules, module_name, _module(module_name, **attrs))

    monkeypatch.delitem(
        sys.modules,
        "macgyvbot_task.task_coordinator_node",
        raising=False,
    )


def _make_cancel_node(TaskCoordinatorNode, *, active_step=False):
    node = object.__new__(TaskCoordinatorNode)
    node.handoff_retry_req = threading.Event()
    node.handoff_fallback_req = threading.Event()
    node.handoff_decision_pending = threading.Event()
    node.exit_req = threading.Event()
    node.pause_req = threading.Event()
    node.drop_req = threading.Event()
    node.resume_req = threading.Event()
    node._queue_lock = threading.RLock()
    node._queue = deque([("pick", object())])
    node._current_step = object() if active_step else None
    node._current_task_name = "pick" if active_step else None
    node._step_thread = None
    node._suspended_step = object()
    node._suspended_task_name = "pick"
    node._pending_drop_recovery_payload = {"event": "tool_dropped"}
    node._active_drop_recovery_snapshot = {"action": "bring"}
    node._drop_recovery_resume_step = object()
    node._drop_recovery_resume_task_name = "pick"
    node._drop_recovery_resume_queue = [("pick", object())]
    node.motion = FakeMotion()
    node.tool_hold_monitor = FakeToolHoldMonitor()
    node.state = types.SimpleNamespace(
        picking=True,
        target_label="screwdriver",
        human_grasped_tool=True,
        current_command={"action": "bring", "tool_name": "screwdriver"},
        drawer_prepared_tool="screwdriver",
        drawer_preparing_tool="screwdriver",
        grasp_detection_mask_images=["mask"],
        grasp_detection_mask_target="screwdriver",
        grasp_detection_yaw_deg=10.0,
        grasp_detection_yaw_target="screwdriver",
        grasp_detection_width_mm=20.0,
        grasp_detection_width_target="screwdriver",
    )
    node._task_log = lambda *_args, **_kwargs: FakeLogger()
    node._motion_log = lambda *_args, **_kwargs: FakeLogger()
    node.published_statuses = []
    node._publish_robot_status = (
        lambda status, **kwargs: node.published_statuses.append((status, kwargs))
    )
    return node


def _make_resume_node(TaskCoordinatorNode):
    node = object.__new__(TaskCoordinatorNode)
    node.pause_req = threading.Event()
    node.drop_req = threading.Event()
    node.resume_req = threading.Event()
    node._queue_lock = threading.RLock()
    node._current_step = None
    node._suspended_step = None
    node._task_log = lambda *_args, **_kwargs: FakeLogger()
    node._step_thread_alive = lambda: False
    node._resume_suspended_step_locked = lambda: False

    def _dispatch_next():
        raise AssertionError("idle resume must not dispatch task queue")

    node._dispatch_next = _dispatch_next
    node.state = types.SimpleNamespace(
        current_command=None,
        recovery_mode=False,
    )
    node.published_statuses = []
    node._publish_robot_status = (
        lambda status, **kwargs: node.published_statuses.append((status, kwargs))
    )
    return node


def _make_bring_node(TaskCoordinatorNode):
    node = object.__new__(TaskCoordinatorNode)
    node.exit_req = threading.Event()
    node.pause_req = threading.Event()
    node.drop_req = threading.Event()
    node.resume_req = threading.Event()
    node._queue_lock = threading.RLock()
    node._queue = deque()
    node._current_step = None
    node._current_task_name = None
    node._step_thread = None
    node._suspended_step = None
    node._suspended_task_name = None
    node.drawer_flow = FakeDrawerFlow()
    node.return_perception = types.SimpleNamespace(
        detect_drawer_tool_labels=lambda: ["screwdriver"]
    )
    node.frame_processor = types.SimpleNamespace(has_camera_state=lambda: False)
    node.detector = types.SimpleNamespace(detect=lambda _image: [])
    node.pick_target_resolver = types.SimpleNamespace(
        should_refine_grasp_point_at_top_view=lambda: False
    )
    node.pick_runner = types.SimpleNamespace(build_steps=lambda *_args: [])
    node.motion = FakeMotion()
    node.tool_hold_monitor = FakeToolHoldMonitor()
    node.state = types.SimpleNamespace(
        picking=False,
        recovery_mode=False,
        target_label=None,
        target_tool=None,
        human_grasped_tool=False,
        current_command=None,
        drawer_prepared_tool=None,
        drawer_preparing_tool=None,
        drawer_open=False,
        opened_drawer_id=None,
        _last_search_status_target=None,
        grasp_detection_mask_images=None,
        grasp_detection_mask_target=None,
        grasp_detection_yaw_deg=None,
        grasp_detection_yaw_target=None,
        grasp_detection_width_mm=None,
        grasp_detection_width_target=None,
    )
    node._task_log = lambda *_args, **_kwargs: FakeLogger()
    node._motion_log = lambda *_args, **_kwargs: FakeLogger()
    node._step_thread_alive = lambda: False
    node._run_cleanup_callbacks = lambda: None
    node.published_statuses = []
    node._publish_robot_status = (
        lambda status, **kwargs: node.published_statuses.append((status, kwargs))
    )
    return node


def _attach_bring_runner(node):
    from macgyvbot_task.application.pick_flow.bring_sequence import BringSequenceRunner

    node.bring_runner = BringSequenceRunner(
        state=node.state,
        drawer_flow=node.drawer_flow,
        return_perception=node.return_perception,
        frame_processor=node.frame_processor,
        detector=node.detector,
        pick_target_resolver=node.pick_target_resolver,
        pick_runner=node.pick_runner,
        publish_robot_status=node._publish_robot_status,
        task_log=node._task_log,
        interrupted=node._motion_interrupted,
        append_task_steps=node._append_task_steps,
        has_queued_task_steps=node._has_queued_task_steps,
        recover_after_drawer_validation_failure=lambda *_args, **_kwargs: True,
    )
    return node


def test_cancel_cleans_paused_suspended_state_without_active_step(monkeypatch):
    _install_task_coordinator_import_stubs(monkeypatch)
    from macgyvbot_task.task_coordinator_node import TaskCoordinatorNode

    node = _make_cancel_node(TaskCoordinatorNode, active_step=False)

    assert node._handle_cancel("취소")

    assert not node.exit_req.is_set()
    assert node.state.picking is False
    assert node.state.target_label is None
    assert node.state.current_command is None
    assert node._queue == deque()
    assert node._suspended_step is None
    assert node._suspended_task_name is None
    assert node._pending_drop_recovery_payload is None
    assert node._active_drop_recovery_snapshot is None
    assert node._drop_recovery_resume_step is None
    assert node._drop_recovery_resume_task_name is None
    assert node._drop_recovery_resume_queue is None
    assert node.motion.cancel_reasons == ["취소"]
    assert node.tool_hold_monitor.stop_reasons == ["task_queue_finished"]
    assert node.published_statuses[-1][0] == "cancelled"


def test_cancel_defers_cleanup_while_step_is_active(monkeypatch):
    _install_task_coordinator_import_stubs(monkeypatch)
    from macgyvbot_task.task_coordinator_node import TaskCoordinatorNode

    node = _make_cancel_node(TaskCoordinatorNode, active_step=True)

    assert node._handle_cancel("취소")

    assert node.exit_req.is_set()
    assert node.state.picking is True
    assert node._queue == deque()
    assert node._suspended_step is None
    assert node._suspended_task_name is None
    assert node._pending_drop_recovery_payload is None
    assert node._active_drop_recovery_snapshot is None
    assert node._drop_recovery_resume_step is None
    assert node._drop_recovery_resume_task_name is None
    assert node._drop_recovery_resume_queue is None
    assert node.tool_hold_monitor.stop_reasons == []
    assert node.published_statuses[-1][0] == "cancelled"


def test_recovery_mode_cancel_is_not_rejected(monkeypatch):
    _install_task_coordinator_import_stubs(monkeypatch)
    from macgyvbot_task.task_coordinator_node import TaskCoordinatorNode

    node = _make_cancel_node(TaskCoordinatorNode, active_step=False)
    node.state.recovery_mode = True

    msg = types.SimpleNamespace(action="cancel", reason="cancel_requested")
    node._task_control_cb(msg)

    assert node.published_statuses[-1][0] == "cancelled"
    assert node._queue == deque()
    assert node._drop_recovery_resume_queue is None


def test_recovery_mode_exit_is_not_rejected_and_clears_recovery_queue(monkeypatch):
    _install_task_coordinator_import_stubs(monkeypatch)
    from macgyvbot_task.task_coordinator_node import TaskCoordinatorNode

    node = _make_cancel_node(TaskCoordinatorNode, active_step=False)
    node.state.recovery_mode = True
    started_exit_home = []
    node._start_exit_home_recovery = lambda reason: started_exit_home.append(reason)

    msg = types.SimpleNamespace(action="exit", reason="exit_requested")
    node._task_control_cb(msg)

    assert node.published_statuses[-1][0] == "cancelled"
    assert node.published_statuses[-1][1]["action"] == "exit"
    assert node._queue == deque()
    assert node._pending_drop_recovery_payload is None
    assert node._active_drop_recovery_snapshot is None
    assert node._drop_recovery_resume_step is None
    assert node._drop_recovery_resume_task_name is None
    assert node._drop_recovery_resume_queue is None
    assert started_exit_home == ["exit_requested"]


def test_resume_without_paused_task_returns_to_idle(monkeypatch):
    _install_task_coordinator_import_stubs(monkeypatch)
    from macgyvbot_task.task_coordinator_node import TaskCoordinatorNode

    node = _make_resume_node(TaskCoordinatorNode)
    node.resume_req.set()

    assert node._handle_resume("재개") is False

    assert not node.resume_req.is_set()
    assert node.published_statuses[-1][0] == "rejected"
    assert node.published_statuses[-1][1]["action"] == "resume"
    assert node.published_statuses[-1][1]["reason"] == "resume_without_paused_task"
    assert "재개할 작업이 없습니다" in node.published_statuses[-1][1]["message"]


def test_bring_request_loads_drawer_prepare_steps_before_search(monkeypatch):
    _install_task_coordinator_import_stubs(monkeypatch)
    from macgyvbot_task.task_coordinator_node import TaskCoordinatorNode

    node = _attach_bring_runner(_make_bring_node(TaskCoordinatorNode))
    loaded = {}

    def _load_queue(task_name, steps):
        loaded["task_name"] = task_name
        loaded["step_names"] = [step.name for step in steps]
        return True

    node._load_queue = _load_queue
    node.is_running = lambda: False
    node._task_request_command = (
        lambda _request: {"action": "bring", "tool_name": "screwdriver"}
    )
    request = types.SimpleNamespace(
        tool_name="screwdriver",
        has_base_target=False,
    )

    node._handle_bring_request(request)

    assert loaded["task_name"] == "bring"
    assert loaded["step_names"] == [
        "bring/drawer_prepare_handle_target",
        "bring/drawer_handle_preapproach",
        "bring/drawer_handle_pose",
        "bring/drawer_grip_handle",
        "bring/drawer_pull_open",
        "bring/drawer_release_handle",
        "bring/drawer_observe",
        "bring/drawer_validate_contents",
        "bring/search_target",
    ]
    assert node.state.picking is True
    assert node.state.target_label == "screwdriver"


def test_drawer_prepare_step_suspends_on_pause(monkeypatch):
    _install_task_coordinator_import_stubs(monkeypatch)
    from macgyvbot_task.application.task_control.task_step import TaskStep
    from macgyvbot_task.task_coordinator_node import TaskCoordinatorNode

    node = _make_bring_node(TaskCoordinatorNode)
    step = TaskStep("bring/drawer_handle_preapproach", lambda: False)
    node.state.target_label = "screwdriver"
    node.state.current_command = {"action": "bring", "tool_name": "screwdriver"}
    node.pause_req.set()
    node._current_step = step
    node._current_task_name = "bring"

    node._finish_step("bring", step, False, None)

    assert node._suspended_step is step
    assert node._suspended_task_name == "bring"
    assert node.state.target_label == "screwdriver"
    assert node.state.current_command == {
        "action": "bring",
        "tool_name": "screwdriver",
    }


def test_drop_recovery_uses_separate_resume_slots(monkeypatch):
    _install_task_coordinator_import_stubs(monkeypatch)
    from macgyvbot_task.application.task_control.task_step import TaskStep
    from macgyvbot_task.task_coordinator_node import TaskCoordinatorNode

    node = _make_bring_node(TaskCoordinatorNode)
    original_step = TaskStep("pick/grasp_tool", lambda: False)
    recovery_step = TaskStep("recovery/attempt_grasp", lambda: False)
    node._drop_recovery_resume_step = original_step
    node._drop_recovery_resume_task_name = "pick"
    node.drop_req.set()

    node._finish_step("pick", original_step, False, None)

    assert node._suspended_step is None
    assert node._drop_recovery_resume_step is original_step
    assert node._drop_recovery_resume_task_name == "pick"

    node.drop_req.clear()
    node.pause_req.set()
    node._current_step = recovery_step
    node._current_task_name = "recovery"

    node._finish_step("recovery", recovery_step, False, None)

    assert node._suspended_step is recovery_step
    assert node._suspended_task_name == "recovery"
    assert node._drop_recovery_resume_step is original_step
    assert node._drop_recovery_resume_task_name == "pick"


def test_paused_recovery_step_keeps_recovery_mode(monkeypatch):
    _install_task_coordinator_import_stubs(monkeypatch)
    from macgyvbot_task.application.task_control.task_step import TaskStep
    from macgyvbot_task.task_coordinator_node import TaskCoordinatorNode

    node = _make_bring_node(TaskCoordinatorNode)
    recovery_step = TaskStep("recovery/cleanup", lambda: False)
    node.state.recovery_mode = False
    node.pause_req.set()

    node._finish_step("recovery", recovery_step, False, None)

    assert node.state.recovery_mode is True
    assert node._suspended_step is recovery_step
    assert node._suspended_task_name == "recovery"


def test_recovery_step_false_after_fast_resume_is_requeued(monkeypatch):
    _install_task_coordinator_import_stubs(monkeypatch)
    from macgyvbot_task.application.task_control.task_step import TaskStep
    from macgyvbot_task.task_coordinator_node import TaskCoordinatorNode

    node = _make_bring_node(TaskCoordinatorNode)
    recovery_step = TaskStep("recovery/return_home", lambda: False)
    dispatched = []
    node.state.recovery_mode = True
    node.resume_req.set()
    node._dispatch_next = lambda: dispatched.append(True)

    node._finish_step("recovery", recovery_step, False, None)

    assert list(node._queue) == [("recovery", recovery_step)]
    assert node._suspended_step is None
    assert node._suspended_task_name is None
    assert not node.pause_req.is_set()
    assert not node.resume_req.is_set()
    assert dispatched == [True]


def test_recovery_resume_keeps_pause_until_active_step_suspends(monkeypatch):
    _install_task_coordinator_import_stubs(monkeypatch)
    from macgyvbot_task.application.task_control.task_step import TaskStep
    from macgyvbot_task.task_coordinator_node import TaskCoordinatorNode

    node = _make_bring_node(TaskCoordinatorNode)
    node.state.recovery_mode = True
    node.state.current_command = {"action": "bring", "tool_name": "wrench"}
    node.pause_req.set()
    node._current_step = TaskStep("recovery/wait_tool_mask_lock", lambda: False)
    node._current_task_name = "recovery"
    node._step_thread_alive = lambda: True
    node._resume_suspended_step_locked = (
        lambda: (_ for _ in ()).throw(
            AssertionError("active recovery step must suspend itself before resume")
        )
    )

    assert node._handle_resume("resume_requested")

    assert node.pause_req.is_set()
    assert node.resume_req.is_set()
    assert node.published_statuses[-1][0] == "resumed"


def test_drop_recovery_success_restores_original_step_before_tail(monkeypatch):
    _install_task_coordinator_import_stubs(monkeypatch)
    from macgyvbot_task.application.task_control.task_step import TaskStep
    from macgyvbot_task.task_coordinator_node import TaskCoordinatorNode

    node = _make_bring_node(TaskCoordinatorNode)
    original_step = TaskStep("pick/grasp_tool", lambda: True)
    tail_step = TaskStep("pick/lift", lambda: True)
    node._active_drop_recovery_snapshot = {
        "action": "bring",
        "task_name": "pick",
        "tool_name": "screwdriver",
        "command": {"action": "bring", "tool_name": "screwdriver"},
        "resume_state": {
            "picking": True,
            "target_label": "screwdriver",
            "target_tool": "screwdriver",
            "current_command": {"action": "bring", "tool_name": "screwdriver"},
        },
    }
    node._pending_drop_recovery_payload = None
    node._drop_recovery_resume_step = original_step
    node._drop_recovery_resume_task_name = "pick"
    node._drop_recovery_resume_queue = [("pick", tail_step)]

    assert node._finish_drop_recovery_queue_locked(True)

    assert list(node._queue) == [("pick", original_step), ("pick", tail_step)]
    assert node._drop_recovery_resume_step is None
    assert node._drop_recovery_resume_task_name is None
    assert node._drop_recovery_resume_queue is None
    assert node._active_drop_recovery_snapshot is None


def test_home_control_does_not_trigger_handoff_fallback(monkeypatch):
    _install_task_coordinator_import_stubs(monkeypatch)
    from macgyvbot_task.task_coordinator_node import TaskCoordinatorNode

    node = object.__new__(TaskCoordinatorNode)
    node.handoff_retry_req = threading.Event()
    node.handoff_fallback_req = threading.Event()
    node.handoff_decision_pending = threading.Event()
    node.handoff_decision_pending.set()
    node.state = types.SimpleNamespace(
        recovery_mode=False,
        picking=True,
        current_command={"action": "bring", "tool_name": "screwdriver"},
    )
    node.is_running = lambda: False
    node._task_log = lambda *_args, **_kwargs: FakeLogger()
    node.published_statuses = []
    node._publish_robot_status = (
        lambda status, **kwargs: node.published_statuses.append((status, kwargs))
    )

    msg = types.SimpleNamespace(action="home", reason="홈위치로 가")
    node._task_control_cb(msg)

    assert not node.handoff_fallback_req.is_set()
    assert node.published_statuses[-1][0] == "busy"
    assert node.published_statuses[-1][1]["action"] == "home"
    assert node.published_statuses[-1][1]["reason"] == "already_picking"


def test_recovery_yolo_mode_uses_grasp_point_target(monkeypatch):
    _install_task_coordinator_import_stubs(monkeypatch)
    from macgyvbot_config.vlm import GRASP_POINT_MODE_YOLO
    from macgyvbot_task.task_coordinator_node import TaskCoordinatorNode

    node = object.__new__(TaskCoordinatorNode)
    node.grasp_point_mode = GRASP_POINT_MODE_YOLO
    node.detector = types.SimpleNamespace(names={0: "wrench"})
    node.grasp_point_selector = types.SimpleNamespace(
        select_yolo_grasp_point=lambda boxes, names, box: (
            11,
            22,
            "yolo",
            None,
        )
    )
    calls = []
    node.pick_target_resolver = types.SimpleNamespace(
        matching_box=lambda boxes, target: (boxes[0], target),
        target_from_selected_grasp=lambda label, target, selected, *_args: (
            calls.append(("selected", label, target, selected))
            or types.SimpleNamespace(
                found=True,
                pixel=(selected[0], selected[1]),
                base_xyz=(0.31, 0.11, 0.2),
                yaw_deg=None,
            )
        ),
        target_from_boxes=lambda *_args, **_kwargs: (
            calls.append(("center",))
            or types.SimpleNamespace(found=True)
        ),
    )

    target = node._resolve_recovery_grasp_point_target(
        boxes=[object()],
        target_tool="wrench",
        color_image=object(),
        depth_image=object(),
        intrinsics={},
    )

    assert target.pixel == (11, 22)
    assert calls == [("selected", "wrench", "wrench", (11, 22, "yolo", None))]


def test_recovery_yolo_mode_falls_back_to_center_when_grasp_point_fails(
    monkeypatch,
):
    _install_task_coordinator_import_stubs(monkeypatch)
    from macgyvbot_config.vlm import GRASP_POINT_MODE_YOLO
    from macgyvbot_task.task_coordinator_node import TaskCoordinatorNode

    node = object.__new__(TaskCoordinatorNode)
    node.grasp_point_mode = GRASP_POINT_MODE_YOLO
    node.detector = types.SimpleNamespace(names={0: "wrench"})
    node.grasp_point_selector = types.SimpleNamespace(
        select_yolo_grasp_point=lambda *_args: None
    )
    node._task_log = lambda *_args, **_kwargs: FakeLogger()
    calls = []
    node.pick_target_resolver = types.SimpleNamespace(
        matching_box=lambda boxes, target: (boxes[0], target),
        target_from_boxes=lambda *args, **kwargs: (
            calls.append(kwargs.get("use_bbox_center"))
            or types.SimpleNamespace(
                found=True,
                pixel=(30, 40),
                base_xyz=(0.3, 0.1, 0.2),
                yaw_deg=None,
            )
        ),
    )

    target = node._resolve_recovery_grasp_point_target(
        boxes=[object()],
        target_tool="wrench",
        color_image=object(),
        depth_image=object(),
        intrinsics={},
    )

    assert target.pixel == (30, 40)
    assert calls == [True]
