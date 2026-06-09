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

moveit_module = types.ModuleType("moveit")
moveit_core_module = types.ModuleType("moveit.core")
moveit_robot_state_module = types.ModuleType("moveit.core.robot_state")
moveit_robot_state_module.RobotState = object
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

from macgyvbot_manipulation.handover_targeting import TargetCandidate
from macgyvbot_task.application.return_flow.return_handoff_flow import (
    ReturnHandoffFlow,
)


class FakeLogger:
    def info(self, message):
        pass

    def warn(self, message):
        pass

    def error(self, message):
        pass


class FakeMotion:
    def __init__(self, results):
        self.results = list(results)
        self.targets = []

    def plan_and_execute(
        self,
        logger,
        pose_goal=None,
        state_goal=None,
        collision_scene_key=None,
    ):
        self.targets.append(pose_goal)
        return self.results.pop(0)


class FakeReporter:
    def __init__(self):
        self.published = []
        self.failed = []

    def publish(self, status, tool_name, message, command, reason=""):
        self.published.append((status, tool_name, message, command, reason))

    def fail(self, tool_name, message, reason, command, logger):
        self.failed.append((tool_name, message, reason, command))


class FakeState:
    def __init__(self):
        self.home_ori = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
        self.last_grasp_result = None
        self.color_image = types.SimpleNamespace(shape=(100, 200, 3))


class FakeGraspVerifier:
    def __init__(self, result):
        self.result = result

    def try_grasp(self, logger, publish_attempt=None, failure_prefix=""):
        if publish_attempt is not None:
            publish_attempt(1, 1)
        return self.result


class FakeClosePolicy:
    def __init__(self, result):
        self.result = result

    def matches(self, image_shape, tool_roi, tool_depth_mm):
        return self.result


def make_flow(motion_results=(True,), close_policy=None):
    flow = ReturnHandoffFlow(
        robot=None,
        motion_controller=FakeMotion(motion_results),
        gripper=object(),
        state=FakeState(),
        reporter=FakeReporter(),
        wait_fn=lambda _duration: None,
        close_policy=close_policy or FakeClosePolicy(True),
    )
    return flow


def test_move_to_candidate_rejects_unsupported_frame():
    flow = make_flow()
    candidate = TargetCandidate(
        found=True,
        x=0.1,
        y=0.2,
        z=0.3,
        frame_id="camera",
        source="test",
    )

    ok, reason = flow.move_to_candidate("hammer", candidate, {}, FakeLogger())

    assert not ok
    assert reason == "return_unsupported_frame"
    assert flow.reporter.failed[-1][2] == "return_unsupported_frame"
    assert flow.motion.targets == []


def test_move_to_candidate_maps_target_move_failure_reason():
    flow = make_flow(motion_results=[False] * 10)
    candidate = TargetCandidate(
        found=True,
        x=0.1,
        y=0.2,
        z=0.3,
        frame_id="base_link",
        source="test",
    )

    ok, reason = flow.move_to_candidate("hammer", candidate, {}, FakeLogger())

    assert not ok
    assert reason == "return_detected_pose_move_failed"
    assert flow.reporter.failed[-1][2] == "return_detected_pose_move_failed"


def test_grasp_at_current_position_returns_timeout_when_close_policy_fails():
    flow = make_flow(close_policy=FakeClosePolicy(False))
    flow_module = sys.modules[ReturnHandoffFlow.__module__]
    original_timeout = flow_module.RETURN_HAND_CLOSE_ROI_TIMEOUT_SEC
    flow_module.RETURN_HAND_CLOSE_ROI_TIMEOUT_SEC = 0.0
    try:
        tool_name, reason = flow.grasp_at_current_position(
            "hammer",
            {},
            FakeLogger(),
        )
    finally:
        flow_module.RETURN_HAND_CLOSE_ROI_TIMEOUT_SEC = original_timeout

    assert tool_name is None
    assert reason == "return_close_roi_timeout"
    assert flow.reporter.failed[-1][2] == "return_close_roi_timeout"


def test_grasp_at_current_position_reports_grasp_failure():
    flow = make_flow(close_policy=FakeClosePolicy(True))
    flow.state.last_grasp_result = {
        "tool_roi": (90, 45, 110, 55),
        "tool_depth_mm": 200.0,
    }
    flow.grasp_verifier = FakeGraspVerifier(False)

    tool_name, reason = flow.grasp_at_current_position(
        "hammer",
        {},
        FakeLogger(),
    )

    assert tool_name is None
    assert reason == "return_grasp_failed"
    assert flow.reporter.failed[-1][2] == "return_grasp_failed"


def test_grasp_at_current_position_reports_success():
    flow = make_flow(close_policy=FakeClosePolicy(True))
    flow.state.last_grasp_result = {
        "tool_roi": (90, 45, 110, 55),
        "tool_depth_mm": 200.0,
    }
    flow.grasp_verifier = FakeGraspVerifier(True)

    tool_name, reason = flow.grasp_at_current_position(
        "hammer",
        {},
        FakeLogger(),
    )

    assert tool_name == "hammer"
    assert reason == ""
    assert flow.reporter.published[-1][0] == "grasp_success"
