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

from macgyvbot_task.application.return_flow.return_target_resolver import (
    RETURN_SOURCE_HAND,
    RETURN_SOURCE_NONE,
    ReturnTargetResolver,
)


class FakeLogger:
    def warn(self, message):
        pass


class FakeState:
    def __init__(self):
        self.last_grasp_result = None
        self.color_image = object()
        self.depth_image = object()
        self.intrinsics = {"fx": 1.0}


def no_wait(_duration_sec):
    pass


def test_hand_result_returns_hand_target():
    state = FakeState()
    state.last_grasp_result = {
        "hand_present": True,
        "tool_label": "hammer",
        "position": {
            "x": 0.3,
            "y": 0.1,
            "z": 0.4,
            "frame_id": "base_link",
        },
    }
    resolver = ReturnTargetResolver(
        state,
        no_wait,
        timeout_sec=0.01,
    )

    target = resolver.resolve("hammer", FakeLogger())

    assert target.source == RETURN_SOURCE_HAND
    assert target.tool_name == "hammer"
    assert target.hand_candidate.x == 0.3
    assert not hasattr(target, "floor_target")


def test_tool_only_target_is_not_treated_as_floor():
    state = FakeState()
    state.last_grasp_result = {"hand_present": False}
    resolver = ReturnTargetResolver(
        state,
        no_wait,
        timeout_sec=0.01,
    )

    target = resolver.resolve("hammer", FakeLogger())

    assert target.source == RETURN_SOURCE_NONE
    assert target.reason == "hand_target_not_found"


def test_no_target_returns_failure_reason():
    state = FakeState()
    state.last_grasp_result = {"hand_present": False}
    resolver = ReturnTargetResolver(
        state,
        no_wait,
        timeout_sec=0.01,
    )

    target = resolver.resolve("hammer", FakeLogger())

    assert target.source == RETURN_SOURCE_NONE
    assert target.reason == "hand_target_not_found"


def test_hand_without_position_blocks_floor_fallback():
    state = FakeState()
    state.last_grasp_result = {
        "hand_present": True,
        "hand_pixel": {"u": 10, "v": 10},
    }
    resolver = ReturnTargetResolver(
        state,
        no_wait,
        timeout_sec=0.01,
    )

    target = resolver.resolve("hammer", FakeLogger())

    assert target.source == RETURN_SOURCE_NONE
    assert target.reason == "hand_position_unavailable"
