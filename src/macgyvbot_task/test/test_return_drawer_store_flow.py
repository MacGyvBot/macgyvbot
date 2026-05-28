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

from macgyvbot_config.drawer import (
    DRAWER_1_SAFE_Z_OFFSET_M,
    DRAWER_STORE_FORCE_DESCENT_START_Z_OFFSET_M,
    DRAWER_STORE_TOOL_OBSERVE_POINT,
)
from macgyvbot_manipulation.robot_safezone import SAFE_Z_MIN
from macgyvbot_task.application.return_flow.return_drawer_placement_flow import (
    ReturnDrawerPlacementFlow,
)
from macgyvbot_task.application.return_flow.return_sequence import (
    ReturnSequenceRunner,
)


class FakeLogger:
    def info(self, _message):
        pass

    def warn(self, _message):
        pass

    def error(self, _message):
        pass


class FakeState:
    def __init__(self):
        self.home_ori = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
        self.human_grasped_tool = False
        self.last_grasp_result = None
        self.target_label = None

    def logger(self):
        return FakeLogger()

    def _publish_status_payload(self, _payload):
        pass


class FakeDrawerFlow:
    def drawer_id_for_tool(self, tool_name):
        return {"screwdriver": 1}.get(tool_name)


def test_drawer_store_observe_point_uses_staging_joint_pose():
    expected_degrees = {
        "joint_1": -22.82,
        "joint_2": 3.95,
        "joint_3": 85.9,
        "joint_4": -0.02,
        "joint_5": 90.16,
        "joint_6": 67.19,
    }
    for joint_name, expected_degree in expected_degrees.items():
        assert math.isclose(
            math.degrees(DRAWER_STORE_TOOL_OBSERVE_POINT[joint_name]),
            expected_degree,
            abs_tol=0.001,
        )

    assert DRAWER_STORE_FORCE_DESCENT_START_Z_OFFSET_M == 0.03


def test_return_drawer_placement_uses_same_drawer_safe_z_min_as_pick():
    assert ReturnDrawerPlacementFlow._safe_z_min_for_drawer(1) == (
        SAFE_Z_MIN + DRAWER_1_SAFE_Z_OFFSET_M
    )
    assert ReturnDrawerPlacementFlow._safe_z_min_for_drawer(0) == SAFE_Z_MIN
    assert ReturnDrawerPlacementFlow._safe_z_min_for_drawer(None) == SAFE_Z_MIN


def test_return_sequence_builds_drawer_store_step_order():
    runner = ReturnSequenceRunner(
        robot=object(),
        motion_controller=object(),
        gripper=object(),
        state=FakeState(),
        drawer_flow=FakeDrawerFlow(),
        detect_store_tool_label=lambda: "screwdriver",
        resolve_store_tool_target=lambda _tool_name: object(),
        resolve_drawer_marker_target=lambda _drawer_id: object(),
    )

    names = [step.name for step in runner.build_steps({"tool_name": "unknown"})]

    assert names == [
        "return/prepare",
        "return/receive_tool",
        "return/place_store_observe_point",
        "return/observe_store_tool_label",
        "return/open_observed_tool_drawer",
        "return/observe_open_drawer",
        "return/resolve_drawer_marker",
        "return/move_to_store_tool_observe_point",
        "return/resolve_store_tool_bbox",
        "return/grasp_store_tool",
        "return/place_tool_at_drawer_marker",
        "return/close_observed_tool_drawer",
        "return/home_after_drawer_close",
        "return/done",
    ]
