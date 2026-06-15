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
    DRAWER_0_SAFE_Z_MIN_M,
    DRAWER_1_SAFE_Z_OFFSET_M,
    DRAWER_2_SAFE_Z_OFFSET_M,
    DRAWER_STORE_MARKER_EXIT_OFFSET_XYZ_M,
    DRAWER_STORE_FORCE_DESCENT_START_Z_OFFSET_M,
    DRAWER_STORE_TOOL_OBSERVE_POINT,
    DRAWER_WALL_CLEARANCE_Z_OFFSET_M,
)
from macgyvbot_config.return_flow import RETURN_DRAWER_PLACE_WRIST_YAW_DEG
from macgyvbot_manipulation.robot_safezone import SAFE_Z_MIN
from macgyvbot_manipulation.robot_safezone import safe_z_min_for_drawer
from macgyvbot_task.application.return_flow.return_drawer_placement_flow import (
    ReturnDrawerPlacementFlow,
)
from macgyvbot_task.application.return_flow.return_perception_adapter import (
    ReturnPerceptionAdapter,
)
from macgyvbot_task.application.drawer_store_motion import (
    drawer_wall_clearance_z_for_drawer,
    rotate_wrist_for_drawer_store,
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


class FakeReporter:
    def __init__(self):
        self.failures = []

    def publish(self, *_args, **_kwargs):
        pass

    def fail(self, tool_name, message, reason, command, logger):
        self.failures.append(
            {
                "tool_name": tool_name,
                "message": message,
                "reason": reason,
                "command": command,
            }
        )


class FakeMotion:
    def __init__(self):
        self.targets = []
        self.min_z_values = []
        self.wrist_rotations = []

    def plan_and_execute(
        self,
        _logger,
        pose_goal,
        min_z=None,
        collision_scene_key=None,
    ):
        self.targets.append(pose_goal)
        self.min_z_values.append(min_z)
        return True

    def rotate_wrist_by_yaw_deg(
        self,
        yaw_deg,
        _logger,
        collision_scene_key=None,
    ):
        self.wrist_rotations.append(yaw_deg)
        return True


class FakeReturnGripper:
    def __init__(self):
        self.close_calls = 0
        self.open_calls = 0

    def close_gripper(self):
        self.close_calls += 1

    def open_gripper(self):
        self.open_calls += 1

    def get_status(self):
        return [False, True]

    def get_width(self):
        return 24.0

    def get_depth(self):
        return 18.0


class FakeReturnTargetPlanner:
    def plan(self, _bx, _by, _bz, _logger):
        return types.SimpleNamespace(
            target_x=0.30,
            target_y=0.10,
            grasp_z=0.27,
        )


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
    assert safe_z_min_for_drawer(1) == (
        SAFE_Z_MIN + DRAWER_1_SAFE_Z_OFFSET_M
    )
    assert safe_z_min_for_drawer(2) == (
        SAFE_Z_MIN + DRAWER_2_SAFE_Z_OFFSET_M
    )
    assert safe_z_min_for_drawer(0) == DRAWER_0_SAFE_Z_MIN_M
    assert safe_z_min_for_drawer(None) == SAFE_Z_MIN
    assert drawer_wall_clearance_z_for_drawer(1) == (
        SAFE_Z_MIN
        + DRAWER_1_SAFE_Z_OFFSET_M
        + DRAWER_WALL_CLEARANCE_Z_OFFSET_M
    )
    assert drawer_wall_clearance_z_for_drawer(0) == (
        DRAWER_0_SAFE_Z_MIN_M + DRAWER_WALL_CLEARANCE_Z_OFFSET_M
    )
    assert drawer_wall_clearance_z_for_drawer(2) == (
        SAFE_Z_MIN
        + DRAWER_2_SAFE_Z_OFFSET_M
        + DRAWER_WALL_CLEARANCE_Z_OFFSET_M
    )
    assert DRAWER_STORE_MARKER_EXIT_OFFSET_XYZ_M == [-0.15, 0.0, 0.0]


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


def test_drawer_store_wrist_rotation_uses_return_policy():
    motion = FakeMotion()

    assert rotate_wrist_for_drawer_store(
        motion,
        FakeLogger(),
        label="test_drawer_store",
    )
    assert motion.wrist_rotations == [RETURN_DRAWER_PLACE_WRIST_YAW_DEG]


def test_return_staged_tool_grasp_pregrasp_depth_adjusts_before_final_grasp(
    monkeypatch,
):
    import macgyvbot_task.application.return_flow.return_drawer_placement_flow as module

    monkeypatch.setattr(
        module,
        "current_ee_orientation",
        lambda _robot: {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    )
    motion = FakeMotion()
    gripper = FakeReturnGripper()
    reporter = FakeReporter()
    flow = ReturnDrawerPlacementFlow(
        robot=object(),
        motion_controller=motion,
        gripper=gripper,
        state=FakeState(),
        reporter=reporter,
        wait_fn=lambda _duration: None,
    )
    flow.target_planner = FakeReturnTargetPlanner()
    target = types.SimpleNamespace(found=True, base_xyz=(0.30, 0.10, 0.20))

    assert flow.grasp_staged_tool(
        target,
        "screwdriver",
        {"action": "return"},
        FakeLogger(),
    )

    z_targets = [pose.pose.position.z for pose in motion.targets]
    assert all(
        math.isclose(actual, expected)
        for actual, expected in zip(z_targets[:2], [0.27, 0.252])
    )
    assert math.isclose(motion.min_z_values[1], 0.252)
    assert gripper.close_calls == 2
    assert gripper.open_calls == 1
    assert reporter.failures == []


def test_return_store_tool_target_uses_center_point_then_refines_yaw():
    import numpy as np

    class FakeTensor:
        def __init__(self, value):
            self.value = value

        def cpu(self):
            return self

        def numpy(self):
            return np.array(self.value)

    class FakeBox:
        xyxy = [FakeTensor([1, 2, 21, 42])]

    class FakeDetector:
        names = {0: "screwdriver"}

        def detect(self, _image):
            return [types.SimpleNamespace(boxes=[FakeBox()])]

    class FakeFrameProcessor:
        def has_camera_state(self):
            return True

    class FakePickTargetResolver:
        def __init__(self):
            self.calls = []

        def target_from_boxes(
            self,
            boxes,
            target_label,
            _color_image,
            _depth_image,
            _intrinsics,
            use_bbox_center=False,
        ):
            self.calls.append((boxes, target_label, use_bbox_center))
            return types.SimpleNamespace(
                found=True,
                label=target_label,
                pixel=(11, 22),
                base_xyz=(0.3, 0.1, 0.2),
                yaw_deg=None,
            )

    state = types.SimpleNamespace(
        color_image=np.zeros((4, 4, 3), dtype=np.uint8),
        depth_image=np.zeros((4, 4), dtype=np.uint16),
        intrinsics={"fx": 1.0, "fy": 1.0, "ppx": 0.0, "ppy": 0.0},
    )
    resolver = FakePickTargetResolver()
    refined = []
    adapter = ReturnPerceptionAdapter(
        state,
        FakeDetector(),
        drawer_flow=object(),
        frame_processor=FakeFrameProcessor(),
        pick_target_resolver=resolver,
        depth_projector=object(),
        logger=FakeLogger(),
        refine_store_tool_target=lambda target, tool: (
            refined.append((target, tool))
            or types.SimpleNamespace(**{**target.__dict__, "yaw_deg": 37.0})
        ),
    )

    target = adapter.resolve_store_tool_target("screwdriver")

    assert target.pixel == (11, 22)
    assert target.yaw_deg == 37.0
    assert len(resolver.calls) == 1
    assert resolver.calls[0][1:] == ("screwdriver", True)
    assert refined[0][1] == "screwdriver"


def test_return_staged_tool_grasp_applies_target_yaw_before_motion(monkeypatch):
    import macgyvbot_task.application.return_flow.return_drawer_placement_flow as module

    monkeypatch.setattr(
        module,
        "current_ee_orientation",
        lambda _robot: {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    )
    motion = FakeMotion()
    gripper = FakeReturnGripper()
    flow = ReturnDrawerPlacementFlow(
        robot=object(),
        motion_controller=motion,
        gripper=gripper,
        state=FakeState(),
        reporter=FakeReporter(),
        wait_fn=lambda _duration: None,
    )
    flow.target_planner = FakeReturnTargetPlanner()
    target = types.SimpleNamespace(
        found=True,
        base_xyz=(0.30, 0.10, 0.20),
        yaw_deg=31.0,
    )

    assert flow.grasp_staged_tool(
        target,
        "screwdriver",
        {"action": "return"},
        FakeLogger(),
    )

    assert motion.wrist_rotations == [31.0]
    assert motion.targets


def test_return_drawer_place_offsets_grasp_yaw_back_to_store_yaw(monkeypatch):
    import numpy as np
    import macgyvbot_task.application.return_flow.return_drawer_placement_flow as module

    monkeypatch.setattr(
        module,
        "current_ee_orientation",
        lambda _robot: {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    )
    monkeypatch.setattr(
        module,
        "get_ee_matrix",
        lambda _robot: np.array(
            [
                [1.0, 0.0, 0.0, 0.30],
                [0.0, 1.0, 0.0, 0.10],
                [0.0, 0.0, 1.0, 0.40],
                [0.0, 0.0, 0.0, 1.0],
            ]
        ),
    )
    motion = FakeMotion()
    gripper = FakeReturnGripper()
    flow = ReturnDrawerPlacementFlow(
        robot=object(),
        motion_controller=motion,
        gripper=gripper,
        state=FakeState(),
        reporter=FakeReporter(),
        wait_fn=lambda _duration: None,
    )
    flow.target_planner = FakeReturnTargetPlanner()
    target = types.SimpleNamespace(
        found=True,
        base_xyz=(0.30, 0.10, 0.20),
        yaw_deg=31.0,
    )
    marker_target = types.SimpleNamespace(
        found=True,
        base_xyz=(0.40, 0.20, 0.20),
    )

    assert flow.grasp_staged_tool(
        target,
        "screwdriver",
        {"action": "return"},
        FakeLogger(),
    )
    assert flow.place_tool_at_marker(
        marker_target,
        "screwdriver",
        {"action": "return"},
        FakeLogger(),
        drawer_id=1,
    )

    assert motion.wrist_rotations == [
        31.0,
        RETURN_DRAWER_PLACE_WRIST_YAW_DEG - 31.0,
    ]
