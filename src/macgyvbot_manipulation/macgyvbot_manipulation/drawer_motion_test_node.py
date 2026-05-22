"""Standalone drawer open/close motion test node."""

from __future__ import annotations

import json
import math
import sys
import threading
import time

import rclpy
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, PlanRequestParameters
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from macgyvbot_config.robot import GROUP_NAME
from macgyvbot_config.topics import DRAWER_COMMAND_TOPIC
from macgyvbot_manipulation.moveit_controller import MoveItController
from macgyvbot_manipulation.onrobot_gripper import RG
from macgyvbot_manipulation.robot_pose import (
    current_ee_orientation,
    get_ee_matrix,
    make_safe_pose,
)


DEFAULT_JOINT_NAMES = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
]
DRAWER_HANDLE_JOINT_DEGREES = {
    # Replace these calibrated handle poses with the measured drawer joints.
    1: [0.86, 18.77, 83.32, 20.08, 77.24, -2.24],
    2: [0.86, 21.67, 90.68, 21.22, 67.64, -6.05],
}
DEFAULT_ZERO_OFFSET_XYZ_M = [0.0, 0.0, 0.0]
DEFAULT_OPEN_OFFSET_XYZ_M = [0.0, 0.12, 0.0]
DEFAULT_OBSERVE_OFFSET_XYZ_M = [0.0, 0.06, 0.20]
DEFAULT_GRIPPER_IP = "192.168.1.1"
DEFAULT_GRIPPER_PORT = 502
DEFAULT_GRIPPER_SETTLE_SEC = 0.8


class DrawerMotionTestNode(Node):
    """Move to a drawer-handle joint pose and test offset-based motions."""

    def __init__(self):
        super().__init__("drawer_motion_test")
        self._declare_parameters()

        self.dry_run = self.get_parameter("dry_run").value
        self.joint_names = DEFAULT_JOINT_NAMES
        self.open_offset_xyz = self._read_xyz("open_offset_xyz")
        self.observe_offset_xyz = self._read_xyz("observe_offset_xyz")
        self.close_offset_xyz = self._read_xyz("close_offset_xyz")
        self.use_reverse_open_for_close = self.get_parameter(
            "use_reverse_open_for_close"
        ).value
        self.gripper_settle_sec = float(
            self.get_parameter("gripper_settle_sec").value
        )

        self.robot = MoveItPy(node_name="drawer_motion_test_moveit_py")
        self.arm = self.robot.get_planning_component(GROUP_NAME)
        self.params = PlanRequestParameters(self.robot)
        self.params.planning_pipeline = self.get_parameter("planning_pipeline").value
        self.params.planner_id = self.get_parameter("planner_id").value
        self.params.max_velocity_scaling_factor = float(
            self.get_parameter("max_velocity_scaling_factor").value
        )
        self.params.max_acceleration_scaling_factor = float(
            self.get_parameter("max_acceleration_scaling_factor").value
        )
        self.motion = MoveItController(
            self.robot,
            self.arm,
            self.params,
            node=self,
        )
        self.gripper = self._create_gripper()
        self._busy_lock = threading.Lock()
        self._active_worker = None
        self.create_subscription(
            String,
            DRAWER_COMMAND_TOPIC,
            self._drawer_command_cb,
            10,
        )
        self.get_logger().info(
            f"drawer motion test 준비 완료: topic={DRAWER_COMMAND_TOPIC}, "
            f"drawers={sorted(DRAWER_HANDLE_JOINT_DEGREES.keys())}, "
            f"dry_run={self.dry_run}"
        )

    def _declare_parameters(self):
        self.declare_parameter("dry_run", True)
        self.declare_parameter("open_offset_xyz", DEFAULT_OPEN_OFFSET_XYZ_M)
        self.declare_parameter("observe_offset_xyz", DEFAULT_OBSERVE_OFFSET_XYZ_M)
        self.declare_parameter("close_offset_xyz", DEFAULT_ZERO_OFFSET_XYZ_M)
        self.declare_parameter("use_reverse_open_for_close", True)
        self.declare_parameter("gripper_ip", DEFAULT_GRIPPER_IP)
        self.declare_parameter("gripper_port", DEFAULT_GRIPPER_PORT)
        self.declare_parameter("gripper_settle_sec", DEFAULT_GRIPPER_SETTLE_SEC)
        self.declare_parameter(
            "planning_pipeline",
            "pilz_industrial_motion_planner",
        )
        self.declare_parameter("planner_id", "PTP")
        self.declare_parameter("max_velocity_scaling_factor", 0.1)
        self.declare_parameter("max_acceleration_scaling_factor", 0.1)

    def _drawer_command_cb(self, msg):
        drawer_id = self._parse_drawer_id(msg.data)
        if drawer_id is None:
            self.get_logger().warn(
                f"서랍 command를 해석하지 못했습니다: {msg.data!r}"
            )
            return

        if drawer_id not in DRAWER_HANDLE_JOINT_DEGREES:
            self.get_logger().warn(f"지원하지 않는 drawer id입니다: {drawer_id}")
            return

        with self._busy_lock:
            if self._active_worker is not None and self._active_worker.is_alive():
                self.get_logger().warn(
                    f"서랍 동작 중이라 drawer {drawer_id} 요청을 무시합니다."
                )
                return

            self._active_worker = threading.Thread(
                target=self.run_for_drawer,
                args=(drawer_id,),
                daemon=True,
            )
            self._active_worker.start()

    def run_for_drawer(self, drawer_id):
        log = self.get_logger()
        handle_joint_values = self._drawer_joint_values(drawer_id)
        log.info(
            "drawer motion test 시작 "
            f"(drawer={drawer_id}, dry_run={self.dry_run}, "
            f"joints={self._format_joint_values(handle_joint_values)})"
        )
        log.info(
            "offsets: "
            f"open={self.open_offset_xyz}, "
            f"observe={self.observe_offset_xyz}, "
            f"close={self._close_offset_xyz()}"
        )

        if not self._move_to_handle_joints(drawer_id, handle_joint_values):
            log.error("서랍 손잡이 joint pose 이동 실패")
            return False

        handle_pose = get_ee_matrix(self.robot)
        handle_xyz = [
            float(handle_pose[0, 3]),
            float(handle_pose[1, 3]),
            float(handle_pose[2, 3]),
        ]
        handle_ori = current_ee_orientation(self.robot)
        log.info(f"손잡이 기준 pose: xyz={self._format_xyz(handle_xyz)}")

        if not self._close_gripper("손잡이 파지"):
            return False

        if not self._move_by_offset(
            handle_xyz,
            handle_ori,
            self.open_offset_xyz,
            label="서랍 열기 offset",
        ):
            return False

        opened_pose = get_ee_matrix(self.robot)
        opened_xyz = [
            float(opened_pose[0, 3]),
            float(opened_pose[1, 3]),
            float(opened_pose[2, 3]),
        ]

        if not self._open_gripper("서랍 열기 후 손잡이 release"):
            return False

        if not self._move_by_offset(
            opened_xyz,
            handle_ori,
            self.observe_offset_xyz,
            label="서랍 내부 관찰 offset",
        ):
            return False

        if not self._move_by_offset(
            opened_xyz,
            handle_ori,
            DEFAULT_ZERO_OFFSET_XYZ_M,
            label="관찰 후 열린 위치 복귀",
        ):
            return False

        if not self._close_gripper("닫기 전 손잡이 재파지"):
            return False

        if not self._move_by_offset(
            opened_xyz,
            handle_ori,
            self._close_offset_xyz(),
            label="서랍 닫기 offset",
        ):
            return False

        if not self._open_gripper("서랍 닫기 후 손잡이 release"):
            return False

        if not self._move_home():
            return False

        log.info("drawer motion test 완료")
        return True

    def _move_to_handle_joints(self, drawer_id, handle_joint_values):
        joint_positions = dict(zip(self.joint_names, handle_joint_values))
        self.get_logger().info(
            f"drawer {drawer_id} 손잡이 joint pose로 이동: "
            + ", ".join(
                f"{name}={value:.3f}rad"
                for name, value in joint_positions.items()
            )
        )
        if self.dry_run:
            return True

        state_goal = RobotState(self.robot.get_robot_model())
        state_goal.joint_positions = joint_positions
        state_goal.update()
        return self.motion.plan_and_execute(self.get_logger(), state_goal=state_goal)

    def _create_gripper(self):
        if self.dry_run:
            self.get_logger().info("dry_run=true라 그리퍼 연결을 생략합니다.")
            return None

        gripper_ip = self.get_parameter("gripper_ip").value
        gripper_port = int(self.get_parameter("gripper_port").value)
        self.get_logger().info(
            f"OnRobot gripper 연결: ip={gripper_ip}, port={gripper_port}"
        )
        return RG("rg2", gripper_ip, gripper_port)

    def _close_gripper(self, label):
        self.get_logger().info(f"{label}: 그리퍼 닫기")
        if self.dry_run:
            return True

        try:
            self.gripper.close_gripper()
            time.sleep(self.gripper_settle_sec)
        except Exception as exc:
            self.get_logger().error(f"그리퍼 닫기 실패: {exc}")
            return False
        return True

    def _open_gripper(self, label):
        self.get_logger().info(f"{label}: 그리퍼 열기")
        if self.dry_run:
            return True

        try:
            self.gripper.open_gripper()
            time.sleep(self.gripper_settle_sec)
        except Exception as exc:
            self.get_logger().error(f"그리퍼 열기 실패: {exc}")
            return False
        return True

    def _move_home(self):
        self.get_logger().info("Home 위치로 이동")
        if self.dry_run:
            return True

        return self.motion.move_to_home_joints(self.get_logger())

    def _move_by_offset(self, base_xyz, ori, offset_xyz, label):
        target_xyz = [
            float(base_xyz[0]) + float(offset_xyz[0]),
            float(base_xyz[1]) + float(offset_xyz[1]),
            float(base_xyz[2]) + float(offset_xyz[2]),
        ]
        self.get_logger().info(
            f"{label}: base={self._format_xyz(base_xyz)}, "
            f"offset={self._format_xyz(offset_xyz)}, "
            f"target={self._format_xyz(target_xyz)}"
        )
        if self.dry_run:
            return True

        return self.motion.plan_and_execute(
            self.get_logger(),
            pose_goal=make_safe_pose(
                target_xyz[0],
                target_xyz[1],
                target_xyz[2],
                ori,
                self.get_logger(),
            ),
        )

    def _close_offset_xyz(self):
        if self.use_reverse_open_for_close:
            return [
                -float(self.open_offset_xyz[0]),
                -float(self.open_offset_xyz[1]),
                -float(self.open_offset_xyz[2]),
            ]
        return self.close_offset_xyz

    def _drawer_joint_values(self, drawer_id):
        degrees = DRAWER_HANDLE_JOINT_DEGREES[drawer_id]
        self._validate_length(
            f"drawer {drawer_id} handle joints",
            degrees,
            len(self.joint_names),
        )
        return [math.radians(float(value)) for value in degrees]

    @staticmethod
    def _parse_drawer_id(payload):
        value = (payload or "").strip().lower()
        if not value:
            return None

        if value.startswith("{"):
            return DrawerMotionTestNode._parse_json_drawer_id(value)

        for prefix in ("drawer_", "drawer-", "drawer", "서랍"):
            if value.startswith(prefix):
                value = value[len(prefix):].strip()
                break

        try:
            return int(value)
        except ValueError:
            return None

    @staticmethod
    def _parse_json_drawer_id(payload):
        try:
            data = json.loads(payload)
        except json.JSONDecodeError:
            return None
        if not isinstance(data, dict):
            return None

        for key in ("drawer_id", "drawer", "id"):
            if key in data:
                try:
                    return int(data[key])
                except (TypeError, ValueError):
                    return None
        return None

    def _read_xyz(self, name):
        values = list(self.get_parameter(name).value)
        self._validate_length(name, values, 3)
        return [float(value) for value in values]

    @staticmethod
    def _validate_length(name, values, expected_length):
        if len(values) != expected_length:
            raise ValueError(
                f"{name} parameter must contain {expected_length} values "
                f"(got {len(values)})"
            )

    def _format_joint_values(self, handle_joint_values):
        return ", ".join(
            f"{name}={math.degrees(value):.1f}deg"
            for name, value in zip(self.joint_names, handle_joint_values)
        )

    @staticmethod
    def _format_xyz(values):
        return (
            f"({float(values[0]):.3f}, "
            f"{float(values[1]):.3f}, "
            f"{float(values[2]):.3f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = DrawerMotionTestNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
        ok = True
    except KeyboardInterrupt:
        ok = True
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
