"""Standalone drawer open/close motion test node."""

from __future__ import annotations

import json
import sys
import threading
import time

import rclpy
from moveit.planning import MoveItPy, PlanRequestParameters
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from macgyvbot_config.drawer import (
    DRAWER_HANDLE_JOINT_DEGREES,
    DRAWER_OBSERVE_OFFSET_XYZ_M,
    DRAWER_OPEN_OFFSET_XYZ_M,
)
from macgyvbot_config.robot import GROUP_NAME
from macgyvbot_config.topics import DRAWER_COMMAND_TOPIC
from macgyvbot_manipulation.drawer_motion import DrawerMotionFlow
from macgyvbot_manipulation.moveit_controller import MoveItController
from macgyvbot_manipulation.onrobot_gripper import RG


DEFAULT_GRIPPER_IP = "192.168.1.1"
DEFAULT_GRIPPER_PORT = 502


class DrawerMotionTestNode(Node):
    """Move to a drawer-handle joint pose and test offset-based motions."""

    def __init__(self):
        super().__init__("drawer_motion_test")
        self._declare_parameters()

        self.dry_run = self.get_parameter("dry_run").value

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
        self.drawer_flow = DrawerMotionFlow(
            self.robot,
            self.motion,
            self.gripper,
            self._cooperative_wait,
            dry_run=self.dry_run,
        )
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
        self.declare_parameter("gripper_ip", DEFAULT_GRIPPER_IP)
        self.declare_parameter("gripper_port", DEFAULT_GRIPPER_PORT)
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
        log.info(
            "drawer motion test 시작 "
            f"(drawer={drawer_id}, dry_run={self.dry_run})"
        )
        log.info(
            "offsets: "
            f"open={DRAWER_OPEN_OFFSET_XYZ_M}, "
            f"observe={DRAWER_OBSERVE_OFFSET_XYZ_M}"
        )

        if not self.drawer_flow.open_drawer(drawer_id, log):
            return False

        if not self.drawer_flow.observe_drawer(drawer_id, log):
            return False

        if not self.drawer_flow.close_drawer(drawer_id, log):
            return False

        if not self._move_home():
            return False

        log.info("drawer motion test 완료")
        return True

    @staticmethod
    def _cooperative_wait(duration_sec):
        end_time = time.monotonic() + max(0.0, float(duration_sec))
        while rclpy.ok() and time.monotonic() < end_time:
            remaining = end_time - time.monotonic()
            time.sleep(min(0.02, max(0.0, remaining)))

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

    def _move_home(self):
        self.get_logger().info("Home 위치로 이동")
        if self.dry_run:
            return True

        return self.motion.move_to_home_joints(self.get_logger())

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
