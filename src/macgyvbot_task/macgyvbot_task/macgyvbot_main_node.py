#!/usr/bin/env python3
"""Main ROS wiring node for the MacGyvBot pick pipeline."""

import json
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import WrenchStamped
from moveit.planning import MoveItPy, PlanRequestParameters
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from macgyvbot_config.models import YOLO_MODEL_NAME
from macgyvbot_config.robot import GROUP_NAME
from macgyvbot_config.topics import (
    FORCE_TORQUE_TOPIC,
    HAND_GRASP_IMAGE_TOPIC,
    HAND_GRASP_MASK_LOCK_TOPIC,
    HAND_GRASP_TOPIC,
    ROBOT_STATUS_TOPIC,
    TOOL_COMMAND_TOPIC,
)
from macgyvbot_config.vlm import DEFAULT_GRASP_POINT_MODE
from macgyvbot_manipulation.moveit_controller import (
    MoveItController,
)
from macgyvbot_manipulation.onrobot_gripper import RG
from macgyvbot_manipulation.robot_pose import get_ee_matrix
from macgyvbot_perception.depth_projection import DepthProjector
from macgyvbot_perception.grasp_point.grasp_point_selector import (
    GraspPointSelector,
    normalize_grasp_point_mode,
)
from macgyvbot_perception.yolo_detector import YoloDetector
from macgyvbot_perception.pick_target_resolver import (
    PickTargetResolver,
)
from macgyvbot_task.application import (
    RobotStatusPublisher,
    TaskRuntimeState,
    ToolCommandController,
)
from macgyvbot_task.application.adapters.hand_grasp_result_adapter import (
    HandGraspResultAdapter,
)
from macgyvbot_task.application.display.debug_display import DebugDisplay
from macgyvbot_task.application.pick_flow.pick_frame_processor import (
    PickFrameProcessor,
)
from macgyvbot_task.application.pick_flow.pick_sequence import (
    PickSequenceRunner,
)
from macgyvbot_task.application.return_flow.return_sequence import (
    ReturnSequenceRunner,
)
from macgyvbot_task.application.robot.robot_home_initializer import (
    RobotHomeInitializer,
)


class MacGyvBotNode(Node):
    def __init__(self):
        super().__init__("macgyvbot_main_node")

        self.bridge = CvBridge()
        self.display = DebugDisplay()

        self.grasp_point_mode = self._read_grasp_point_mode()
        self.yolo_model = self._read_yolo_model()
        self.force_torque_topic = self._read_force_torque_topic()
        self.robot_status_pub = self.create_publisher(
            String,
            ROBOT_STATUS_TOPIC,
            10,
        )
        self.state = TaskRuntimeState(
            logger_provider=self.get_logger,
            publish_robot_status=self._publish_robot_status,
            publish_status_payload=self._publish_status_payload,
        )
        self.detector = YoloDetector(self.yolo_model)
        self.grasp_point_selector = GraspPointSelector(
            self.grasp_point_mode,
            self.get_logger(),
        )

        calib_file = self._resolve_calibration_file("T_gripper2camera.npy")
        self.gripper2cam = np.load(str(calib_file)).astype(float)
        self.gripper2cam[:3, 3] /= 1000.0

        self.gripper = RG("rg2", "192.168.1.1", 502)
        self.robot = MoveItPy(node_name="yolo_pick_moveit_py")
        self.arm = self.robot.get_planning_component(GROUP_NAME)
        self.depth_projector = DepthProjector(self._base_to_camera_matrix)
        self.hand_grasp_adapter = HandGraspResultAdapter(
            self.state,
            self.depth_projector,
            self.get_logger(),
        )
        self.pick_target_resolver = PickTargetResolver(
            self.detector,
            self.grasp_point_selector,
            self.depth_projector,
            self.get_logger(),
        )

        self.pilz_params = PlanRequestParameters(self.robot)
        self.pilz_params.planning_pipeline = "pilz_industrial_motion_planner"
        self.pilz_params.planner_id = "PTP"
        self.pilz_params.max_velocity_scaling_factor = 0.2

        self.motion = MoveItController(self.robot, self.arm, self.pilz_params)
        self.home_initializer = RobotHomeInitializer(
            self.robot,
            self.motion,
            self.state,
        )
        self.status_publisher = RobotStatusPublisher(
            self._publish_status_payload,
            target_label_provider=lambda: self.state.target_label,
        )
        self.task_controller = ToolCommandController(
            self.get_logger(),
            self.status_publisher,
            is_busy=lambda: self.state.picking,
            set_target=self._set_active_target,
            clear_target=self._clear_pending_target,
            reset_search_status=self._reset_search_status,
            start_return=self.start_return_sequence,
            release_gripper=self.gripper.open_gripper,
        )
        self.frame_processor = PickFrameProcessor(
            self.state,
            self.detector,
            self.pick_target_resolver,
            self.start_pick_sequence,
            self._publish_robot_status,
            self.get_logger(),
        )
        self.pick_runner = PickSequenceRunner(
            self.robot,
            self.motion,
            self.gripper,
            self.state,
        )
        self.return_runner = ReturnSequenceRunner(
            self.robot,
            self.motion,
            self.gripper,
            self.state,
        )

        self._create_subscriptions()
        self._log_startup()

    def logger(self):
        return self.get_logger()

    def _base_to_camera_matrix(self):
        return get_ee_matrix(self.robot) @ self.gripper2cam

    @staticmethod
    def _resolve_calibration_file(filename):
        try:
            package_share = Path(
                get_package_share_directory("macgyvbot_resources")
            )
            candidate = package_share / "calibration" / filename
            if candidate.exists():
                return candidate
        except Exception:
            pass

        workspace_src = Path(__file__).resolve().parents[3]
        return workspace_src / "macgyvbot_resources" / "calibration" / filename

    def _read_grasp_point_mode(self):
        self.declare_parameter("grasp_point_mode", DEFAULT_GRASP_POINT_MODE)
        grasp_point_mode = (
            self.get_parameter("grasp_point_mode")
            .get_parameter_value()
            .string_value
            .strip()
            .lower()
        )
        return normalize_grasp_point_mode(grasp_point_mode, self.get_logger())

    def _read_yolo_model(self):
        self.declare_parameter("yolo_model", YOLO_MODEL_NAME)
        return (
            self.get_parameter("yolo_model")
            .get_parameter_value()
            .string_value
            .strip()
        ) or YOLO_MODEL_NAME

    def _read_force_torque_topic(self):
        self.declare_parameter("force_torque_topic", FORCE_TORQUE_TOPIC)
        return (
            self.get_parameter("force_torque_topic")
            .get_parameter_value()
            .string_value
            .strip()
        ) or FORCE_TORQUE_TOPIC

    def _create_subscriptions(self):
        self.create_subscription(
            CameraInfo,
            "/camera/camera/color/camera_info",
            self._cam_info_cb,
            10,
        )
        self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self._color_cb,
            10,
        )
        self.create_subscription(
            Image,
            "/camera/camera/aligned_depth_to_color/image_raw",
            self._depth_cb,
            10,
        )
        self.create_subscription(
            String,
            "/target_label",
            self._target_label_cb,
            10,
        )
        self.create_subscription(
            String,
            TOOL_COMMAND_TOPIC,
            self._tool_command_cb,
            10,
        )
        self.create_subscription(
            String,
            HAND_GRASP_TOPIC,
            self._hand_grasp_cb,
            10,
        )
        self.create_subscription(
            Image,
            HAND_GRASP_IMAGE_TOPIC,
            self._hand_grasp_image_cb,
            10,
        )
        self.create_subscription(
            String,
            HAND_GRASP_MASK_LOCK_TOPIC,
            self._tool_mask_lock_cb,
            10,
        )
        self.create_subscription(
            WrenchStamped,
            self.force_torque_topic,
            self._wrench_cb,
            10,
        )

    def _log_startup(self):
        self.get_logger().info("노드 초기화 완료")
        self.get_logger().info(f"YOLO model: {self.yolo_model}")
        self.get_logger().info(f"grasp point mode: {self.grasp_point_mode}")
        self.get_logger().info(
            "객체 입력 예시: ros2 topic pub --once /target_label "
            "std_msgs/msg/String \"{data: cup}\""
        )
        self.get_logger().info(f"공구 명령 토픽: {TOOL_COMMAND_TOPIC}")
        self.get_logger().info(f"로봇 상태 토픽: {ROBOT_STATUS_TOPIC}")
        self.get_logger().info(f"잡기 인식 결과 토픽: {HAND_GRASP_TOPIC}")
        self.get_logger().info(f"잡기 인식 화면 토픽: {HAND_GRASP_IMAGE_TOPIC}")
        self.get_logger().info(f"공구 mask lock 토픽: {HAND_GRASP_MASK_LOCK_TOPIC}")
        self.get_logger().info(f"힘/토크 입력 토픽: {self.force_torque_topic}")

    def _target_label_cb(self, msg):
        val = msg.data.strip()

        if not val:
            return

        self.task_controller.handle_target_label(val, source="/target_label")

    def _tool_command_cb(self, msg):
        try:
            command = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"/tool_command JSON 파싱 실패: {msg.data}")
            self._publish_robot_status(
                "rejected",
                message="명령 JSON을 해석하지 못했습니다.",
                reason="invalid_json",
            )
            return

        self.task_controller.handle_command(command)

    def _wrench_cb(self, msg):
        self.state.latest_wrench = msg.wrench

    def _set_active_target(self, tool_name, command=None):
        self.state.target_label = tool_name
        self.state.current_command = command

    def _clear_pending_target(self):
        self.state.target_label = None
        self.state.current_command = None

    def _reset_search_status(self):
        self.state._last_search_status_target = None

    def _hand_grasp_cb(self, msg):
        try:
            result = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(
                f"잡기 인식 결과 JSON 파싱 실패: {msg.data}"
            )
            return

        result["_received_monotonic_sec"] = time.monotonic()
        self.hand_grasp_adapter.attach_base_position(result)
        self.state.last_grasp_result = result
        self.state.human_grasped_tool = bool(result.get("human_grasped_tool", False))

    def _tool_mask_lock_cb(self, msg):
        try:
            result = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"공구 mask lock 결과 JSON 파싱 실패: {msg.data}")
            return

        self.state.last_tool_mask_lock_result = result
        self.state.tool_mask_locked = bool(result.get("locked", False))

    def _hand_grasp_image_cb(self, msg):
        try:
            self.state.hand_grasp_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as exc:
            self.get_logger().warn(f"잡기 인식 화면 변환 실패: {exc}")

    def _cam_info_cb(self, msg):
        self.state.intrinsics = {
            "fx": msg.k[0],
            "fy": msg.k[4],
            "ppx": msg.k[2],
            "ppy": msg.k[5],
        }

    def _color_cb(self, msg):
        self.state.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def _depth_cb(self, msg):
        self.state.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def start_pick_sequence(self, bx, by, bz, z_m, vlm_yaw_deg=None):
        if self.state.picking:
            self.get_logger().warn("이미 pick 동작 중이라 새 pick 요청을 무시합니다.")
            return

        self.state.picking = True
        self._publish_robot_status(
            "picking",
            tool_name=self.state.target_label,
            action="bring",
            message=f"{self.state.target_label} pick 동작을 시작합니다.",
            command=self.state.current_command,
        )
        self.state.pending_pick_thread = threading.Thread(
            target=self.pick_runner.run,
            args=(bx, by, bz, z_m, vlm_yaw_deg),
            daemon=True,
        )
        self.state.pending_pick_thread.start()

    def start_return_sequence(self, command):
        if self.state.picking:
            tool_name = command.get("tool_name", "unknown")
            self.get_logger().warn(
                "이미 로봇 동작 중이라 반납 요청을 무시합니다."
            )
            self._publish_robot_status(
                "busy",
                tool_name=tool_name,
                action="return",
                message="이미 로봇 동작 중이라 반납 요청을 무시합니다.",
                reason="already_picking",
                command=command,
            )
            return

        tool_name = command.get("tool_name", "unknown")
        self.get_logger().info(f"반납 시퀀스 시작: tool={tool_name}")
        self.state.picking = True
        self.state.target_label = None
        self.state.current_command = command
        self.state.pending_return_thread = threading.Thread(
            target=self.return_runner.run,
            args=(command,),
            daemon=True,
        )
        self.state.pending_return_thread.start()

    def run(self):
        self.home_initializer.initialize()
        self._process_frames()

    def _process_frames(self):
        if self.state.home_xyz is None or self.state.home_ori is None:
            return

        self.get_logger().info("카메라 영상 대기 중...")
        self.get_logger().info(
            "다른 터미널에서 객체명을 publish 하세요. 예: "
            "ros2 topic pub --once /target_label "
            "std_msgs/msg/String \"{data: cup}\""
        )

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            if not self.frame_processor.has_camera_state():
                continue

            if self.state.picking:
                self.display.show_robot(self.state.color_image)
                self.display.show_hand_grasp(self.state.hand_grasp_image)
                if self.display.wait_key() == 27:
                    break
                continue

            annotated_frame = self.frame_processor.process_current_frame()

            self.display.show_robot(annotated_frame)
            self.display.show_hand_grasp(self.state.hand_grasp_image)

            if self.display.wait_key() == 27:
                break

        self.display.close()

    def _latest_camera_state(self):
        return self.state.color_image, self.state.depth_image, self.state.intrinsics

    def _publish_robot_status(
        self,
        status,
        tool_name=None,
        action=None,
        message="",
        reason="",
        command=None,
    ):
        self.status_publisher.publish(
            status,
            tool_name=tool_name,
            action=action,
            message=message,
            reason=reason,
            command=command,
        )

    def _publish_status_payload(self, payload):
        self.robot_status_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )

def main():
    rclpy.init()
    node = MacGyvBotNode()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
