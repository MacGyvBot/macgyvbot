#!/usr/bin/env python3
"""Main ROS wiring node for the MacGyvBot pick pipeline."""

import json
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import WrenchStamped
from moveit.planning import MoveItPy, PlanRequestParameters
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from macgyvbot.config.models import YOLO_MODEL_NAME
from macgyvbot.config.robot import (
    BASE_FRAME,
    GROUP_NAME,
)
from macgyvbot.config.topics import (
    FORCE_TORQUE_TOPIC,
    HAND_GRASP_IMAGE_TOPIC,
    HAND_GRASP_MASK_LOCK_TOPIC,
    HAND_GRASP_TOPIC,
    ROBOT_STATUS_TOPIC,
    TOOL_COMMAND_TOPIC,
)
from macgyvbot.config.ui import HAND_GRASP_WINDOW_NAME, ROBOT_WINDOW_NAME
from macgyvbot.config.vlm import DEFAULT_GRASP_POINT_MODE
from macgyvbot.control.moveit_controller import (
    MoveItController,
)
from macgyvbot.control.onrobot_gripper import RG
from macgyvbot.control.robot_pose import get_ee_matrix
from macgyvbot.perception.depth_projection import (
    DepthProjector,
    pixel_to_camera_point,
)
from macgyvbot.perception.grasp_mechanism.grasp_point_selector import (
    GraspPointSelector,
    normalize_grasp_point_mode,
)
from macgyvbot.perception.yolo_detector import YoloDetector
from macgyvbot.perception.pick_target_resolver import (
    PickTargetResolver,
)
from macgyvbot.application import (
    RobotStatusPublisher,
    ToolCommandController,
)
from macgyvbot.application.robot_home_initializer import RobotHomeInitializer
from macgyvbot.application.pick_frame_processor import PickFrameProcessor
from macgyvbot.application.pick_sequence import (
    PickSequenceRunner,
)
from macgyvbot.application.return_sequence import (
    ReturnSequenceRunner,
)


class MacGyvBotNode(Node):
    def __init__(self):
        super().__init__("macgyvbot_main_node")

        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None
        self.intrinsics = None
        self.picking = False
        self.target_label = None
        self.pending_pick_thread = None
        self.pending_return_thread = None
        self.human_grasped_tool = False
        self.last_grasp_result = None
        self.tool_mask_locked = False
        self.last_tool_mask_lock_result = None
        self.hand_grasp_image = None
        self.latest_wrench = None
        self.home_xyz = None
        self.home_ori = None
        self.current_command = None
        self._last_search_status_target = None

        self.grasp_point_mode = self._read_grasp_point_mode()
        self.yolo_model = self._read_yolo_model()
        self.force_torque_topic = self._read_force_torque_topic()
        self.detector = YoloDetector(self.yolo_model)
        self.grasp_point_selector = GraspPointSelector(
            self.grasp_point_mode,
            self.get_logger(),
        )

        calib_file = (
            Path(get_package_share_directory("macgyvbot"))
            / "calibration"
            / "T_gripper2camera.npy"
        )
        self.gripper2cam = np.load(str(calib_file)).astype(float)
        self.gripper2cam[:3, 3] /= 1000.0

        self.gripper = RG("rg2", "192.168.1.1", 502)
        self.robot = MoveItPy(node_name="yolo_pick_moveit_py")
        self.arm = self.robot.get_planning_component(GROUP_NAME)
        self.depth_projector = DepthProjector(self._base_to_camera_matrix)
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
            self,
        )
        self.robot_status_pub = self.create_publisher(
            String,
            ROBOT_STATUS_TOPIC,
            10,
        )
        self.status_publisher = RobotStatusPublisher(
            self._publish_status_payload,
            target_label_provider=lambda: self.target_label,
        )
        self.task_controller = ToolCommandController(
            self.get_logger(),
            self.status_publisher,
            is_busy=lambda: self.picking,
            set_target=self._set_active_target,
            clear_target=self._clear_pending_target,
            reset_search_status=self._reset_search_status,
            start_return=self.start_return_sequence,
            release_gripper=self.gripper.open_gripper,
        )
        self.frame_processor = PickFrameProcessor(
            self,
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
            self,
        )
        self.return_runner = ReturnSequenceRunner(
            self.robot,
            self.motion,
            self.gripper,
            self,
        )

        self._create_subscriptions()
        self._log_startup()

    def logger(self):
        return self.get_logger()

    def _base_to_camera_matrix(self):
        return get_ee_matrix(self.robot) @ self.gripper2cam

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
        self.latest_wrench = msg.wrench

    def _set_active_target(self, tool_name, command=None):
        self.target_label = tool_name
        self.current_command = command

    def _clear_pending_target(self):
        self.target_label = None
        self.current_command = None

    def _reset_search_status(self):
        self._last_search_status_target = None

    def _hand_grasp_cb(self, msg):
        try:
            result = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(
                f"잡기 인식 결과 JSON 파싱 실패: {msg.data}"
            )
            return

        result["_received_monotonic_sec"] = time.monotonic()
        self._attach_base_position_to_grasp_result(result)
        self.last_grasp_result = result
        self.human_grasped_tool = bool(result.get("human_grasped_tool", False))

    def _tool_mask_lock_cb(self, msg):
        try:
            result = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"공구 mask lock 결과 JSON 파싱 실패: {msg.data}")
            return

        self.last_tool_mask_lock_result = result
        self.tool_mask_locked = bool(result.get("locked", False))

    def _attach_base_position_to_grasp_result(self, result):
        position = result.get("position")
        if isinstance(position, dict) and all(k in position for k in ("x", "y", "z")):
            return

        hand_pixel = result.get("hand_pixel")
        if not isinstance(hand_pixel, dict):
            return
        if self.depth_image is None or self.intrinsics is None:
            return

        try:
            u = int(hand_pixel["u"])
            v = int(hand_pixel["v"])
        except (KeyError, TypeError, ValueError):
            return

        height, width = self.depth_image.shape[:2]
        clamped_u = min(max(u, 0), max(width - 1, 0))
        clamped_v = min(max(v, 0), max(height - 1, 0))
        if clamped_u != u or clamped_v != v:
            self.get_logger().warn(
                "handover_hand_pixel 경계 클램프 적용: "
                f"raw=({u}, {v}), clamped=({clamped_u}, {clamped_v}), "
                f"size=({width}, {height})"
            )
        u, v = clamped_u, clamped_v

        camera_point = pixel_to_camera_point(
            u,
            v,
            self.depth_image,
            self.intrinsics,
            logger=self.get_logger(),
            source="handover_hand_pixel",
        )
        if camera_point is None:
            return

        bx, by, bz = self.depth_projector.camera_to_base(camera_point)
        result["position"] = {
            "x": float(bx),
            "y": float(by),
            "z": float(bz),
            "frame_id": BASE_FRAME,
        }
        result["position_observed_monotonic_sec"] = result.get(
            "_received_monotonic_sec",
            time.monotonic(),
        )

    def _hand_grasp_image_cb(self, msg):
        try:
            self.hand_grasp_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as exc:
            self.get_logger().warn(f"잡기 인식 화면 변환 실패: {exc}")

    def _cam_info_cb(self, msg):
        self.intrinsics = {
            "fx": msg.k[0],
            "fy": msg.k[4],
            "ppx": msg.k[2],
            "ppy": msg.k[5],
        }

    def _color_cb(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def _depth_cb(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def start_pick_sequence(self, bx, by, bz, z_m, vlm_yaw_deg=None):
        if self.picking:
            self.get_logger().warn("이미 pick 동작 중이라 새 pick 요청을 무시합니다.")
            return

        self.picking = True
        self._publish_robot_status(
            "picking",
            tool_name=self.target_label,
            action="bring",
            message=f"{self.target_label} pick 동작을 시작합니다.",
            command=self.current_command,
        )
        self.pending_pick_thread = threading.Thread(
            target=self.pick_runner.run,
            args=(bx, by, bz, z_m, vlm_yaw_deg),
            daemon=True,
        )
        self.pending_pick_thread.start()

    def start_return_sequence(self, command):
        if self.picking:
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
        self.picking = True
        self.target_label = None
        self.current_command = command
        self.pending_return_thread = threading.Thread(
            target=self.return_runner.run,
            args=(command,),
            daemon=True,
        )
        self.pending_return_thread.start()

    def run(self):
        self.home_initializer.initialize()
        self._process_frames()

    def _process_frames(self):
        if self.home_xyz is None or self.home_ori is None:
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

            if self.picking:
                self._show_robot_window(self.color_image)
                self._show_hand_grasp_window()
                if self._debug_wait_key() == 27:
                    break
                continue

            annotated_frame = self.frame_processor.process_current_frame()

            self._show_robot_window(annotated_frame)
            self._show_hand_grasp_window()

            if self._debug_wait_key() == 27:
                break

        self._close_windows()

    def _latest_camera_state(self):
        return self.color_image, self.depth_image, self.intrinsics

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

    def _show_robot_window(self, image):
        cv2.imshow(ROBOT_WINDOW_NAME, image)

    def _show_hand_grasp_window(self):
        if self.hand_grasp_image is not None:
            cv2.imshow(HAND_GRASP_WINDOW_NAME, self.hand_grasp_image)

    @staticmethod
    def _debug_wait_key(delay_ms=1):
        return cv2.waitKey(delay_ms)

    @staticmethod
    def _close_windows():
        cv2.destroyAllWindows()


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
