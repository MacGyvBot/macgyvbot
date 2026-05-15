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
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from macgyvbot.config.config import (
    BASE_FRAME,
    DEFAULT_GRASP_POINT_MODE,
    FORCE_TORQUE_TOPIC,
    GROUP_NAME,
    HAND_GRASP_IMAGE_TOPIC,
    HAND_GRASP_TOPIC,
    HAND_GRASP_WINDOW_NAME,
    ROBOT_WINDOW_NAME,
    ROBOT_STATUS_TOPIC,
    TOOL_COMMAND_TOPIC,
    YOLO_MODEL_NAME,
)
from macgyvbot.util.macgyvbot_main.model_control.moveit_controller import (
    MoveItController,
)
from macgyvbot.util.macgyvbot_main.model_control.onrobot_gripper import RG
from macgyvbot.util.macgyvbot_main.model_control.robot_pose import get_ee_matrix
from macgyvbot.util.macgyvbot_main.perception.depth_projection import (
    DepthProjector,
    pixel_to_camera_point,
)
from macgyvbot.util.macgyvbot_main.grasp_mechanism.grasp_point_selector import (
    GraspPointSelector,
    normalize_grasp_point_mode,
)
from macgyvbot.util.macgyvbot_main.perception.yolo_detector import YoloDetector
from macgyvbot.util.macgyvbot_main.task_pipeline.pick_sequence import (
    PickSequenceRunner,
)
from macgyvbot.util.macgyvbot_main.task_pipeline.return_sequence import (
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
        self.depth_projector = DepthProjector(self.robot, self.gripper2cam)

        self.pilz_params = PlanRequestParameters(self.robot)
        self.pilz_params.planning_pipeline = "pilz_industrial_motion_planner"
        self.pilz_params.planner_id = "PTP"
        self.pilz_params.max_velocity_scaling_factor = 0.2

        self.motion = MoveItController(self.robot, self.arm, self.pilz_params)
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
        self.robot_status_pub = self.create_publisher(
            String,
            ROBOT_STATUS_TOPIC,
            10,
        )

        self._create_subscriptions()
        self._log_startup()

    def logger(self):
        return self.get_logger()

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
        self.get_logger().info(f"힘/토크 입력 토픽: {self.force_torque_topic}")

    def _target_label_cb(self, msg):
        val = msg.data.strip()

        if not val:
            return

        self._set_target_label(val, source="/target_label")

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

        action = command.get("action", "unknown")
        tool_name = command.get("tool_name", "unknown")

        if action == "bring":
            self._set_target_label(tool_name, source="/tool_command", command=command)
            return

        if action == "return":
            self.start_return_sequence(command)
            return

        if action == "release":
            if self.picking:
                self.get_logger().warn("pick 동작 중 release 명령은 수동 실행하지 않습니다.")
                self._publish_robot_status(
                    "busy",
                    tool_name=tool_name,
                    action=action,
                    message=(
                        "pick 동작 중에는 자동 핸드오프 절차가 그리퍼를 제어합니다."
                    ),
                    reason="handoff_controls_release",
                    command=command,
                )
                return

            self.get_logger().info("release 명령 수신: 그리퍼를 엽니다.")
            self.gripper.open_gripper()
            self._publish_robot_status(
                "done",
                tool_name=tool_name,
                action=action,
                message="그리퍼를 열었습니다.",
                command=command,
            )
            return

        if action == "stop":
            self.get_logger().warn("stop 명령 수신")
            if self.picking:
                self._publish_robot_status(
                    "busy",
                    tool_name=tool_name,
                    action=action,
                    message=(
                        "이미 실행 중인 MoveIt 동작은 안전 중단을 지원하지 않아 "
                        "완료를 기다립니다."
                    ),
                    reason="active_motion_not_interruptible",
                    command=command,
                )
                return

            self.target_label = None
            self.current_command = None
            self._publish_robot_status(
                "cancelled",
                tool_name=tool_name,
                action=action,
                message="대기 중인 pick 요청을 취소했습니다.",
                command=command,
            )
            return

        self.get_logger().warn(f"지원하지 않는 action: {action}")
        self._publish_robot_status(
            "rejected",
            tool_name=tool_name,
            action=action,
            message="지원하지 않는 명령입니다.",
            reason="unsupported_action",
            command=command,
        )

    def _wrench_cb(self, msg):
        self.latest_wrench = msg.wrench

    def _set_target_label(self, tool_name, source, command=None):
        val = (tool_name or "").strip()

        if not val or val == "unknown":
            self._publish_robot_status(
                "rejected",
                tool_name=val or "unknown",
                message="대상 공구가 비어 있거나 unknown입니다.",
                reason="unknown_tool",
                command=command,
            )
            return

        if self.picking:
            self.get_logger().warn(
                f"현재 pick 동작 중이라 새 타겟 '{val}' 입력은 무시합니다."
            )
            self._publish_robot_status(
                "busy",
                tool_name=val,
                message=f"현재 pick 동작 중이라 새 타겟 '{val}' 입력은 무시합니다.",
                reason="already_picking",
                command=command,
            )
            return

        self.target_label = val
        self.current_command = command
        self._last_search_status_target = None
        self.get_logger().info(f"타겟 객체 설정: {self.target_label} ({source})")
        self._publish_robot_status(
            "accepted",
            tool_name=val,
            action="bring",
            message=f"{val} 탐색을 시작합니다.",
            command=command,
        )

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
        self._move_home_and_capture_pose()
        self._process_frames()

    def _move_home_and_capture_pose(self):
        self.get_logger().info("시스템 준비 중... Home으로 이동합니다.")

        ok = self.motion.move_to_home_joints(self.get_logger())
        if not ok:
            self.get_logger().error("초기 Home 이동 실패")
            return

        transform = get_ee_matrix(self.robot)
        self.home_xyz = (
            transform[0, 3],
            transform[1, 3],
            transform[2, 3],
        )

        qx, qy, qz, qw = Rotation.from_matrix(transform[:3, :3]).as_quat()
        self.home_ori = {
            "x": float(qx),
            "y": float(qy),
            "z": float(qz),
            "w": float(qw),
        }

        self.get_logger().info(
            f"Home 저장 완료: x={self.home_xyz[0]:.3f}, "
            f"y={self.home_xyz[1]:.3f}, z={self.home_xyz[2]:.3f}"
        )

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

            if not self._has_camera_state():
                continue

            if self.picking:
                self._show_robot_window(self.color_image)
                self._show_hand_grasp_window()
                if self._debug_wait_key() == 27:
                    break
                continue

            results = self.detector.detect(self.color_image)
            annotated_frame = results[0].plot()

            if self.target_label:
                self._handle_target_detection(
                    results[0].boxes,
                    annotated_frame,
                )

            self._show_robot_window(annotated_frame)
            self._show_hand_grasp_window()

            if self._debug_wait_key() == 27:
                break

        self._close_windows()

    def _has_camera_state(self):
        return (
            self.color_image is not None
            and self.depth_image is not None
            and self.intrinsics is not None
        )

    def _handle_target_detection(self, boxes, annotated_frame):
        found = False

        for box in boxes:
            label = self.detector.names[int(box.cls)]
            if label != self.target_label:
                continue

            found = True
            u, v, source, vlm_rpy_deg = self.grasp_point_selector.select(
                box,
                label,
                self.color_image,
                self.depth_image,
                self.intrinsics,
                self.target_label,
            )
            target = self.depth_projector.pixel_to_base_target(
                u,
                v,
                label,
                source,
                self.depth_image,
                self.intrinsics,
                self.get_logger(),
                vlm_rpy_deg,
            )
            if target is None:
                continue

            bx, by, bz, z_m, vlm_rpy_deg = target
            vlm_yaw_deg = self._extract_vlm_yaw(vlm_rpy_deg)
            self._draw_grasp_marker(annotated_frame, u, v, source)
            self.start_pick_sequence(bx, by, bz, z_m, vlm_yaw_deg)
            break

        if not found:
            self.get_logger().info(
                f"'{self.target_label}' 탐색 중... 현재 프레임에서는 미검출"
            )
            if self._last_search_status_target != self.target_label:
                self._last_search_status_target = self.target_label
                self._publish_robot_status(
                    "searching",
                    tool_name=self.target_label,
                    action="bring",
                    message=f"{self.target_label} 탐색 중입니다.",
                    command=self.current_command,
                )

    def _extract_vlm_yaw(self, vlm_rpy_deg):
        if vlm_rpy_deg is None or len(vlm_rpy_deg) < 3:
            return None

        try:
            return float(vlm_rpy_deg[2])
        except (TypeError, ValueError):
            self.get_logger().warn(f"VLM yaw 파싱 실패: {vlm_rpy_deg}")
            return None

    @staticmethod
    def _draw_grasp_marker(frame, u, v, source):
        cv2.circle(frame, (u, v), 6, (0, 255, 255), -1)
        cv2.putText(
            frame,
            source,
            (u + 8, v - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )

    def _publish_robot_status(
        self,
        status,
        tool_name=None,
        action=None,
        message="",
        reason="",
        command=None,
    ):
        payload = {
            "status": status,
            "tool_name": tool_name or self.target_label or "unknown",
            "action": action or (command or {}).get("action", "unknown"),
            "message": message,
        }
        if reason:
            payload["reason"] = reason
        if command is not None:
            payload["command"] = command

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
