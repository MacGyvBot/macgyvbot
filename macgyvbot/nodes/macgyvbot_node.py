#!/usr/bin/env python3
"""Main ROS wiring node for the MacGyvBot pick pipeline."""

import json
import threading
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, PlanRequestParameters
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from macgyvbot.core.config import (
    DEFAULT_GRASP_POINT_MODE,
    GRASP_POINT_MODE_CENTER,
    GRASP_POINT_MODE_VLA,
    GRASP_POINT_MODE_VLM,
    GROUP_NAME,
    HAND_GRASP_IMAGE_TOPIC,
    HAND_GRASP_TOPIC,
    HOME_JOINTS,
    YOLO_MODEL_NAME,
)
from macgyvbot.core.pick_sequence import PickSequenceRunner
from macgyvbot.motion.moveit_controller import (
    MoveItController,
    plan_and_execute,
)
from macgyvbot.motion.pose_utils import get_ee_matrix
from macgyvbot.onrobot import RG
from macgyvbot.perception.depth_projection import DepthProjector
from macgyvbot.perception.grasp_point_selector import GraspPointSelector
from macgyvbot.perception.yolo_detector import YoloDetector
from macgyvbot.ui.debug_windows import DebugWindows


class MacGyvBotNode(Node):
    def __init__(self):
        super().__init__("macgyvbot_node")

        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None
        self.intrinsics = None
        self.picking = False
        self.target_label = None
        self.pending_pick_thread = None
        self.human_grasped_tool = False
        self.last_grasp_result = None
        self.hand_grasp_image = None
        self.vla_state_model = None
        self.home_xyz = None
        self.home_ori = None

        self.grasp_point_mode = self._read_grasp_point_mode()
        self._preload_vla_or_fallback_to_center()
        self.yolo_model = self._read_yolo_model()
        self.detector = YoloDetector(self.yolo_model)

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

        self.pilz_params = PlanRequestParameters(self.robot)
        self.pilz_params.planning_pipeline = "pilz_industrial_motion_planner"
        self.pilz_params.planner_id = "PTP"
        self.pilz_params.max_velocity_scaling_factor = 0.2

        self.motion = MoveItController(self.robot, self.arm, self.pilz_params)
        self.grasp_selector = GraspPointSelector(
            self.grasp_point_mode,
            self.get_logger(),
        )
        self.projector = DepthProjector(
            self.robot,
            self.gripper2cam,
            self.get_logger(),
        )
        self.pick_runner = PickSequenceRunner(
            self.robot,
            self.motion,
            self.gripper,
            self,
        )
        self.debug_windows = DebugWindows(self)

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
        if grasp_point_mode not in (
            GRASP_POINT_MODE_CENTER,
            GRASP_POINT_MODE_VLM,
            GRASP_POINT_MODE_VLA,
        ):
            self.get_logger().warn(
                f"알 수 없는 grasp_point_mode '{grasp_point_mode}'. "
                f"'{GRASP_POINT_MODE_CENTER}'로 대체합니다."
            )
            return GRASP_POINT_MODE_CENTER
        return grasp_point_mode

    def _preload_vla_or_fallback_to_center(self):
        if self.grasp_point_mode != GRASP_POINT_MODE_VLA:
            return

        self.get_logger().info(
            "VLA 모드가 선택되어 시작 시점에 VLA 가중치 로드를 확인합니다."
        )
        if self.ensure_vla_state_model_loaded() is not None:
            return

        self.get_logger().warn(
            "VLA 가중치 로드에 실패하여 grasp_point_mode를 "
            f"'{GRASP_POINT_MODE_CENTER}'로 fallback합니다."
        )
        self.grasp_point_mode = GRASP_POINT_MODE_CENTER

    def ensure_vla_state_model_loaded(self):
        try:
            from macgyvbot.grasp_point_vla import VLAStateModel
        except ImportError as exc:
            self.get_logger().warn(f"VLA grasp 모듈 import 실패: {exc}")
            return None

        if self.vla_state_model is not None:
            if (
                self.vla_state_model.model is not None
                and self.vla_state_model.processor is not None
            ):
                config = self.vla_state_model.config
                self.get_logger().info(
                    f"VLA 가중치 사용 준비 완료: model_id={config.model_id}, "
                    f"device={self.vla_state_model.device}, "
                    f"dtype={self.vla_state_model.torch_dtype}"
                )
                return self.vla_state_model
        else:
            self.get_logger().info("VLA grasp 모델 인스턴스 초기화를 시작합니다.")
            self.vla_state_model = VLAStateModel()

        config = self.vla_state_model.config
        self.get_logger().info(f"VLA 가중치 로드 시작: model_id={config.model_id}")

        try:
            self.vla_state_model.load()
        except Exception as exc:
            self.get_logger().warn(
                f"VLA 가중치 로드 실패: model_id={config.model_id}, error={exc}"
            )
            try:
                self.vla_state_model.unload()
            except Exception:
                pass
            self.vla_state_model = None
            return None

        quantization = "4bit" if config.use_4bit else "full-precision"
        self.get_logger().info(
            f"VLA 가중치 로드 성공: model_id={config.model_id}, "
            f"device={self.vla_state_model.device}, "
            f"dtype={self.vla_state_model.torch_dtype}, "
            f"quantization={quantization}"
        )
        return self.vla_state_model

    def _read_yolo_model(self):
        self.declare_parameter("yolo_model", YOLO_MODEL_NAME)
        return (
            self.get_parameter("yolo_model")
            .get_parameter_value()
            .string_value
            .strip()
        ) or YOLO_MODEL_NAME

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

    def _log_startup(self):
        self.get_logger().info("노드 초기화 완료")
        self.get_logger().info(f"YOLO model: {self.yolo_model}")
        self.get_logger().info(f"grasp point mode: {self.grasp_point_mode}")
        self.get_logger().info(
            "객체 입력 예시: ros2 topic pub --once /target_label "
            "std_msgs/msg/String \"{data: cup}\""
        )
        self.get_logger().info(f"잡기 인식 결과 토픽: {HAND_GRASP_TOPIC}")
        self.get_logger().info(f"잡기 인식 화면 토픽: {HAND_GRASP_IMAGE_TOPIC}")

    def _target_label_cb(self, msg):
        val = msg.data.strip()

        if not val:
            return

        if self.picking:
            self.get_logger().warn(
                f"현재 pick 동작 중이라 새 타겟 '{val}' 입력은 무시합니다."
            )
            return

        self.target_label = val
        self.get_logger().info(f"타겟 객체 설정: {self.target_label}")

    def _hand_grasp_cb(self, msg):
        try:
            result = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"잡기 인식 결과 JSON 파싱 실패: {msg.data}")
            return

        self.last_grasp_result = result
        self.human_grasped_tool = bool(result.get("human_grasped_tool", False))

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

    def start_pick_sequence(
        self,
        bx,
        by,
        bz,
        z_m,
        vlm_yaw_deg=None,
        label=None,
        bbox=None,
    ):
        if self.picking:
            self.get_logger().warn("이미 pick 동작 중이라 새 pick 요청을 무시합니다.")
            return

        self.picking = True
        self.pending_pick_thread = threading.Thread(
            target=self.pick_runner.run,
            args=(
                bx,
                by,
                bz,
                z_m,
                vlm_yaw_deg,
                label,
                bbox,
                None if self.color_image is None else self.color_image.copy(),
                self.target_label,
            ),
            daemon=True,
        )
        self.pending_pick_thread.start()

    def run(self):
        self._move_home_and_capture_pose()
        self._process_frames()

    def _move_home_and_capture_pose(self):
        self.get_logger().info("시스템 준비 중... Home으로 이동합니다.")

        home_state = RobotState(self.robot.get_robot_model())
        home_state.joint_positions = HOME_JOINTS

        ok = plan_and_execute(
            self.robot,
            self.arm,
            self.get_logger(),
            state_goal=home_state,
        )
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
                self.debug_windows.show_robot(self.color_image)
                self.debug_windows.show_hand_grasp()
                if self.debug_windows.wait_key() == 27:
                    break
                continue

            results = self.detector.detect(self.color_image)
            annotated_frame = results[0].plot()

            if self.target_label:
                self._handle_target_detection(
                    results[0].boxes,
                    annotated_frame,
                )

            self.debug_windows.show_robot(annotated_frame)
            self.debug_windows.show_hand_grasp()

            if self.debug_windows.wait_key() == 27:
                break

        self.debug_windows.close_all()

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
            u, v, source, vlm_rpy_deg = self.grasp_selector.select_grasp_pixel(
                box,
                label,
                self.color_image,
                self.depth_image,
                self.intrinsics,
                self.target_label,
            )
            target = self.projector.pixel_to_base_target(
                self.depth_image,
                self.intrinsics,
                u,
                v,
                label,
                source,
                vlm_rpy_deg,
            )
            if target is None:
                continue

            bx, by, bz, z_m, vlm_rpy_deg = target
            vlm_yaw_deg = self._extract_vlm_yaw(vlm_rpy_deg)
            self._draw_grasp_marker(annotated_frame, u, v, source)
            self.start_pick_sequence(
                bx,
                by,
                bz,
                z_m,
                vlm_yaw_deg,
                label=label,
                bbox=box.xyxy[0].cpu().numpy(),
            )
            break

        if not found:
            self.get_logger().info(
                f"'{self.target_label}' 탐색 중... 현재 프레임에서는 미검출"
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
