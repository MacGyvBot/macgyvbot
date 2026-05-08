#!/usr/bin/env python3
import json
import math
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from ultralytics import YOLO

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge

from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, PlanRequestParameters

from .onrobot import RG
from .safety import clamp_to_safe_workspace


# ═══════════════════════════════════════════
# 설정
# ═══════════════════════════════════════════
GROUP_NAME = "manipulator"
BASE_FRAME = "base_link"
EE_LINK = "link_6"

HOME_JOINTS = {
    "joint_1": math.radians(0.0),
    "joint_2": math.radians(0.0),
    "joint_3": math.radians(90.0),
    "joint_4": math.radians(0.0),
    "joint_5": math.radians(90.0),
    "joint_6": math.radians(90.0),
}
WRIST_JOINT_NAME = "joint_6"

SAFE_Z = 0.40
APPROACH_Z_OFFSET = 0.18
GRASP_Z_OFFSET = 0.3
COLLISION_MARGIN = 0.02
MIN_GRASP_CLEARANCE = 0.02
MIN_TRAVEL_Z = 0.06
MIN_PICK_Z = 0.30
MAX_DESCENT_FROM_APPROACH = 0.08
YOLO_MODEL_NAME = "yolov11_best.pt"
HAND_GRASP_TOPIC = "/human_grasped_tool"
HAND_GRASP_IMAGE_TOPIC = "/hand_grasp_detection/annotated_image"
HAND_GRASP_TIMEOUT_SEC = 20.0
ROBOT_WINDOW_NAME = "YOLO Robot Pick"
HAND_GRASP_WINDOW_NAME = "Hand Grasp Detection"
GRASP_POINT_MODE_CENTER = "center"
GRASP_POINT_MODE_VLM = "vlm"
DEFAULT_GRASP_POINT_MODE = GRASP_POINT_MODE_CENTER
VLM_GRASP_GRID_SIZES = ((3, 3), (4, 4))


# ═══════════════════════════════════════════
# 유틸리티 함수
# ═══════════════════════════════════════════
def make_pose(x, y, z, ori):
    p = PoseStamped()
    p.header.frame_id = BASE_FRAME

    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.position.z = float(z)

    p.pose.orientation.x = ori["x"]
    p.pose.orientation.y = ori["y"]
    p.pose.orientation.z = ori["z"]
    p.pose.orientation.w = ori["w"]

    return p


def make_safe_pose(x, y, z, ori, logger):
    safe_x, safe_y, safe_z = clamp_to_safe_workspace(x, y, z, logger)
    return make_pose(safe_x, safe_y, safe_z, ori)


def plan_and_execute(robot, arm, logger, pose_goal=None, state_goal=None, params=None):
    arm.set_start_state_to_current_state()

    if pose_goal:
        x = pose_goal.pose.position.x
        y = pose_goal.pose.position.y
        z = pose_goal.pose.position.z
        sx, sy, sz = clamp_to_safe_workspace(x, y, z, logger)
        pose_goal.pose.position.x = sx
        pose_goal.pose.position.y = sy
        pose_goal.pose.position.z = sz

        arm.set_goal_state(
            pose_stamped_msg=pose_goal,
            pose_link=EE_LINK,
        )
    elif state_goal:
        arm.set_goal_state(robot_state=state_goal)

    plan_result = arm.plan(parameters=params) if params else arm.plan()

    if plan_result:
        robot.execute(
            GROUP_NAME,
            plan_result.trajectory,
            blocking=True,
        )
        return True

    logger.error("Planning 실패")
    return False


def get_ee_matrix(moveit_robot):
    psm = moveit_robot.get_planning_scene_monitor()

    with psm.read_only() as scene:
        T = scene.current_state.get_global_link_transform(EE_LINK)

    return np.asarray(T, dtype=float)


def resolve_model_path(model_name):
    package_share = Path(get_package_share_directory("macgyvbot"))
    candidates = [
        package_share / "models" / model_name,
        package_share / model_name,
        Path.cwd() / "models" / model_name,
        Path.cwd() / model_name,
    ]

    for candidate in candidates:
        if candidate.exists():
            return str(candidate)

    return model_name


# ═══════════════════════════════════════════
# 메인 노드 클래스
# ═══════════════════════════════════════════
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
        self.vlm_grasp_model = None

        self.home_xyz = None
        self.home_ori = None

        self.declare_parameter("grasp_point_mode", DEFAULT_GRASP_POINT_MODE)
        self.grasp_point_mode = (
            self.get_parameter("grasp_point_mode")
            .get_parameter_value()
            .string_value
            .strip()
            .lower()
        )
        if self.grasp_point_mode not in (
            GRASP_POINT_MODE_CENTER,
            GRASP_POINT_MODE_VLM,
        ):
            self.get_logger().warn(
                f"알 수 없는 grasp_point_mode '{self.grasp_point_mode}'. "
                f"'{GRASP_POINT_MODE_CENTER}'로 대체합니다."
            )
            self.grasp_point_mode = GRASP_POINT_MODE_CENTER

        # YOLO 모델 로드
        self.model = YOLO(resolve_model_path(YOLO_MODEL_NAME))

        # Hand-Eye 매트릭스 로드
        calib_file = (
            Path(get_package_share_directory("macgyvbot"))
            / "calibration"
            / "T_gripper2camera.npy"
        )

        self.gripper2cam = np.load(str(calib_file)).astype(float)
        self.gripper2cam[:3, 3] /= 1000.0

        # 그리퍼 및 MoveIt 설정
        self.gripper = RG("rg2", "192.168.1.1", 502)

        self.robot = MoveItPy(node_name="yolo_pick_moveit_py")
        self.arm = self.robot.get_planning_component(GROUP_NAME)

        # Planner 파라미터
        self.pilz_params = PlanRequestParameters(self.robot)
        self.pilz_params.planning_pipeline = "pilz_industrial_motion_planner"
        self.pilz_params.planner_id = "PTP"
        self.pilz_params.max_velocity_scaling_factor = 0.2

        # 카메라 구독
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

        # 객체명 입력용 토픽 구독
        self.create_subscription(
            String,
            "/target_label",
            self._target_label_cb,
            10,
        )

        # 사용자 손의 공구 잡기 인식 결과 구독
        self.create_subscription(
            String,
            HAND_GRASP_TOPIC,
            self._hand_grasp_cb,
            10,
        )

        # 사용자 손 인식 annotation 이미지 구독
        self.create_subscription(
            Image,
            HAND_GRASP_IMAGE_TOPIC,
            self._hand_grasp_image_cb,
            10,
        )

        self.get_logger().info("노드 초기화 완료")
        self.get_logger().info(f"grasp point mode: {self.grasp_point_mode}")
        self.get_logger().info(
            "객체 입력 예시: ros2 topic pub --once /target_label std_msgs/msg/String \"{data: cup}\""
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

    def transform_to_base(self, cam_xyz):
        coord = np.append(np.array(cam_xyz), 1.0)

        base2cam = get_ee_matrix(self.robot) @ self.gripper2cam
        base_xyz = (base2cam @ coord)[:3]

        return base_xyz

    def select_grasp_pixel(self, box, label):
        b = box.xyxy[0].cpu().numpy()

        if self.grasp_point_mode == GRASP_POINT_MODE_VLM:
            vlm_pixel = self.select_vlm_grasp_pixel(b, label)
            if vlm_pixel is not None:
                return vlm_pixel

            self.get_logger().warn(
                "VLM grasp point 실패. box 중심점으로 대체합니다."
            )

        return self.select_box_center_pixel(b)

    def select_box_center_pixel(self, bbox):
        u = int((bbox[0] + bbox[2]) / 2)
        v = int((bbox[1] + bbox[3]) / 2)
        return u, v, GRASP_POINT_MODE_CENTER, None

    @staticmethod
    def apply_rpy_deg_to_orientation(base_ori, rpy_deg):
        if rpy_deg is None:
            return base_ori
        roll_deg, pitch_deg, yaw_deg = rpy_deg
        base_q = [base_ori["x"], base_ori["y"], base_ori["z"], base_ori["w"]]
        base_rot = Rotation.from_quat(base_q)
        delta_rot = Rotation.from_euler("xyz", [roll_deg, pitch_deg, yaw_deg], degrees=True)
        out = (base_rot * delta_rot).as_quat()
        return {"x": float(out[0]), "y": float(out[1]), "z": float(out[2]), "w": float(out[3])}

    @staticmethod
    def normalize_angle_deg(angle_deg):
        return ((angle_deg + 180.0) % 360.0) - 180.0

    def get_current_ee_orientation(self):
        T = get_ee_matrix(self.robot)
        qx, qy, qz, qw = Rotation.from_matrix(T[:3, :3]).as_quat()
        return {
            "x": float(qx),
            "y": float(qy),
            "z": float(qz),
            "w": float(qw),
        }

    def rotate_joint6_by_yaw_deg(self, yaw_deg, logger):
        if yaw_deg is None:
            return True

        try:
            yaw_deg = float(yaw_deg)
        except (TypeError, ValueError):
            logger.warn(f"유효하지 않은 yaw 값이라 J6 회전을 생략합니다: {yaw_deg}")
            return False

        if not math.isfinite(yaw_deg):
            logger.warn(f"비정상 yaw 값이라 J6 회전을 생략합니다: {yaw_deg}")
            return False

        yaw_deg = self.normalize_angle_deg(yaw_deg)
        if abs(yaw_deg) < 0.1:
            logger.info("VLM yaw가 매우 작아 J6 회전을 생략합니다.")
            return True

        robot_model = self.robot.get_robot_model()
        jmg = robot_model.get_joint_model_group(GROUP_NAME)
        joint_names = list(jmg.active_joint_model_names)
        if WRIST_JOINT_NAME not in joint_names:
            logger.error(
                f"{GROUP_NAME} 그룹에 {WRIST_JOINT_NAME}이 없어 J6 회전을 수행할 수 없습니다. "
                f"(active joints={joint_names})"
            )
            return False

        j6_idx = joint_names.index(WRIST_JOINT_NAME)
        psm = self.robot.get_planning_scene_monitor()
        with psm.read_only() as scene:
            current_positions = np.array(
                scene.current_state.get_joint_group_positions(GROUP_NAME),
                dtype=float,
            )

        if j6_idx >= len(current_positions):
            logger.error(
                f"J6 인덱스({j6_idx})가 현재 조인트 벡터 길이({len(current_positions)})를 초과합니다."
            )
            return False

        current_j6 = float(current_positions[j6_idx])
        target_j6 = current_j6 + math.radians(yaw_deg)
        target_j6 = math.atan2(math.sin(target_j6), math.cos(target_j6))

        target_positions = np.array(current_positions, copy=True)
        target_positions[j6_idx] = target_j6

        logger.info(
            "2-1단계: VLM yaw를 J6에 적용 "
            f"(yaw_offset={yaw_deg:.2f}deg, "
            f"j6={math.degrees(current_j6):.2f}deg -> {math.degrees(target_j6):.2f}deg)"
        )

        state_goal = RobotState(robot_model)
        state_goal.set_joint_group_positions(GROUP_NAME, target_positions)
        state_goal.update()

        return plan_and_execute(
            self.robot,
            self.arm,
            logger,
            state_goal=state_goal,
            params=self.pilz_params,
        )

    def select_vlm_grasp_pixel(self, bbox, label):
        x1, y1, x2, y2 = self.clamp_bbox_to_image(bbox, self.color_image)

        if x2 <= x1 or y2 <= y1:
            self.get_logger().warn("VLM crop bbox가 비어 있습니다.")
            return None

        try:
            from PIL import Image as PILImage

            from .grasp_point_detection import VLMModel
        except ImportError as exc:
            self.get_logger().warn(f"VLM grasp 모듈 import 실패: {exc}")
            return None

        if self.vlm_grasp_model is None:
            self.get_logger().info("VLM grasp 모델을 lazy load 준비합니다.")
            self.vlm_grasp_model = VLMModel()
            runtime = self.vlm_grasp_model.get_runtime_info()
            self.get_logger().info(
                "VLM runtime: "
                f"device={runtime['device']}, "
                f"dtype={runtime['dtype']}, "
                f"local_weights={runtime['using_local_weights']}, "
                f"source={runtime['model_source']}"
            )
            if runtime["device"] == "cuda":
                self.get_logger().info("VLM은 CUDA를 사용합니다.")
            else:
                self.get_logger().warn("VLM이 CUDA를 사용하지 않습니다. (CPU 실행)")
            self.get_logger().info("VLM 가중치 로드 시작...")
            self.vlm_grasp_model.load()
            self.get_logger().info("VLM 가중치 로드 완료.")

        crop_bgr = self.color_image[y1:y2, x1:x2]
        crop_rgb = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)
        crop_image = PILImage.fromarray(crop_rgb)

        try:
            result = self.vlm_grasp_model.select_grasp_region(
                crop_image,
                object_label=label,
                user_request=self.target_label,
                grid_sizes=VLM_GRASP_GRID_SIZES,
            )
        except Exception as exc:
            self.get_logger().warn(f"VLM grasp point 추론 실패: {exc}")
            return None

        u = x1 + int(round(result.point[0]))
        v = y1 + int(round(result.point[1]))
        source = GRASP_POINT_MODE_VLM

        refined = VLMModel.refine_grasp_point_with_depth(
            self.depth_image,
            (u, v),
            focal_px=(self.intrinsics["fx"] + self.intrinsics["fy"]) / 2.0,
        )
        if refined is not None:
            u, v = refined.point
            source = f"{GRASP_POINT_MODE_VLM}+depth"

        self.get_logger().info(
            f"VLM grasp point 선택: pixel=({u}, {v}), "
            f"angle={result.angle_deg:.1f}deg, "
            f"rpy_deg={result.orientation_rpy_deg}, source={source}"
        )

        return u, v, source, result.orientation_rpy_deg

    @staticmethod
    def clamp_bbox_to_image(bbox, image):
        height, width = image.shape[:2]
        x1 = max(0, min(width - 1, int(math.floor(bbox[0]))))
        y1 = max(0, min(height - 1, int(math.floor(bbox[1]))))
        x2 = max(0, min(width, int(math.ceil(bbox[2]))))
        y2 = max(0, min(height, int(math.ceil(bbox[3]))))
        return x1, y1, x2, y2

    def pixel_to_base_target(self, u, v, label, source, vlm_rpy_deg=None):
        h, w = self.depth_image.shape[:2]

        if not (0 <= u < w and 0 <= v < h):
            self.get_logger().warn(
                f"{source} 픽셀이 depth 이미지 범위를 벗어남: u={u}, v={v}"
            )
            return None

        z_raw = self.depth_image[v, u]

        if z_raw == 0:
            self.get_logger().warn(
                f"{label} 검출됨, 하지만 {source} depth 값이 0입니다."
            )
            return None

        z_m = float(z_raw) / 1000.0
        cam_x = (u - self.intrinsics["ppx"]) * z_m / self.intrinsics["fx"]
        cam_y = (v - self.intrinsics["ppy"]) * z_m / self.intrinsics["fy"]
        bx, by, bz = self.transform_to_base((cam_x, cam_y, z_m))

        self.get_logger().info(
            f"'{label}' 검출: source={source}, "
            f"pixel=({u}, {v}), "
            f"camera=({cam_x:.3f}, {cam_y:.3f}, {z_m:.3f}), "
            f"base=({bx:.3f}, {by:.3f}, {bz:.3f})"
        )

        return bx, by, bz, z_m, vlm_rpy_deg

    def return_tool_to_original_position(self, target_x, target_y, travel_z, grasp_z, ori, logger):
        logger.info("반환 1단계: 원래 공구 위치 상단으로 이동")
        ok = plan_and_execute(
            self.robot,
            self.arm,
            logger,
            pose_goal=make_safe_pose(target_x, target_y, travel_z, ori, logger),
            params=self.pilz_params,
        )
        if not ok:
            logger.error("원래 공구 위치 상단 이동 실패. 공구를 잡은 상태로 중단합니다.")
            return False

        logger.info("반환 2단계: 원래 공구 위치로 하강")
        ok = plan_and_execute(
            self.robot,
            self.arm,
            logger,
            pose_goal=make_safe_pose(target_x, target_y, grasp_z, ori, logger),
            params=self.pilz_params,
        )
        if not ok:
            logger.error("원래 공구 위치 하강 실패. 공구를 잡은 상태로 중단합니다.")
            return False

        logger.info("반환 3단계: 원래 위치에 공구 놓기")
        self.gripper.open_gripper()
        time.sleep(0.8)

        logger.info("반환 4단계: 공구를 놓은 뒤 안전 높이로 복귀")
        ok = plan_and_execute(
            self.robot,
            self.arm,
            logger,
            pose_goal=make_safe_pose(target_x, target_y, travel_z, ori, logger),
            params=self.pilz_params,
        )
        if not ok:
            logger.error("공구를 놓은 뒤 안전 높이 복귀 실패")
            return False

        logger.info("반환 5단계: Home 위치로 복귀")
        ok = plan_and_execute(
            self.robot,
            self.arm,
            logger,
            pose_goal=make_safe_pose(
                self.home_xyz[0],
                self.home_xyz[1],
                travel_z,
                ori,
                logger,
            ),
            params=self.pilz_params,
        )
        if not ok:
            logger.error("공구 반환 후 Home 복귀 실패")
            return False

        return True

    def pick_sequence(self, bx, by, bz, z_m, vlm_yaw_deg=None):
        self.human_grasped_tool = False
        self.last_grasp_result = None

        log = self.get_logger()
        ori = self.home_ori

        # Depth 기반 안전 파지 높이 계산
        # z_m: camera->object depth (m)
        # base/object z 좌표(bz) 기준으로 너무 과도한 하강을 막고
        # 최소 여유 간격을 유지하도록 제한한다.
        safe_grasp_offset = max(
            GRASP_Z_OFFSET,
            z_m * 0.35 + MIN_GRASP_CLEARANCE,
        )
        safe_grasp_offset += COLLISION_MARGIN

        approach_z = bz + APPROACH_Z_OFFSET + COLLISION_MARGIN
        grasp_z = bz + safe_grasp_offset
        grasp_z = max(grasp_z, MIN_TRAVEL_Z, MIN_PICK_Z)
        if grasp_z > approach_z:
            grasp_z = approach_z - 0.01
        # 급격한 하강 제한: 보정 오차로 과도하게 내려가는 것을 방지
        min_safe_grasp_z = approach_z - MAX_DESCENT_FROM_APPROACH
        if grasp_z < min_safe_grasp_z:
            log.warn(
                f"grasp_z({grasp_z:.3f})가 과도하게 낮아 "
                f"하강 제한 적용: {min_safe_grasp_z:.3f}"
            )
            grasp_z = min_safe_grasp_z

        current_pose = get_ee_matrix(self.robot)
        current_x = current_pose[0, 3]
        current_y = current_pose[1, 3]

        target_x, target_y, _ = clamp_to_safe_workspace(bx, by, SAFE_Z, log)
        _, _, travel_z = clamp_to_safe_workspace(target_x, target_y, SAFE_Z, log)
        _, _, approach_z = clamp_to_safe_workspace(target_x, target_y, approach_z, log)
        _, _, grasp_z = clamp_to_safe_workspace(target_x, target_y, grasp_z, log)

        if grasp_z > approach_z:
            log.warn(
                f"안전영역 적용 후 grasp_z({grasp_z:.3f})가 "
                f"approach_z({approach_z:.3f})보다 높아 approach_z로 맞춥니다."
            )
            grasp_z = approach_z

        should_descend_to_grasp = abs(approach_z - grasp_z) > 0.005

        try:
            log.info(
                f"시퀀스 시작: Target({target_x:.3f}, {target_y:.3f}), "
                f"depth={z_m:.3f}, travel_z={travel_z:.3f}, "
                f"approach_z={approach_z:.3f}, grasp_z={grasp_z:.3f}"
            )

            # 0. 파지 전 그리퍼 오픈
            self.gripper.open_gripper()
            time.sleep(0.5)

            # 1. 안전 이동 높이 확보
            log.info("1단계: 안전 이동 높이 확보")
            ok = plan_and_execute(
                self.robot,
                self.arm,
                log,
                pose_goal=make_safe_pose(
                    current_x,
                    current_y,
                    travel_z,
                    ori,
                    log,
                ),
                params=self.pilz_params,
            )

            if not ok:
                log.error("안전 이동 높이 확보 실패. Pick 시퀀스 중단")
                return

            # 2. 안전 높이에서 XY 이동
            log.info("2단계: 안전 높이에서 XY 수평 이동")
            ok = plan_and_execute(
                self.robot,
                self.arm,
                log,
                pose_goal=make_safe_pose(target_x, target_y, travel_z, ori, log),
                params=self.pilz_params,
            )

            if not ok:
                log.error("XY 이동 실패. Pick 시퀀스 중단")
                return

            # 2-1. VLM yaw를 J6 단일 조인트 회전으로 적용
            if vlm_yaw_deg is not None:
                ok = self.rotate_joint6_by_yaw_deg(vlm_yaw_deg, log)
                if not ok:
                    log.error("J6 회전 실패. Pick 시퀀스 중단")
                    return
                # 이후 pose goal이 방금 회전한 손목 자세를 유지하도록 현재 orientation을 갱신.
                ori = self.get_current_ee_orientation()

            # 3. 타겟 상단으로 접근 (offset 적용)
            log.info("3단계: 타겟 상단 접근")
            ok = plan_and_execute(
                self.robot,
                self.arm,
                log,
                pose_goal=make_safe_pose(target_x, target_y, approach_z, ori, log),
                params=self.pilz_params,
            )

            if not ok:
                log.error("상단 접근 실패. Pick 시퀀스 중단")
                return

            # 4. 파지 높이로 하강 (offset 적용)
            if should_descend_to_grasp:
                log.info("4단계: 파지 높이 하강")
                ok = plan_and_execute(
                    self.robot,
                    self.arm,
                    log,
                    pose_goal=make_safe_pose(target_x, target_y, grasp_z, ori, log),
                    params=self.pilz_params,
                )

                if not ok:
                    log.error("파지 높이 하강 실패. Pick 시퀀스 중단")
                    return
            else:
                log.info("4단계: approach_z와 grasp_z가 같아 추가 하강 생략")

            # 5. 그리퍼 닫기
            log.info("5단계: 그리퍼 닫기")
            self.gripper.close_gripper()
            time.sleep(1.0)

            # 6. 안전 높이로 복귀
            log.info("6단계: 안전 높이 복귀")
            ok = plan_and_execute(
                self.robot,
                self.arm,
                log,
                pose_goal=make_safe_pose(target_x, target_y, travel_z, ori, log),
                params=self.pilz_params,
            )

            if not ok:
                log.error("안전 높이 복귀 실패")
                return

            # 7. Home 위치 근처로 복귀
            log.info("7단계: Home XY 복귀")
            ok = plan_and_execute(
                self.robot,
                self.arm,
                log,
                pose_goal=make_safe_pose(
                    self.home_xyz[0],
                    self.home_xyz[1],
                    travel_z,
                    ori,
                    log,
                ),
                params=self.pilz_params,
            )

            if not ok:
                log.error("Home 복귀 실패")
                return

            # 8. 사용자 손이 공구를 잡았는지 확인 후 놓기
            log.info("8단계: 사용자 잡기 인식 대기")
            if not self.wait_for_human_grasp(log):
                log.error("사용자 잡기 인식 실패. 원래 공구 위치로 반환합니다.")
                self.return_tool_to_original_position(
                    target_x,
                    target_y,
                    travel_z,
                    grasp_z,
                    ori,
                    log,
                )
                return

            log.info("9단계: 사용자 잡기 확인 후 그리퍼 오픈(놓기)")
            self.gripper.open_gripper()
            time.sleep(0.8)

            log.info("Pick 시퀀스 완료")

        finally:
            self.picking = False
            self.target_label = None
            self.human_grasped_tool = False

    def wait_for_human_grasp(self, logger):
        start_time = time.monotonic()

        while rclpy.ok():
            if self.human_grasped_tool:
                logger.info("사용자가 공구를 잡은 것으로 확인됨")
                return True

            if time.monotonic() - start_time >= HAND_GRASP_TIMEOUT_SEC:
                logger.warn(
                    f"{HAND_GRASP_TIMEOUT_SEC:.1f}초 동안 사용자 잡기 인식이 없어 대기 종료"
                )
                return False

            time.sleep(0.1)

        return False

    def start_pick_sequence(self, bx, by, bz, z_m, vlm_yaw_deg=None):
        if self.picking:
            self.get_logger().warn("이미 pick 동작 중이라 새 pick 요청을 무시합니다.")
            return

        self.picking = True
        self.pending_pick_thread = threading.Thread(
            target=self.pick_sequence,
            args=(bx, by, bz, z_m, vlm_yaw_deg),
            daemon=True,
        )
        self.pending_pick_thread.start()

    def show_hand_grasp_window(self):
        if self.hand_grasp_image is not None:
            cv2.imshow(HAND_GRASP_WINDOW_NAME, self.hand_grasp_image)

    def run(self):
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

        T = get_ee_matrix(self.robot)

        self.home_xyz = (
            T[0, 3],
            T[1, 3],
            T[2, 3],
        )

        qx, qy, qz, qw = Rotation.from_matrix(T[:3, :3]).as_quat()

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

        self.get_logger().info("카메라 영상 대기 중...")
        self.get_logger().info(
            "다른 터미널에서 객체명을 publish 하세요. 예: "
            "ros2 topic pub --once /target_label std_msgs/msg/String \"{data: cup}\""
        )

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.color_image is None:
                continue

            if self.depth_image is None:
                continue

            if self.intrinsics is None:
                continue

            if self.picking:
                cv2.imshow(ROBOT_WINDOW_NAME, self.color_image)
                self.show_hand_grasp_window()
                if cv2.waitKey(1) == 27:
                    break
                continue

            # YOLO 추론
            results = self.model(self.color_image, verbose=False)
            annotated_frame = results[0].plot()

            if self.target_label:
                found = False

                for box in results[0].boxes:
                    label = self.model.names[int(box.cls)]

                    if label != self.target_label:
                        continue

                    found = True

                    u, v, source, vlm_rpy_deg = self.select_grasp_pixel(box, label)
                    target = self.pixel_to_base_target(u, v, label, source, vlm_rpy_deg)

                    if target is None:
                        continue

                    bx, by, bz, z_m, vlm_rpy_deg = target
                    vlm_yaw_deg = None
                    if vlm_rpy_deg is not None and len(vlm_rpy_deg) >= 3:
                        try:
                            vlm_yaw_deg = float(vlm_rpy_deg[2])
                        except (TypeError, ValueError):
                            self.get_logger().warn(
                                f"VLM yaw 파싱 실패: {vlm_rpy_deg}"
                            )

                    cv2.circle(annotated_frame, (u, v), 6, (0, 255, 255), -1)
                    cv2.putText(
                        annotated_frame,
                        source,
                        (u + 8, v - 8),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.45,
                        (0, 255, 255),
                        1,
                        cv2.LINE_AA,
                    )

                    self.start_pick_sequence(bx, by, bz, z_m, vlm_yaw_deg)
                    break

                if not found:
                    self.get_logger().info(
                        f"'{self.target_label}' 탐색 중... 현재 프레임에서는 미검출"
                    )

            cv2.imshow(ROBOT_WINDOW_NAME, annotated_frame)
            self.show_hand_grasp_window()

            if cv2.waitKey(1) == 27:
                break

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
