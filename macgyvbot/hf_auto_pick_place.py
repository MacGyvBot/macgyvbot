#!/usr/bin/env python3
import math
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

SAFE_Z = 0.40
APPROACH_Z_OFFSET = 0.18
GRASP_Z_OFFSET = 0.12
COLLISION_MARGIN = 0.02
MIN_GRASP_CLEARANCE = 0.02
MIN_TRAVEL_Z = 0.06
MIN_PICK_Z = 0.30
MAX_DESCENT_FROM_APPROACH = 0.08


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


def plan_and_execute(robot, arm, logger, pose_goal=None, state_goal=None, params=None):
    arm.set_start_state_to_current_state()

    if pose_goal:
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


# ═══════════════════════════════════════════
# 메인 노드 클래스
# ═══════════════════════════════════════════
class ClickPickNode(Node):
    def __init__(self):
        super().__init__("yolo_pick_moveit_node")

        self.bridge = CvBridge()

        self.color_image = None
        self.depth_image = None
        self.intrinsics = None

        self.picking = False
        self.target_label = None

        self.home_xyz = None
        self.home_ori = None

        # YOLO 모델 로드
        self.model = YOLO("yolov8n.pt")

        # Hand-Eye 매트릭스 로드
        calib_file = (
            Path(get_package_share_directory("dsr_practice"))
            / "config"
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

        self.get_logger().info("노드 초기화 완료")
        self.get_logger().info(
            "객체 입력 예시: ros2 topic pub --once /target_label std_msgs/msg/String \"{data: cup}\""
        )

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

    def pick_sequence(self, bx, by, bz, z_m):
        self.picking = True

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

        cur_z = get_ee_matrix(self.robot)[2, 3]

        try:
            log.info(
                f"시퀀스 시작: Target({bx:.3f}, {by:.3f}), "
                f"depth={z_m:.3f}, approach_z={approach_z:.3f}, grasp_z={grasp_z:.3f}"
            )

            # 0. 파지 전 그리퍼 오픈
            self.gripper.open_gripper()
            time.sleep(0.5)

            # 1. XY 우선 이동
            log.info("1단계: XY 수평 이동")
            ok = plan_and_execute(
                self.robot,
                self.arm,
                log,
                pose_goal=make_pose(bx, by, cur_z, ori),
                params=self.pilz_params,
            )

            if not ok:
                log.error("XY 이동 실패. Pick 시퀀스 중단")
                return

            # 2. 타겟 상단으로 접근 (offset 적용)
            log.info("2단계: 타겟 상단 접근")
            ok = plan_and_execute(
                self.robot,
                self.arm,
                log,
                pose_goal=make_pose(bx, by, approach_z, ori),
                params=self.pilz_params,
            )

            if not ok:
                log.error("상단 접근 실패. Pick 시퀀스 중단")
                return

            # 3. 파지 높이로 하강 (offset 적용)
            log.info("3단계: 파지 높이 하강")
            ok = plan_and_execute(
                self.robot,
                self.arm,
                log,
                pose_goal=make_pose(bx, by, grasp_z, ori),
                params=self.pilz_params,
            )

            if not ok:
                log.error("파지 높이 하강 실패. Pick 시퀀스 중단")
                return

            # 4. 그리퍼 닫기
            log.info("4단계: 그리퍼 닫기")
            self.gripper.close_gripper()
            time.sleep(1.0)

            # 5. 안전 높이로 복귀
            log.info("5단계: 안전 높이 복귀")
            ok = plan_and_execute(
                self.robot,
                self.arm,
                log,
                pose_goal=make_pose(bx, by, SAFE_Z, ori),
                params=self.pilz_params,
            )

            if not ok:
                log.error("안전 높이 복귀 실패")
                return

            # 6. Home 위치 근처로 복귀
            log.info("6단계: Home XY 복귀")
            ok = plan_and_execute(
                self.robot,
                self.arm,
                log,
                pose_goal=make_pose(self.home_xyz[0], self.home_xyz[1], SAFE_Z, ori),
                params=self.pilz_params,
            )

            if not ok:
                log.error("Home 복귀 실패")
                return

            # 7. 놓기 (Home 위치에서 그리퍼 오픈)
            log.info("7단계: Home 위치에서 그리퍼 오픈(놓기)")
            self.gripper.open_gripper()
            time.sleep(0.8)

            log.info("Pick 시퀀스 완료")

        finally:
            self.picking = False
            self.target_label = None

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

                    b = box.xyxy[0].cpu().numpy()

                    u = int((b[0] + b[2]) / 2)
                    v = int((b[1] + b[3]) / 2)

                    h, w = self.depth_image.shape[:2]

                    if not (0 <= u < w and 0 <= v < h):
                        self.get_logger().warn(
                            f"검출 중심점이 depth 이미지 범위를 벗어남: u={u}, v={v}"
                        )
                        continue

                    z_raw = self.depth_image[v, u]

                    if z_raw == 0:
                        self.get_logger().warn(
                            f"{self.target_label} 검출됨, 하지만 depth 값이 0입니다."
                        )
                        continue

                    z_m = float(z_raw) / 1000.0

                    cam_x = (
                        (u - self.intrinsics["ppx"])
                        * z_m
                        / self.intrinsics["fx"]
                    )
                    cam_y = (
                        (v - self.intrinsics["ppy"])
                        * z_m
                        / self.intrinsics["fy"]
                    )

                    bx, by, bz = self.transform_to_base((cam_x, cam_y, z_m))

                    self.get_logger().info(
                        f"'{self.target_label}' 검출: "
                        f"pixel=({u}, {v}), "
                        f"camera=({cam_x:.3f}, {cam_y:.3f}, {z_m:.3f}), "
                        f"base=({bx:.3f}, {by:.3f}, {bz:.3f})"
                    )

                    self.pick_sequence(bx, by, bz, z_m)
                    break

                if not found:
                    self.get_logger().info(
                        f"'{self.target_label}' 탐색 중... 현재 프레임에서는 미검출"
                    )

            cv2.imshow("YOLO Robot Pick", annotated_frame)

            if cv2.waitKey(1) == 27:
                break

        cv2.destroyAllWindows()


def main():
    rclpy.init()

    node = ClickPickNode()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
