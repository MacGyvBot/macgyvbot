"""GraspNet pose buffering and frame conversion helpers."""

import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation

from macgyvbot.core.config import BASE_FRAME, GRASP_POINT_MODE_GRASPNET
from macgyvbot.motion.pose_utils import get_ee_matrix
from macgyvbot.perception.graspnet_pose import GraspNetPoseBuffer


class GraspNetPoseSelector:
    def __init__(
        self,
        moveit_robot,
        gripper2cam,
        pose_topic,
        pose_timeout_sec,
        wait_timeout_sec,
        target_distance_tolerance_m,
        use_orientation,
    ):
        self.moveit_robot = moveit_robot
        self.gripper2cam = gripper2cam
        self.pose_topic = pose_topic
        self.wait_timeout_sec = wait_timeout_sec
        self.pose_buffer = GraspNetPoseBuffer(
            base_frame=BASE_FRAME,
            timeout_sec=pose_timeout_sec,
            target_distance_tolerance_m=target_distance_tolerance_m,
            use_orientation=use_orientation,
            use_position=False,
        )

    def update_pose(self, msg, logger):
        pose_msg = self.transform_pose_to_base(msg, logger)
        if pose_msg is None:
            return

        self.pose_buffer.update(pose_msg, logger)

    def transform_pose_to_base(self, msg, logger):
        frame_id = msg.header.frame_id
        if not frame_id or frame_id == BASE_FRAME:
            return msg

        try:
            cam2grasp = np.eye(4, dtype=float)
            cam2grasp[:3, 3] = [
                float(msg.pose.position.x),
                float(msg.pose.position.y),
                float(msg.pose.position.z),
            ]
            quat = [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ]
            cam2grasp[:3, :3] = Rotation.from_quat(quat).as_matrix()
            base2cam = get_ee_matrix(self.moveit_robot) @ self.gripper2cam
            base2grasp = base2cam @ cam2grasp
            qx, qy, qz, qw = Rotation.from_matrix(
                base2grasp[:3, :3]
            ).as_quat()

            pose_msg = PoseStamped()
            pose_msg.header = msg.header
            pose_msg.header.frame_id = BASE_FRAME
            pose_msg.pose.position.x = float(base2grasp[0, 3])
            pose_msg.pose.position.y = float(base2grasp[1, 3])
            pose_msg.pose.position.z = float(base2grasp[2, 3])
            pose_msg.pose.orientation.x = float(qx)
            pose_msg.pose.orientation.y = float(qy)
            pose_msg.pose.orientation.z = float(qz)
            pose_msg.pose.orientation.w = float(qw)
            return pose_msg
        except Exception as exc:
            logger.warn(f"GraspNet pose base 변환 실패: {exc}")
            return None

    def wait_for_pose_if_needed(self, mode, logger):
        if mode != GRASP_POINT_MODE_GRASPNET:
            return

        if self.pose_buffer.has_fresh_pose():
            return

        start_time = time.monotonic()
        logger.info(
            f"GraspNet pose 대기 중... 최대 {self.wait_timeout_sec:.1f}초"
        )
        while rclpy.ok():
            if self.pose_buffer.has_fresh_pose():
                logger.info("GraspNet pose 수신 완료")
                return

            if time.monotonic() - start_time >= self.wait_timeout_sec:
                logger.warn(
                    "GraspNet pose 대기 시간 초과. "
                    "Home orientation으로 진행합니다."
                )
                return

            time.sleep(0.05)

    def select(self, yolo_xyz, fallback_orientation, logger):
        return self.pose_buffer.select(yolo_xyz, fallback_orientation, logger)
