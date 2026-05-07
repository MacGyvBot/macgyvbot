from __future__ import annotations

import time

import numpy as np


def pose_orientation_to_dict(pose):
    return {
        "x": float(pose.orientation.x),
        "y": float(pose.orientation.y),
        "z": float(pose.orientation.z),
        "w": float(pose.orientation.w),
    }


def normalize_quaternion(ori):
    quat = np.array([ori["x"], ori["y"], ori["z"], ori["w"]], dtype=float)
    norm = np.linalg.norm(quat)

    if not np.isfinite(norm) or norm < 1e-6:
        return None

    quat /= norm
    return {
        "x": float(quat[0]),
        "y": float(quat[1]),
        "z": float(quat[2]),
        "w": float(quat[3]),
    }


class GraspNetPoseBuffer:
    def __init__(
        self,
        base_frame,
        timeout_sec,
        target_distance_tolerance_m,
        use_orientation,
        use_position,
    ):
        self.base_frame = base_frame
        self.timeout_sec = timeout_sec
        self.target_distance_tolerance_m = target_distance_tolerance_m
        self.use_orientation = use_orientation
        self.use_position = use_position
        self.last_pose = None
        self.last_pose_time = None

    def update(self, msg, logger):
        if msg.header.frame_id and msg.header.frame_id != self.base_frame:
            logger.warn(
                f"GraspNet pose frame이 {msg.header.frame_id}입니다. "
                f"현재는 {self.base_frame} frame pose만 사용합니다."
            )
            return

        ori = normalize_quaternion(pose_orientation_to_dict(msg.pose))
        if ori is None:
            logger.warn("GraspNet pose quaternion이 유효하지 않아 무시합니다.")
            return

        self.last_pose = msg
        self.last_pose_time = time.monotonic()

    def has_fresh_pose(self):
        if self.last_pose is None or self.last_pose_time is None:
            return False

        return time.monotonic() - self.last_pose_time <= self.timeout_sec

    def select(self, yolo_xyz, fallback_orientation, logger):
        target_xyz = np.array(yolo_xyz, dtype=float)

        if not self.use_orientation:
            return target_xyz, fallback_orientation, False

        if self.last_pose is None or self.last_pose_time is None:
            logger.warn("GraspNet pose가 없어 Home orientation으로 pick합니다.")
            return target_xyz, fallback_orientation, False

        pose_age = time.monotonic() - self.last_pose_time
        if pose_age > self.timeout_sec:
            logger.warn(
                f"GraspNet pose가 오래됨({pose_age:.2f}s). "
                "Home orientation으로 pick합니다."
            )
            return target_xyz, fallback_orientation, False

        graspnet_pose = self.last_pose.pose
        graspnet_xyz = np.array(
            [
                graspnet_pose.position.x,
                graspnet_pose.position.y,
                graspnet_pose.position.z,
            ],
            dtype=float,
        )
        distance_to_yolo = float(np.linalg.norm(graspnet_xyz - target_xyz))

        if distance_to_yolo > self.target_distance_tolerance_m:
            logger.warn(
                f"GraspNet pose와 YOLO 타겟 거리 차이가 큼: "
                f"{distance_to_yolo:.3f}m. Home orientation으로 pick합니다."
            )
            return target_xyz, fallback_orientation, False

        graspnet_ori = normalize_quaternion(pose_orientation_to_dict(graspnet_pose))
        if graspnet_ori is None:
            logger.warn("GraspNet orientation이 유효하지 않아 Home orientation으로 pick합니다.")
            return target_xyz, fallback_orientation, False

        if self.use_position:
            target_xyz = graspnet_xyz

        logger.info(
            "GraspNet pose 적용: "
            f"position_used={self.use_position}, "
            f"distance_to_yolo={distance_to_yolo:.3f}m, "
            f"quat=({graspnet_ori['x']:.3f}, {graspnet_ori['y']:.3f}, "
            f"{graspnet_ori['z']:.3f}, {graspnet_ori['w']:.3f})"
        )
        return target_xyz, graspnet_ori, True
