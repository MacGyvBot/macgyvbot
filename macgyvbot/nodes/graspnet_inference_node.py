#!/usr/bin/env python3
from __future__ import annotations

import math
import os
import sys
from typing import Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from scipy.spatial.transform import Rotation
from std_msgs.msg import String


def normalize_quaternion(
    x: float,
    y: float,
    z: float,
    w: float,
) -> tuple[float, float, float, float]:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm <= 1e-9:
        return 0.0, 0.0, 0.0, 1.0

    return x / norm, y / norm, z / norm, w / norm


def depth_image_to_meters(
    depth_image: np.ndarray,
    encoding: str,
) -> np.ndarray:
    depth = np.asarray(depth_image)
    encoding = (encoding or "").lower()

    if encoding in ("16uc1", "mono16") or depth.dtype == np.uint16:
        return depth.astype(np.float32) / 1000.0

    return depth.astype(np.float32)


def build_point_cloud(
    depth_m: np.ndarray,
    camera_info: CameraInfo,
) -> np.ndarray:
    height, width = depth_m.shape[:2]
    xs, ys = np.meshgrid(np.arange(width), np.arange(height))
    fx = float(camera_info.k[0])
    fy = float(camera_info.k[4])
    cx = float(camera_info.k[2])
    cy = float(camera_info.k[5])

    z = depth_m
    x = (xs.astype(np.float32) - cx) * z / fx
    y = (ys.astype(np.float32) - cy) * z / fy
    return np.stack((x, y, z), axis=-1)


def make_pose_from_grasp(
    *,
    stamp,
    frame_id: str,
    translation,
    rotation_matrix,
) -> PoseStamped:
    qx, qy, qz, qw = Rotation.from_matrix(rotation_matrix).as_quat()
    qx, qy, qz, qw = normalize_quaternion(qx, qy, qz, qw)

    pose = PoseStamped()
    pose.header.stamp = stamp
    pose.header.frame_id = frame_id
    pose.pose.position.x = float(translation[0])
    pose.pose.position.y = float(translation[1])
    pose.pose.position.z = float(translation[2])
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


class GraspNetInferenceNode(Node):
    def __init__(self):
        super().__init__("graspnet_inference_node")
        self.bridge = CvBridge()

        self.checkpoint_path = str(
            self.declare_parameter("checkpoint_path", "").value
        )
        self.publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", 5.0).value
        )
        self.graspnet_baseline_path = str(
            self.declare_parameter("graspnet_baseline_path", "").value
        )
        self.num_point = int(self.declare_parameter("num_point", 20000).value)
        self.num_view = int(self.declare_parameter("num_view", 300).value)
        self.collision_thresh = float(
            self.declare_parameter("collision_thresh", -1.0).value
        )
        self.voxel_size = float(
            self.declare_parameter("voxel_size", 0.01).value
        )
        self.device_name = str(self.declare_parameter("device", "auto").value)
        self.color_topic = str(
            self.declare_parameter(
                "color_topic",
                "/camera/camera/color/image_raw",
            ).value
        )
        self.depth_topic = str(
            self.declare_parameter(
                "depth_topic",
                "/camera/camera/aligned_depth_to_color/image_raw",
            ).value
        )
        self.camera_info_topic = str(
            self.declare_parameter(
                "camera_info_topic",
                "/camera/camera/color/camera_info",
            ).value
        )
        self.target_label_topic = str(
            self.declare_parameter("target_label_topic", "/target_label").value
        )
        self.pose_topic = str(
            self.declare_parameter("pose_topic", "/graspnet/target_pose").value
        )
        self.camera_frame = str(
            self.declare_parameter(
                "camera_frame",
                "camera_color_optical_frame",
            ).value
        )
        self.min_depth_m = float(
            self.declare_parameter("min_depth_m", 0.15).value
        )
        self.max_depth_m = float(
            self.declare_parameter("max_depth_m", 1.20).value
        )

        self.latest_color_msg: Optional[Image] = None
        self.latest_depth_msg: Optional[Image] = None
        self.latest_camera_info_msg: Optional[CameraInfo] = None
        self.target_label = ""
        self.model = None
        self.torch = None
        self.GraspNet = None
        self.GraspGroup = None
        self.ModelFreeCollisionDetector = None
        self.pred_decode = None
        self.device = None

        self.create_subscription(Image, self.color_topic, self._color_cb, 10)
        self.create_subscription(Image, self.depth_topic, self._depth_cb, 10)
        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._camera_info_cb,
            10,
        )
        self.create_subscription(
            String,
            self.target_label_topic,
            self._target_label_cb,
            10,
        )
        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)

        timer_period = 1.0 / max(self.publish_rate_hz, 1e-6)
        self.create_timer(timer_period, self._timer_cb)

        self.model = self._load_graspnet_model()

        self.get_logger().info(
            f"GraspNet inference node started: pose_topic={self.pose_topic}, "
            f"frame={self.camera_frame}"
        )

    def _color_cb(self, msg: Image) -> None:
        self.latest_color_msg = msg

    def _depth_cb(self, msg: Image) -> None:
        self.latest_depth_msg = msg

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        self.latest_camera_info_msg = msg

    def _target_label_cb(self, msg: String) -> None:
        self.target_label = msg.data.strip()

    def _timer_cb(self) -> None:
        pose = self.predict_grasp_pose(
            self.latest_color_msg,
            self.latest_depth_msg,
            self.latest_camera_info_msg,
            self.target_label,
        )

        if pose is not None:
            self.pose_pub.publish(pose)

    def _load_graspnet_model(self):
        if not self.checkpoint_path:
            self.get_logger().error(
                "checkpoint_path is empty. Provide a GraspNet checkpoint_path."
            )
            return None

        if not self.graspnet_baseline_path:
            self.get_logger().error(
                "graspnet_baseline_path is empty. Clone graspnet-baseline and "
                "pass graspnet_baseline_path:=/path/to/graspnet-baseline."
            )
            return None

        baseline_path = os.path.abspath(
            os.path.expanduser(self.graspnet_baseline_path)
        )
        if not os.path.isdir(baseline_path):
            self.get_logger().error(
                f"graspnet_baseline_path not found: {baseline_path}"
            )
            return None

        checkpoint_path = os.path.abspath(
            os.path.expanduser(self.checkpoint_path)
        )
        if not os.path.isfile(checkpoint_path):
            self.get_logger().error(
                f"checkpoint_path not found: {checkpoint_path}"
            )
            return None

        for relative_path in reversed(
            ("models", "dataset", "utils", "pointnet2", "knn")
        ):
            path = os.path.join(baseline_path, relative_path)
            if os.path.isdir(path) and path not in sys.path:
                sys.path.insert(0, path)

        try:
            import torch
            from collision_detector import ModelFreeCollisionDetector
            from graspnet import GraspNet, pred_decode
            from graspnetAPI import GraspGroup
        except ImportError as exc:
            self.get_logger().error(
                f"Failed to import GraspNet dependencies: {exc}"
            )
            return None

        self.torch = torch
        self.GraspNet = GraspNet
        self.GraspGroup = GraspGroup
        self.ModelFreeCollisionDetector = ModelFreeCollisionDetector
        self.pred_decode = pred_decode
        self.device = self._select_device(torch)

        try:
            net = GraspNet(
                input_feature_dim=0,
                num_view=self.num_view,
                num_angle=12,
                num_depth=4,
                cylinder_radius=0.05,
                hmin=-0.02,
                hmax_list=[0.01, 0.02, 0.03, 0.04],
                is_training=False,
            )
            net.to(self.device)
            try:
                checkpoint = torch.load(
                    checkpoint_path,
                    map_location=self.device,
                    weights_only=False,
                )
            except TypeError:
                checkpoint = torch.load(
                    checkpoint_path,
                    map_location=self.device,
                )
            net.load_state_dict(checkpoint["model_state_dict"])
            net.eval()
        except Exception as exc:
            self.get_logger().error(
                f"Failed to load GraspNet checkpoint: {exc}"
            )
            return None

        epoch = checkpoint.get("epoch", "unknown")
        self.get_logger().info(
            f"Loaded GraspNet checkpoint: {checkpoint_path}, epoch={epoch}, "
            f"device={self.device}"
        )
        return net

    def _select_device(self, torch):
        if self.device_name.startswith("cuda"):
            if torch.cuda.is_available():
                return torch.device(self.device_name)

            self.get_logger().warn(
                "CUDA was requested but is not available. Using CPU."
            )
            return torch.device("cpu")

        if self.device_name == "cpu":
            return torch.device("cpu")

        return torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

    def predict_grasp_pose(
        self,
        color_msg: Optional[Image],
        depth_msg: Optional[Image],
        camera_info_msg: Optional[CameraInfo],
        target_label: str,
    ) -> Optional[PoseStamped]:
        """
        Returns:
            geometry_msgs.msg.PoseStamped or None
        """
        if self.model is None:
            self.get_logger().warn(
                "GraspNet model is not available. "
                "No grasp pose will be published.",
                throttle_duration_sec=5.0,
            )
            return None

        if color_msg is None or depth_msg is None or camera_info_msg is None:
            self.get_logger().warn(
                "Waiting for color, depth, and camera_info before "
                "GraspNet inference.",
                throttle_duration_sec=5.0,
            )
            return None

        try:
            color = self.bridge.imgmsg_to_cv2(
                color_msg,
                desired_encoding="rgb8",
            )
            depth = self.bridge.imgmsg_to_cv2(
                depth_msg,
                desired_encoding="passthrough",
            )
            end_points, cloud_points = self._prepare_graspnet_input(
                color,
                depth,
                depth_msg.encoding,
                camera_info_msg,
            )
            grasp_group = self._run_graspnet(end_points, cloud_points)
            grasp = self._select_best_grasp(grasp_group, target_label)
        except Exception as exc:
            self.get_logger().warn(
                f"GraspNet inference failed: {exc}",
                throttle_duration_sec=5.0,
            )
            return None

        if grasp is None:
            self.get_logger().warn(
                "GraspNet produced no valid grasp candidates.",
                throttle_duration_sec=5.0,
            )
            return None

        return make_pose_from_grasp(
            stamp=self.get_clock().now().to_msg(),
            frame_id=self.camera_frame,
            translation=grasp.translation,
            rotation_matrix=grasp.rotation_matrix,
        )

    def _prepare_graspnet_input(
        self,
        color: np.ndarray,
        depth,
        depth_encoding: str,
        camera_info_msg: CameraInfo,
    ):
        depth_m = depth_image_to_meters(depth, depth_encoding)
        cloud = build_point_cloud(depth_m, camera_info_msg)
        mask = np.isfinite(depth_m) & (depth_m >= self.min_depth_m)
        mask &= depth_m <= self.max_depth_m

        cloud_masked = cloud[mask]
        color_masked = color.astype(np.float32)[mask] / 255.0
        if len(cloud_masked) == 0:
            raise RuntimeError(
                "no valid depth points in configured depth range"
            )

        sample_count = max(self.num_point, 1)
        if len(cloud_masked) >= sample_count:
            idxs = np.random.choice(
                len(cloud_masked),
                sample_count,
                replace=False,
            )
        else:
            idxs1 = np.arange(len(cloud_masked))
            idxs2 = np.random.choice(
                len(cloud_masked),
                sample_count - len(cloud_masked),
                replace=True,
            )
            idxs = np.concatenate([idxs1, idxs2], axis=0)

        cloud_sampled = cloud_masked[idxs].astype(np.float32)
        cloud_tensor = self.torch.from_numpy(cloud_sampled[np.newaxis])
        cloud_tensor = cloud_tensor.to(self.device)

        end_points = {
            "point_clouds": cloud_tensor,
            "cloud_colors": color_masked[idxs],
        }
        return end_points, cloud_masked.astype(np.float32)

    def _run_graspnet(self, end_points, cloud_points: np.ndarray):
        with self.torch.no_grad():
            end_points = self.model(end_points)
            grasp_preds = self.pred_decode(end_points)

        gg_array = grasp_preds[0].detach().cpu().numpy()
        grasp_group = self.GraspGroup(gg_array)

        if self.collision_thresh > 0:
            detector = self.ModelFreeCollisionDetector(
                cloud_points,
                voxel_size=self.voxel_size,
            )
            collision_mask = detector.detect(
                grasp_group,
                approach_dist=0.05,
                collision_thresh=self.collision_thresh,
            )
            grasp_group = grasp_group[~collision_mask]

        return grasp_group

    def _select_best_grasp(self, grasp_group, target_label: str):
        if len(grasp_group) == 0:
            return None

        grasp_group = grasp_group.nms()
        grasp_group.sort_by_score()
        grasp = grasp_group[0]
        label_text = (
            f" for target_label={target_label}" if target_label else ""
        )
        self.get_logger().info(
            f"Selected GraspNet grasp{label_text}: score={grasp.score:.3f}, "
            f"width={grasp.width:.3f}, "
            f"translation={grasp.translation.tolist()}"
        )
        return grasp


def main():
    rclpy.init()
    node = GraspNetInferenceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
