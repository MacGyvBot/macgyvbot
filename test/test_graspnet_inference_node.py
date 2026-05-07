from geometry_msgs.msg import PoseStamped
import numpy as np
from sensor_msgs.msg import CameraInfo

from macgyvbot.graspnet_inference_node import (
    build_point_cloud,
    depth_image_to_meters,
    make_mock_pose,
    make_pose_from_grasp,
    normalize_quaternion,
)


def test_graspnet_inference_node_importable():
    import macgyvbot.graspnet_inference_node as node_module

    assert node_module.GraspNetInferenceNode is not None


def test_normalize_quaternion():
    qx, qy, qz, qw = normalize_quaternion(0.0, 2.0, 0.0, 0.0)

    assert qx == 0.0
    assert qy == 1.0
    assert qz == 0.0
    assert qw == 0.0


def test_make_mock_pose_returns_pose_stamped():
    stamp = PoseStamped().header.stamp
    stamp.sec = 1
    stamp.nanosec = 2

    pose = make_mock_pose(
        stamp=stamp,
        frame_id="base_link",
        x=0.45,
        y=0.0,
        z=0.20,
    )

    assert isinstance(pose, PoseStamped)
    assert pose.header.stamp.sec == 1
    assert pose.header.stamp.nanosec == 2
    assert pose.header.frame_id == "base_link"
    assert pose.pose.position.x == 0.45
    assert pose.pose.position.y == 0.0
    assert pose.pose.position.z == 0.20
    assert pose.pose.orientation.x == 0.0
    assert pose.pose.orientation.y == 1.0
    assert pose.pose.orientation.z == 0.0
    assert pose.pose.orientation.w == 0.0


def test_make_mock_pose_uses_frame_id_parameter():
    stamp = PoseStamped().header.stamp
    pose = make_mock_pose(
        stamp=stamp,
        frame_id="camera_color_optical_frame",
        x=0.1,
        y=0.2,
        z=0.3,
    )

    assert pose.header.frame_id == "camera_color_optical_frame"


def test_depth_image_to_meters_handles_uint16_mm():
    depth = np.array([[1000, 250]], dtype=np.uint16)

    depth_m = depth_image_to_meters(depth, "16UC1")

    np.testing.assert_allclose(depth_m, np.array([[1.0, 0.25]], dtype=np.float32))


def test_build_point_cloud_uses_camera_intrinsics():
    depth_m = np.array([[1.0, 1.0]], dtype=np.float32)
    camera_info = CameraInfo()
    camera_info.k = [2.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 1.0]

    cloud = build_point_cloud(depth_m, camera_info)

    assert cloud.shape == (1, 2, 3)
    np.testing.assert_allclose(cloud[0, 0], [0.0, 0.0, 1.0])
    np.testing.assert_allclose(cloud[0, 1], [0.5, 0.0, 1.0])


def test_make_pose_from_grasp_converts_rotation_to_quaternion():
    stamp = PoseStamped().header.stamp
    pose = make_pose_from_grasp(
        stamp=stamp,
        frame_id="camera_color_optical_frame",
        translation=np.array([0.1, 0.2, 0.3]),
        rotation_matrix=np.eye(3),
    )

    assert pose.header.frame_id == "camera_color_optical_frame"
    assert pose.pose.position.x == 0.1
    assert pose.pose.position.y == 0.2
    assert pose.pose.position.z == 0.3
    assert pose.pose.orientation.x == 0.0
    assert pose.pose.orientation.y == 0.0
    assert pose.pose.orientation.z == 0.0
    assert pose.pose.orientation.w == 1.0
