#!/usr/bin/env python3
"""Click RGB pixels and print their camera/base coordinates."""

from __future__ import annotations

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from moveit.planning import MoveItPy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from macgyvbot_config.robot import GROUP_NAME
from macgyvbot_config.topics import (
    CAMERA_COLOR_TOPIC,
    CAMERA_DEPTH_TOPIC,
    CAMERA_INFO_TOPIC,
)
from macgyvbot_manipulation.robot_pose import get_ee_matrix
from macgyvbot_perception.depth_projection import (
    pixel_to_camera_point,
    transform_point_to_base,
)
from macgyvbot_resources.calibration import resolve_calibration_file


WINDOW_NAME = "Click Base Point"
CALIBRATION_FILE = "T_gripper2camera.npy"


class ClickBasePointNode(Node):
    """Subscribe to camera streams and print clicked point coordinates."""

    def __init__(self):
        super().__init__("click_base_point")
        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None
        self.intrinsics = None
        self.last_click = None

        self.robot = MoveItPy(node_name="click_base_point_moveit_py")
        self.robot.get_planning_component(GROUP_NAME)
        self.gripper2cam = self._load_gripper_to_camera()

        self.create_subscription(
            Image,
            CAMERA_COLOR_TOPIC,
            self._color_cb,
            10,
        )
        self.create_subscription(
            Image,
            CAMERA_DEPTH_TOPIC,
            self._depth_cb,
            10,
        )
        self.create_subscription(
            CameraInfo,
            CAMERA_INFO_TOPIC,
            self._cam_info_cb,
            10,
        )

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(WINDOW_NAME, self._mouse_cb)
        self.get_logger().info(
            "Click a point in the RGB window to print camera/base coordinates. "
            "Press ESC to exit."
        )

    def spin_ui_once(self):
        if self.color_image is None:
            return True

        frame = self.color_image.copy()
        if self.last_click is not None:
            u, v = self.last_click
            cv2.circle(frame, (u, v), 6, (0, 255, 255), -1)
            cv2.putText(
                frame,
                f"({u}, {v})",
                (u + 8, v - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                1,
                cv2.LINE_AA,
            )

        cv2.imshow(WINDOW_NAME, frame)
        return cv2.waitKey(1) != 27

    def _mouse_cb(self, event, x, y, flags, userdata):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        self.last_click = (int(x), int(y))
        self._print_clicked_point(int(x), int(y))

    def _print_clicked_point(self, u, v):
        if self.depth_image is None or self.intrinsics is None:
            self.get_logger().warn("Depth image or camera info is not ready yet.")
            return

        camera_point = pixel_to_camera_point(
            u,
            v,
            self.depth_image,
            self.intrinsics,
            logger=self.get_logger(),
            source="clicked_pixel",
        )
        if camera_point is None:
            return

        base_to_camera = self._base_to_camera_matrix()
        base_point = transform_point_to_base(camera_point, base_to_camera)
        cam_x, cam_y, cam_z = camera_point
        bx, by, bz = [float(value) for value in base_point]
        self.get_logger().info(
            "clicked point: "
            f"pixel=({u}, {v}), "
            f"camera=({cam_x:.4f}, {cam_y:.4f}, {cam_z:.4f}) m, "
            f"base=({bx:.4f}, {by:.4f}, {bz:.4f}) m"
        )

    def _base_to_camera_matrix(self):
        return get_ee_matrix(self.robot) @ self.gripper2cam

    def _color_cb(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def _depth_cb(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def _cam_info_cb(self, msg):
        self.intrinsics = {
            "fx": msg.k[0],
            "fy": msg.k[4],
            "ppx": msg.k[2],
            "ppy": msg.k[5],
        }

    def _load_gripper_to_camera(self):
        calib_file = resolve_calibration_file(CALIBRATION_FILE)
        transform = np.load(str(calib_file)).astype(float)
        transform[:3, 3] /= 1000.0
        self.get_logger().info(f"Loaded calibration: {calib_file}")
        return transform


def main():
    rclpy.init()
    node = ClickBasePointNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            if not node.spin_ui_once():
                break
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
