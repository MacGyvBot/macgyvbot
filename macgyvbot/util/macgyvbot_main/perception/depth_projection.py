"""Depth image projection helpers."""

from __future__ import annotations

import numpy as np

from macgyvbot.util.macgyvbot_main.model_control.robot_pose import get_ee_matrix


class DepthProjector:
    """Project image-space grasp pixels into the robot base frame."""

    def __init__(self, robot, gripper_to_camera):
        self.robot = robot
        self.gripper_to_camera = gripper_to_camera

    def pixel_to_base_target(
        self,
        u,
        v,
        label,
        source,
        depth_image,
        intrinsics,
        logger,
        vlm_rpy_deg=None,
    ):
        height, width = depth_image.shape[:2]

        if not (0 <= u < width and 0 <= v < height):
            logger.warn(
                f"{source} 픽셀이 depth 이미지 범위를 벗어남: u={u}, v={v}"
            )
            return None

        z_raw = depth_image[v, u]
        if z_raw == 0:
            logger.warn(f"{label} 검출됨, 하지만 {source} depth 값이 0입니다.")
            return None

        z_m = float(z_raw) / 1000.0
        cam_x = (u - intrinsics["ppx"]) * z_m / intrinsics["fx"]
        cam_y = (v - intrinsics["ppy"]) * z_m / intrinsics["fy"]
        bx, by, bz = self.camera_to_base((cam_x, cam_y, z_m))

        logger.info(
            f"'{label}' 검출: source={source}, "
            f"pixel=({u}, {v}), "
            f"camera=({cam_x:.3f}, {cam_y:.3f}, {z_m:.3f}), "
            f"base=({bx:.3f}, {by:.3f}, {bz:.3f})"
        )

        return bx, by, bz, z_m, vlm_rpy_deg

    def camera_to_base(self, cam_xyz):
        coord = np.append(np.array(cam_xyz), 1.0)
        base_to_camera = get_ee_matrix(self.robot) @ self.gripper_to_camera
        return (base_to_camera @ coord)[:3]
