"""Depth image projection and camera-to-base transformation."""

import numpy as np

from macgyvbot.motion.pose_utils import get_ee_matrix


class DepthProjector:
    def __init__(self, moveit_robot, gripper2cam, logger):
        self.moveit_robot = moveit_robot
        self.gripper2cam = gripper2cam
        self.logger = logger

    def transform_to_base(self, cam_xyz):
        coord = np.append(np.array(cam_xyz), 1.0)
        base2cam = get_ee_matrix(self.moveit_robot) @ self.gripper2cam
        return (base2cam @ coord)[:3]

    def pixel_to_base_target(
        self,
        depth_image,
        intrinsics,
        u,
        v,
        label,
        source,
        vlm_rpy_deg=None,
    ):
        h, w = depth_image.shape[:2]

        if not (0 <= u < w and 0 <= v < h):
            self.logger.warn(
                f"{source} 픽셀이 depth 이미지 범위를 벗어남: u={u}, v={v}"
            )
            return None

        z_raw = depth_image[v, u]

        if z_raw == 0:
            self.logger.warn(
                f"{label} 검출됨, 하지만 {source} depth 값이 0입니다."
            )
            return None

        z_m = float(z_raw) / 1000.0
        cam_x = (u - intrinsics["ppx"]) * z_m / intrinsics["fx"]
        cam_y = (v - intrinsics["ppy"]) * z_m / intrinsics["fy"]
        bx, by, bz = self.transform_to_base((cam_x, cam_y, z_m))

        self.logger.info(
            f"'{label}' 검출: source={source}, "
            f"pixel=({u}, {v}), "
            f"camera=({cam_x:.3f}, {cam_y:.3f}, {z_m:.3f}), "
            f"base=({bx:.3f}, {by:.3f}, {bz:.3f})"
        )

        return bx, by, bz, z_m, vlm_rpy_deg
