"""Home pose initialization for robot workflows."""

from __future__ import annotations

from scipy.spatial.transform import Rotation

from macgyvbot.control.robot_pose import get_ee_matrix


class RobotHomeInitializer:
    """Move the robot Home and capture Home position/orientation on state."""

    def __init__(self, robot, motion_controller, state):
        self.robot = robot
        self.motion = motion_controller
        self.state = state

    def initialize(self):
        logger = self.state.get_logger()
        logger.info("시스템 준비 중... Home으로 이동합니다.")

        ok = self.motion.move_to_home_joints(logger)
        if not ok:
            logger.error("초기 Home 이동 실패")
            return False

        transform = get_ee_matrix(self.robot)
        self.state.home_xyz = (
            transform[0, 3],
            transform[1, 3],
            transform[2, 3],
        )

        qx, qy, qz, qw = Rotation.from_matrix(transform[:3, :3]).as_quat()
        self.state.home_ori = {
            "x": float(qx),
            "y": float(qy),
            "z": float(qz),
            "w": float(qw),
        }

        logger.info(
            f"Home 저장 완료: x={self.state.home_xyz[0]:.3f}, "
            f"y={self.state.home_xyz[1]:.3f}, z={self.state.home_xyz[2]:.3f}"
        )
        return True
