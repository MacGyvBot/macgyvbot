"""Pose and robot-state utilities shared by motion and perception."""

import math

import numpy as np
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation

from macgyvbot_config.robot import BASE_FRAME, EE_LINK


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
    """Build a pose goal; MoveItController clamps it before planning."""
    return make_pose(x, y, z, ori)


def get_ee_matrix(moveit_robot):
    psm = moveit_robot.get_planning_scene_monitor()

    with psm.read_only() as scene:
        transform = scene.current_state.get_global_link_transform(EE_LINK)

    return np.asarray(transform, dtype=float)


def current_ee_orientation(moveit_robot):
    transform = get_ee_matrix(moveit_robot)
    qx, qy, qz, qw = Rotation.from_matrix(transform[:3, :3]).as_quat()
    return {
        "x": float(qx),
        "y": float(qy),
        "z": float(qz),
        "w": float(qw),
    }


def normalize_angle_deg(angle_deg):
    return ((angle_deg + 180.0) % 360.0) - 180.0


def is_finite_angle_deg(angle_deg):
    try:
        angle = float(angle_deg)
    except (TypeError, ValueError):
        return False
    return math.isfinite(angle)
