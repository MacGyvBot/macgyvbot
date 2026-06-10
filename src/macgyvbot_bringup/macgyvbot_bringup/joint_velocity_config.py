"""Launch-time joint velocity patching for MoveIt configuration."""

from __future__ import annotations

from copy import deepcopy
import xml.etree.ElementTree as ET

from macgyvbot_config.joint_velocity import JOINT_VELOCITY_LIMITS_RAD_PER_SEC


def validate_joint_velocity_limits(joint_velocity_limits=None):
    """Validate joint velocity limits before applying them to MoveIt config."""
    limits = (
        JOINT_VELOCITY_LIMITS_RAD_PER_SEC
        if joint_velocity_limits is None
        else joint_velocity_limits
    )
    expected_joints = {f"joint_{index}" for index in range(1, 7)}
    configured_joints = set(limits)

    missing_joints = expected_joints - configured_joints
    if missing_joints:
        names = ", ".join(sorted(missing_joints))
        raise ValueError(f"Missing joint velocity limits for: {names}")

    for joint_name, velocity in limits.items():
        if velocity <= 0.0:
            raise ValueError(
                f"{joint_name} velocity limit must be positive: {velocity}"
            )


def apply_joint_velocity_limits_to_urdf(
    robot_description,
    joint_velocity_limits=None,
):
    """Return robot_description URDF with configured joint velocity limits."""
    limits = (
        JOINT_VELOCITY_LIMITS_RAD_PER_SEC
        if joint_velocity_limits is None
        else joint_velocity_limits
    )
    validate_joint_velocity_limits(limits)

    root = ET.fromstring(robot_description)
    for joint_name, velocity in limits.items():
        joint = root.find(f".//joint[@name='{joint_name}']")
        if joint is None:
            raise ValueError(f"Joint not found in robot_description: {joint_name}")
        limit = joint.find("limit")
        if limit is None:
            raise ValueError(
                f"Joint limit not found in robot_description: {joint_name}"
            )
        limit.set("velocity", str(velocity))

    return ET.tostring(root, encoding="unicode")


def apply_joint_velocity_limits_to_moveit_config(
    moveit_config,
    joint_velocity_limits=None,
):
    """Return a MoveIt config dict with URDF and planning limits updated."""
    limits = (
        JOINT_VELOCITY_LIMITS_RAD_PER_SEC
        if joint_velocity_limits is None
        else joint_velocity_limits
    )
    validate_joint_velocity_limits(limits)

    config = deepcopy(moveit_config)
    robot_description = config.get("robot_description")
    if isinstance(robot_description, str):
        config["robot_description"] = apply_joint_velocity_limits_to_urdf(
            robot_description,
            limits,
        )

    planning = config.setdefault("robot_description_planning", {})
    joint_limits = planning.setdefault("joint_limits", {})
    for joint_name, velocity in limits.items():
        joint_config = joint_limits.setdefault(joint_name, {})
        joint_config["has_velocity_limits"] = True
        joint_config["max_velocity"] = velocity

    return config
