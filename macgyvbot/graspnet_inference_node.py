#!/usr/bin/env python3
"""Backward-compatible entrypoint for the GraspNet inference node."""

from macgyvbot.nodes.graspnet_inference_node import (
    GraspNetInferenceNode,
    build_point_cloud,
    depth_image_to_meters,
    main,
    make_pose_from_grasp,
    normalize_quaternion,
)

__all__ = [
    "GraspNetInferenceNode",
    "build_point_cloud",
    "depth_image_to_meters",
    "main",
    "make_pose_from_grasp",
    "normalize_quaternion",
]


if __name__ == "__main__":
    main()
