"""Geometry-first grasp planning for narrow-space parallel-jaw grasps.

This module intentionally has no ROS, MoveIt, YOLO, or gripper driver imports.
It turns segmentation/depth geometry into a stepped grasp plan that can later be
wired into ``hf_auto_pick_place.py`` without changing the robot control loop.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole camera intrinsics in pixels."""

    fx: float
    fy: float
    ppx: float
    ppy: float


@dataclass(frozen=True)
class GripperGeometry:
    """Parallel-jaw gripper geometry.

    Widths are stored in meters. ``to_register_width`` converts a metric width
    to the OnRobot driver convention used by ``RG.move_gripper``: 1/10 mm.
    """

    min_width_m: float = 0.0
    max_width_m: float = 0.110
    finger_thickness_m: float = 0.008
    pregrasp_offset_m: float = 0.015
    final_grasp_offset_m: float = 0.004
    approach_clearance_m: float = 0.08
    descend_clearance_m: float = 0.01
    lift_distance_m: float = 0.12

    def clamp_width(self, width_m: float) -> float:
        return float(np.clip(width_m, self.min_width_m, self.max_width_m))

    @staticmethod
    def to_register_width(width_m: float) -> int:
        return int(round(width_m * 10000.0))


@dataclass(frozen=True)
class HardGraspConfig:
    """Tuning parameters for segmentation/depth to grasp conversion."""

    min_points: int = 40
    bbox_shrink_ratio: float = 0.08
    depth_scale: float | None = None
    z_percentile_clip: tuple[float, float] = (2.0, 98.0)
    max_points: int = 5000
    table_normal_world: tuple[float, float, float] = (0.0, 0.0, 1.0)


@dataclass(frozen=True)
class PlanarFootprint:
    center_xy: np.ndarray
    long_axis_xy: np.ndarray
    short_axis_xy: np.ndarray
    long_width_m: float
    narrow_width_m: float


@dataclass(frozen=True)
class GraspStep:
    name: str
    description: str
    target_width_m: float | None = None
    target_width_register: int | None = None
    pose_xyz: tuple[float, float, float] | None = None


@dataclass(frozen=True)
class HardGraspPlan:
    """Result of the narrow-space grasp algorithm."""

    center_world: np.ndarray
    pregrasp_world: np.ndarray
    descend_world: np.ndarray
    lift_world: np.ndarray
    approach_world: np.ndarray
    closing_axis_world: np.ndarray
    long_axis_world: np.ndarray
    object_narrow_width_m: float
    object_long_width_m: float
    pregrasp_width_m: float
    final_grasp_width_m: float
    pregrasp_width_register: int
    final_grasp_width_register: int
    score: float
    feasible: bool
    reason: str
    steps: tuple[GraspStep, ...] = field(default_factory=tuple)


class HardGraspPlanner:
    """Step-by-step planner for constrained parallel-jaw grasping."""

    def __init__(
        self,
        gripper: GripperGeometry | None = None,
        config: HardGraspConfig | None = None,
    ) -> None:
        self.gripper = gripper or GripperGeometry()
        self.config = config or HardGraspConfig()

    def plan_from_depth_mask(
        self,
        depth_image: np.ndarray,
        intrinsics: CameraIntrinsics | dict,
        mask: np.ndarray | None = None,
        bbox: tuple[int, int, int, int] | Iterable[float] | None = None,
        camera_to_world: np.ndarray | None = None,
    ) -> HardGraspPlan:
        """Build a grasp plan from depth plus segmentation mask or bbox.

        ``mask`` is preferred. If only ``bbox`` is available, the bbox interior
        is used as a coarse mask so this can be integrated before true instance
        segmentation is ready.
        """

        intrinsics = _coerce_intrinsics(intrinsics)
        depth_m = normalize_depth_to_m(depth_image, self.config.depth_scale)
        object_mask = make_object_mask(
            depth_m.shape,
            mask=mask,
            bbox=bbox,
            shrink_ratio=self.config.bbox_shrink_ratio,
        )
        pixels_u, pixels_v, pixels_z = extract_valid_depth_pixels(depth_m, object_mask)
        points_camera = unproject_pixels_to_camera(
            pixels_u,
            pixels_v,
            pixels_z,
            intrinsics,
        )
        points_world = transform_points(points_camera, camera_to_world)
        return self.plan_from_points(points_world)

    def plan_from_points(self, object_points_world: np.ndarray) -> HardGraspPlan:
        """Build a grasp plan from an object-only world-frame point cloud."""

        points = filter_object_points(
            object_points_world,
            min_points=self.config.min_points,
            z_percentile_clip=self.config.z_percentile_clip,
            max_points=self.config.max_points,
        )
        footprint = estimate_planar_footprint(points)
        return self._plan_from_footprint(points, footprint)

    def _plan_from_footprint(
        self,
        points_world: np.ndarray,
        footprint: PlanarFootprint,
    ) -> HardGraspPlan:
        gripper = self.gripper
        narrow_width = footprint.narrow_width_m
        long_width = footprint.long_width_m

        pregrasp_width = gripper.clamp_width(narrow_width + gripper.pregrasp_offset_m)
        final_width = gripper.clamp_width(narrow_width + gripper.final_grasp_offset_m)
        pregrasp_register = gripper.to_register_width(pregrasp_width)
        final_register = gripper.to_register_width(final_width)

        center_z = float(np.percentile(points_world[:, 2], 98.0))
        center = np.array(
            [footprint.center_xy[0], footprint.center_xy[1], center_z],
            dtype=float,
        )
        normal = _normalize(
            np.asarray(self.config.table_normal_world, dtype=float),
            np.array([0.0, 0.0, 1.0]),
        )
        approach = -normal
        closing_axis = _normalize(
            np.array([footprint.short_axis_xy[0], footprint.short_axis_xy[1], 0.0]),
            np.array([1.0, 0.0, 0.0]),
        )
        long_axis = _normalize(
            np.array([footprint.long_axis_xy[0], footprint.long_axis_xy[1], 0.0]),
            np.array([0.0, 1.0, 0.0]),
        )

        pregrasp = center - approach * gripper.approach_clearance_m
        descend = center - approach * gripper.descend_clearance_m
        lift = descend - approach * gripper.lift_distance_m

        feasible, reason = self._check_feasibility(narrow_width, pregrasp_width)
        score = self._score(footprint, pregrasp_width, feasible)
        steps = self._make_steps(pregrasp, descend, lift, pregrasp_width, final_width)

        return HardGraspPlan(
            center_world=center,
            pregrasp_world=pregrasp,
            descend_world=descend,
            lift_world=lift,
            approach_world=approach,
            closing_axis_world=closing_axis,
            long_axis_world=long_axis,
            object_narrow_width_m=float(narrow_width),
            object_long_width_m=float(long_width),
            pregrasp_width_m=float(pregrasp_width),
            final_grasp_width_m=float(final_width),
            pregrasp_width_register=pregrasp_register,
            final_grasp_width_register=final_register,
            score=score,
            feasible=feasible,
            reason=reason,
            steps=steps,
        )

    def _check_feasibility(
        self,
        narrow_width_m: float,
        pregrasp_width_m: float,
    ) -> tuple[bool, str]:
        gripper = self.gripper
        outer_width = pregrasp_width_m + 2.0 * gripper.finger_thickness_m

        if narrow_width_m > gripper.max_width_m:
            return False, "object narrow width is larger than max gripper opening"
        if outer_width > gripper.max_width_m + 2.0 * gripper.finger_thickness_m:
            return False, "pregrasp opening plus finger thickness exceeds usable width"
        if math.isclose(pregrasp_width_m, gripper.max_width_m):
            return True, "pregrasp width clamped to max gripper opening"

        return True, "ok"

    def _score(
        self,
        footprint: PlanarFootprint,
        pregrasp_width_m: float,
        feasible: bool,
    ) -> float:
        if not feasible:
            return 0.0

        long_width = max(footprint.long_width_m, 1e-6)
        aspect_score = np.clip(
            (footprint.long_width_m - footprint.narrow_width_m) / long_width,
            0.0,
            1.0,
        )
        compact_opening_score = 1.0 - np.clip(
            pregrasp_width_m / max(self.gripper.max_width_m, 1e-6),
            0.0,
            1.0,
        )
        return float(0.65 * aspect_score + 0.35 * compact_opening_score)

    def _make_steps(
        self,
        pregrasp: np.ndarray,
        descend: np.ndarray,
        lift: np.ndarray,
        pregrasp_width_m: float,
        final_width_m: float,
    ) -> tuple[GraspStep, ...]:
        return (
            GraspStep(
                name="preshape",
                description="Open gripper only to object narrow width plus entry offset.",
                target_width_m=float(pregrasp_width_m),
                target_width_register=self.gripper.to_register_width(pregrasp_width_m),
            ),
            GraspStep(
                name="move_pregrasp",
                description="Move above the selected grasp center with compact opening.",
                pose_xyz=_tuple3(pregrasp),
            ),
            GraspStep(
                name="descend",
                description="Descend into the narrow space while keeping compact opening.",
                pose_xyz=_tuple3(descend),
            ),
            GraspStep(
                name="close",
                description="Close toward object width plus final grasp offset.",
                target_width_m=float(final_width_m),
                target_width_register=self.gripper.to_register_width(final_width_m),
            ),
            GraspStep(
                name="lift",
                description="Lift from the grasp pose.",
                pose_xyz=_tuple3(lift),
            ),
        )


def normalize_depth_to_m(depth_image: np.ndarray, depth_scale: float | None = None) -> np.ndarray:
    """Convert common ROS depth encodings to meters."""

    depth = np.asarray(depth_image)
    if depth.ndim < 2:
        raise ValueError("depth_image must be at least 2D")

    depth = depth.astype(np.float32)
    depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)

    if depth_scale is not None:
        return depth * float(depth_scale)
    if np.issubdtype(np.asarray(depth_image).dtype, np.integer):
        return depth * 0.001

    max_depth = float(np.max(depth)) if depth.size else 0.0
    return depth * 0.001 if max_depth > 20.0 else depth


def make_object_mask(
    image_shape: tuple[int, ...],
    mask: np.ndarray | None = None,
    bbox: tuple[int, int, int, int] | Iterable[float] | None = None,
    shrink_ratio: float = 0.0,
) -> np.ndarray:
    """Return a boolean object mask from segmentation mask or bbox fallback."""

    height, width = int(image_shape[0]), int(image_shape[1])

    if mask is not None:
        object_mask = np.asarray(mask).astype(bool)
        if object_mask.shape[:2] != (height, width):
            raise ValueError("mask shape must match depth image shape")
        return object_mask

    if bbox is None:
        raise ValueError("Either mask or bbox must be provided")

    x1, y1, x2, y2 = _clamp_bbox(bbox, width, height)
    box_w = max(0, x2 - x1)
    box_h = max(0, y2 - y1)
    shrink_x = int(round(box_w * shrink_ratio))
    shrink_y = int(round(box_h * shrink_ratio))
    x1, y1, x2, y2 = (
        x1 + shrink_x,
        y1 + shrink_y,
        x2 - shrink_x,
        y2 - shrink_y,
    )

    object_mask = np.zeros((height, width), dtype=bool)
    if x2 > x1 and y2 > y1:
        object_mask[y1:y2, x1:x2] = True
    return object_mask


def extract_valid_depth_pixels(
    depth_m: np.ndarray,
    object_mask: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Extract valid depth pixels from the selected object region."""

    valid = object_mask & np.isfinite(depth_m) & (depth_m > 0.0)
    pixels_v, pixels_u = np.nonzero(valid)

    if len(pixels_u) == 0:
        raise ValueError("No valid depth pixels inside object mask")

    return pixels_u.astype(float), pixels_v.astype(float), depth_m[pixels_v, pixels_u]


def unproject_pixels_to_camera(
    pixels_u: np.ndarray,
    pixels_v: np.ndarray,
    depth_m: np.ndarray,
    intrinsics: CameraIntrinsics,
) -> np.ndarray:
    """Unproject aligned depth pixels into camera-frame XYZ points."""

    z = np.asarray(depth_m, dtype=float)
    x = (np.asarray(pixels_u, dtype=float) - intrinsics.ppx) * z / intrinsics.fx
    y = (np.asarray(pixels_v, dtype=float) - intrinsics.ppy) * z / intrinsics.fy
    return np.column_stack((x, y, z))


def transform_points(
    points: np.ndarray,
    transform: np.ndarray | None = None,
) -> np.ndarray:
    """Transform Nx3 points with a 4x4 homogeneous matrix."""

    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape Nx3")
    if transform is None:
        return points

    transform = np.asarray(transform, dtype=float)
    if transform.shape != (4, 4):
        raise ValueError("transform must have shape 4x4")

    homogeneous = np.column_stack((points, np.ones(len(points))))
    return (transform @ homogeneous.T).T[:, :3]


def filter_object_points(
    points: np.ndarray,
    min_points: int,
    z_percentile_clip: tuple[float, float],
    max_points: int,
) -> np.ndarray:
    """Remove invalid/outlier points and optionally downsample."""

    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("object points must have shape Nx3")

    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < min_points:
        raise ValueError(f"Need at least {min_points} finite object points")

    low_p, high_p = z_percentile_clip
    z_low, z_high = np.percentile(points[:, 2], [low_p, high_p])
    clipped = points[(points[:, 2] >= z_low) & (points[:, 2] <= z_high)]
    if len(clipped) >= min_points:
        points = clipped

    if max_points > 0 and len(points) > max_points:
        indices = np.linspace(0, len(points) - 1, max_points).astype(int)
        points = points[indices]

    return points


def estimate_planar_footprint(points_world: np.ndarray) -> PlanarFootprint:
    """Estimate an oriented 2D footprint using PCA on world XY points."""

    points_xy = np.asarray(points_world, dtype=float)[:, :2]
    center = points_xy.mean(axis=0)
    centered = points_xy - center

    if np.linalg.norm(centered) < 1e-8:
        long_axis = np.array([1.0, 0.0])
        short_axis = np.array([0.0, 1.0])
    else:
        covariance = np.cov(centered.T)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        order = np.argsort(eigenvalues)[::-1]
        long_axis = _normalize(eigenvectors[:, order[0]], np.array([1.0, 0.0]))
        short_axis = _normalize(eigenvectors[:, order[1]], np.array([0.0, 1.0]))

    long_width = extent_along_axis(points_xy, long_axis)
    narrow_width = extent_along_axis(points_xy, short_axis)

    return PlanarFootprint(
        center_xy=center,
        long_axis_xy=long_axis,
        short_axis_xy=short_axis,
        long_width_m=long_width,
        narrow_width_m=narrow_width,
    )


def extent_along_axis(points_xy: np.ndarray, axis_xy: np.ndarray) -> float:
    axis_xy = _normalize(np.asarray(axis_xy, dtype=float), np.array([1.0, 0.0]))
    projected = np.asarray(points_xy, dtype=float) @ axis_xy
    return float(projected.max() - projected.min())


def _coerce_intrinsics(intrinsics: CameraIntrinsics | dict) -> CameraIntrinsics:
    if isinstance(intrinsics, CameraIntrinsics):
        return intrinsics

    return CameraIntrinsics(
        fx=float(intrinsics["fx"]),
        fy=float(intrinsics["fy"]),
        ppx=float(intrinsics["ppx"]),
        ppy=float(intrinsics["ppy"]),
    )


def _normalize(vector: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    vector = np.asarray(vector, dtype=float)
    norm = float(np.linalg.norm(vector))
    if norm < 1e-8:
        return np.asarray(fallback, dtype=float)
    return vector / norm


def _clamp_bbox(
    bbox: tuple[int, int, int, int] | Iterable[float],
    width: int,
    height: int,
) -> tuple[int, int, int, int]:
    x1, y1, x2, y2 = [float(value) for value in bbox]
    return (
        max(0, min(width - 1, int(math.floor(x1)))),
        max(0, min(height - 1, int(math.floor(y1)))),
        max(0, min(width, int(math.ceil(x2)))),
        max(0, min(height, int(math.ceil(y2)))),
    )


def _tuple3(vector: np.ndarray) -> tuple[float, float, float]:
    return (float(vector[0]), float(vector[1]), float(vector[2]))

