"""Resolve drawer ArUco marker centers into robot-base targets."""

from __future__ import annotations

import numpy as np

from macgyvbot_domain.target_models import PickTarget

try:
    import cv2
except ImportError:  # pragma: no cover - depends on robot image environment.
    cv2 = None


class DrawerMarkerResolver:
    """Detect an ArUco marker and project its center into the base frame."""

    def __init__(
        self,
        depth_projector,
        logger,
        dictionary_name="DICT_4X4_50",
    ):
        self.depth_projector = depth_projector
        self.logger = logger
        self.dictionary_name = dictionary_name

    def resolve_marker_target(
        self,
        color_image,
        depth_image,
        intrinsics,
        marker_id,
    ) -> PickTarget:
        marker_id = int(marker_id)
        if color_image is None or depth_image is None or intrinsics is None:
            return self._not_found(marker_id, "return_drawer_marker_projection_failed")

        marker_center = self._detect_marker_center(color_image, marker_id)
        if marker_center is None:
            return self._not_found(marker_id, "return_drawer_marker_not_found")

        u, v = marker_center
        target = self.depth_projector.pixel_to_base_target(
            u,
            v,
            f"drawer_marker_{marker_id}",
            "aruco_marker_center",
            depth_image,
            intrinsics,
            self.logger,
        )
        if target is None:
            return self._not_found(
                marker_id,
                "return_drawer_marker_projection_failed",
            )

        bx, by, bz, z_m, _ = target
        return PickTarget(
            found=True,
            label=f"drawer_marker_{marker_id}",
            pixel=(u, v),
            base_xyz=(bx, by, bz),
            depth_m=z_m,
            yaw_deg=None,
            source="aruco_marker_center",
        )

    def _detect_marker_center(self, color_image, marker_id):
        if cv2 is None or not hasattr(cv2, "aruco"):
            self.logger.error("OpenCV aruco 모듈을 사용할 수 없습니다.")
            return None

        corners, ids = self._detect_markers(color_image)
        if ids is None or len(ids) == 0:
            return None

        flat_ids = np.asarray(ids).reshape(-1)
        for index, found_id in enumerate(flat_ids):
            if int(found_id) != int(marker_id):
                continue

            marker_corners = np.asarray(corners[index], dtype=float).reshape(-1, 2)
            center = marker_corners.mean(axis=0)
            return int(round(float(center[0]))), int(round(float(center[1])))

        return None

    def _detect_markers(self, color_image):
        aruco = cv2.aruco
        dictionary_id = getattr(aruco, self.dictionary_name)
        dictionary = aruco.getPredefinedDictionary(dictionary_id)
        image = self._gray_image(color_image)

        if hasattr(aruco, "ArucoDetector"):
            parameters = aruco.DetectorParameters()
            detector = aruco.ArucoDetector(dictionary, parameters)
            corners, ids, _ = detector.detectMarkers(image)
            return corners, ids

        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(
            image,
            dictionary,
            parameters=parameters,
        )
        return corners, ids

    @staticmethod
    def _gray_image(color_image):
        if len(color_image.shape) == 2:
            return color_image
        return cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

    @staticmethod
    def _not_found(marker_id, reason) -> PickTarget:
        return PickTarget(
            found=False,
            label=f"drawer_marker_{marker_id}",
            pixel=None,
            base_xyz=None,
            depth_m=None,
            yaw_deg=None,
            reason=reason,
        )
