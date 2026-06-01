import sys
import types

import numpy as np


try:
    import cv2 as cv2_module
except ImportError:
    cv2_module = types.ModuleType("cv2")
sys.modules.setdefault("cv2", cv2_module)

import macgyvbot_perception.drawer_marker_resolver as drawer_marker_resolver

DrawerMarkerResolver = drawer_marker_resolver.DrawerMarkerResolver


class FakeLogger:
    def info(self, _message):
        pass

    def warn(self, _message):
        pass

    def error(self, _message):
        pass


class FakeDepthProjector:
    def __init__(self, target=None):
        self.target = target

    def pixel_to_base_target(self, *_args, **_kwargs):
        return self.target


def test_marker_resolver_returns_not_found_without_aruco_module():
    drawer_marker_resolver.cv2 = types.SimpleNamespace()
    resolver = DrawerMarkerResolver(FakeDepthProjector(), FakeLogger())

    result = resolver.resolve_marker_target(
        np.zeros((10, 10, 3), dtype=np.uint8),
        np.ones((10, 10), dtype=np.float32),
        {"fx": 1.0, "fy": 1.0, "ppx": 0.0, "ppy": 0.0},
        10,
    )

    assert not result.found
    assert result.reason == "return_drawer_marker_not_found"


def test_marker_resolver_returns_projection_failed_for_missing_camera_state():
    resolver = DrawerMarkerResolver(FakeDepthProjector(), FakeLogger())

    result = resolver.resolve_marker_target(None, None, None, 10)

    assert not result.found
    assert result.reason == "return_drawer_marker_projection_failed"


def test_marker_resolver_returns_projection_failed_when_depth_projection_fails():
    class FakeDetector:
        def __init__(self, *_args, **_kwargs):
            pass

        def detectMarkers(self, _image):
            corners = [
                np.array(
                    [[[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]]],
                    dtype=np.float32,
                )
            ]
            return corners, np.array([[10]], dtype=np.int32), None

    aruco = types.SimpleNamespace(
        DICT_4X4_50=0,
        getPredefinedDictionary=lambda _dictionary_id: object(),
        DetectorParameters=lambda: object(),
        ArucoDetector=FakeDetector,
    )
    drawer_marker_resolver.cv2 = types.SimpleNamespace(
        aruco=aruco,
        COLOR_BGR2GRAY=0,
        cvtColor=lambda image, _code: image[:, :, 0],
    )
    resolver = DrawerMarkerResolver(FakeDepthProjector(target=None), FakeLogger())

    result = resolver.resolve_marker_target(
        np.zeros((10, 10, 3), dtype=np.uint8),
        np.ones((10, 10), dtype=np.float32),
        {"fx": 1.0, "fy": 1.0, "ppx": 0.0, "ppy": 0.0},
        10,
    )

    assert not result.found
    assert result.reason == "return_drawer_marker_projection_failed"
