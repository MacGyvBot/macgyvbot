from macgyvbot_config.vlm import GRASP_POINT_MODE_YOLO
from macgyvbot_perception.grasp_point.yolo_method.selector import (
    YoloGraspPointSelector,
)


class _Logger:
    def warn(self, _message):
        pass


class _ArrayLike:
    def __init__(self, values):
        self.values = values

    def cpu(self):
        return self

    def numpy(self):
        return self.values


class _Box:
    def __init__(self, cls_id, confidence, bbox):
        self.cls = [cls_id]
        self.conf = [confidence]
        self.xyxy = [_ArrayLike(bbox)]


def test_selects_grasp_point_bbox_center_inside_target_bbox():
    selector = YoloGraspPointSelector(_Logger())
    boxes = [
        _Box(0, 0.95, [10, 10, 90, 90]),
        _Box(1, 0.80, [40, 50, 60, 70]),
    ]

    selected = selector.select_grasp_pixel(
        boxes,
        names={0: "screwdriver", 1: "grasp_point"},
        target_bbox=[10, 10, 90, 90],
    )

    assert selected == (50, 60, GRASP_POINT_MODE_YOLO, None)


def test_ignores_grasp_point_bbox_outside_target_bbox():
    selector = YoloGraspPointSelector(_Logger())
    boxes = [_Box(1, 0.80, [140, 150, 160, 170])]

    selected = selector.select_grasp_pixel(
        boxes,
        names={1: "grasp_point"},
        target_bbox=[10, 10, 90, 90],
    )

    assert selected is None
