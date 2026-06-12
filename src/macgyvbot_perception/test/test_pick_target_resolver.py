import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
for package_path in (
    REPO_ROOT / "src" / "macgyvbot_config",
    REPO_ROOT / "src" / "macgyvbot_domain",
    REPO_ROOT / "src" / "macgyvbot_perception",
):
    if str(package_path) not in sys.path:
        sys.path.insert(0, str(package_path))

from macgyvbot_perception.pick_target_resolver import PickTargetResolver


class _ArrayLike:
    def __init__(self, values):
        self.values = values

    def cpu(self):
        return self

    def numpy(self):
        return self.values


class _Box:
    def __init__(self, cls_id, confidence, bbox=(10, 20, 50, 80)):
        self.cls = [cls_id]
        self.conf = [confidence]
        self.xyxy = [_ArrayLike(bbox)]


class _Detector:
    names = {0: "wrench", 1: "hammer"}
    confidence_threshold = 0.4


class _GraspPointSelector:
    def __init__(self):
        self.selected_boxes = []

    def select(self, box, *_args):
        self.selected_boxes.append(box)
        return (30, 40, "center", None)

    def select_bbox_center(self, box):
        self.selected_boxes.append(box)
        return (30, 40, "center", None)

    def should_refine_grasp_point_at_top_view(self):
        return False


class _DepthProjector:
    def pixel_to_base_target(self, *_args):
        return (0.1, 0.2, 0.3, 0.7, None)


class _Logger:
    pass


def _resolver(selector=None):
    return PickTargetResolver(
        _Detector(),
        selector or _GraspPointSelector(),
        _DepthProjector(),
        _Logger(),
    )


def test_matching_box_ignores_target_below_confidence_threshold():
    resolver = _resolver()

    matched = resolver.matching_box([_Box(0, 0.18)], "wrench")

    assert matched is None


def test_matching_box_selects_highest_confident_target_label():
    resolver = _resolver()
    low_confident = _Box(0, 0.41, bbox=(0, 0, 10, 10))
    high_confident = _Box(0, 0.72, bbox=(20, 20, 60, 60))

    matched = resolver.matching_box(
        [
            _Box(1, 0.99, bbox=(100, 100, 120, 120)),
            low_confident,
            _Box(0, 0.18, bbox=(70, 70, 90, 90)),
            high_confident,
        ],
        "wrench",
    )

    assert matched == (high_confident, "wrench")


def test_target_from_boxes_blocks_low_confidence_before_grasp_and_depth():
    selector = _GraspPointSelector()
    resolver = _resolver(selector)

    target = resolver.target_from_boxes(
        [_Box(0, 0.18)],
        "wrench",
        color_image=object(),
        depth_image=object(),
        intrinsics={},
    )

    assert target.found is False
    assert target.reason == "target_not_found"
    assert selector.selected_boxes == []
