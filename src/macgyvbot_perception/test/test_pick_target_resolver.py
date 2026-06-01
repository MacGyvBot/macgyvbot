from macgyvbot_perception.pick_target_resolver import PickTargetResolver


class FakeBox:
    cls = [0]


class FakeDetector:
    names = {0: "screwdriver"}


class FakeSelector:
    def select(self, *_args):
        return 12, 34, "center", (0.0, 0.0, 28.3)


class FakeDepthProjector:
    def pixel_to_base_target(self, *_args):
        return None


class FakeLogger:
    def warn(self, _message):
        pass


def test_target_from_selected_grasp_preserves_yaw_when_depth_projection_fails():
    resolver = PickTargetResolver(
        FakeDetector(),
        FakeSelector(),
        FakeDepthProjector(),
        FakeLogger(),
    )

    target = resolver.target_from_selected_grasp(
        "screwdriver",
        "screwdriver",
        (12, 34, "center", (0.0, 0.0, 28.3)),
        depth_image=None,
        intrinsics={},
    )

    assert target.found is False
    assert target.reason == "depth_projection_failed"
    assert target.pixel == (12, 34)
    assert target.source == "center"
    assert target.yaw_deg == 28.3
