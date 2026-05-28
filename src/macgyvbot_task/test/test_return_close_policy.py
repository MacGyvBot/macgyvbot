from macgyvbot_task.application.return_flow.return_close_policy import (
    ReturnClosePolicy,
)


def test_accepts_roi_center_and_depth_boundaries():
    policy = ReturnClosePolicy(
        center_x_ratio=0.5,
        center_y_ratio=0.5,
        width_ratio=0.25,
        height_ratio=0.25,
        min_depth_mm=100.0,
        max_depth_mm=200.0,
    )

    assert policy.matches((100, 200, 3), (90, 45, 110, 55), 100.0)
    assert policy.matches((100, 200, 3), (90, 45, 110, 55), 200.0)


def test_rejects_roi_outside_close_region():
    policy = ReturnClosePolicy(
        center_x_ratio=0.5,
        center_y_ratio=0.5,
        width_ratio=0.2,
        height_ratio=0.2,
        min_depth_mm=100.0,
        max_depth_mm=200.0,
    )

    assert not policy.matches((100, 200, 3), (10, 45, 30, 55), 150.0)


def test_rejects_missing_image_or_bad_roi():
    policy = ReturnClosePolicy()

    assert not policy.matches(None, (1, 2, 3, 4), 200.0)
    assert not policy.matches((100, 200, 3), (1, 2, 3), 200.0)
    assert not policy.matches((100, 200, 3), ("bad", 2, 3, 4), 200.0)


def test_rejects_depth_outside_or_missing():
    policy = ReturnClosePolicy(min_depth_mm=100.0, max_depth_mm=200.0)

    assert not policy.depth_in_close_range(None)
    assert not policy.depth_in_close_range("bad")
    assert not policy.depth_in_close_range(99.9)
    assert not policy.depth_in_close_range(200.1)
