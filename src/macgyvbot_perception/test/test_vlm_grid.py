import math

from PIL import Image

from macgyvbot_perception.grasp_point.vlm.models import GridChoice
from macgyvbot_perception.grasp_point.vlm_method.grid import GridPolicy


def test_draw_numbered_grid_preserves_size():
    image = Image.new("RGB", (120, 80), color=(255, 255, 255))
    grid = GridPolicy()

    overlay = grid.draw_numbered_grid(image, rows=3, cols=4)

    assert overlay.size == image.size


def test_has_converged_for_nearby_choices():
    grid = GridPolicy()
    choices = [
        GridChoice(3, 3, 1, 1, 1, (0, 0, 10, 10), (50.0, 50.0), None, "", ""),
        GridChoice(4, 4, 1, 1, 1, (0, 0, 10, 10), (55.0, 52.0), None, "", ""),
    ]

    assert grid.has_converged(choices, image_size=(100, 100), radius_factor=0.1)


def test_estimate_grasp_pose_returns_mean_and_angle():
    grid = GridPolicy()
    choices = [
        GridChoice(3, 3, 1, 1, 1, (0, 0, 10, 10), (10.0, 10.0), None, "", ""),
        GridChoice(3, 3, 2, 2, 5, (0, 0, 10, 10), (20.0, 20.0), None, "", ""),
    ]

    point, angle = grid.estimate_grasp_pose(choices)

    assert point == (15.0, 15.0)
    assert math.isclose(angle, math.pi / 4.0)
