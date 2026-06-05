import math

from macgyvbot_config.grasp import PREGRASP_MAX_EXTRA_DESCENT_M
from macgyvbot_task.application.pick_flow.pick_grasp_flow import (
    calculate_pregrasp_extra_descent,
)


def test_pregrasp_extra_descent_uses_actual_depth_mm():
    assert math.isclose(calculate_pregrasp_extra_descent(12.0), 0.012)


def test_pregrasp_extra_descent_uses_absolute_depth():
    assert math.isclose(calculate_pregrasp_extra_descent(-12.0), 0.012)


def test_pregrasp_extra_descent_caps_depth_compensation():
    assert calculate_pregrasp_extra_descent(120.0) == PREGRASP_MAX_EXTRA_DESCENT_M
