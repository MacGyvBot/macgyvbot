from macgyvbot_perception.grasp_point.vlm.models import VLMResult
from macgyvbot_perception.grasp_point.vlm.parser import Parser


def test_parse_vlm_only_result_from_json():
    parser = Parser()
    result = VLMResult(
        text='{"x_px": 12, "y_px": 18, "yaw_deg": -45, "confidence": 0.9}',
        data={"x_px": 12, "y_px": 18, "yaw_deg": -45, "confidence": 0.9},
    )

    parsed = parser.parse_vlm_only_result(result, width=100, height=80)

    assert parsed == ((12.0, 18.0), -45.0)


def test_parse_vlm_only_result_clamps_and_normalizes():
    parser = Parser()
    result = VLMResult(
        text='{"x_px": 120, "y_px": -5, "yaw_deg": 270}',
        data={"x_px": 120, "y_px": -5, "yaw_deg": 270},
    )

    parsed = parser.parse_vlm_only_result(result, width=100, height=80)

    assert parsed == ((99.0, 0.0), -90.0)


def test_parse_grid_result_from_text_tuple():
    parser = Parser()
    result = VLMResult(text="I choose (2,3)", data=None)

    choice = parser.parse_vlm_grid_result(
        result,
        rows=3,
        cols=4,
        image_size=(400, 300),
    )

    assert choice is not None
    assert choice.row == 2
    assert choice.col == 3
    assert choice.cell == 7
    assert choice.center == (250.0, 150.0)


def test_parse_orientation_uses_yaw_fallback():
    parser = Parser()
    result = VLMResult(text="{}", data={})

    assert parser.parse_vlm_orientation_result(result, yaw_fallback_deg=30.0) == (
        0.0,
        0.0,
        30.0,
    )
