import csv

from PIL import Image

from macgyvbot_perception.grasp_point.vlm.inference_history_recode import (
    InferenceHistoryConfig,
    InferenceHistoryRecode,
)


def test_history_disabled_writes_nothing(tmp_path):
    recorder = InferenceHistoryRecode(
        InferenceHistoryConfig(enabled=False, root_dir=str(tmp_path))
    )

    recorder.record(
        image=Image.new("RGB", (10, 10)),
        mode="vlm_only_qwen3b",
        model_id="model",
        target_label="hammer",
        detected_label="hammer",
        bbox_xyxy=(0, 0, 10, 10),
    )

    assert not any(tmp_path.iterdir())


def test_history_enabled_writes_image_and_csv(tmp_path):
    recorder = InferenceHistoryRecode(
        InferenceHistoryConfig(enabled=True, root_dir=str(tmp_path))
    )

    recorder.record(
        image=Image.new("RGB", (10, 10)),
        mode="vlm_only_qwen3b",
        model_id="model",
        target_label="hammer",
        detected_label="hammer",
        bbox_xyxy=(0, 0, 10, 10),
        raw_response='{"x_px": 1}',
        parsed_point=(1, 2),
        yaw_deg=30.0,
        orientation_rpy_deg=(0.0, 0.0, 30.0),
        success=True,
    )

    csv_path = tmp_path / "inference_history.csv"
    assert csv_path.exists()
    assert len(list((tmp_path / "crop_image").glob("*.jpg"))) == 1
    assert not (tmp_path / "frame_image").exists()
    with csv_path.open(encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    assert rows[0]["mode"] == "vlm_only_qwen3b"
    assert rows[0]["success"] == "True"
