"""Record VLM inference inputs and outputs for offline review."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

from PIL import Image


@dataclass(frozen=True)
class InferenceHistoryConfig:
    """Configuration for optional inference history recording."""

    enabled: bool = False
    root_dir: str = "src/macgyvbot_perception/data/vlm_traces"
    csv_name: str = "inference_history.csv"


class InferenceHistoryRecode:
    """Best-effort image and CSV recorder for VLM inference."""

    FIELD_NAMES = (
        "timestamp",
        "mode",
        "model_id",
        "target_label",
        "detected_label",
        "bbox_xyxy",
        "image_path",
        "raw_response",
        "parsed_point",
        "yaw_deg",
        "orientation_rpy_deg",
        "success",
        "error",
    )

    def __init__(self, config: InferenceHistoryConfig | None = None, logger=None):
        self.config = config or InferenceHistoryConfig()
        self.logger = logger

    def record(
        self,
        *,
        image: Image.Image | None,
        mode: str,
        model_id: str,
        target_label: str,
        detected_label: str,
        bbox_xyxy,
        raw_response: str = "",
        parsed_point=None,
        yaw_deg=None,
        orientation_rpy_deg=None,
        success: bool = True,
        error: str = "",
    ):
        """Save one input image and one CSV row when recording is enabled."""
        if not self.config.enabled:
            return

        try:
            root = Path(self.config.root_dir)
            images_dir = root / "images"
            images_dir.mkdir(parents=True, exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            image_path = ""

            if image is not None:
                safe_mode = self._safe_name(mode)
                image_path = str(images_dir / f"{timestamp}_{safe_mode}.jpg")
                image.convert("RGB").save(image_path, format="JPEG", quality=90)

            csv_path = root / self.config.csv_name
            is_new = not csv_path.exists()
            with csv_path.open("a", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=self.FIELD_NAMES)
                if is_new:
                    writer.writeheader()
                writer.writerow(
                    {
                        "timestamp": timestamp,
                        "mode": mode,
                        "model_id": model_id,
                        "target_label": target_label,
                        "detected_label": detected_label,
                        "bbox_xyxy": self._stringify(bbox_xyxy),
                        "image_path": image_path,
                        "raw_response": raw_response,
                        "parsed_point": self._stringify(parsed_point),
                        "yaw_deg": self._stringify(yaw_deg),
                        "orientation_rpy_deg": self._stringify(orientation_rpy_deg),
                        "success": str(bool(success)),
                        "error": error,
                    }
                )
        except Exception as exc:
            if self.logger is not None:
                self.logger.warn(f"VLM inference history recording failed: {exc}")

    @staticmethod
    def _safe_name(value: str) -> str:
        return "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in value)

    @staticmethod
    def _stringify(value: Any) -> str:
        if value is None:
            return ""
        return str(value)

