from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional, Tuple

from macgyvbot_perception.model_paths import (
    resolve_weight_path,
)

Rect = Tuple[int, int, int, int]

DEFAULT_MODEL_PATH = "yolov11_best.pt"
DEFAULT_TOOL_CLASSES = (
    "drill",
    "hammer",
    "pliers",
    "screwdriver",
    "tape_measure",
    "wrench",
)


@dataclass(frozen=True)
class ToolDetection:
    roi: Rect
    label: str
    confidence: float


class ToolDetector:
    """YOLO wrapper that returns the best detected tool bbox as a grasp ROI."""

    def __init__(
        self,
        model_path: str = DEFAULT_MODEL_PATH,
        target_classes: Iterable[str] = DEFAULT_TOOL_CLASSES,
        confidence_threshold: float = 0.20,
        image_size: int = 640,
    ) -> None:
        from ultralytics import YOLO

        resolved_model_path = self._resolve_model_path(model_path)
        self.model_path = str(resolved_model_path)
        self.target_classes = {
            name.strip().lower()
            for name in target_classes
            if name.strip()
        }
        self.confidence_threshold = confidence_threshold
        self.image_size = image_size
        self.model = YOLO(str(resolved_model_path))

    def detect(self, frame) -> Optional[ToolDetection]:
        """Return highest-confidence target tool detection, or None."""
        results = self.model.predict(
            source=frame,
            imgsz=self.image_size,
            conf=self.confidence_threshold,
            verbose=False,
        )
        if not results:
            return None

        result = results[0]
        if result.boxes is None:
            return None

        best_detection: Optional[ToolDetection] = None
        best_confidence = -1.0
        names = result.names

        for box in result.boxes:
            confidence = float(box.conf[0])
            class_id = int(box.cls[0])
            label = str(names.get(class_id, class_id)).lower()

            if self.target_classes and label not in self.target_classes:
                continue

            if confidence <= best_confidence:
                continue

            x1, y1, x2, y2 = box.xyxy[0].tolist()
            best_detection = ToolDetection(
                roi=(int(x1), int(y1), int(x2), int(y2)),
                label=label,
                confidence=confidence,
            )
            best_confidence = confidence

        return best_detection

    @staticmethod
    def is_builtin_model(model_path: str) -> bool:
        return Path(model_path).suffix == ".pt" and not Path(model_path).exists()

    @staticmethod
    def _resolve_model_path(model_path: str) -> Path | str:
        return resolve_weight_path(model_path, default_model_name=DEFAULT_MODEL_PATH)
