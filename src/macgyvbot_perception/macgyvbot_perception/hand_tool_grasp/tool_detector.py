from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional, Tuple

from macgyvbot_config.models import YOLO_CONFIDENCE_THRESHOLD, YOLO_MODEL_NAME
from macgyvbot_perception.model_paths import (
    resolve_weight_path,
)

Rect = Tuple[int, int, int, int]

DEFAULT_MODEL_PATH = YOLO_MODEL_NAME
DEFAULT_TOOL_CLASSES = (
    "hammer",
    "pliers",
    "screwdriver",
    "tape_measure",
    "wrench",
)
DEFAULT_GRASP_POINT_CLASSES = (
    "grasp_point",
    "grasp-point",
    "grasp point",
    "grasp",
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
        grasp_point_classes: Iterable[str] = DEFAULT_GRASP_POINT_CLASSES,
        confidence_threshold: float = YOLO_CONFIDENCE_THRESHOLD,
        image_size: int = 640,
    ) -> None:
        from ultralytics import YOLO

        resolved_model_path = self._resolve_model_path(model_path)
        self.model_path = str(resolved_model_path)
        self.target_classes = {
            self._normalize_label(name)
            for name in target_classes
            if name.strip()
        }
        self.grasp_point_classes = {
            self._normalize_label(name)
            for name in grasp_point_classes
            if name.strip()
        }
        self.confidence_threshold = confidence_threshold
        self.image_size = image_size
        self.last_grasp_point_detection: Optional[ToolDetection] = None
        self.model = YOLO(str(resolved_model_path))

    def detect(self, frame, target_label: str | None = None) -> Optional[ToolDetection]:
        """Return highest-confidence matching tool detection, or None."""
        self.last_grasp_point_detection = None
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
        best_grasp_point_confidence = -1.0
        names = result.names

        requested_label = self._normalize_label(target_label)

        for box in result.boxes:
            confidence = float(box.conf[0])
            if confidence < self.confidence_threshold:
                continue

            class_id = int(box.cls[0])
            label = self._normalize_label(names.get(class_id, class_id))
            x1, y1, x2, y2 = box.xyxy[0].tolist()

            if label in self.grasp_point_classes:
                if confidence > best_grasp_point_confidence:
                    self.last_grasp_point_detection = ToolDetection(
                        roi=(int(x1), int(y1), int(x2), int(y2)),
                        label=label,
                        confidence=confidence,
                    )
                    best_grasp_point_confidence = confidence
                continue

            if self.target_classes and label not in self.target_classes:
                continue

            if requested_label and label != requested_label:
                continue

            if confidence <= best_confidence:
                continue

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
    def _normalize_label(label) -> str:
        return str(label or "").strip().lower().replace(" ", "_").replace("-", "_")

    @staticmethod
    def _resolve_model_path(model_path: str) -> Path | str:
        return resolve_weight_path(model_path, default_model_name=DEFAULT_MODEL_PATH)
