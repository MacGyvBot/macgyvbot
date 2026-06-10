"""YOLO detector construction and model path resolution."""

from macgyvbot_config.models import YOLO_MODEL_NAME
from macgyvbot_perception.model_paths import resolve_weight_path

DEFAULT_MODEL_PATH = YOLO_MODEL_NAME


def resolve_model_path(model_name):
    return str(resolve_weight_path(model_name, default_model_name=DEFAULT_MODEL_PATH))


class YoloDetector:
    """Small wrapper around Ultralytics YOLO to keep node wiring thin."""

    def __init__(self, model_name, confidence_threshold=0.20):
        from ultralytics import YOLO

        self.model_path = resolve_model_path(model_name)
        self.confidence_threshold = float(confidence_threshold)
        self.model = YOLO(self.model_path)

    @property
    def names(self):
        return self.model.names

    def detect(self, image):
        return self.model(image, conf=self.confidence_threshold, verbose=False)
