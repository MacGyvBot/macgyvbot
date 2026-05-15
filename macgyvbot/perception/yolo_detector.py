"""YOLO detector construction and model path resolution."""

from macgyvbot.perception.model_paths import resolve_weight_path

DEFAULT_MODEL_PATH = "yolov11_best.pt"


def resolve_model_path(model_name):
    return str(resolve_weight_path(model_name, default_model_name=DEFAULT_MODEL_PATH))


class YoloDetector:
    """Small wrapper around Ultralytics YOLO to keep node wiring thin."""

    def __init__(self, model_name):
        from ultralytics import YOLO

        self.model = YOLO(resolve_model_path(model_name))

    @property
    def names(self):
        return self.model.names

    def detect(self, image):
        return self.model(image, verbose=False)
