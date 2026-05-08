"""YOLO detector construction and model path resolution."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO


def resolve_model_path(model_name):
    package_share = Path(get_package_share_directory("macgyvbot"))
    candidates = [
        package_share / "weights" / model_name,
        package_share / model_name,
        Path.cwd() / "weights" / model_name,
        Path.cwd() / model_name,
    ]

    for candidate in candidates:
        if candidate.exists():
            return str(candidate)

    return model_name


class YoloDetector:
    """Small wrapper around Ultralytics YOLO to keep node wiring thin."""

    def __init__(self, model_name):
        self.model = YOLO(resolve_model_path(model_name))

    @property
    def names(self):
        return self.model.names

    def detect(self, image):
        return self.model(image, verbose=False)
