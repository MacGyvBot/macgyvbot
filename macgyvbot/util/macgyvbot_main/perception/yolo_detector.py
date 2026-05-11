"""YOLO detector construction and model path resolution."""

from pathlib import Path

try:
    from ament_index_python.packages import (
        PackageNotFoundError,
        get_package_share_directory,
    )
except ModuleNotFoundError:
    PackageNotFoundError = Exception
    get_package_share_directory = None


def resolve_model_path(model_name):
    path = Path(model_name).expanduser()
    if path.exists() or path.is_absolute():
        return str(path)

    package_share = _package_share_path()
    source_root = Path(__file__).resolve().parents[4]
    cwd = Path.cwd()
    candidates = [
        package_share / "weights" / path if package_share else None,
        package_share / path if package_share else None,
        cwd / "weights" / path,
        cwd / path,
        source_root / "weights" / path,
        source_root / path,
    ]

    for candidate in candidates:
        if candidate is not None and candidate.exists():
            return str(candidate)

    return model_name


def _package_share_path():
    if get_package_share_directory is None:
        return None

    try:
        return Path(get_package_share_directory("macgyvbot"))
    except PackageNotFoundError:
        return None


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
