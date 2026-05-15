"""YOLO detector construction and model path resolution."""

from pathlib import Path

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)

DEFAULT_MODEL_PATH = "yolov11_best.pt"


def resolve_model_path(model_name):
    path = Path(model_name).expanduser()
    if path.exists() or path.is_absolute():
        return str(path)

    package_root = Path(__file__).resolve().parents[3]
    project_root = Path(__file__).resolve().parents[4]
    cwd = Path.cwd()
    try:
        package_share = Path(get_package_share_directory("macgyvbot"))
    except PackageNotFoundError:
        package_share = None

    candidates = [
        *((package_share / "weights" / path,) if package_share is not None else ()),
        cwd / "weights" / path,
        cwd / path,
        project_root / "weights" / path,
        package_root / "weights" / path,
        package_root / path,
    ]

    for candidate in candidates:
        if candidate.exists():
            return str(candidate)

    if model_name == "yolo11_best.pt":
        roots = [project_root, cwd, package_root]
        if package_share is not None:
            roots.insert(0, package_share)
        for root in roots:
            corrected_path = root / "weights" / DEFAULT_MODEL_PATH
            if corrected_path.exists():
                print(
                    "WARNING: yolo11_best.pt not found. "
                    f"Using {DEFAULT_MODEL_PATH}."
                )
                return str(corrected_path)

    return model_name


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
