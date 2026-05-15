"""Shared model path resolution helpers."""

from __future__ import annotations

from pathlib import Path


def resolve_weight_path(model_name, default_model_name=None):
    """Resolve a model file from package share, cwd, and source tree weights."""
    path = Path(model_name).expanduser()
    if path.exists() or path.is_absolute():
        return path

    package_root = Path(__file__).resolve().parents[1]
    project_root = Path(__file__).resolve().parents[2]
    cwd = Path.cwd()
    package_share = None
    try:
        from ament_index_python.packages import get_package_share_directory

        package_share = Path(get_package_share_directory("macgyvbot"))
    except Exception:
        pass

    candidates = [
        *(
            (package_share / path, package_share / "weights" / path)
            if package_share is not None
            else ()
        ),
        cwd / "weights" / path,
        cwd / path,
        project_root / "weights" / path,
        package_root / "weights" / path,
        package_root / path,
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate

    if default_model_name is not None and str(model_name) == "yolo11_best.pt":
        roots = [project_root, cwd, package_root]
        if package_share is not None:
            roots.insert(0, package_share)
        for root in roots:
            corrected_path = root / "weights" / default_model_name
            if corrected_path.exists():
                print(
                    "WARNING: yolo11_best.pt not found. "
                    f"Using {default_model_name}."
                )
                return corrected_path

    return model_name
