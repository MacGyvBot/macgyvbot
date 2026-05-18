"""Shared model path resolution helpers."""

from __future__ import annotations

from pathlib import Path


def _package_share(package_name):
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory(package_name))
    except Exception:
        return None


def resolve_weight_path(model_name, default_model_name=None):
    """Resolve a model file from package share, cwd, and source tree weights."""
    path = Path(model_name).expanduser()
    if path.exists() or path.is_absolute():
        return path

    package_root = Path(__file__).resolve().parents[1]
    workspace_src = Path(__file__).resolve().parents[3]
    resources_root = workspace_src / "macgyvbot_resources"
    cwd = Path.cwd()
    package_shares = [
        share
        for share in (_package_share("macgyvbot_resources"),)
        if share is not None
    ]

    candidates = [
        *(
            candidate
            for share in package_shares
            for candidate in (share / path, share / "weights" / path)
        ),
        cwd / "weights" / path,
        cwd / path,
        resources_root / "weights" / path,
        package_root / "weights" / path,
        package_root / path,
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate

    if default_model_name is not None and str(model_name) == "yolo11_best.pt":
        roots = [*package_shares, resources_root, cwd, package_root]
        for root in roots:
            corrected_path = root / "weights" / default_model_name
            if corrected_path.exists():
                print(
                    "WARNING: yolo11_best.pt not found. "
                    f"Using {default_model_name}."
                )
                return corrected_path

    return model_name
