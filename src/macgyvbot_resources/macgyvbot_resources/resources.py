"""Shared resource path resolution helpers."""

from __future__ import annotations

from pathlib import Path


def package_share(package_name: str = "macgyvbot_resources") -> Path | None:
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory(package_name))
    except Exception:
        return None


def package_root() -> Path:
    return Path(__file__).resolve().parents[1]


def resolve_resource_file(*parts: str | Path) -> Path:
    """Resolve a resource from installed share or source checkout."""
    path = Path(*parts).expanduser()
    if path.exists() or path.is_absolute():
        return path

    share = package_share()
    if share is not None:
        candidate = share / path
        if candidate.exists():
            return candidate

    return package_root() / path


def resolve_weight_file(model_name, default_model_name=None):
    """Resolve a weight file from package share, cwd, and source checkout."""
    path = Path(model_name).expanduser()
    if path.exists() or path.is_absolute():
        return path

    root = package_root()
    cwd = Path.cwd()
    shares = [share for share in (package_share(),) if share is not None]

    legacy_yolo_names = ("yolo11_best.pt", "yolov11_best.pt")
    if default_model_name is not None and str(model_name) in legacy_yolo_names:
        default_path = Path(default_model_name).expanduser()
        default_candidates = [
            *(
                candidate
                for share in shares
                for candidate in (share / default_path, share / "weights" / default_path)
            ),
            cwd / "weights" / default_path,
            cwd / default_path,
            root / "weights" / default_path,
            root / default_path,
        ]
        for candidate in default_candidates:
            if candidate.exists():
                print(
                    f"WARNING: {model_name} is a legacy YOLO name. "
                    f"Using {default_model_name}."
                )
                return candidate

    candidates = [
        *(
            candidate
            for share in shares
            for candidate in (share / path, share / "weights" / path)
        ),
        cwd / "weights" / path,
        cwd / path,
        root / "weights" / path,
        root / path,
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate

    return model_name


def resolve_env_file(filename: str = ".env") -> Path:
    """Resolve an optional environment file path under macgyvbot_resources."""
    path = Path(filename).expanduser()
    if path.exists() or path.is_absolute():
        return path

    candidates = []
    share = package_share()
    if share is not None:
        candidates.append(share / path)

    cwd = Path.cwd()
    root = package_root()
    candidates.extend(
        [
            cwd / "src" / "macgyvbot_resources" / path,
            cwd / "macgyvbot_resources" / path,
            root / path,
        ]
    )

    for candidate in candidates:
        if candidate.exists() and candidate.is_file():
            return candidate
    for candidate in candidates:
        if candidate.parent.exists():
            return candidate
    return root / path
