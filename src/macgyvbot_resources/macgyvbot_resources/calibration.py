"""Calibration resource path helpers."""

from __future__ import annotations

from pathlib import Path


def resolve_calibration_file(filename: str) -> Path:
    """Resolve a calibration file from installed resources or source checkout."""
    try:
        from ament_index_python.packages import get_package_share_directory

        package_share = Path(get_package_share_directory("macgyvbot_resources"))
        candidate = package_share / "calibration" / filename
        if candidate.exists():
            return candidate
    except Exception:
        pass

    package_root = Path(__file__).resolve().parents[1]
    return package_root / "calibration" / filename
