"""Calibration resource path helpers."""

from __future__ import annotations

from pathlib import Path

from macgyvbot_resources.resources import resolve_resource_file


def resolve_calibration_file(filename: str) -> Path:
    """Resolve a calibration file from installed resources or source checkout."""
    return resolve_resource_file("calibration", filename)
