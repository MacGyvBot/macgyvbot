"""Shared timing helpers for robot motion workflows."""

from __future__ import annotations

import time

from macgyvbot_config.timing import SEQUENCE_WAIT_POLL_SEC


def cooperative_wait(duration_sec, poll_sec=SEQUENCE_WAIT_POLL_SEC):
    """Sleep in short ROS-aware intervals so control callbacks can interrupt."""
    import rclpy

    end_time = time.monotonic() + max(0.0, float(duration_sec))
    while rclpy.ok() and time.monotonic() < end_time:
        remaining = end_time - time.monotonic()
        time.sleep(min(float(poll_sec), max(0.0, remaining)))
