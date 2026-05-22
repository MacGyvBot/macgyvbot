"""Task queue step contract."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable


@dataclass(frozen=True)
class TaskStep:
    """One executable unit in a robot task queue."""

    name: str
    execute: Callable[[], bool]
    retry_on_pause: bool = True
