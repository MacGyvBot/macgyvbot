"""Resolve a user-held return target from hand observation data."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

import rclpy

from macgyvbot_config.handoff import OBSERVATION_TIMEOUT_SEC
from macgyvbot_config.timing import SEQUENCE_WAIT_POLL_SEC
from macgyvbot_config.robot import BASE_FRAME
from macgyvbot_manipulation.handover_targeting import (
    TargetCandidate,
    candidate_from_grasp_result,
)
from macgyvbot_task.application.logging_utils import log_warn


RETURN_SOURCE_HAND = "hand"
RETURN_SOURCE_NONE = "none"


@dataclass(frozen=True)
class ReturnTarget:
    """Selected source for a return command target."""

    source: str
    tool_name: str
    hand_candidate: Optional[TargetCandidate] = None
    reason: str = ""


class ReturnTargetResolver:
    """Choose a hand-held receive target from observation data."""

    def __init__(
        self,
        state,
        wait_fn,
        timeout_sec=OBSERVATION_TIMEOUT_SEC,
        poll_sec=SEQUENCE_WAIT_POLL_SEC,
    ):
        self.state = state
        self.wait_fn = wait_fn
        self.timeout_sec = timeout_sec
        self.poll_sec = poll_sec

    def resolve(self, requested_tool, logger):
        tool_name = requested_tool or "unknown"
        deadline = time.monotonic() + max(0.0, float(self.timeout_sec))
        saw_hand_without_position = False

        while rclpy.ok() and time.monotonic() < deadline:
            result = self.state.last_grasp_result or {}
            if self._hand_present(result):
                candidate = self._hand_candidate_from_result(result)
                if candidate is not None:
                    detected_tool = result.get("tool_label") or "unknown"
                    return ReturnTarget(
                        source=RETURN_SOURCE_HAND,
                        tool_name=self._resolve_tool_name(tool_name, detected_tool),
                        hand_candidate=candidate,
                    )
                saw_hand_without_position = True

            self.wait_fn(self.poll_sec)

        if saw_hand_without_position:
            log_warn(
                logger,
                "hand detected without usable base position",
                step="target_resolve",
                event="fail",
                reason="hand_position_unavailable",
            )
            return ReturnTarget(
                source=RETURN_SOURCE_NONE,
                tool_name=tool_name,
                reason="hand_position_unavailable",
            )

        return ReturnTarget(
            source=RETURN_SOURCE_NONE,
            tool_name=tool_name,
            reason="hand_target_not_found",
        )

    @staticmethod
    def _hand_present(result):
        if bool(result.get("hand_present", False)):
            return True
        return result.get("hand_pixel") is not None

    @staticmethod
    def _hand_candidate_from_result(result):
        return candidate_from_grasp_result(
            result,
            source="return_hand_presence",
            default_frame=BASE_FRAME,
        )

    @staticmethod
    def _resolve_tool_name(requested_tool, detected_tool):
        if detected_tool and detected_tool != "unknown":
            return detected_tool
        if requested_tool and requested_tool != "unknown":
            return requested_tool
        return "unknown"
