"""Resolve whether a returned tool should be received from hand or floor."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

import rclpy

from macgyvbot_config.handoff import OBSERVATION_TIMEOUT_SEC
from macgyvbot_config.return_flow import RETURN_FLOOR_MAX_BASE_Z_M
from macgyvbot_config.robot import BASE_FRAME
from macgyvbot_config.timing import SEQUENCE_WAIT_POLL_SEC
from macgyvbot_domain.target_models import PickTarget
from macgyvbot_manipulation.handover_targeting import TargetCandidate


RETURN_SOURCE_HAND = "hand"
RETURN_SOURCE_FLOOR = "floor"
RETURN_SOURCE_NONE = "none"
RETURN_HAND_PRESENCE_GRACE_SEC = 0.5


@dataclass(frozen=True)
class ReturnTarget:
    """Selected source for a return command target."""

    source: str
    tool_name: str
    hand_candidate: Optional[TargetCandidate] = None
    floor_target: Optional[PickTarget] = None
    reason: str = ""


class ReturnTargetResolver:
    """Choose hand-held receive or floor pickup from observation data."""

    def __init__(
        self,
        state,
        pick_target_resolver,
        wait_fn,
        timeout_sec=OBSERVATION_TIMEOUT_SEC,
        poll_sec=SEQUENCE_WAIT_POLL_SEC,
    ):
        self.state = state
        self.pick_target_resolver = pick_target_resolver
        self.wait_fn = wait_fn
        self.timeout_sec = timeout_sec
        self.poll_sec = poll_sec

    def resolve(self, requested_tool, logger):
        tool_name = requested_tool or "unknown"
        start_time = time.monotonic()
        deadline = start_time + max(0.0, float(self.timeout_sec))
        floor_allowed_at = start_time + RETURN_HAND_PRESENCE_GRACE_SEC
        saw_hand_without_position = False
        last_floor_reason = "target_not_observed"

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

            floor_allowed = bool(result) or time.monotonic() >= floor_allowed_at
            if floor_allowed and not saw_hand_without_position:
                floor_target = self._floor_target(tool_name)
                if floor_target.found:
                    if self._is_floor_height(floor_target, logger):
                        return ReturnTarget(
                            source=RETURN_SOURCE_FLOOR,
                            tool_name=floor_target.label or tool_name,
                            floor_target=floor_target,
                        )
                    last_floor_reason = "floor_target_z_too_high"
                    self.wait_fn(self.poll_sec)
                    continue
                last_floor_reason = floor_target.reason or last_floor_reason

            self.wait_fn(self.poll_sec)

        if saw_hand_without_position:
            logger.warn("손은 감지했지만 base 좌표를 계산하지 못했습니다.")
            return ReturnTarget(
                source=RETURN_SOURCE_NONE,
                tool_name=tool_name,
                reason="hand_position_unavailable",
            )

        return ReturnTarget(
            source=RETURN_SOURCE_NONE,
            tool_name=tool_name,
            reason=last_floor_reason or "return_target_not_found",
        )

    def _floor_target(self, tool_name):
        if (
            self.state.color_image is None
            or self.state.depth_image is None
            or self.state.intrinsics is None
        ):
            return PickTarget(
                found=False,
                label=tool_name,
                pixel=None,
                base_xyz=None,
                depth_m=None,
                yaw_deg=None,
                reason="camera_state_unavailable",
            )

        results = self.pick_target_resolver.detector.detect(self.state.color_image)
        if not results:
            return PickTarget(
                found=False,
                label=tool_name,
                pixel=None,
                base_xyz=None,
                depth_m=None,
                yaw_deg=None,
                reason="detector_result_unavailable",
            )

        return self.pick_target_resolver.target_from_boxes(
            results[0].boxes,
            tool_name,
            self.state.color_image,
            self.state.depth_image,
            self.state.intrinsics,
        )

    @staticmethod
    def _is_floor_height(target, logger):
        if target.base_xyz is None or len(target.base_xyz) < 3:
            return False

        base_z = float(target.base_xyz[2])
        if base_z < RETURN_FLOOR_MAX_BASE_Z_M:
            return True

        logger.warn(
            "공구만 인식되었지만 floor 후보 z가 높아 바닥 공구로 보지 않습니다: "
            f"base_z={base_z:.3f}m, "
            f"threshold={RETURN_FLOOR_MAX_BASE_Z_M:.3f}m"
        )
        return False

    @staticmethod
    def _hand_present(result):
        if bool(result.get("hand_present", False)):
            return True
        return result.get("hand_pixel") is not None

    @staticmethod
    def _hand_candidate_from_result(result):
        position = result.get("position")
        if not isinstance(position, dict):
            return None
        if not all(key in position for key in ("x", "y", "z")):
            return None

        return TargetCandidate(
            found=True,
            x=float(position["x"]),
            y=float(position["y"]),
            z=float(position["z"]),
            frame_id=str(position.get("frame_id", BASE_FRAME)),
            source="return_hand_presence",
            observed_at_sec=float(
                result.get(
                    "position_observed_monotonic_sec",
                    result.get("_received_monotonic_sec", time.monotonic()),
                )
            ),
        )

    @staticmethod
    def _resolve_tool_name(requested_tool, detected_tool):
        if detected_tool and detected_tool != "unknown":
            return detected_tool
        if requested_tool and requested_tool != "unknown":
            return requested_tool
        return "unknown"
