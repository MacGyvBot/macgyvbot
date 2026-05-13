"""Reusable helpers for handover target search and movement."""

from __future__ import annotations

import time
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
from typing import Optional

from moveit.core.robot_state import RobotState

from macgyvbot.config.config import (
    BASE_FRAME,
    EE_LINK,
    HANDOVER_SEARCH_TIMEOUT_SEC,
    OBSERVATION_JOINTS,
    SEQUENCE_WAIT_POLL_SEC,
)
from macgyvbot.util.macgyvbot_main.model_control.robot_pose import make_safe_pose
from macgyvbot.util.macgyvbot_main.model_control.robot_safezone import (
    clamp_to_safe_workspace,
)


@dataclass(frozen=True)
class SearchStartPose:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class TargetCandidate:
    found: bool
    x: float
    y: float
    z: float
    frame_id: str
    source: str


_SEARCH_EXECUTOR = ThreadPoolExecutor(max_workers=2)


def move_to_observation_pose(
    motion,
    robot,
    logger,
) -> tuple[bool, SearchStartPose]:
    """Move robot to fixed observation joint pose for handover search."""
    robot_model = robot.get_robot_model()
    state_goal = RobotState(robot_model)
    state_goal.joint_positions = OBSERVATION_JOINTS
    state_goal.update()

    logger.info(
        "관찰 자세 이동(joint): "
        "j1=0, j2=-40, j3=55, j4=0, j5=120, j6=90"
    )
    ok = motion.plan_and_execute(logger, state_goal=state_goal)
    if not ok:
        logger.warn("관찰 자세 이동 실패")
        return False, SearchStartPose(0.0, 0.0, 0.0)

    # Keep contract compatible for existing callers/logging.
    with robot.get_planning_scene_monitor().read_only() as scene:
        current_state = scene.current_state
        t = current_state.get_pose(EE_LINK).position
    safe_x, safe_y, safe_z = clamp_to_safe_workspace(t.x, t.y, t.z, logger)
    return True, SearchStartPose(safe_x, safe_y, safe_z)


def start_async_observation_search(
    state,
    logger,
    timeout_sec: float = HANDOVER_SEARCH_TIMEOUT_SEC,
    poll_sec: float = SEQUENCE_WAIT_POLL_SEC,
) -> Future:
    """Start an async, observation-only target search from grasp results."""
    return _SEARCH_EXECUTOR.submit(
        observe_target_candidate,
        state,
        logger,
        timeout_sec,
        poll_sec,
    )


def observe_target_candidate(
    state,
    logger,
    timeout_sec: float = HANDOVER_SEARCH_TIMEOUT_SEC,
    poll_sec: float = SEQUENCE_WAIT_POLL_SEC,
) -> TargetCandidate:
    """Observe target candidate from latest grasp result without moving robot."""
    deadline = time.monotonic() + max(0.0, float(timeout_sec))
    sleep_step = min(max(0.01, float(poll_sec)), 0.1)

    while time.monotonic() < deadline:
        result = getattr(state, "last_grasp_result", None) or {}
        candidate = _candidate_from_grasp_result(result)
        if candidate is not None:
            return candidate
        time.sleep(sleep_step)

    logger.warn(
        "관측 기반 목표 탐색 타임아웃: "
        f"timeout={timeout_sec:.1f}s"
    )
    return TargetCandidate(
        found=False,
        x=0.0,
        y=0.0,
        z=0.0,
        frame_id="world",
        source="observation_timeout",
    )


def _candidate_from_grasp_result(result: dict) -> Optional[TargetCandidate]:
    position = result.get("position")
    if isinstance(position, dict) and all(k in position for k in ("x", "y", "z")):
        frame_id = str(position.get("frame_id", "world"))
        return TargetCandidate(
            found=True,
            x=float(position["x"]),
            y=float(position["y"]),
            z=float(position["z"]),
            frame_id=frame_id,
            source="observation_position",
        )
    return None


def move_to_candidate_with_offset(
    motion,
    candidate: TargetCandidate,
    ori,
    logger,
    x_offset_m: float,
    z_offset_m: float,
) -> tuple[bool, SearchStartPose, str]:
    """
    Move to the final handover pose derived from a candidate.
    Returns (ok, pose, reason_if_failed).
    """
    if not candidate.found:
        return False, SearchStartPose(candidate.x, candidate.y, candidate.z), "target_not_found"

    if candidate.frame_id not in ("world", BASE_FRAME):
        return False, SearchStartPose(candidate.x, candidate.y, candidate.z), "unsupported_frame"

    target_x = candidate.x + float(x_offset_m)
    target_y = candidate.y
    target_z = candidate.z + float(z_offset_m)
    safe_x, safe_y, safe_z = clamp_to_safe_workspace(target_x, target_y, target_z, logger)

    ok = motion.plan_and_execute(
        logger,
        pose_goal=make_safe_pose(safe_x, safe_y, safe_z, ori, logger),
    )
    if not ok:
        return False, SearchStartPose(safe_x, safe_y, safe_z), "target_move_failed"
    return True, SearchStartPose(safe_x, safe_y, safe_z), ""
