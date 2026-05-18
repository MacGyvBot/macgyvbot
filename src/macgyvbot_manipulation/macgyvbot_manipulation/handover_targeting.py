"""Reusable helpers for handover target search and movement."""

from __future__ import annotations

import time
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
from typing import Optional

from moveit.core.robot_state import RobotState

from macgyvbot_config.handoff import (
    HAND_POSE_STABLE_TOLERANCE_M,
    HAND_POSE_WAIT_AFTER_DETECTION_SEC,
    HANDOVER_REPLAN_MAX_ATTEMPTS,
    HANDOVER_REPLAN_X_STEP_M,
    HANDOVER_SEARCH_TIMEOUT_SEC,
)
from macgyvbot_config.robot import (
    BASE_FRAME,
    EE_LINK,
    OBSERVATION_JOINTS,
)
from macgyvbot_config.timing import SEQUENCE_WAIT_POLL_SEC
from macgyvbot_manipulation.robot_pose import make_safe_pose
from macgyvbot_manipulation.robot_safezone import SAFE_Z_MIN


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
    observed_at_sec: float = 0.0


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
    return True, SearchStartPose(t.x, t.y, t.z)


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
    """Observe a hand target only after its position is stable long enough."""
    return _observe_stable_candidate(
        state,
        logger,
        timeout_sec,
        poll_sec,
    )


def _candidate_from_grasp_result(result: dict) -> Optional[TargetCandidate]:
    position = result.get("position")
    if isinstance(position, dict) and all(k in position for k in ("x", "y", "z")):
        frame_id = str(position.get("frame_id", "world"))
        observed_at_sec = float(
            result.get(
                "position_observed_monotonic_sec",
                result.get("_received_monotonic_sec", time.monotonic()),
            )
        )
        return TargetCandidate(
            found=True,
            x=float(position["x"]),
            y=float(position["y"]),
            z=float(position["z"]),
            frame_id=frame_id,
            source="observation_position",
            observed_at_sec=observed_at_sec,
        )
    return None


def _candidate_distance(a: TargetCandidate, b: TargetCandidate) -> float:
    return max(abs(a.x - b.x), abs(a.y - b.y), abs(a.z - b.z))


def _observe_stable_candidate(
    state,
    logger,
    timeout_sec: float,
    poll_sec: float,
    stable_duration_sec: float = HAND_POSE_WAIT_AFTER_DETECTION_SEC,
    stable_tolerance_m: float = HAND_POSE_STABLE_TOLERANCE_M,
) -> TargetCandidate:
    deadline = time.monotonic() + max(0.0, float(timeout_sec))
    sleep_step = min(max(0.01, float(poll_sec)), 0.1)
    stable_anchor = None
    stable_since_observed_at = None
    last_observed_at = None
    last_log_time = 0.0

    while time.monotonic() < deadline:
        result = getattr(state, "last_grasp_result", None) or {}
        candidate = _candidate_from_grasp_result(result)
        if candidate is None:
            stable_anchor = None
            stable_since_observed_at = None
            last_observed_at = None
            time.sleep(sleep_step)
            continue

        if candidate.observed_at_sec == last_observed_at:
            time.sleep(sleep_step)
            continue
        last_observed_at = candidate.observed_at_sec

        if (
            stable_anchor is None
            or candidate.frame_id != stable_anchor.frame_id
            or _candidate_distance(candidate, stable_anchor) > stable_tolerance_m
        ):
            stable_anchor = candidate
            stable_since_observed_at = candidate.observed_at_sec
            logger.info(
                "사용자 손 위치 안정화 시작: "
                f"xyz=({candidate.x:.3f},{candidate.y:.3f},{candidate.z:.3f}), "
                f"tol={stable_tolerance_m:.3f}m"
            )
            time.sleep(sleep_step)
            continue

        stable_elapsed = candidate.observed_at_sec - stable_since_observed_at
        if stable_elapsed >= stable_duration_sec:
            logger.info(
                "사용자 손 위치 안정화 완료: "
                f"duration={stable_elapsed:.2f}s, "
                f"xyz=({candidate.x:.3f},{candidate.y:.3f},{candidate.z:.3f})"
            )
            return candidate

        now = time.monotonic()
        if now - last_log_time >= 0.7:
            logger.info(
                "사용자 손 위치 안정화 대기: "
                f"{stable_elapsed:.2f}/{stable_duration_sec:.2f}s"
            )
            last_log_time = now
        time.sleep(sleep_step)

    logger.warn(
        "관측 기반 목표 탐색 타임아웃: "
        f"timeout={timeout_sec:.1f}s, stable_required={stable_duration_sec:.1f}s"
    )
    return TargetCandidate(
        found=False,
        x=0.0,
        y=0.0,
        z=0.0,
        frame_id="world",
        source="observation_timeout",
        observed_at_sec=time.monotonic(),
    )


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
        return (
            False,
            SearchStartPose(candidate.x, candidate.y, candidate.z),
            "target_not_found",
        )

    if candidate.frame_id not in ("world", BASE_FRAME):
        return (
            False,
            SearchStartPose(candidate.x, candidate.y, candidate.z),
            "unsupported_frame",
        )

    target_x = candidate.x + float(x_offset_m)
    target_y = candidate.y
    target_z = max(SAFE_Z_MIN + 0.15, candidate.z + float(z_offset_m))
    attempts = [(target_x, target_y, target_z)]
    retry_count = max(0, int(HANDOVER_REPLAN_MAX_ATTEMPTS) - 1)
    for index in range(1, retry_count + 1):
        y_ratio = max(0.0, 1.0 - index / float(max(1, retry_count)))
        attempts.append(
            (
                target_x - HANDOVER_REPLAN_X_STEP_M * index,
                target_y * y_ratio,
                target_z,
            )
        )

    last_pose = SearchStartPose(*attempts[0])
    for attempt_index, (target_x, target_y, target_z) in enumerate(attempts, start=1):
        logger.info(
            "사용자 손 위치 전달 플래닝 시도: "
            f"{attempt_index}/{len(attempts)}, "
            f"target=({target_x:.3f},{target_y:.3f},{target_z:.3f})"
        )
        pose_goal = make_safe_pose(target_x, target_y, target_z, ori, logger)
        ok = motion.plan_and_execute(
            logger,
            pose_goal=pose_goal,
        )
        last_pose = SearchStartPose(
            float(pose_goal.pose.position.x),
            float(pose_goal.pose.position.y),
            float(pose_goal.pose.position.z),
        )
        if ok:
            return True, last_pose, ""

        if attempt_index < len(attempts):
            logger.warn(
                "사용자 손 위치 전달 플래닝 실패. "
                "x를 줄이고 y를 0에 가깝게 보정해 재시도합니다."
            )

    return False, last_pose, "target_move_failed"
