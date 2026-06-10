"""User-facing chat messages for abnormal runtime events.

This module stays GUI-side: it maps existing ROS topic/status payloads to short
operator-facing chat text without changing task recovery or robot motion logic.
"""

from __future__ import annotations

import re


HAND_NOT_FOUND_MESSAGE = "손이 인식되지 않았습니다. 움직여서 카메라에 나오게 하세요!"
HAND_DETECTED_MESSAGE = "손이 인식되었습니다."
BRING_WAIT_HANDOFF_MESSAGE = "공구를 받아주세요."
BRING_DONE_MESSAGE = "공구 전달을 완료했습니다."
BRING_RETURNED_HOME_MESSAGE = "공구를 원래 위치에 두고 Home으로 복귀했습니다."
RETURN_SEARCH_HAND_MESSAGE = "손 위치를 탐색합니다."
RETURN_WAIT_TOOL_MESSAGE = "공구를 그리퍼 위치에 둬주세요."
RETURN_STORE_START_MESSAGE = "정리 중입니다."
RETURN_DONE_MESSAGE = "정리를 완료했습니다."
PARSE_FAILED_MESSAGE = "명령을 이해하지 못했습니다. 다시 말씀해주세요."
TOOL_DROPPED_MESSAGE = "공구를 떨어트렸습니다. inspection하여 다시 찾습니다."
GRASP_RETRY_MESSAGE = "공구를 잡지 못했습니다. 다시 시도합니다."
GRASP_FAILED_MESSAGE = "공구를 잡지 못했습니다. 다시 확인이 필요합니다."

_HAND_NOT_FOUND_REASONS = {
    "handoff_search_failed",
    "hand_target_not_found",
    "hand_position_unavailable",
}
_PARSE_FAILURE_REASONS = {
    "llm_failed",
    "local_parser_failed",
    "unknown_action",
    "unknown_tool",
    "low_confidence",
}
_GRASP_FAILURE_REASONS = {
    "robot_grasp_failed",
    "grasp_failed",
    "gripper_grasp_failed",
}
_GRASP_ATTEMPT_RE = re.compile(r"(\d+)\s*/\s*(\d+)")


def command_feedback_chat(status, reason, message=""):
    """Return chat guidance for command parser feedback."""
    normalized_status = _normalize(status)
    normalized_reason = _normalize(reason)
    if normalized_status != "rejected":
        return ""
    if normalized_reason in _PARSE_FAILURE_REASONS:
        return PARSE_FAILED_MESSAGE
    if not str(message or "").strip():
        return PARSE_FAILED_MESSAGE
    return ""


def robot_status_chat(status, reason="", message=""):
    """Return abnormal-event chat guidance for robot task status."""
    normalized_status = _normalize(status)
    normalized_reason = _normalize(reason)
    raw_message = str(message or "").strip()

    if normalized_status == "tool_dropped":
        return TOOL_DROPPED_MESSAGE
    if normalized_reason in _HAND_NOT_FOUND_REASONS:
        return HAND_NOT_FOUND_MESSAGE
    if normalized_reason in _GRASP_FAILURE_REASONS:
        return GRASP_FAILED_MESSAGE
    if normalized_status == "grasping" and _is_retry_attempt(raw_message):
        return GRASP_RETRY_MESSAGE
    return ""


def normal_robot_status_chat(status, action="", reason=""):
    """Return minimal user-facing chat for ordinary task progress."""
    normalized_status = _normalize(status)
    normalized_action = _normalize(action)
    normalized_reason = _normalize(reason)

    if normalized_status == "handoff_inspection_pending":
        return (
            "사용자의 손을 인식하지 못했습니다. "
            "다시 인식할까요, 복귀할까요?"
        )

    if normalized_action == "bring":
        if normalized_status == "searching_hand":
            return BRING_WAIT_HANDOFF_MESSAGE
        if normalized_status == "waiting_handoff":
            return ""
        if normalized_status in {"done", "completed", "success"}:
            return BRING_DONE_MESSAGE
        if normalized_status == "returned":
            return BRING_RETURNED_HOME_MESSAGE
        return ""

    if normalized_action == "return":
        if normalized_status == "moving_return_grasp_pose":
            return RETURN_SEARCH_HAND_MESSAGE
        if normalized_status == "checking_return_target":
            if normalized_reason in _HAND_NOT_FOUND_REASONS:
                return ""
            return RETURN_WAIT_TOOL_MESSAGE
        if normalized_status == "grasp_success":
            return RETURN_STORE_START_MESSAGE
        if normalized_status in {"done", "returned", "completed", "success"}:
            return RETURN_DONE_MESSAGE
        return ""

    return ""


def hand_detection_chat(previous_hand_present, hand_present):
    """Do not chat for raw hand topic transitions.

    HumanGraspResult is a perception stream and may toggle outside handoff
    inspection. User-facing hand messages are emitted from task status instead.
    """
    _ = previous_hand_present, hand_present
    return ""


def tool_drop_chat(event):
    """Return chat text for gripper hold-monitor events."""
    if _normalize(event) == "tool_dropped":
        return TOOL_DROPPED_MESSAGE
    return ""


def _is_retry_attempt(message):
    match = _GRASP_ATTEMPT_RE.search(message)
    if match is None:
        return False

    try:
        attempt = int(match.group(1))
    except ValueError:
        return False
    return attempt > 1


def _normalize(value):
    return str(value or "").strip().lower()
