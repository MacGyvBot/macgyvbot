"""One-line service log helpers for MacGyvBot packages."""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass, field
from pathlib import Path
import time
from typing import Any


DEFAULT_VALUE_LIMIT = 240


@dataclass(frozen=True)
class LogEvent:
    """Structured MacGyvBot service log event."""

    svc: str
    pipe: str
    step: str
    event: str
    target: str | None = None
    reason: str | None = None
    dur_ms: int | None = None
    file: str | None = None
    msg: str | None = None
    fields: dict[str, Any] = field(default_factory=dict)


def format_log_event(event: LogEvent, value_limit: int = DEFAULT_VALUE_LIMIT) -> str:
    """Return a stable one-line key-value representation."""

    ordered_fields = {
        "svc": event.svc,
        "pipe": event.pipe,
        "step": event.step,
        "event": event.event,
        "target": event.target,
        "reason": event.reason,
        "dur_ms": event.dur_ms,
        "file": event.file,
        "msg": event.msg,
    }
    ordered_fields.update(event.fields)

    parts = []
    for key, value in ordered_fields.items():
        if value is None or value == "":
            continue
        parts.append(f"{key}={_format_value(value, value_limit)}")
    return " ".join(parts)


def exception_log_fields(exc: BaseException) -> dict[str, str]:
    """Return compact exception fields suitable for one-line logs."""

    fields = {
        "exc_type": type(exc).__name__,
        "reason": _single_line(str(exc)) or type(exc).__name__,
    }
    filename = _exception_filename(exc)
    if filename:
        fields["file"] = filename
    return fields


class MacGyvbotLogger:
    """Adapter that emits formatted MacGyvBot service logs through a logger."""

    def __init__(self, logger, svc: str, pipe: str = "system"):
        self._logger = logger
        self._svc = svc
        self._pipe = pipe

    def bind(self, pipe: str | None = None) -> "MacGyvbotLogger":
        return MacGyvbotLogger(self._logger, self._svc, pipe or self._pipe)

    def debug(self, *args, **fields):
        self._emit_call("debug", *args, **fields)

    def info(self, *args, **fields):
        self._emit_call("info", *args, **fields)

    def warn(self, *args, **fields):
        self._emit_call("warn", *args, **fields)

    def warning(self, *args, **fields):
        self._emit_call("warn", *args, **fields)

    def error(self, *args, **fields):
        self._emit_call("error", *args, **fields)

    @contextmanager
    def duration(self, level: str, step: str, event: str, **fields):
        start = time.monotonic()
        try:
            yield
        finally:
            dur_ms = int((time.monotonic() - start) * 1000)
            self._emit(level, step, event, dur_ms=dur_ms, **fields)

    def _emit_call(self, level: str, *args, **fields):
        if len(args) == 1:
            self._emit(
                level,
                fields.pop("step", "log"),
                fields.pop("event", "status"),
                msg=args[0],
                **fields,
            )
            return
        if len(args) >= 2:
            step, event = args[:2]
            self._emit(level, step, event, **fields)
            return
        self._emit(level, fields.pop("step", "log"), fields.pop("event", "status"), **fields)

    def _emit(self, level: str, step: str, event: str, **fields):
        step, event, fields = _normalize_log_fields(
            level,
            self._svc,
            fields.get("pipe", self._pipe),
            step,
            event,
            fields,
        )
        event_fields = {
            name: fields.pop(name, None)
            for name in ("target", "reason", "dur_ms", "file", "msg")
        }
        text = format_log_event(
            LogEvent(
                svc=self._svc,
                pipe=fields.pop("pipe", self._pipe),
                step=step,
                event=event,
                fields=fields,
                **event_fields,
            )
        )

        if level == "debug" and hasattr(self._logger, "debug"):
            self._logger.debug(text)
        elif level == "error":
            self._logger.error(text)
        elif level in ("warn", "warning"):
            self._logger.warn(text)
        else:
            self._logger.info(text)


def emit_structured_log(
    logger,
    level: str,
    step: str,
    event: str,
    *,
    svc: str = "system",
    pipe: str = "system",
    **fields,
) -> None:
    """Emit a structured log through either MacGyvbotLogger or a plain logger."""

    step, event, fields = _normalize_log_fields(level, svc, pipe, step, event, fields)

    method_name = "warn" if level == "warning" else level
    method = getattr(logger, method_name, None)
    if method is None and method_name == "warn":
        method = getattr(logger, "warning", None)
    if method is None:
        method = getattr(logger, "info", None)
    if method is None:
        return

    method_fields = dict(fields)
    method_fields.setdefault("pipe", pipe)
    try:
        method(step, event, **method_fields)
        return
    except TypeError:
        pass

    event_fields = {
        name: fields.pop(name, None)
        for name in ("target", "reason", "dur_ms", "file", "msg")
    }
    text = format_log_event(
        LogEvent(
            svc=svc,
            pipe=fields.pop("pipe", pipe),
            step=step,
            event=event,
            fields=fields,
            **event_fields,
        )
    )
    method(text)


def _normalize_log_fields(
    level: str,
    svc: str,
    pipe: str,
    step: str,
    event: str,
    fields: dict[str, Any],
) -> tuple[str, str, dict[str, Any]]:
    normalized = dict(fields)
    if step != "log" or event != "status":
        _fill_reason_for_result(level, event, normalized)
        return step, event, normalized

    original_msg = normalized.get("msg")
    msg_text = _single_line(str(original_msg or ""))
    inferred_step = _infer_step(svc, pipe, msg_text)
    inferred_event = _infer_event(level, msg_text)
    reason = normalized.get("reason") or _infer_reason(level, inferred_event, msg_text)
    if reason:
        normalized["reason"] = reason

    summary = _korean_log_summary(
        level=level,
        svc=svc,
        pipe=pipe,
        step=inferred_step,
        event=inferred_event,
        reason=reason,
        msg=msg_text,
    )
    if msg_text and summary != msg_text:
        normalized.setdefault("detail", msg_text)
    normalized["msg"] = summary
    return inferred_step, inferred_event, normalized


def _fill_reason_for_result(level: str, event: str, fields: dict[str, Any]) -> None:
    if fields.get("reason"):
        return
    if event in {"failed", "skipped", "rejected", "timeout", "unavailable", "interrupted"}:
        fields["reason"] = _default_reason(level, event)


def _infer_step(svc: str, pipe: str, msg: str) -> str:
    lower = msg.lower()
    if pipe == "command":
        if "stt" in lower:
            return "stt_input"
        if "tts" in lower:
            return "tts_feedback"
        if "task_control" in lower or "pause" in lower or "resume" in lower or "cancel" in lower:
            return "task_control_publish"
        if "shutdown" in lower or "종료" in msg:
            return "shutdown"
        if "tool_command" in lower or "명령" in msg or "command" in lower:
            return "command_parse"
        return "command_input"
    if pipe == "moveit":
        if "ik" in lower:
            return "pose_goal_ik"
        if "followjointtrajectory" in lower or "trajectory" in lower or "goal" in lower:
            return "trajectory_execution"
        if "home" in lower:
            return "home_motion"
        if "yaw" in lower or "j6" in lower:
            return "wrist_yaw"
        if "drawer" in lower or "서랍" in msg:
            return "drawer_motion"
        if "planning" in lower or "plan" in lower:
            return "planning"
        return "moveit_motion"
    if pipe == "force":
        return "force_descent"
    if pipe == "gripper":
        if "mask" in lower:
            return "tool_mask_lock"
        if "yolo" in lower:
            return "tool_detection"
        if "sam" in lower:
            return "sam_mask"
        if "ml" in lower or "classifier" in lower:
            return "grasp_classifier"
        if "depth" in lower:
            return "depth_stream"
        if "image" in lower:
            return "image_stream"
        return "grasp_verify" if svc == "manipulation" else "hand_grasp_detection"
    if pipe == "handoff":
        return "handoff_targeting"
    if pipe == "vlm":
        return "vlm_history" if "history" in lower else "vlm_inference"
    if pipe == "pick":
        if "mask" in lower:
            return "pick_mask_lock"
        if "handoff" in lower or "사용자" in msg or "손 위치" in msg:
            return "pick_handoff"
        if "vlm" in lower or "view" in lower:
            return "pick_refine"
        if "grasp" in lower:
            return "pick_grasp"
        if "반환" in msg:
            return "pick_return"
        return "pick_sequence"
    if pipe == "return":
        if "drawer" in lower or "서랍" in msg:
            return "return_drawer"
        if "손 위치" in msg or "사용자" in msg:
            return "return_handoff"
        if "임시" in msg or "staging" in lower:
            return "return_staging"
        if "bbox" in lower or "camera" in lower or "marker" in lower:
            return "return_perception"
        return "return_sequence"
    if pipe == "motion":
        return "home_initializer"
    if pipe == "adapter":
        return "hand_grasp_adapter"
    if svc == "ui":
        return "operator_ui"
    return pipe or "system"


def _infer_event(level: str, msg: str) -> str:
    lower = msg.lower()
    if "timeout" in lower or "타임아웃" in msg:
        return "timeout"
    if "stop/pause" in lower or "중단" in msg or "cancel" in lower or "취소" in msg:
        return "interrupted"
    if "실패" in msg or "못했습니다" in msg or "failed" in lower or "abort" in lower or level == "error":
        return "failed"
    if "무시" in msg or "skip" in lower or "생략" in msg:
        return "skipped"
    if "거부" in msg or "reject" in lower:
        return "rejected"
    if "없" in msg or "찾지 못" in msg or "not found" in lower or "unavailable" in lower or "disabled" in lower:
        return "unavailable"
    if "clamp" in lower or "맞춥니다" in msg or "보정" in msg:
        return "clamped"
    if "수신" in msg or "received" in lower:
        return "received"
    if "발행" in msg or "publish" in lower or "전송" in msg:
        return "published"
    if "시작" in msg or "start" in lower:
        return "started"
    if "완료" in msg or "success" in lower or "성공" in msg or "확인" in msg or "enabled" in lower:
        return "completed"
    if "이동" in msg or "복귀" in msg:
        return "requested"
    if level in {"warn", "warning"}:
        return "warning"
    return "status"


def _infer_reason(level: str, event: str, msg: str) -> str | None:
    lower = msg.lower()
    if "stop/pause" in lower or "중단" in msg:
        return "stop_requested"
    if "timeout" in lower or "타임아웃" in msg:
        return "timeout"
    if "ik" in lower and (
        "실패" in msg or "없" in msg or "찾지 못" in msg or "failed" in lower
    ):
        return "ik_failed"
    if "joint bounds" in lower:
        return "joint_bounds"
    if "joint delta" in lower or "안전 한계" in msg:
        return "joint_delta_limit"
    if "safe_z_min" in lower or "safety limit" in lower:
        return "workspace_limit"
    if "depth" in lower:
        return "depth_unavailable" if event in {"failed", "unavailable"} else "depth_condition"
    if "mask" in lower:
        return "mask_unavailable" if event in {"failed", "unavailable", "timeout"} else "mask_condition"
    if "sam" in lower:
        return "sam_unavailable" if event in {"failed", "unavailable"} else "sam_condition"
    if "yolo" in lower or "bbox" in lower or "roi" in lower:
        return "detection_unavailable" if event in {"failed", "unavailable", "timeout"} else "detection_condition"
    if "camera" in lower or "image" in lower:
        return "camera_unavailable" if event in {"failed", "unavailable"} else "camera_condition"
    if "frame" in lower:
        return "unsupported_frame"
    if "action server" in lower:
        return "action_server_unavailable"
    if "goal" in lower and "거부" in msg:
        return "goal_rejected"
    if "unsupported" in lower or "지원하지" in msg:
        return "unsupported_request"
    if "drop" in lower:
        return "tool_drop_detected"
    if "grasp" in lower or "그리퍼" in msg:
        return "grasp_failed" if event in {"failed", "timeout"} else "grasp_condition"
    if event in {"failed", "skipped", "rejected", "timeout", "unavailable"}:
        return _default_reason(level, event)
    if level in {"warn", "warning", "error"}:
        return "check_required"
    return None


def _default_reason(level: str, event: str) -> str:
    if event == "timeout":
        return "timeout"
    if event == "interrupted":
        return "stop_requested"
    if event == "unavailable":
        return "resource_unavailable"
    if event == "skipped":
        return "condition_not_met"
    if event == "rejected":
        return "request_rejected"
    if level == "error":
        return "operation_failed"
    return "check_required"


def _korean_log_summary(
    *,
    level: str,
    svc: str,
    pipe: str,
    step: str,
    event: str,
    reason: str | None,
    msg: str,
) -> str:
    if msg and _looks_korean(msg) and len(msg) <= 80:
        return msg

    step_name = _KOREAN_STEP_NAMES.get(step, step)
    event_name = _KOREAN_EVENT_NAMES.get(event, event)
    reason_name = _KOREAN_REASON_NAMES.get(reason or "", "")
    if reason_name:
        return f"{step_name} {event_name}: {reason_name}"
    if event in {"failed", "warning", "timeout", "unavailable", "skipped", "rejected"}:
        return f"{step_name} {event_name}"
    if level in {"warn", "warning"}:
        return f"{step_name} 확인 필요"
    return f"{step_name} {event_name}"


def _looks_korean(text: str) -> bool:
    return any("\uac00" <= char <= "\ud7a3" for char in text)


_KOREAN_STEP_NAMES = {
    "stt_input": "음성 입력",
    "tts_feedback": "음성 응답",
    "task_control_publish": "작업 제어 발행",
    "shutdown": "종료 처리",
    "command_parse": "명령 해석",
    "command_input": "명령 입력",
    "pose_goal_ik": "IK 계산",
    "trajectory_execution": "궤적 실행",
    "home_motion": "Home 이동",
    "wrist_yaw": "손목 yaw 보정",
    "drawer_motion": "서랍 동작",
    "planning": "경로 계획",
    "moveit_motion": "MoveIt 동작",
    "force_descent": "반력 하강",
    "tool_mask_lock": "공구 마스크 고정",
    "tool_detection": "공구 검출",
    "sam_mask": "SAM 마스크",
    "grasp_classifier": "잡기 분류",
    "depth_stream": "Depth 입력",
    "image_stream": "이미지 입력",
    "grasp_verify": "그리퍼 잡기 확인",
    "hand_grasp_detection": "손 잡기 인식",
    "handoff_targeting": "전달 위치 탐색",
    "vlm_history": "VLM 이력 저장",
    "vlm_inference": "VLM 추론",
    "pick_mask_lock": "픽업 마스크 고정",
    "pick_handoff": "픽업 전달",
    "pick_refine": "픽업 목표 보정",
    "pick_grasp": "픽업 잡기",
    "pick_return": "픽업 원위치 반환",
    "pick_sequence": "픽업 시퀀스",
    "return_drawer": "반납 서랍 처리",
    "return_handoff": "반납 수령",
    "return_staging": "반납 임시 배치",
    "return_perception": "반납 인식",
    "return_sequence": "반납 시퀀스",
    "home_initializer": "초기 Home 설정",
    "hand_grasp_adapter": "손 위치 어댑터",
    "operator_ui": "운영 UI",
}

_KOREAN_EVENT_NAMES = {
    "status": "상태",
    "started": "시작",
    "completed": "완료",
    "requested": "요청",
    "received": "수신",
    "published": "발행",
    "updated": "갱신",
    "failed": "실패",
    "warning": "확인 필요",
    "timeout": "시간 초과",
    "unavailable": "사용 불가",
    "skipped": "생략",
    "rejected": "거부",
    "interrupted": "중단",
    "clamped": "보정",
}

_KOREAN_REASON_NAMES = {
    "stop_requested": "중단 요청",
    "timeout": "대기 시간 초과",
    "ik_failed": "IK 후보 없음",
    "joint_bounds": "조인트 범위 초과",
    "joint_delta_limit": "조인트 변화량 한계 초과",
    "workspace_limit": "안전 작업공간 한계 적용",
    "depth_unavailable": "Depth 정보 없음",
    "depth_condition": "Depth 조건 확인",
    "mask_unavailable": "마스크 없음",
    "mask_condition": "마스크 조건 확인",
    "sam_unavailable": "SAM 사용 불가",
    "sam_condition": "SAM 조건 확인",
    "detection_unavailable": "검출 결과 없음",
    "detection_condition": "검출 조건 확인",
    "camera_unavailable": "카메라 정보 없음",
    "camera_condition": "카메라 조건 확인",
    "unsupported_frame": "지원하지 않는 프레임",
    "action_server_unavailable": "액션 서버 없음",
    "goal_rejected": "액션 goal 거부",
    "unsupported_request": "지원하지 않는 요청",
    "tool_drop_detected": "공구 떨어짐 감지",
    "grasp_failed": "잡기 실패",
    "grasp_condition": "잡기 조건 확인",
    "resource_unavailable": "필요 리소스 없음",
    "condition_not_met": "조건 미충족",
    "request_rejected": "요청 거부",
    "operation_failed": "작업 실패",
    "check_required": "상태 확인 필요",
}


def _format_value(value: Any, value_limit: int) -> str:
    text = _single_line(str(value))
    if len(text) > value_limit:
        text = text[: max(0, value_limit - 3)] + "..."
    if not text:
        return '""'
    if any(ch.isspace() for ch in text) or '"' in text:
        text = text.replace("\\", "\\\\").replace('"', '\\"')
        return f'"{text}"'
    return text


def _single_line(text: str) -> str:
    return " ".join(str(text or "").replace("\r", " ").replace("\n", " ").split())


def _exception_filename(exc: BaseException) -> str | None:
    tb = exc.__traceback__
    last = None
    while tb is not None:
        last = tb
        tb = tb.tb_next
    if last is None:
        return None
    return Path(last.tb_frame.f_code.co_filename).name
