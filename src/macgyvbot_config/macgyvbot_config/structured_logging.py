"""Shared structured logging helpers for MacGyvBot runtime nodes."""

from __future__ import annotations

PKG_WIDTH = 21
PIPE_WIDTH = 36

PKG_LABELS = {
    "command": "command",
    "macgyvbot_command": "command",
    "manipulation": "manipulation",
    "macgyvbot_manipulation": "manipulation",
    "perception": "perception",
    "macgyvbot_perception": "perception",
    "task": "task",
    "macgyvbot_task": "task",
    "ui": "ui",
    "macgyvbot_ui": "ui",
}

PIPE_LABELS = {
    "input": "command_input",
    "router": "task_router",
    "operator": "operator_ui",
    "system": "task_system",
    "moveit": "moveit_control",
    "task": "task_runtime",
    "pick": "pick_flow",
    "return": "return_flow",
    "perception": "perception_runtime",
    "hand_grasp": "hand_grasp_detection",
    "vlm_service": "vlm_grasp_service",
    "control": "task_control",
    "gripper": "manual_gripper",
    "camera": "camera_state",
    "cleanup": "task_cleanup",
    "safety": "tool_drop_safety",
    "request": "task_request",
    "vlm": "vlm_selector",
}

MESSAGE_TRANSLATIONS = {
    "main router ready": "메인 라우터 준비 완료",
    "task request published": "작업 요청 발행",
    "release request forwarded to task coordinator.": "release 요청을 task coordinator로 전달했습니다.",
    "home request forwarded to task coordinator.": "Home 복귀 요청을 task coordinator로 전달했습니다.",
    "stt ready": "STT 준비 완료",
    "microphone stt disabled": "마이크 STT 비활성화",
    "command input ready": "명령 입력 노드 준비 완료",
    "stt text received": "STT 텍스트 수신",
    "ignored recent tts echo": "최근 TTS 에코 입력 무시",
    "command interpretation requested": "명령 해석 요청",
    "task control interpreted": "작업 제어 명령 해석 완료",
    "exit interpreted": "종료 명령 해석 완료",
    "exit already pending": "종료 요청이 이미 진행 중입니다",
    "fast control command matched": "빠른 제어 명령 인식",
    "tool command published": "공구 명령 발행",
    "task control published": "작업 제어 명령 발행",
    "operator ui shutdown received": "UI 종료 신호 수신",
    "exit completed": "종료 처리 완료",
    "exit failed": "종료 처리 실패",
    "task coordinator initialized": "노드 준비 완료",
    "topic ready": "토픽 준비 완료",
    "YOLO model ready": "YOLO 모델 준비 완료",
    "grasp point mode ready": "grasp point 모드 준비 완료",
    "manual gripper service ready": "manual gripper 서비스 준비 완료",
    "robot grasp": "로봇 파지 시작",
    "robot grasp failed": "로봇 파지 실패",
    "grasp descent skipped": "파지 하강 생략",
    "grasp mask image generation failed": "grasp 마스크 이미지 생성 실패",
    "handoff pose unavailable": "handover 자세를 계산할 수 없습니다",
    "wait for human grasp": "사용자 파지 대기",
    "wait for tool mask lock": "공구 마스크 고정 대기",
    "pick sequence done": "픽 시퀀스 완료",
    "pregrasp depth measurement": "pre-grasp 깊이 측정 시작",
    "pregrasp depth unavailable": "pre-grasp 깊이 측정 불가",
    "pregrasp descent calculated": "pre-grasp 하강량 계산 완료",
    "pregrasp descent skipped": "pre-grasp 하강 생략",
    "pregrasp gripper measurement": "pre-grasp 그리퍼 측정 시작",
    "pregrasp gripper depth read failed": "pre-grasp 그리퍼 깊이 읽기 실패",
    "pregrasp measurement interrupted": "pre-grasp 측정 중단",
    "pregrasp measurement timed out": "pre-grasp 측정 시간 초과",
    "tool mask lock failed": "공구 마스크 고정 실패",
    "wrist rotation failed": "손목 회전 실패",
    "release to human": "사용자에게 공구 전달",
    "home after handoff": "handover 이후 Home 복귀",
    "close drawer after handoff": "handover 이후 서랍 닫기",
    "close drawer after recovery": "복구 이후 서랍 닫기",
    "pick target refined": "픽 타겟 보정 완료",
    "pick target refinement unavailable": "픽 타겟 보정 불가",
    "refine pick target": "픽 타겟 보정 시작",
    "refine pick target failed": "픽 타겟 보정 실패",
    "pick plan ready": "픽 계획 준비 완료",
    "target search started": "타겟 탐색 시작",
    "approach z clamped to grasp z": "접근 Z를 grasp Z에 맞춰 제한했습니다",
    "approach z clamped to safe minimum": "접근 Z를 안전 최소값으로 제한했습니다",
    "grasp z clamped to safe minimum": "grasp Z를 안전 최소값으로 제한했습니다",
    "drawer marker camera state unavailable": "서랍 마커 카메라 상태를 사용할 수 없습니다",
    "drawer marker id missing": "서랍 마커 ID가 없습니다",
    "drawer store exit move failed": "서랍 수납 종료 이동 실패",
    "move to drawer store exit": "서랍 수납 종료 위치로 이동",
    "store tool target camera state unavailable": "공구 수납 타겟 카메라 상태를 사용할 수 없습니다",
    "return flow failed": "반납 흐름 실패",
    "grasp state changed": "파지 상태 변경",
    "Robot grasp success received. Tool mask lock requested.": "로봇 파지 성공 상태를 받아 공구 마스크 고정을 요청했습니다.",
    "YOLO tool detector enabled": "YOLO 공구 검출기 사용 가능",
    "depth recognition enabled": "depth recognition 사용 가능",
    "depth recognition disabled": "depth recognition 비활성화",
    "ML grasp classifier enabled": "ML grasp classifier 사용 가능",
    "SAM tool mask enabled": "SAM 공구 마스크 사용 가능",
    "SAM tool mask disabled": "SAM 공구 마스크 비활성화",
    "YOLO tool detector init failed": "YOLO 공구 검출기 초기화 실패",
    "ML grasp classifier init failed": "ML grasp classifier 초기화 실패",
    "SAM tool mask init failed": "SAM 공구 마스크 초기화 실패",
    "Depth locked tool mask accumulation started.": "깊이 기반 공구 마스크 누적을 시작합니다.",
    "Pre-grasp tool mask lock failed.": "pre-grasp 공구 마스크 고정에 실패했습니다.",
    "Pre-grasp SAM returned an empty tool mask.": "pre-grasp SAM이 비어 있는 공구 마스크를 반환했습니다.",
    "SAM returned an empty tool mask; falling back if allowed.": "SAM이 비어 있는 공구 마스크를 반환했습니다. 허용되면 fallback을 수행합니다.",
    "VLM crop bbox is empty.": "VLM crop bbox가 비어 있습니다.",
    "VLM-only crop bbox is empty.": "VLM-only crop bbox가 비어 있습니다.",
    "API grasp crop bbox is empty.": "API grasp crop bbox가 비어 있습니다.",
    "VLM grasp model lazy load preparing.": "VLM grasp 모델 지연 로딩을 준비합니다.",
    "VLM is not using CUDA. Running on CPU.": "VLM이 CUDA를 사용하지 않고 CPU에서 실행 중입니다.",
    "VLM service request bbox is invalid.": "VLM 서비스 요청 bbox가 올바르지 않습니다.",
    "VLM grasp point failed. Falling back to bbox center.": "VLM grasp point 선택에 실패하여 bbox 중심으로 대체합니다.",
    "VLM-only grasp point failed. Falling back to bbox center.": "VLM-only grasp point 선택에 실패하여 bbox 중심으로 대체합니다.",
    "VLM grasp service ready:": "노드 준비 완료:",
    "node ready": "노드 준비 완료",
    "VLM service request created": "VLM 서비스 요청 생성",
    "VLM service unavailable within timeout": "VLM 서비스를 제한 시간 안에 사용할 수 없습니다",
    "VLM service request sent": "VLM 서비스 요청 전송",
    "VLM service request interrupted while waiting for response": "VLM 서비스 응답 대기 중 요청 중단",
    "VLM service response timeout": "VLM 서비스 응답 시간 초과",
    "VLM service response received": "VLM 서비스 응답 수신",
    "VLM service future failed": "VLM 서비스 future 실패",
    "VLM service returned no response object": "VLM 서비스가 응답 객체를 반환하지 않았습니다",
    "VLM service reported failure": "VLM 서비스 실패 응답 수신",
    "manual gripper command failed": "manual gripper 명령 실패",
    "manual gripper command accepted": "manual gripper 명령 수락",
    "manual gripper status read failed": "manual gripper 상태 읽기 실패",
    "manual gripper command rejected": "manual gripper 명령 거부",
    "unsupported task request": "지원하지 않는 task request",
    "release task request received": "release task request 수신",
    "unsupported task control action": "지원하지 않는 task control action",
    "task queue pause requested": "task queue pause 요청",
    "task queue resume requested": "task queue resume 요청",
    "task queue cancel requested": "task queue cancel 요청",
    "task queue exit requested": "task queue exit 요청",
    "moving home after exit request": "exit 요청 후 Home 복귀 이동",
    "task queue shutdown wait timed out": "task queue shutdown 대기 시간 초과",
    "tool drop handled as task exit": "tool drop을 task exit로 처리",
    "pick request ignored while task is running": "task 실행 중 pick 요청 무시",
    "task request ignored while queue is running": "queue 실행 중 task request 무시",
    "task queue loaded": "task queue 로드 완료",
    "task step started": "task step 시작",
    "task step completed": "task step 완료",
    "task step suspended": "task step 일시 중단",
    "task queue stopped by exit request": "exit 요청으로 task queue 중단",
    "task step failed": "task step 실패",
    "task queue failed": "task queue 실패",
    "task queue completed": "task queue 완료",
    "task cleanup callback failed": "task cleanup callback 실패",
    "preparing drawer before target search": "타겟 탐색 전 서랍 준비",
    "camera state unavailable for top-view VLM refine": "top-view VLM 보정용 camera state 사용 불가",
    "top-view VLM service refine failed; keeping existing pick plan": "top-view VLM 서비스 보정 실패. 기존 pick plan 유지",
    "grasp detection SAM mask disabled": "grasp detection SAM mask 비활성화",
    "grasp detection SAM init failed": "grasp detection SAM 초기화 실패",
    "grasp detection binary crop PCA yaw failed": "grasp detection binary crop PCA yaw 계산 실패",
    "grasp detection binary crop PCA yaw applied": "grasp detection binary crop PCA yaw 적용",
    "camera state unavailable for grasp detection mask image": "grasp detection mask image용 camera state 사용 불가",
    "YOLO bbox unavailable for grasp detection mask image": "grasp detection mask image용 YOLO bbox 사용 불가",
    "SAM depth mask generation failed at VLM observe pose": "VLM 관찰 자세에서 SAM depth mask 생성 실패",
    "SAM depth grasp detection mask images saved": "SAM depth grasp detection mask image 저장 완료",
    "hand grasp image conversion failed": "hand grasp image 변환 실패",
    "gripper release failed after exit home": "exit home 이후 gripper release 실패",
    "task coordinator camera loop waiting": "task coordinator camera loop 대기 중",
    "VLM service future failed": "VLM 서비스 future 실패",
    "Tool mask lock requested without YOLO ROI; using latest bbox fallback.": "YOLO ROI 없이 공구 마스크 고정이 요청되어 최신 bbox fallback을 사용합니다.",
    "Tool mask lock requested, but no YOLO tool ROI is available.": "공구 마스크 고정이 요청되었지만 YOLO 공구 ROI를 사용할 수 없습니다.",
    "Clearing SAM candidate mask because requested YOLO target has not been seen recently.": "요청된 YOLO 타겟이 최근 감지되지 않아 SAM 후보 마스크를 초기화합니다.",
}

PREFIX_TRANSLATIONS = {
    "Depth image conversion failed: ": "깊이 이미지 변환 실패: ",
    "Color image conversion failed: ": "컬러 이미지 변환 실패: ",
    "ML grasp update failed: ": "ML grasp 갱신 실패: ",
    "robot grasp attempt ": "로봇 파지 시도 ",
    "VLM service image conversion failed: ": "VLM 서비스 이미지 변환 실패: ",
    "VLM grasp service ready: ": "노드 준비 완료: ",
    "VLM grasp point inference failed: ": "VLM grasp point 추론 실패: ",
    "VLM-only grasp point inference failed: ": "VLM-only grasp point 추론 실패: ",
    "API grasp point inference failed: ": "API grasp point 추론 실패: ",
    "VLM grasp module import failed: ": "VLM grasp 모듈 import 실패: ",
    "VLM-only grasp module import failed: ": "VLM-only grasp 모듈 import 실패: ",
    "VLM preload failed: ": "VLM preload 실패: ",
    "VLM-only preload failed: ": "VLM-only preload 실패: ",
    "VLM service request received: ": "VLM 서비스 요청 수신: ",
    "VLM service inference started: ": "VLM 서비스 추론 시작: ",
    "VLM service inference completed: ": "VLM 서비스 추론 완료: ",
    "VLM service request completed: ": "VLM 서비스 요청 처리 완료: ",
    "Preloading VLM service selector: ": "VLM 서비스 selector 사전 로딩: ",
    "VLM service preload skipped: ": "VLM 서비스 사전 로딩 생략: ",
    "VLM service preload started: ": "VLM 서비스 사전 로딩 시작: ",
    "VLM service preload completed: ": "VLM 서비스 사전 로딩 완료: ",
    "VLM runtime: ": "VLM 런타임 정보: ",
    "VLM-only runtime: ": "VLM-only 런타임 정보: ",
    "VLM grasp point selected: ": "VLM grasp point 선택 완료: ",
    "VLM-only grasp point selected: ": "VLM-only grasp point 선택 완료: ",
    "API grasp point selected: ": "API grasp point 선택 완료: ",
    "VLM-only response did not contain valid x_px, y_px, yaw_deg: ": "VLM-only 응답에 유효한 x_px, y_px, yaw_deg가 없습니다: ",
    "VLM-only grasp model lazy load preparing: ": "VLM-only grasp 모델 지연 로딩 준비: ",
    "Tool mask tracking target label set: ": "공구 마스크 추적 타겟 라벨 설정: ",
    "SAM+Depth tool mask ready: ": "SAM+Depth 공구 마스크 준비 완료: ",
    "Depth lock ROI initialized from full depth frame: ": "전체 depth frame에서 depth lock ROI 초기화: ",
    "Depth locked tool mask ready: ": "depth 기반 공구 마스크 준비 완료: ",
    "Pre-grasp tool mask locked: ": "pre-grasp 공구 마스크 고정 완료: ",
    "Pre-grasp SAM mask locked: ": "pre-grasp SAM 마스크 고정 완료: ",
    "Locked depth tool mask: ": "depth 공구 마스크 고정: ",
    "Locked SAM tool mask: ": "SAM 공구 마스크 고정: ",
    "Locked bbox tool mask: ": "bbox 공구 마스크 고정: ",
    "Rejected SAM tracked tool mask: ": "SAM 추적 공구 마스크 거부: ",
    "Rejected scaled pre-grasp mask because it disagrees with YOLO: ": "YOLO와 일치하지 않아 scale 보정된 pre-grasp 마스크 거부: ",
    "Using scaled pre-grasp mask for final lock: ": "최종 고정에 scale 보정된 pre-grasp 마스크 사용: ",
    "Unknown grasp_point_mode ": "알 수 없는 grasp_point_mode ",
    "VLM inference progress: ": "VLM 추론 진행률: ",
    "VLM inference device check: ": "VLM 추론 device 확인: ",
    "VLM inference complete: ": "VLM 추론 완료: ",
    "bbox center history recording failed: ": "bbox center history 기록 실패: ",
    "grasp detection recording failed: ": "grasp detection 기록 실패: ",
    "pose_goal IK best candidate: ": "pose_goal IK 최적 후보: ",
}


def translate_log_message(message):
    text = str(message or "")
    if not text:
        return ""
    translated = MESSAGE_TRANSLATIONS.get(text)
    if translated is not None:
        return translated
    for prefix, localized in PREFIX_TRANSLATIONS.items():
        if text.startswith(prefix):
            return localized + text[len(prefix):]
    return text


def describe_pipe(pipe):
    text = str(pipe or "").strip()
    if not text:
        return ""
    return PIPE_LABELS.get(text, text)


def describe_pkg(pkg):
    text = str(pkg or "").strip()
    if not text:
        return ""
    return PKG_LABELS.get(text, text)


def format_log_value(value):
    text = " ".join(str(value or "").replace("\r", " ").replace("\n", " ").split())
    if not text:
        return '""'
    if len(text) > 240:
        text = text[:237] + "..."
    if any(char.isspace() for char in text) or '"' in text:
        text = text.replace("\\", "\\\\").replace('"', '\\"')
        return f'"{text}"'
    return text


def _pad_segment(text, width):
    if len(text) >= width:
        return text
    return text + (" " * (width - len(text)))


def format_structured_log(
    *,
    pkg=None,
    svc=None,
    pipe="",
    step="",
    event="",
    msg="",
    translate=True,
    **fields,
):
    package_name = describe_pkg(pkg or svc or "")
    pipe_name = describe_pipe(pipe)
    message = translate_log_message(msg) if translate else str(msg or "")

    parts = [
        _pad_segment(f"[pkg] {format_log_value(package_name)}", PKG_WIDTH),
        _pad_segment(f"[pipe] {format_log_value(pipe_name)}", PIPE_WIDTH),
    ]
    if message != "":
        parts.append(f"[msg] {format_log_value(message)}")
    for key, value in fields.items():
        if key in {"step", "event", "svc", "pkg"}:
            continue
        if value is None or value == "":
            continue
        parts.append(f"[{key}] {format_log_value(value)}")
    return " | ".join(parts)
