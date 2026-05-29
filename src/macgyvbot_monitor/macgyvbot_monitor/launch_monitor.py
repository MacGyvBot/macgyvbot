#!/usr/bin/env python3
"""
launch_monitor — ros2 launch 출력을 감시해 에러 줄을 Discord webhook으로 전송한다.
전체 로그와 신호 로그(에러 + 수치값) 두 파일로 저장한다. 표준 라이브러리만 사용.

저장 파일 (~/macgyvbot_monitor/macgyvbot_log/):
  launch_{stamp}.log         — 전체 로그 (블랙박스)
  launch_{stamp}_signal.log  — 에러 줄 + 수치값 줄만 추려낸 압축 로그

Discord 전송:
  - 에러 감지 즉시: 에러 줄 + 앞 N줄 맥락 (텍스트)
  - launch 종료 시: signal log 파일 첨부

사용법 (둘 중 하나):

  # 방법 A) 파이프 입력
  ros2 launch macgyvbot_bringup macgyvbot.launch.py 2>&1 | ros2 run macgyvbot_monitor launch_monitor

  # 방법 B) 스크립트가 직접 launch 실행
  ros2 run macgyvbot_monitor launch_monitor -- ros2 launch macgyvbot_bringup macgyvbot.launch.py

webhook URL 은 환경변수로 주입한다 (코드에 넣지 말 것):
  export DISCORD_WEBHOOK="https://discord.com/api/webhooks/xxxx/yyyy"
"""

import json
import os
import re
import subprocess
import sys
import time
import uuid
from datetime import datetime
from urllib.error import HTTPError

# ── 설정 (환경변수로 덮어쓸 수 있음) ─────────────────────────────────────────
WEBHOOK = os.environ.get("DISCORD_WEBHOOK", "").strip()
LOG_DIR = os.path.expanduser(
    os.environ.get("MACGYVBOT_LOG_DIR", "~/macgyvbot_monitor/macgyvbot_log")
)
CONTEXT_LINES = int(os.environ.get("CONTEXT_LINES", "5"))
COOLDOWN_SEC = float(os.environ.get("COOLDOWN_SEC", "10"))

ERROR_PATTERN = re.compile(
    r"\b(ERROR|FATAL|CRITICAL|Traceback \(most recent call last\)|"
    r"\[ERROR\]|process has died|required process .* exited|"
    r"No such file|ModuleNotFoundError|ImportError|"
    r"Exception|RLException)\b",
    re.IGNORECASE,
)
IGNORE_PATTERN = re.compile(
    r"(0 errors|error_code\s*=\s*0|errors:\s*0)",
    re.IGNORECASE,
)
# 수치값이 의미 있을 가능성이 높은 줄 — 실제 코드베이스 로그 형식 기준
METRIC_PATTERN = re.compile(
    # gripper width/threshold(mm), VLM yaw/joint 각도(deg, rad)
    r"\b\d+\.?\d*\s*(?:mm|deg|rad)\b|"
    # pick 시퀀스 z축 계획값: depth=, travel_z=, approach_z=, grasp_z=, corrected_bz= 등
    r"\b(?:depth|travel_z|approach_z|grasp_z|corrected_bz|raw_bz|safe_z_min)\s*=\s*-?\d|"
    # pick Target 좌표 튜플: Target(x, y)
    r"\bTarget\s*\(-?\d|"
    # 3D 좌표 출력: xyz=, base=, camera=, offset=, target=, pixel=
    r"\b(?:xyz|base|camera|offset|pixel)\s*[=:]\s*[\-\d(]|"
    # grasp 시도 횟수: 시도 N/M
    r"시도\s+\d+/\d+|"
    # trajectory points 수
    r"\bpoints\s*=\s*\d|"
    # 일반 ROS 인프라 · 카메라 드라이버 등에서 나오는 단위
    r"\b\d+\.?\d*\s*(?:Hz|ms|fps|sec)\b",
    re.IGNORECASE,
)
# ─────────────────────────────────────────────────────────────────────────────


def _format_post_error(exc: Exception) -> str:
    if not isinstance(exc, HTTPError):
        return str(exc)

    try:
        body = exc.read().decode("utf-8", errors="replace").strip()
    except Exception:
        body = ""

    detail = f"HTTP {exc.code} {exc.reason}"
    if body:
        detail = f"{detail}: {body[:500]}"
    return detail


def _post(content: str) -> None:
    """Discord webhook 텍스트 전송. 미설정·실패해도 스크립트가 죽지 않는다."""
    if not WEBHOOK:
        print("[launch_monitor] DISCORD_WEBHOOK 미설정 — 전송 생략", file=sys.stderr)
        return
    import urllib.request

    data = json.dumps({"content": content[:1900]}).encode("utf-8")
    req = urllib.request.Request(
        WEBHOOK,
        data=data,
        headers={
            "Content-Type": "application/json",
            "User-Agent": "macgyvbot-monitor/0.0.0",
        },
        method="POST",
    )
    try:
        urllib.request.urlopen(req, timeout=10)
    except Exception as exc:
        print(f"[launch_monitor] 전송 실패: {_format_post_error(exc)}", file=sys.stderr)


def _post_file(content: str, filepath: str) -> None:
    """Discord webhook 파일 첨부 전송. 파일이 비어있거나 실패하면 텍스트만 전송한다."""
    if not WEBHOOK:
        print("[launch_monitor] DISCORD_WEBHOOK 미설정 — 전송 생략", file=sys.stderr)
        return
    import urllib.request

    try:
        with open(filepath, "rb") as f:
            file_data = f.read()
    except Exception as exc:
        print(f"[launch_monitor] 파일 읽기 실패: {exc}", file=sys.stderr)
        _post(content)
        return

    if not file_data.strip():
        _post(content)
        return

    filename = os.path.basename(filepath)
    boundary = uuid.uuid4().hex
    body = (
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="content"\r\n\r\n'
        f"{content[:1900]}\r\n"
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="file"; filename="{filename}"\r\n'
        f"Content-Type: text/plain; charset=utf-8\r\n\r\n"
    ).encode("utf-8") + file_data + f"\r\n--{boundary}--\r\n".encode("utf-8")

    req = urllib.request.Request(
        WEBHOOK,
        data=body,
        headers={
            "Content-Type": f"multipart/form-data; boundary={boundary}",
            "User-Agent": "macgyvbot-monitor/0.0.0",
        },
        method="POST",
    )
    try:
        urllib.request.urlopen(req, timeout=30)
    except Exception as exc:
        print(
            f"[launch_monitor] 파일 전송 실패: {_format_post_error(exc)}",
            file=sys.stderr,
        )
        _post(content)


def _make_source():
    """'--' 뒤에 명령이 있으면 subprocess로 실행, 없으면 stdin을 반환한다."""
    if "--" in sys.argv:
        idx = sys.argv.index("--")
        cmd = sys.argv[idx + 1 :]
        if not cmd:
            print("'--' 뒤에 실행할 명령이 없습니다.", file=sys.stderr)
            sys.exit(2)
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        return proc.stdout, proc
    return sys.stdin, None


def main() -> None:
    os.makedirs(LOG_DIR, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = os.path.join(LOG_DIR, f"launch_{stamp}.log")
    signal_path = os.path.join(LOG_DIR, f"launch_{stamp}_signal.log")

    src, proc = _make_source()
    recent: list[str] = []
    last_sent = 0.0
    error_count = 0

    host = os.uname().nodename
    print(f"[launch_monitor] full log: {log_path}", file=sys.stderr)
    print(f"[launch_monitor] signal log: {signal_path}", file=sys.stderr)
    _post(f"🚀 launch 시작 — `{host}` / 로그: `{log_path}`")

    with open(log_path, "w") as logf, open(signal_path, "w") as sigf:
        try:
            for raw in src:
                line = raw.rstrip("\n")
                print(line)
                logf.write(line + "\n")
                logf.flush()

                recent.append(line)
                if len(recent) > CONTEXT_LINES:
                    recent.pop(0)

                is_error = ERROR_PATTERN.search(line) and not IGNORE_PATTERN.search(line)
                is_metric = METRIC_PATTERN.search(line)

                if is_error or is_metric:
                    tag = "[E]" if is_error else "[M]"
                    sigf.write(f"{tag} {line}\n")
                    sigf.flush()

                if is_error:
                    error_count += 1
                    now = time.time()
                    if now - last_sent >= COOLDOWN_SEC:
                        last_sent = now
                        ctx = "\n".join(recent)
                        _post(
                            f"❌ **launch 에러 감지** (`{host}`)\n"
                            f"```\n{ctx}\n```\n"
                            f"누적 에러 줄: {error_count} · 전체 로그: `{log_path}`"
                        )
        except KeyboardInterrupt:
            pass

    rc = proc.wait() if proc is not None else 0
    summary = "✅ 에러 없이 종료" if error_count == 0 else f"⚠️ 에러 {error_count}건"
    print(
        f"[launch_monitor] 종료: {summary} / full={log_path} / signal={signal_path}",
        file=sys.stderr,
    )
    _post_file(
        f"🏁 launch 종료 — `{host}` · {summary}",
        signal_path,
    )
    sys.exit(rc)


if __name__ == "__main__":
    main()
