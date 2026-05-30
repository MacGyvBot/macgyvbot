#!/usr/bin/env python3
"""Monitor ros2 launch output, persist logs, and send Discord alerts."""

from __future__ import annotations

import json
import os
import re
import subprocess
import sys
import time
import uuid
from datetime import datetime
from urllib.error import HTTPError

from macgyvbot_domain.logging import LogEvent, format_log_event


WEBHOOK = os.environ.get("DISCORD_WEBHOOK", "").strip()
LOG_DIR = os.environ.get(
    "MACGYVBOT_LOG_DIR",
    os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "log"),
)
CONTEXT_LINES = int(os.environ.get("CONTEXT_LINES", "5"))
COOLDOWN_SEC = float(os.environ.get("COOLDOWN_SEC", "10"))

ERROR_PATTERN = re.compile(
    r"\b(ERROR|FATAL|CRITICAL|Traceback \(most recent call last\)|"
    r"\[ERROR\]|process has died|required process .* exited|"
    r"No such file|ModuleNotFoundError|ImportError|Exception|RLException)\b",
    re.IGNORECASE,
)
IGNORE_PATTERN = re.compile(r"(0 errors|error_code\s*=\s*0|errors:\s*0)", re.IGNORECASE)
METRIC_PATTERN = re.compile(
    r"\b\d+\.?\d*\s*(?:mm|deg|rad|Hz|ms|fps|sec)\b|"
    r"\b(?:depth|travel_z|approach_z|grasp_z|corrected_bz|raw_bz|safe_z_min)"
    r"\s*=\s*-?\d|"
    r"\bTarget\s*\(-?\d|"
    r"\b(?:xyz|base|camera|offset|pixel)\s*[=:]\s*[\-\d(]|"
    r"\bpoints\s*=\s*\d",
    re.IGNORECASE,
)


def _log(level: str, step: str, event: str, **fields) -> None:
    text = format_log_event(
        LogEvent(
            svc="monitor",
            pipe="launch",
            step=step,
            event=event,
            fields=fields,
        )
    )
    sys.stderr.write(f"{level.upper()} {text}\n")
    sys.stderr.flush()


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
    if not WEBHOOK:
        _log("warn", "webhook", "skip", reason="missing_webhook")
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
        _log("error", "webhook", "post_failed", reason=_format_post_error(exc))


def _post_files(content: str, *filepaths: str) -> None:
    if not WEBHOOK:
        _log("warn", "webhook", "skip", reason="missing_webhook")
        return

    import urllib.request

    loaded: list[tuple[str, bytes]] = []
    for filepath in filepaths:
        try:
            with open(filepath, "rb") as f:
                data = f.read()
            if data.strip():
                loaded.append((filepath, data))
        except Exception as exc:
            _log("error", "file", "read_failed", path=filepath, reason=exc)

    if not loaded:
        _post(content)
        return

    boundary = uuid.uuid4().hex
    body = (
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="content"\r\n\r\n'
        f"{content[:1900]}\r\n"
    ).encode("utf-8")
    for i, (filepath, data) in enumerate(loaded):
        filename = os.path.basename(filepath)
        body += (
            f"--{boundary}\r\n"
            f'Content-Disposition: form-data; name="file{i}"; filename="{filename}"\r\n'
            f"Content-Type: text/plain; charset=utf-8\r\n\r\n"
        ).encode("utf-8") + data + b"\r\n"
    body += f"--{boundary}--\r\n".encode("utf-8")

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
        _log("error", "webhook", "file_post_failed", reason=_format_post_error(exc))
        _post(content)


def _make_source():
    if "--" in sys.argv:
        idx = sys.argv.index("--")
        cmd = sys.argv[idx + 1 :]
        if not cmd:
            _log("error", "source", "missing_command", reason="empty_command_after_separator")
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

    _log("info", "launch", "started", full_log=log_path, signal_log=signal_path)
    _post("MacGyvBot launch started")

    with open(log_path, "w", encoding="utf-8") as logf, open(
        signal_path,
        "w",
        encoding="utf-8",
    ) as sigf:
        try:
            for raw in src:
                line = raw.rstrip("\n")
                sys.stdout.write(line + "\n")
                sys.stdout.flush()
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
                        _post(f"MacGyvBot launch error detected\n```\n{ctx}\n```")
        except KeyboardInterrupt:
            _log("warn", "launch", "interrupted")

    rc = proc.wait() if proc is not None else 0
    status = "ok" if error_count == 0 and rc == 0 else "error"
    _log(
        "info",
        "launch",
        "finished",
        status=status,
        return_code=rc,
        error_count=error_count,
        full_log=log_path,
        signal_log=signal_path,
    )
    _post_files("MacGyvBot launch full log", log_path)
    _post_files("MacGyvBot launch signal log", signal_path)
    sys.exit(rc)


if __name__ == "__main__":
    main()
