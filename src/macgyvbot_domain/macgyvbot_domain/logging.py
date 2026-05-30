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
