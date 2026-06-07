"""Small logging helpers for flow-level structured messages."""

from __future__ import annotations


def log_debug(logger, message, **fields):
    _emit(logger, "debug", message, fields)


def log_info(logger, message, **fields):
    _emit(logger, "info", message, fields)


def log_warn(logger, message, **fields):
    _emit(logger, "warn", message, fields)


def log_error(logger, message, **fields):
    _emit(logger, "error", message, fields)


def _emit(logger, level, message, fields):
    method = getattr(logger, level, None)
    if method is None and level == "warn":
        method = getattr(logger, "warning", None)
    if method is None:
        return

    try:
        method(message, **fields)
    except TypeError:
        method(_format_message(message, fields))


def _format_message(message, fields):
    if not fields:
        return str(message)
    suffix = " ".join(
        f"{key}={_format_value(value)}"
        for key, value in fields.items()
        if value is not None
    )
    if not suffix:
        return str(message)
    return f"{message} {suffix}"


def _format_value(value):
    if isinstance(value, float):
        return f"{value:.3f}"
    return str(value)
