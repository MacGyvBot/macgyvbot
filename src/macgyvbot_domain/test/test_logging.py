from macgyvbot_domain.logging import (
    LogEvent,
    MacGyvbotLogger,
    exception_log_fields,
    format_log_event,
)


def test_format_log_event_uses_stable_one_line_key_values():
    text = format_log_event(
        LogEvent(
            svc="task",
            pipe="pick",
            step="grasp",
            event="fail",
            target="screwdriver",
            reason="bad status",
            dur_ms=12,
            file="grasp.py",
            msg="retry needed",
            fields={"attempt": 2},
        )
    )

    assert "\n" not in text
    assert text == (
        'svc=task pipe=pick step=grasp event=fail target=screwdriver '
        'reason="bad status" dur_ms=12 file=grasp.py msg="retry needed" '
        "attempt=2"
    )


def test_exception_log_fields_are_compact():
    try:
        raise RuntimeError("first line\nsecond line")
    except RuntimeError as exc:
        fields = exception_log_fields(exc)

    assert fields["exc_type"] == "RuntimeError"
    assert fields["reason"] == "first line second line"
    assert fields["file"] == "test_logging.py"


def test_logger_adapter_accepts_legacy_single_message_calls():
    class FakeLogger:
        def __init__(self):
            self.lines = []

        def info(self, message):
            self.lines.append(message)

    fake = FakeLogger()
    MacGyvbotLogger(fake, svc="task", pipe="pick").info("plain message")

    assert fake.lines == [
        'svc=task pipe=pick step=log event=status msg="plain message"'
    ]
