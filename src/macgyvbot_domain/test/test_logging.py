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


def test_logger_adapter_normalizes_legacy_single_message_calls():
    class FakeLogger:
        def __init__(self):
            self.lines = []

        def info(self, message):
            self.lines.append(message)

    fake = FakeLogger()
    MacGyvbotLogger(fake, svc="task", pipe="pick").info("plain message")

    assert fake.lines == [
        'svc=task pipe=pick step=pick_sequence event=status '
        'msg="픽업 시퀀스 상태" detail="plain message"'
    ]


def test_logger_adapter_adds_reason_and_korean_msg_for_failures():
    class FakeLogger:
        def __init__(self):
            self.lines = []

        def error(self, message):
            self.lines.append(message)

    fake = FakeLogger()
    MacGyvbotLogger(fake, svc="manipulation", pipe="moveit").error(
        "pose_goal IK 후보를 찾지 못했습니다."
    )

    assert fake.lines == [
        'svc=manipulation pipe=moveit step=pose_goal_ik event=failed '
        'reason=ik_failed msg="pose_goal IK 후보를 찾지 못했습니다."'
    ]
