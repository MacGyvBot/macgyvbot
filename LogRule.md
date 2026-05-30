# MacGyvBot Log Rule

이 파일은 Codex와 개발자가 MacGyvBot 코드에 로그를 추가하거나 수정할 때 반드시 따르는 규칙이다.
새 로그를 만들기 전에 이 문서를 먼저 확인한다.

## 핵심 원칙

- MacGyvBot 서비스 로그는 반드시 한 줄 key-value 형식으로 남긴다.
- 직접 `self.get_logger().info(...)`, `node.get_logger().warn(...)`, `print(...)`로 서비스 로그를 만들지 않는다.
- ROS 2, MoveIt, 카메라 드라이버 같은 외부 라이브러리 로그는 이 규칙의 직접 대상이 아니다.
- 사용자가 봐야 하는 문장은 로그가 아니라 `/robot_task_status`, `/command_feedback`, GUI 상태 메시지로 전달한다.
- 로그는 개발자 진단용이다. 너무 긴 설명문보다 원인 추적 가능한 필드를 남긴다.
- 로봇 동작 파라미터, 안전 임계값, motion 정책은 로그 리팩토링 중 변경하지 않는다.

## 표준 형식

```text
svc=<service> pipe=<pipeline> step=<step> event=<event> target=<target> reason=<reason> dur_ms=<duration> file=<file> msg=<message>
```

필수 필드:

- `svc`: 서비스 영역. `command`, `task`, `perception`, `manipulation`, `ui`, `monitor`, `system`
- `pipe`: 파이프라인. `stt`, `command`, `pick`, `return`, `vlm`, `yolo`, `hand_grasp`, `moveit`, `gripper`, `launch`
- `step`: 작업 단계. `startup`, `request`, `model_load`, `detect`, `inference`, `step`, `queue`, `motion`, `trajectory`
- `event`: 단계 상태. `start`, `done`, `fail`, `skip`, `status`, `pause`, `resume`, `cancel`, `timeout`, `interrupted`

선택 필드:

- `target`: 공구, 객체, 토픽, 서비스 대상
- `reason`: 실패, skip, timeout, cancel 원인
- `dur_ms`: 처리 시간. 밀리초 정수
- `file`: 예외 발생 파일명
- `msg`: 보조 설명. 가능하면 짧게 작성하고, 원인은 `reason`에 넣는다.

## 사용 방법

ROS 노드에서는 `MacGyvbotLogger`를 노드 초기화 시 한 번 만든다.

```python
from macgyvbot_domain.logging import MacGyvbotLogger, exception_log_fields

self.service_log = MacGyvbotLogger(super().get_logger(), svc="task", pipe="pick")
```

로그를 남길 때는 `step`, `event`, 필요한 필드를 분리한다.

```python
self.service_log.info(
    "step",
    "start",
    pipe="pick",
    target=tool_name,
    step_name=step.name,
    msg="task step started",
)
```

예외는 `exception_log_fields(exc)`를 붙인다.

```python
try:
    result = run_inference()
except Exception as exc:
    self.service_log.warn(
        "inference",
        "fail",
        pipe="vlm",
        target=target_label,
        **exception_log_fields(exc),
    )
```

ROS 노드가 아닌 파일에서 임시 출력이 필요하면 `format_log_event(LogEvent(...))`를 사용한다.
단, 가능하면 호출부에서 `MacGyvbotLogger`를 전달받는 구조를 우선한다.

## 좋은 로그 예시

```text
svc=task pipe=pick step=queue event=done target=screwdriver step_count=8
svc=task pipe=pick step=step event=fail target=screwdriver step_name=grasp reason=task_step_failed dur_ms=1230
svc=perception pipe=vlm step=inference event=done target=screwdriver mode=qwen3b dur_ms=21431
svc=manipulation pipe=gripper step=motion event=close_start force=400
```

## 피해야 할 로그

아래처럼 자유 문장을 직접 출력하지 않는다.

```python
self.get_logger().info("VLM inference failed")
print("Start closing gripper.")
```

아래처럼 너무 긴 문장을 `msg`에만 넣고 원인 필드를 비워두지 않는다.

```text
svc=task pipe=legacy step=log event=status msg="긴 설명..."
```

대신 이렇게 쓴다.

```text
svc=task pipe=pick step=step event=fail target=screwdriver reason=motion_failed step_name=approach dur_ms=820
```

## GUI 전달 기준

GUI 채팅창에는 모든 로그를 보여주지 않는다. 기본적으로 아래 이벤트만 후보로 삼는다.

- 작업 요청 접수
- 작업 시작
- 사용자 행동 필요
- 완료
- 실패
- 안전 관련 중단
- pause, resume, cancel, exit
- VLM 로딩 지연, VLM 준비 완료, VLM 실패

반복 루프, 내부 retry, 좌표 계산, 모델 내부 디버그, 외부 라이브러리 로그는 GUI 채팅창으로 보내지 않는다.

## 새 로그 추가 전 체크리스트

- [ ] `svc`, `pipe`, `step`, `event`가 명확한가?
- [ ] 실패 로그라면 `reason`이 있는가?
- [ ] 예외 로그라면 `exception_log_fields(exc)`를 사용했는가?
- [ ] 한 줄로 끝나는가?
- [ ] 같은 의미의 중복 로그를 만들지 않았는가?
- [ ] 사용자에게 보여야 하는 메시지라면 로그가 아니라 상태/피드백 토픽으로 전달했는가?
- [ ] 로봇 안전 파라미터나 동작 정책을 건드리지 않았는가?

