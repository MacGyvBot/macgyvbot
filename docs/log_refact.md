# 로그 리팩토링 계획 및 진행 현황

## 목표

MacGyvBot 서비스 코드에서 직접 출력하던 로그를 하나의 형식으로 통일한다.
터미널에서는 ROS 2, MoveIt 등 외부 로그와 섞여도 우리 서비스 로그를 한 줄로 식별할 수 있어야 하며,
추후 GUI 채팅창에는 이 통일 로그 중 핵심 이벤트만 전달할 수 있어야 한다.

## 표준 로그 형식

기본 형식은 한 줄 key-value 로그다.

```text
svc=<service> pipe=<pipeline> step=<step> event=<event> target=<target> reason=<reason> dur_ms=<duration> file=<file> msg=<message>
```

필수 필드:

- `svc`: 서비스 또는 패키지 영역. 예: `task`, `perception`, `manipulation`, `command`, `ui`, `monitor`
- `pipe`: 파이프라인. 예: `pick`, `return`, `vlm`, `yolo`, `hand_grasp`, `motion`
- `step`: 현재 단계. 예: `startup`, `request`, `response`, `trajectory`
- `event`: 상태 변화. 예: `started`, `ready`, `published`, `failed`, `timeout`

선택 필드:

- `target`: 대상 공구, 객체, 토픽 등
- `reason`: 실패 또는 분기 원인
- `dur_ms`: 처리 시간
- `file`: 예외 발생 파일
- `msg`: 기존 자유 문장 로그를 임시로 담는 필드

## 진행 체크리스트

- [x] `macgyvbot_domain.logging`에 공용 로그 포맷터와 `MacGyvbotLogger` 어댑터를 추가했다.
- [x] 예외 로그용 `exception_log_fields()`를 추가해 예외 타입, 원인, 파일명을 한 줄 필드로 정리했다.
- [x] 기존 한 문장 로그도 어댑터를 통과하면 `svc=... pipe=... step=log event=status msg=...` 형식으로 출력되도록 했다.
- [x] task coordinator 로그를 `service_log` 기반으로 통일했다.
- [x] main task router 로그를 `service_log` 기반으로 통일했다.
- [x] command input 로그를 `service_log` 기반으로 통일했다.
- [x] operator UI 로그를 `service_log` 기반으로 통일했다.
- [x] VLM grasp service 로그를 `service_log` 기반으로 통일했다.
- [x] VLM grasp client 로그를 `service_log` 기반으로 통일했다.
- [x] YOLO detector 로그를 통일 로그 어댑터로 출력하도록 정리했다.
- [x] hand grasp detection 로그를 `service_log` 기반으로 통일했다.
- [x] grasp point selector 로그를 통일 로그 어댑터와 상태 발행 흐름에 맞게 정리했다.
- [x] manipulation의 grasp verifier, force detection 로그를 통일했다.
- [x] OnRobot gripper의 `print()` 기반 출력 로그를 통일 형식으로 교체했다.
- [x] launch monitor 자체 로그를 `svc=monitor pipe=launch ...` 형식으로 교체했다.
- [x] 서비스 런타임 영역에서 직접 `self.get_logger().info/warn/error`, `node.get_logger().info/warn/error`, `print()`를 사용하는 지점을 제거했다.
- [x] 수동 테스트 스크립트의 `print()`는 서비스 런타임 로그가 아니므로 리팩토링 범위에서 제외했다.
- [x] `README.md`에 `macgyvbot_domain`의 공용 로그 헬퍼 역할을 반영했다.
- [x] `docs/architecture/logging.md`에 표준 로그 정책을 추가했다.
- [x] 로그 포맷터 단위 테스트를 추가했다.
- [x] `python -m compileall -q src`로 전체 컴파일을 확인했다.
- [x] 로그 및 관련 task/manipulation/perception 테스트를 실행했다.

## 후속 작업 후보

- GUI 채팅창으로 전달할 핵심 이벤트 목록을 별도 정책으로 확정한다.
- ROS 2/MoveIt 외부 로그와 우리 서비스 로그를 터미널에서 분리해서 보는 실행 옵션을 정리한다.
- 기존 `msg=` 기반 임시 로그 중 중요한 흐름은 단계별 `step/event/reason` 필드로 더 세분화한다.

## 검증 명령

```bash
python -m compileall -q src
```

```bash
PYTHONPATH=src/macgyvbot_task:src/macgyvbot_manipulation:src/macgyvbot_perception:src/macgyvbot_config:src/macgyvbot_domain:src/macgyvbot_interfaces:src/macgyvbot_resources \
python -m pytest -q \
  src/macgyvbot_domain/test/test_logging.py \
  src/macgyvbot_task/test \
  src/macgyvbot_manipulation/test/test_handover_targeting.py \
  src/macgyvbot_manipulation/test/test_gripper_grasp.py \
  src/macgyvbot_perception/test/test_hand_grasp_ml_mask.py
```
