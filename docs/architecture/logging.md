# MacGyvBot 로그 계약

이 문서는 MacGyvBot 서비스 코드가 출력하는 런타임 로그의 공통 계약을
정의한다. ROS 2, MoveIt, 컨트롤러, 카메라, 외부 라이브러리 로그는 이 계약의
직접 대상이 아니다.

## 기본 원칙

- MacGyvBot 서비스 로그는 한 줄 key-value 형식을 사용한다.
- 한 이벤트는 한 줄에 끝나야 한다.
- 사용자가 봐야 하는 문장은 `/robot_task_status`와 `/command_feedback`을 통해
  GUI로 전달한다.
- 콘솔 로그는 원인 분석과 개발자 진단을 위한 정보에 집중한다.
- 로봇 동작 파라미터, 안전 임계값, 토픽 계약은 로그 리팩토링 과정에서
  변경하지 않는다.

## 표준 형식

```text
svc=task pipe=pick step=grasp event=start target=screwdriver msg="grasp attempt" attempt=1
svc=task pipe=pick step=grasp event=fail target=screwdriver reason=gripper_status dur_ms=820 file=grasp_verifier.py
svc=perception pipe=vlm step=model_load event=done model=qwen3b dur_ms=21431
```

## 필드

- `svc`: 서비스 영역. 예: `command`, `task`, `perception`, `manipulation`,
  `ui`, `monitor`, `system`.
- `pipe`: 상위 파이프라인. 예: `pick`, `return`, `vlm`, `stt`, `tts`,
  `moveit`, `drawer`, `handoff`, `system`.
- `step`: 현재 작업 단계.
- `event`: 단계 생명주기. 예: `start`, `done`, `fail`, `skip`, `retry`,
  `cancel`, `pause`, `resume`, `timeout`, `status`.
- `target`: 공구, 서랍, 손, 모델, 프레임 등 작업 대상.
- `reason`: 실패나 skip의 기계 판독 가능한 원인.
- `dur_ms`: 선택 필드. 작업 소요 시간, 밀리초 단위.
- `file`: 선택 필드. 실패가 발생한 짧은 파일명.
- `msg`: 짧은 사람용 설명.

## GUI 표시 정책

GUI 채팅에는 모든 로그를 올리지 않는다. 기본적으로 다음 이벤트만 사용자에게
노출한다.

- 작업 요청 접수
- 작업 시작
- 사용자 행동 필요
- 완료
- 실패
- 안전 관련 중단
- pause, resume, cancel, exit
- VLM 로딩 지연, VLM 준비 완료, VLM 실패

상세 디버그 로그, 반복 폴링, 내부 retry, 외부 라이브러리 로그는 GUI 채팅에
올리지 않는다.

## 외부 로그 소음 제어

MacGyvBot service event는 위 key-value 형식에 의존한다. ROS 2, MoveIt,
컨트롤러, 카메라, 외부 라이브러리 로그는 같은 형식으로 강제하지 않는다.

운영 중 콘솔 가독성이 필요하면 launch 또는 실행 명령에서 외부 노드 log level을
`warn` 이상으로 낮춘다. 예:

```bash
ros2 launch macgyvbot_bringup macgyvbot.launch.py --ros-args --log-level warn
```

진단 시에는 MacGyvBot 노드만 `info` 또는 `debug`로 올리고, MoveIt/카메라/외부
드라이버는 가능한 한 `warn`으로 둔다. 특정 노드만 자세히 보고 싶을 때는 ROS 2
logger 이름을 지정해 개별 log level을 조정한다.

```bash
ros2 run macgyvbot_task task_coordinator_node --ros-args \
  --log-level task_coordinator_node:=debug \
  --log-level moveit:=warn
```
