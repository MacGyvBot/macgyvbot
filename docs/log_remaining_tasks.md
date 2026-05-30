# 로그 리팩토링 남은 작업

현재 서비스 로그는 `MacGyvbotLogger`와 key-value 포맷을 통과하도록 정리되어 있다.
다음 단계의 목표는 중복 로그와 `legacy` 로그를 줄이고, GUI 채팅창에 올릴 핵심 이벤트만 안정적으로 선별하는 것이다.

## 1. legacy 로그 제거

- [ ] `service_log.bind("legacy")` 호출 목록을 패키지별로 다시 확인한다.
- [ ] 같은 위치에 구조화 로그와 legacy 로그가 함께 있으면 legacy 로그를 삭제한다.
- [ ] 필요한 정보가 legacy `msg`에만 있으면 `target`, `reason`, `step_name`, `mode`, `topic` 같은 필드로 옮긴다.
- [ ] 제거 후 `rg "legacy"`로 남은 지점을 확인한다.

## 2. task 파이프라인 로그 정리

- [ ] `task_coordinator_node.py`의 queue/step 로그를 기준 로그로 확정한다.
- [ ] pick, return 세부 flow에서 반복적으로 나오는 자유 문장 로그를 `step/event/reason` 구조로 바꾼다.
- [ ] pause/resume/cancel/exit 로그의 `pipe`, `reason`, `target` 필드 사용을 통일한다.
- [ ] task 실패 시 `task_step_failed`, `task_step_exception`, `motion_failed`, `perception_failed` 같은 reason 값을 정리한다.

## 3. perception 로그 정리

- [ ] YOLO detect 로그가 너무 자주 찍히면 frame 단위 로그 레벨 또는 샘플링 정책을 정한다.
- [ ] VLM service/client 로그의 request id 또는 timestamp 연결 방법을 정한다.
- [ ] `grasp_point_selector.py`와 하위 selector의 자유 문장 로그를 구조화 필드로 더 세분화한다.
- [ ] hand grasp detection의 image/model/segmentation/classification 로그 reason 값을 표준화한다.

## 4. manipulation 로그 정리

- [ ] gripper 로그의 `svc`와 `pipe`가 호출 맥락과 맞는지 확인한다.
- [ ] grasp verifier의 legacy-style `msg` 로그를 구조화 로그로 바꾼다.
- [ ] force descent 로그에서 정상 종료와 안전 종료의 `event`/`reason`을 구분한다.
- [ ] MoveIt motion 실패 로그에 가능한 경우 `target`, `pose`, `step_name`을 추가한다.

## 5. command/UI 로그 정리

- [ ] command input의 STT, command publish, task control publish 로그를 기준 이벤트로 확정한다.
- [ ] TTS 서비스 내부 로그도 같은 포맷으로 맞출지 결정한다.
- [ ] operator UI의 `_append_log()`가 ROS 로그와 GUI 표시를 동시에 담당하는 구조를 분리할지 검토한다.
- [ ] GUI 채팅창에 보여줄 상태 이벤트 목록을 코드 상수로 만든다.

## 6. GUI 핵심 이벤트 전달 정책

- [ ] GUI에 표시할 이벤트 필터 조건을 정의한다.
- [ ] 후보 필드: `svc`, `pipe`, `step`, `event`, `target`, `reason`
- [ ] 표시 대상 예시: request accepted, task started, user action required, done, fail, safety stop, pause/resume/cancel/exit, VLM ready/fail
- [ ] 표시 제외 예시: 반복 detect, 내부 retry, 좌표 계산, 모델 디버그, 외부 라이브러리 로그

## 7. 검증

- [ ] `python -m compileall -q src`
- [ ] 로그 관련 단위 테스트 실행
- [ ] task/manipulation/perception 기존 테스트 실행
- [ ] `rg "self.get_logger\\(\\)\\.|node.get_logger\\(\\)\\.|print\\(" src -g "*.py"`로 원시 로그 재확인
- [ ] 실제 launch 시 터미널에서 한 줄 로그가 깨지지 않는지 확인

## 참고 문서

- 루트 규칙: `LogRule.md`
- 상세 아키텍처 문서: `docs/architecture/logging.md`
- 완료된 1차 리팩토링 기록: `docs/log_refact.md`

