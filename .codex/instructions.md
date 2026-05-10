# MacGyvBot Code Conventions

이 저장소에서 코드를 작성하거나 수정할 때는 아래 규칙을 따른다. 브랜치, 커밋, PR, 안전 리뷰 규칙은 `CONTRIBUTING.md`를 우선한다.

## 기본 원칙

- 변경 범위는 요청된 기능이나 버그에 필요한 파일로 제한한다.
- 로봇 제어, 그리퍼, 서랍 개폐, 사용자 전달 동작은 안전 영향을 먼저 검토한다.
- 실제 장비 실행이 필요한 변경은 dry-run, 시뮬레이션, 낮은 속도 제한을 우선 적용한다.
- 대용량 모델 파일, 데이터셋, 영상, ROS bag 파일은 저장소에 추가하지 않는다.
- 새 기능은 역할별 모듈에 배치하고, ROS node 파일은 wiring 중심으로 얇게 유지한다.

## 패키지 구조 기준

- `macgyvbot/nodes/`: ROS node entrypoint, subscription/publisher, parameter, launch wiring
- `macgyvbot/config/`: topic, frame, offset, mode 같은 공통 설정
- `macgyvbot/util/command_input/`: STT, GUI 입력, local parser, LLM fallback 같은 명령 입력 도메인 로직
- `macgyvbot/util/hand_grasp_detection/`: 사용자 손과 공구 grasp 상태 판단 로직
- `macgyvbot/util/macgyvbot_main/perception/`: YOLO 인식, depth projection, 좌표 변환
- `macgyvbot/util/macgyvbot_main/grasp_mechanism/`: bbox center/VLM 기반 grasp point 선택
- `macgyvbot/util/macgyvbot_main/model_control/`: MoveIt planning, pose helper, gripper/robot motion adapter
- `macgyvbot/util/macgyvbot_main/task_pipeline/`: pick, handoff, return 같은 시퀀스 orchestration
- `macgyvbot/ui/`: PyQt command input GUI처럼 ROS node에서 분리 가능한 UI helper
- 기존 standalone 노드가 있는 경우, 새 노드는 우선 `macgyvbot/nodes/`에 추가하고 `setup.py` console script만 연결한다.
- 여러 노드가 공유하는 topic, frame, offset, mode 상수는 가능한 한 `macgyvbot/config/config.py` 또는 도메인별 config 모듈로 모은다.
- README의 패키지 구조 트리와 실제 파일 배치가 어긋나면 함께 수정한다.

## Python 스타일

- Python 3.10과 ROS 2 Humble 환경을 기준으로 작성한다.
- PEP 8을 따르고 `ament_flake8`, `ament_pep257` 테스트를 통과하도록 유지한다.
- 들여쓰기는 공백 4칸을 사용한다.
- import는 표준 라이브러리, 서드파티, ROS/프로젝트 모듈 순서로 그룹화한다.
- 상수는 모듈 상단에 `UPPER_SNAKE_CASE`로 선언한다.
- 함수와 변수는 `snake_case`, 클래스는 `PascalCase`를 사용한다.
- 한 함수가 너무 많은 책임을 갖지 않도록 좌표 변환, 계획 실행, 안전 제한 계산, ROS 콜백을 분리한다.
- 복잡한 로봇 동작이나 안전 제한에는 짧은 주석을 남기되, 코드가 그대로 설명하는 내용은 주석으로 반복하지 않는다.

## ROS 2 규칙

- 노드 이름, topic, service, action 이름은 역할이 명확하게 드러나야 한다.
- 새 topic, service, action, parameter를 추가하면 README 또는 관련 launch/config 문서도 함께 갱신한다.
- 하드코딩된 장비 주소, frame 이름, 속도 제한, 경로는 가능한 한 launch parameter 또는 YAML 설정으로 분리한다.
- 새 노드나 실행 파일을 추가하면 `setup.py`, `package.xml`, launch 파일을 함께 확인한다.
- frame 변환은 단위를 명확히 유지한다. depth, 좌표, 오프셋 값은 meter 기준을 기본으로 한다.
- ROS 로그는 `get_logger()`를 사용하고, 실제 작업자가 이해하기 쉬운 한국어 메시지를 선호한다.
- `macgyvbot.launch.py`에 launch argument를 추가하면 해당 노드 parameter에도 실제로 전달되는지 확인한다.
- 기존 topic은 즉시 제거하지 말고 remap, parameter, wrapper 등을 통해 가능한 한 호환성을 유지한다.

## 안전 규칙

- 로봇팔 또는 그리퍼 이동을 추가할 때는 최소한 다음 제한을 검토한다.
  - 작업 공간 경계
  - 속도 및 가속도 제한
  - 접근 높이와 파지 높이 제한
  - 사용자 손과 공구 사이의 최소 거리
  - 실패 시 정지 또는 복귀 동작
- 안전 관련 상수는 이름만 보고 의도를 알 수 있게 작성한다.
- 보정값, 좌표계, hand-eye transform을 바꾸는 변경은 로그와 검증 절차를 함께 남긴다.
- planning 실패, depth 결측, 검출 범위 초과, 비상정지 상황은 무시하지 말고 명시적으로 처리한다.

## 테스트와 검증

- Python 코드 변경 후 가능한 경우 아래 테스트를 실행한다.

```bash
colcon test --packages-select macgyvbot
colcon test-result --verbose
```

- ROS 2/MoveIt/카메라 장비가 필요한 테스트를 실행하지 못했다면, 최종 응답에 실행하지 못한 이유와 남은 검증 항목을 적는다.
- 로봇 동작 변경은 실제 장비 실행 전에 시뮬레이션 또는 낮은 속도 dry-run 결과를 확인한다.

## 문서화

- 사용자 실행 방법, topic 예시, launch 명령이 바뀌면 README를 갱신한다.
- 협업 규칙 변경은 `CONTRIBUTING.md`에 반영하고, Codex 작업 규칙만 `.codex/instructions.md`에 둔다.

## 커밋 메시지

커밋 메시지는 아래 형식을 따른다.

```text
type: 변경 내용 요약
```

type:

- `feat`: 신규 기능 추가
- `fix`: 버그 수정
- `docs`: 문서 수정
- `refactor`: 기능 변화 없는 구조 개선
- `test`: 테스트 추가 또는 수정
- `task`: 설정, 빌드, 기타 일반 작업
- `experiment`: 모델 학습, 로봇 제어, 시뮬레이션 실험
- `safety`: 안전 제한, 비상정지, 전달 동작 관련 수정
