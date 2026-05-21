# MacGyvBot Development Strategy

이 저장소에서 코드를 작성하거나 수정할 때는 아래 규칙을 따른다. 브랜치, 커밋, PR, 안전 리뷰 규칙은 `CONTRIBUTING.md`를 우선한다.

## 기본 원칙

- 변경 범위는 요청된 기능이나 버그에 필요한 파일로 제한한다.
- 로봇 제어, 그리퍼, 서랍 개폐, 사용자 전달 동작은 안전 영향을 먼저 검토한다.
- 실제 장비 실행이 필요한 변경은 dry-run, 시뮬레이션, 낮은 속도 제한을 우선 적용한다.
- 대용량 모델 파일, 데이터셋, 영상, ROS bag 파일은 저장소에 추가하지 않는다.
- 새 기능은 역할별 모듈에 배치하고, ROS node 파일은 topic/parameter wiring 중심으로 얇게 유지한다.
- 새 코드는 `util/` 경로를 만들거나 참조하지 않는다. 현재 구조의 `application/`, `perception/`, `control/`, `domain/`, `command_input/`, `config/` 경계를 따른다.
- 기능 추가는 기존 workflow에 큰 조건문을 누적하기보다, 작은 객체를 만들고 workflow에 주입하는 방식으로 진행한다.
- 리팩토링 PR에서는 기능 동작 변경을 최소화한다. 기능 변경이 필요한 경우 PR 설명과 테스트 범위에 명확히 적는다.

## 패키지 구조 기준

- `macgyvbot/nodes/`: ROS node entrypoint, subscription/publisher, parameter, launch wiring
- `macgyvbot/application/`: pick, return, handoff, command routing 같은 workflow orchestration
- `macgyvbot/domain/`: workflow 간 전달되는 순수 dataclass/model
- `macgyvbot/perception/`: YOLO, depth projection, grasp point selection, hand grasp perception
- `macgyvbot/control/`: MoveIt, pose, safe workspace, gripper, force detection 같은 robot/hardware adapter
- `macgyvbot/recovery/`: 실패 후 복구 정책을 위한 확장 지점. 현재 동작 추가 없이 placeholder로 유지한다.
- `macgyvbot/command_input/`: local parser, LLM fallback, STT, TTS
- `macgyvbot/ui/`: PyQt command input GUI처럼 ROS node에서 분리 가능한 UI helper
- `macgyvbot/config/`: 역할별 runtime constants
- 기존 standalone 노드가 있는 경우, 새 노드는 우선 `macgyvbot/nodes/`에 추가하고 `setup.py` console script만 연결한다.
- 여러 노드가 공유하는 topic, frame, offset, mode 상수는 역할별 `macgyvbot/config/*.py` 모듈로 모은다.
- README의 패키지 구조 트리와 실제 파일 배치가 어긋나면 함께 수정한다.

## `src/` 다중 패키지 전환 기준

- `src/` 아래 패키지는 워크스페이스형 ROS 패키지로 전환하기 위한 스캐폴드이다.
- 현재 production source of truth는 루트 `macgyvbot/` 패키지이다.
- 기존 파일을 그대로 둔 채 전환을 시작한다. 새 패키지로 코드를 옮길 때는 한 책임 단위씩 이동하고, 기존 import 경로 제거는 새 경로가 검증된 뒤 진행한다.
- 루트 패키지와 `src/` 패키지에 같은 production 로직을 장기간 중복 유지하지 않는다.
- 새 shared message/service/action은 `src/macgyvbot_interfaces/`에 둔다.
- 여러 패키지가 공유하는 Python runtime constant는 `src/macgyvbot_config/`에 둔다.
- 여러 패키지가 공유하는 Python dataclass는 `src/macgyvbot_domain/`에 둔다.
- launch와 runtime composition은 `src/macgyvbot_bringup/`으로 이동한다.
- command input, parser, STT/TTS는 `src/macgyvbot_command/`로 이동한다.
- camera, detection, depth, hand grasp 판단은 `src/macgyvbot_perception/`로 이동한다.
- MoveIt, gripper, force, safe workspace, low-level robot adapter는 `src/macgyvbot_manipulation/`으로 이동한다.
- pick/return workflow, command routing, status publishing은 `src/macgyvbot_task/`로 이동한다.
- operator UI adapter는 `src/macgyvbot_ui/`로 이동한다.
- `src/` 패키지에 새 executable을 추가하면 해당 패키지의 `setup.py`, `package.xml`, bringup launch 연결까지 함께 확인한다.

## 현재 주요 객체 역할

- `nodes/macgyvbot_main_node.py`
  - 메인 ROS wiring node이다.
  - camera/depth/camera_info, command, hand grasp result, force/torque topic을 연결한다.
  - command routing, target resolve, pick/return workflow, status payload 생성은 하위 객체에 위임한다.

- `application/tool_command_controller.py`
  - parsed command를 `bring`, `return`, `release`, `stop` 실행 경로로 라우팅한다.
  - ROS message 타입을 직접 알지 않고 callback과 status publisher를 통해 동작한다.

- `application/robot_status_publisher.py`
  - `/robot_task_status` payload shape을 한 곳에서 조립한다.
  - status JSON shape이 바뀌면 여기와 관련 UI 표시 로직을 같이 검토한다.

- `application/pick_frame_processor.py`
  - camera frame에서 YOLO detection을 수행하고, target label이 있을 때 pick target 생성을 요청한다.
  - target 발견 시 pick sequence 시작 callback을 호출한다.

- `perception/pick_target_resolver.py`
  - YOLO box, grasp point selection, depth projection을 묶어 `PickTarget`을 만든다.
  - pick 대상 선택 방식이 바뀌면 이 객체 또는 `grasp_mechanism/`을 먼저 본다.

- `application/pick_target_planner.py`
  - `PickTarget`의 base 좌표를 `PickMotionPlan`으로 변환한다.
  - safe workspace clamp는 여기서 하지 않는다. 실제 pose goal clamp는 `control/moveit_controller.py`에서 planning 직전에 수행한다.

- `application/pick_sequence.py`
  - pick workflow의 순서만 조율한다.
  - target planning, initial grasp, handoff는 각각 `PickTargetPlanner`, `PickGraspFlow`, `PickHandoffFlow`에 위임한다.

- `application/return_sequence.py`
  - return workflow의 순서만 조율한다.
  - 사용자에게서 공구 받기, Home 배치, status reporting은 하위 flow/reporter에 위임한다.

- `nodes/hand_grasp_detection_node.py`
  - human grasp 판단 ROS node이다.
  - 최종 `human_grasped_tool`은 ML grasp, depth contact, locked mask contact가 모두 참일 때만 참으로 판단한다.
  - depth/mask 신호는 payload에 진단값으로 유지한다.

- `control/moveit_controller.py`
  - MoveIt planning/execution adapter이다.
  - pose goal에 대한 `clamp_to_safe_workspace()`는 이 파일에서만 수행한다.

- `control/handover_targeting.py`
  - handoff target 안정화와 handoff 재플랜 후보 생성을 담당한다.
  - planning 실패 시 x를 줄이고 y를 0에 가깝게 보정하며 재시도한다.

## 기능 추가 전략

새 기능을 추가할 때는 먼저 변경의 성격을 분류한다.

- ROS topic, service, action, parameter wiring이면 `nodes/`에 둔다.
- 작업 순서, 상태 전이, pick/return 절차 변경이면 `application/`에 둔다.
- image, depth, YOLO, VLM, mask, hand landmark 판단이면 `perception/`에 둔다.
- MoveIt, gripper, force/torque, pose, safe workspace, hardware I/O이면 `control/`에 둔다.
- 여러 계층에서 공유하는 순수 데이터이면 `domain/`에 dataclass로 둔다.
- 명령어 parsing, STT, TTS, GUI 입력이면 `command_input/` 또는 `ui/`에 둔다.
- 실패 이후 복구 정책이면 `recovery/`에 둔다.
- 설정값이면 역할별 `config/*.py`에 둔다.

일반적인 기능 추가 흐름:

1. 입력과 출력이 ROS인지 순수 Python인지 구분한다.
2. 순수 판단 로직을 먼저 `application/`, `perception/`, `control/` 중 맞는 위치에 작성한다.
3. ROS node는 그 객체를 생성하고 callback에 연결만 한다.
4. workflow에 새 단계를 넣을 때는 sequence 파일에 큰 조건문을 직접 늘리지 말고, 작은 flow/helper 객체를 만든다.
5. 새 payload field를 추가하면 publisher, subscriber, UI 표시, README/EXPLAIN을 같이 확인한다.
6. 실제 로봇 이동이 바뀌면 안전 제한, 실패 처리, 수동 검증 항목을 PR에 적는다.

## 설정 파일 전략

- `config/config.py`는 기존 import 호환용 facade이다. 신규 코드는 가능한 한 역할별 모듈을 직접 import한다.
- `config/robot.py`: MoveIt group, frame, link, Home/observation joint pose
- `config/topics.py`: ROS topic 이름
- `config/models.py`: model/weight 파일명
- `config/pick.py`: pick waypoint 계산 상수
- `config/grasp.py`: gripper grasp 검증 상수
- `config/hand_grasp.py`: human grasp timeout, ML confidence 기준
- `config/handoff.py`: handoff 탐색, 안정화, offset, replan 설정
- `config/return_flow.py`: return force-guided descent 설정
- `config/timing.py`: 공통 polling interval
- `config/ui.py`: OpenCV debug window 이름
- `config/vlm.py`: VLM grasp point mode와 grid 설정
- 상수를 추가할 때는 이름만 보고 단위와 의도가 드러나게 작성한다.
- meter, millimeter, second, degree 같은 단위는 이름에 `_M`, `_MM`, `_SEC`, `_DEG`를 붙인다.

## Python 스타일

- Python 3.10과 ROS 2 Humble 환경을 기준으로 작성한다.
- PEP 8을 따르고 `ament_flake8`, `ament_pep257` 테스트를 통과하도록 유지한다.
- 들여쓰기는 공백 4칸을 사용한다.
- import는 표준 라이브러리, 서드파티, ROS/프로젝트 모듈 순서로 그룹화한다.
- 상수는 모듈 상단에 `UPPER_SNAKE_CASE`로 선언한다.
- 함수와 변수는 `snake_case`, 클래스는 `PascalCase`를 사용한다.
- 한 함수가 너무 많은 책임을 갖지 않도록 좌표 변환, 계획 실행, 안전 제한 계산, ROS 콜백을 분리한다.
- 복잡한 로봇 동작이나 안전 제한에는 짧은 주석을 남기되, 코드가 그대로 설명하는 내용은 주석으로 반복하지 않는다.
- 공개 workflow 결과나 계층 간 전달값은 가능한 한 dict 대신 `domain/` dataclass를 사용한다.
- 파일명은 역할이 드러나게 쓴다. 예: `pick_target_resolver.py`, `return_home_placement_flow.py`, `robot_status_publisher.py`.

## ROS 2 규칙

- 노드 이름, topic, service, action 이름은 역할이 명확하게 드러나야 한다.
- 새 topic, service, action, parameter를 추가하면 README 또는 관련 launch/config 문서도 함께 갱신한다.
- 하드코딩된 장비 주소, frame 이름, 속도 제한, 경로는 가능한 한 launch parameter 또는 YAML 설정으로 분리한다.
- 새 노드나 실행 파일을 추가하면 `setup.py`, `package.xml`, launch 파일을 함께 확인한다.
- frame 변환은 단위를 명확히 유지한다. depth, 좌표, 오프셋 값은 meter 기준을 기본으로 한다.
- ROS 로그는 `get_logger()`를 사용하고, 실제 작업자가 이해하기 쉬운 한국어 메시지를 선호한다.
- `macgyvbot.launch.py`에 launch argument를 추가하면 해당 노드 parameter에도 실제로 전달되는지 확인한다.
- 기존 topic은 즉시 제거하지 말고 remap, parameter, wrapper 등을 통해 가능한 한 호환성을 유지한다.
- node callback 내부에서는 긴 판단 로직을 작성하지 않는다. callback은 parse, validate, delegate, publish 정도만 수행한다.

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
- pose goal의 safe workspace clamp는 `control/moveit_controller.py`의 `plan_and_execute()`에서 수행한다.
- `plan_and_execute()` 호출 전 단계에서 중복으로 `clamp_to_safe_workspace()`를 호출하지 않는다.
- planning 실패 후 재시도 target을 만드는 로직은 `control/handover_targeting.py`처럼 별도 helper에 둔다.
- user handoff 또는 return 관련 변경은 사용자의 손 위치, 공구 위치, gripper open/close 타이밍을 함께 검토한다.

## 테스트와 검증

- Python 코드 변경 후 가능한 경우 아래 테스트를 실행한다.

```bash
colcon test --packages-select macgyvbot
colcon test-result --verbose
```

- ROS 2 의존이 없는 변경은 우선 일반 Python 테스트와 syntax check를 실행한다.

```bash
python -c "from pathlib import Path; files=list(Path('macgyvbot').rglob('*.py'))+list(Path('test').rglob('*.py'))+list(Path('launch').rglob('*.py')); [compile(p.read_text(encoding='utf-8'), str(p), 'exec') for p in files]; print('syntax ok', len(files))"
python -m pytest -q test/test_handover_targeting.py
python -m pytest -q test/test_hand_grasp_ml_mask.py
```

- ROS 2/MoveIt/카메라 장비가 필요한 테스트를 실행하지 못했다면, 최종 응답에 실행하지 못한 이유와 남은 검증 항목을 적는다.
- 로봇 동작 변경은 실제 장비 실행 전에 시뮬레이션 또는 낮은 속도 dry-run 결과를 확인한다.
- 테스트가 ROS/MoveIt/numpy 같은 외부 모듈을 stub 처리하면, 다른 테스트를 오염시키지 않도록 실제 모듈 존재 여부를 먼저 확인한다.

## 문서화

- 사용자 실행 방법, topic 예시, launch 명령이 바뀌면 README를 갱신한다.
- 협업 규칙 변경은 `CONTRIBUTING.md`에 반영하고, Codex 작업 규칙만 `.codex/instructions.md`에 둔다.
- 파일 구조, 주요 workflow, config 구조가 바뀌면 `README.md`와 `EXPLAIN.md`를 함께 갱신한다.
- PR 설명에는 변경 방향, 주요 파일 역할, 실행하지 못한 검증 항목을 명시한다.

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
