# MacGyvBot 다중 패키지 마이그레이션

이 문서는 기존 루트 `macgyvbot` ROS 패키지를 `src/` 아래의 workspace형
다중 ROS 2 패키지 구조로 전환한 방식과, 앞으로의 개발 기준을 정리한다.

현재 `src/` 아래 패키지들이 실제 production runtime 경로다. 저장소 루트는
더 이상 ROS 패키지가 아니라 colcon workspace root다.

## 적용된 마이그레이션 구조

현재 MacGyvBot은 다음 ROS 2 workspace 구조를 따른다.

```text
macgyvbot/
  src/
    macgyvbot_interfaces/
    macgyvbot_config/
    macgyvbot_domain/
    macgyvbot_resources/
    macgyvbot_bringup/
    macgyvbot_command/
    macgyvbot_perception/
    macgyvbot_manipulation/
    macgyvbot_task/
    macgyvbot_ui/
```

저장소 루트는 문서, `requirements.txt`, `.gitignore`, colcon 산출물 같은
workspace 수준 파일만 소유한다. 루트에는 ROS 패키지 메타데이터나 런타임
리소스를 다시 두지 않는다.

루트에 다시 만들지 말아야 할 항목:

- `package.xml`
- `setup.py`
- `setup.cfg`
- `resource/<package>`
- `launch/`
- `config/`
- `weights/`
- `calibration/`
- `test/`

현재 실행 entrypoint는 다음 명령이다.

```bash
ros2 launch macgyvbot_bringup macgyvbot.launch.py
```

아래 옛 entrypoint는 폐기됐다.

```bash
ros2 launch macgyvbot macgyvbot.launch.py
```

명시적으로 별도 호환 패키지를 설계하는 경우가 아니라면 이 entrypoint를 다시
도입하지 않는다.

## 패키지별 책임

### `macgyvbot_interfaces`

공유 ROS 메시지 계약을 소유한다.

현재 포함된 메시지:

- `CommandFeedback`
- `ToolCommand`
- `RobotTaskStatus`
- `HumanGraspResult`
- `ToolMaskLock`

새로운 ROS topic/service/action 계약이 여러 패키지 사이의 경계가 된다면 이
패키지에 추가한다.

### `macgyvbot_config`

공유 Python runtime 상수를 소유한다.

여기에 둘 항목:

- frame 이름
- link 이름
- topic 이름
- 기본 모델 파일명
- grasp/pick/return/handoff threshold
- timing constant
- UI 이름
- VLM 기본값

launch YAML은 여기에 두지 않는다. launch/runtime YAML은
`macgyvbot_bringup`의 책임이다.

### `macgyvbot_domain`

패키지 사이에서 공유되는 in-process Python dataclass와 domain model을 소유한다.

ROS serialization 경계가 아니라 Python 코드 내부에서 여러 패키지가 같은 typed
object를 써야 할 때 이 패키지에 둔다.

### `macgyvbot_resources`

코드가 아닌 runtime asset을 소유한다.

소스 트리 기준 위치:

- `src/macgyvbot_resources/calibration/`
- `src/macgyvbot_resources/weights/`
- `src/macgyvbot_resources/weights/vlm/`

설치 후에는 다음 package share 경로로 접근한다.

```text
share/macgyvbot_resources/calibration/
share/macgyvbot_resources/weights/
```

대용량 모델 파일은 Git에 포함하지 않는다. 다운로드 스크립트와 `.gitkeep` 같은
placeholder만 추적한다.

### `macgyvbot_bringup`

launch 파일과 launch/runtime YAML을 소유한다.

현재 `moveit_py.yaml` 위치:

```text
src/macgyvbot_bringup/config/moveit_py.yaml
```

launch 파일에서는 repository path를 하드코딩하지 않는다.
`FindPackageShare`와 `PathJoinSubstitution`으로 package share 경로를 조합한다.

`macgyvbot_bringup`은 노드와 파라미터를 조합하는 패키지다. business logic,
명령 파싱, perception 알고리즘, robot control 로직은 여기에 두지 않는다.

### `macgyvbot_command`

명령 입력 계층을 소유한다.

여기에 둘 항목:

- command input node
- command parser
- command validation
- STT helper
- TTS helper
- command feedback 처리
- command-coupled GUI widget

수동 LLM 평가 스크립트는 `src/macgyvbot_command/test/`에 둔다.

### `macgyvbot_perception`

perception 계층을 소유한다.

여기에 둘 항목:

- camera input 처리
- YOLO wrapper
- depth projection
- pick target resolution
- grasp point selection
- hand grasp detection
- SAM/mask helper
- VLM grasp-point inference

perception 테스트는 `src/macgyvbot_perception/test/`에 둔다.

내부 perception 네이밍 규칙:

- `grasp_point/`: 로봇이 잡을 image-space grasp point를 선택하는 코드
- `hand_tool_grasp/`: 사용자 손이 공구를 잡았는지 판단하는 코드

ROS topic, executable, model filename의 `hand_grasp` 이름은 외부 호환성을 위해
유지한다.

### `macgyvbot_manipulation`

로봇 조작 계층을 소유한다.

여기에 둘 항목:

- MoveIt adapter
- gripper control
- force sensing
- robot pose helper
- safe workspace clamp
- grasp verification
- handoff targeting

manipulation 테스트는 `src/macgyvbot_manipulation/test/`에 둔다.

### `macgyvbot_task`

main task 실행과 workflow orchestration을 소유한다.

여기에 둘 항목:

- main task executable
- command routing
- robot task status publishing
- pick workflow
- return workflow
- handoff workflow 연결

task 패키지는 orchestration 계층이다. robot adapter 세부 구현이나 perception
알고리즘을 여기로 가져오지 않는다.

### `macgyvbot_ui`

미래의 독립 UI 경계로 예약된 패키지다.

현재 command와 강하게 결합된 GUI는 `macgyvbot_command` 안에 둔다. UI 코드가
command 입력과 분리되어 독립적으로 재사용되거나, 별도 operator UI로 커질 때
`macgyvbot_ui`로 이동한다.

## 현재 적용 중인 규칙

- 루트 `macgyvbot` Python 구현 패키지는 폐기됐다.
- 새 production 코드는 책임을 소유한 패키지 아래에 둔다.
- 같은 production 구현을 두 패키지에 중복 보관하지 않는다.
- 루트에 ROS 패키지 메타데이터를 다시 만들지 않는다.
- 루트에 runtime asset 디렉터리를 다시 만들지 않는다.
- 테스트는 검증 대상 패키지의 `test/` 아래에 둔다.
- runtime asset은 package share 경로를 먼저 사용하고, 개발 편의를 위해
  package-local source-tree fallback만 허용한다.
- 현재 demo 안정성을 위해 JSON topic payload는 유지한다.
- typed message는 `macgyvbot_interfaces`를 통해 점진적으로 병행 도입한다.

## 검증 기준

마이그레이션 또는 구조 변경 후 기본 검증 명령은 다음과 같다.

```bash
cd /mnt/c/Users/lmhst/git/MacGyvBot/macgyvbot
source /opt/ros/humble/setup.bash
colcon build --symlink-install
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test --event-handlers console_direct+
```

현재 WSL 환경에서는 사용자 local pytest plugin 충돌을 피하기 위해
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`을 사용한다.

기대되는 package discovery:

```text
macgyvbot_bringup
macgyvbot_command
macgyvbot_config
macgyvbot_domain
macgyvbot_interfaces
macgyvbot_manipulation
macgyvbot_perception
macgyvbot_resources
macgyvbot_task
macgyvbot_ui
```

## 마이그레이션 단계 기록

### M1. 패키지 경계 준비

- 초기 단계에서는 루트 패키지 실행을 유지했다.
- `src/macgyvbot_*` 패키지들이 각각 build 가능하도록 만들었다.
- 공유 typed interface를 `macgyvbot_interfaces`에 정의했다.
- 공유 Python 상수를 `macgyvbot_config`로 분리했다.
- 공유 Python dataclass를 `macgyvbot_domain`으로 분리했다.
- `macgyvbot_bringup`에 compatibility launch를 추가했다.
- target package에 wrapper executable을 추가해 기존 루트 node 구현을 위임했다.
- wrapper executable 기반 migrated launch path를 추가했다.

완료 기준:

- 각 `package.xml`이 parse된다.
- Python entrypoint가 compile된다.
- ROS 2 환경에서 `macgyvbot_bringup`이 wrapper executable을 launch할 수 있다.

### M2. Command 패키지 마이그레이션

- command parser, STT, TTS, command input node 소유권을
  `macgyvbot_command`로 이동했다.
- command와 강하게 결합된 UI는 아직 `macgyvbot_command` 안에 둔다.
- 기존 JSON topic은 유지하고 typed topic은 병행 도입 대상으로 남겼다.

완료 기준:

- 기존 command input 동작이 유지된다.
- `/tool_command`, `/command_feedback` 호환성이 유지된다.
- command package가 executable과 test를 소유한다.

### M3. Perception 패키지 마이그레이션

- depth projection, target resolution, grasp-point selection 같은 순수
  perception helper를 먼저 이동했다.
- model/weight path 처리를 확인한 뒤 hand grasp detection helper와 node를
  `macgyvbot_perception`으로 이동했다.
- model weight 설치는 `macgyvbot_resources`가 소유하도록 했다.

완료 기준:

- perception helper를 `macgyvbot_perception`에서 import할 수 있다.
- hand grasp JSON payload 호환성이 유지된다.
- typed `HumanGraspResult`가 병행 전환 대상으로 준비되어 있다.

### M4. Manipulation 패키지 마이그레이션

- MoveIt, gripper, force sensing, robot pose, safe workspace, handoff targeting
  adapter를 `macgyvbot_manipulation`으로 이동했다.
- safety-critical clamp는 MoveIt 실행 adapter 직전에 유지한다.

완료 기준:

- 기존 pick/return motion 동작이 유지된다.
- safe workspace clamp가 planning/execution 직전에 적용된다.
- gripper와 force adapter에 package-level test 또는 mock check가 있다.

### M5. Task 패키지 마이그레이션

- pick/return workflow, command routing, status publishing을
  `macgyvbot_task`로 이동했다.
- 공유 Python dataclass는 `macgyvbot_domain`에 유지했다.
- command, perception, manipulation contract가 안정된 뒤 root import를 package
  import로 교체했다.

완료 기준:

- main task executable을 `macgyvbot_task`가 소유한다.
- pick/return flow가 기존 JSON topic shape를 유지한다.
- workflow dependency가 명시적인 interface를 통해 주입된다.

### M6. Bringup 소유권 전환

- `macgyvbot_bringup` launch가 migrated package executable을 실행하도록
  수정했다.
- launch/config 소유권을 root package에서 제거했다.
- 전환 기간 확인용 package-local launch alias를 유지했다.

완료 기준:

- `ros2 launch macgyvbot_bringup macgyvbot.launch.py`가 기본 entrypoint다.
- root launch는 확인 후 제거된다.

### M7. 루트 패키지 폐기

- 모든 runtime 소유권을 옮긴 뒤 root `macgyvbot` 패키지를 제거했다.
- 중복 console script를 제거했다.
- 오래된 import path와 문서를 정리했다.

완료 기준:

- production node가 root `macgyvbot.*` 구현 경로에 의존하지 않는다.
- bringup, command, perception, manipulation, task, UI 패키지가 ROS 2
  workspace에서 함께 build된다.

## 현재 상태

- M6 완료: `macgyvbot_bringup/macgyvbot.launch.py`가 migrated package
  executable을 실행한다.
- M7 완료: root `macgyvbot` Python 구현 패키지를 제거했다.
- root 중복 console script를 제거했다.
- 저장소 루트는 workspace root만 담당한다.
- runtime resource는 `src/macgyvbot_resources` 아래로 이동했다.
- launch/runtime YAML은 `src/macgyvbot_bringup/config` 아래로 이동했다.
- test는 package-local `test/` 디렉터리로 이동했다.
- WSL에서 workspace build와 package-local test가 통과했다.

## 앞으로의 개발 방향

1. 패키지 경계를 유지한다.
   - 새 코드는 책임을 소유한 패키지에 추가한다.
   - 여러 unrelated package를 동시에 import해야 하는 함수가 생기면 shared type,
     constant, interface 위치를 재검토한다.
   - shared Python type은 `macgyvbot_domain`, shared constant는
     `macgyvbot_config`, ROS 경계 계약은 `macgyvbot_interfaces`로 이동한다.

2. JSON compatibility topic을 typed interface로 점진 전환한다.
   - demo 안정성을 위해 기존 JSON topic은 유지한다.
   - contract가 안정된 경로부터 typed publisher/subscriber를 병행 추가한다.
   - launch와 UI consumer가 모두 전환된 뒤에만 JSON compatibility를 제거한다.

3. `macgyvbot_bringup`은 선언적으로 유지한다.
   - launch 파일은 node와 parameter를 조합한다.
   - business logic, parser logic, perception algorithm, robot control behavior를
     launch 파일에 넣지 않는다.

4. resource는 package-owned로 유지한다.
   - calibration file, model checkpoint, model download helper, local VLM cache는
     `macgyvbot_resources` 아래에 둔다.
   - 코드는 repository root에 `weights/`나 `calibration/`이 있다고 가정하지
     않는다.

5. test는 package-local로 유지한다.
   - perception 동작 테스트는 `macgyvbot_perception/test`에 둔다.
   - gripper, force, pose, MoveIt, safe-zone 테스트는
     `macgyvbot_manipulation/test`에 둔다.
   - command parser, STT/TTS, command UI 테스트는 `macgyvbot_command/test`에
     둔다.
   - launch/config/linter 테스트는 `macgyvbot_bringup/test`에 둔다.

6. `macgyvbot_ui`는 미래의 독립 UI 경계로 다룬다.
   - UI 코드가 command-specific이면 `macgyvbot_command`에 둔다.
   - UI 코드가 독립 operator UI로 커지면 `macgyvbot_ui`로 이동한다.

7. 구조 변경 후에는 반드시 검증한다.
   - 최소한 영향을 받은 package build와 test를 실행한다.
   - migration 작업을 완료했다고 보기 전에는 full workspace build를 실행한다.
