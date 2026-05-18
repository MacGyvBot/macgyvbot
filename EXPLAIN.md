# MacGyvBot File Roles

이 문서는 현재 코드 디렉터리를 빠르게 이해하기 위한 구조 설명서입니다.
MacGyvBot은 음성/GUI 명령으로 공구를 가져오고 반납하는 ROS 2 기반 로봇팔
데모이며, 현재 production runtime은 `src/` 아래의 다중 ROS 패키지 구조를
사용합니다.

루트 `macgyvbot` Python 구현 패키지는 마이그레이션 과정에서 제거되었습니다.
저장소 루트는 ROS 패키지가 아니라 colcon workspace root이며, 실행 entrypoint는
`macgyvbot_bringup`이 소유합니다.

## Workspace Shape

```text
.
├── README.md                 # 설치, 실행, 운영 흐름
├── EXPLAIN.md                # 코드 구조와 파일 역할
├── requirements.txt          # Python runtime 의존성
├── CONTRIBUTING.md           # 협업/안전 규칙
├── REFACT.md                 # 리팩토링 기록
├── docs/                     # 브랜치별/보조 문서
└── src/
    ├── macgyvbot_bringup/
    ├── macgyvbot_task/
    ├── macgyvbot_command/
    ├── macgyvbot_perception/
    ├── macgyvbot_manipulation/
    ├── macgyvbot_config/
    ├── macgyvbot_domain/
    ├── macgyvbot_resources/
    ├── macgyvbot_interfaces/
    └── macgyvbot_ui/
```

루트에는 ROS `package.xml`, `setup.py`, `launch/`, `weights/`, `calibration/`을
다시 만들지 않습니다. 런타임 코드와 리소스는 책임을 소유한 `src/macgyvbot_*`
패키지 안에 둡니다.

## Runtime Packages

- `src/macgyvbot_bringup`
  - 기본 launch entrypoint를 소유합니다.
  - `launch/macgyvbot.launch.py`가 `macgyvbot_task`, `macgyvbot_perception`,
    `macgyvbot_command` executable을 함께 실행합니다.
  - `launch/macgyvbot_compat.launch.py`는 같은 primary launch로 위임하는
    전환용 alias입니다.
  - `config/moveit_py.yaml` 같은 launch/runtime YAML을 설치합니다.

- `src/macgyvbot_task`
  - `macgyvbot_main_node.py`와 pick/return application workflow를 소유합니다.
  - `/tool_command`, `/target_label`, 카메라 입력, hand grasp 결과, force/torque
    입력을 받아 task queue로 로봇 작업 시퀀스를 실행합니다.
  - `/robot_task_status`를 발행하고 `/robot_task_control`로 stop/pause/resume
    요청을 받습니다.
  - 주요 하위 경로는 `application/pick_flow/`, `application/return_flow/`,
    `application/task_control/`, `application/commands/`입니다.

- `src/macgyvbot_command`
  - command input node, command parser, STT/TTS helper, command-coupled GUI를
    소유합니다.
  - GUI 키보드 입력과 마이크 STT를 같은 `/stt_text` 흐름으로 처리합니다.
  - 자연어를 `bring`, `return`, `release`, `stop` 명령 JSON으로 해석해
    `/tool_command` 또는 `/robot_task_control`로 보냅니다.
  - `/command_feedback`과 `/robot_task_status`를 GUI/TTS 메시지로 보여줍니다.

- `src/macgyvbot_perception`
  - YOLO detection, depth projection, pick target resolution, grasp point selection,
    hand grasp detection node, SAM/mask helper를 소유합니다.
  - `grasp_point/`는 로봇이 잡을 image-space grasp point 선택 코드입니다.
  - `hand_tool_grasp/`는 사용자의 손이 공구를 잡았는지 판단하는 코드입니다.
  - model path lookup은 `macgyvbot_resources` package share를 우선 사용하고,
    개발 편의를 위해 source-tree fallback을 둡니다.

- `src/macgyvbot_manipulation`
  - MoveIt controller, robot pose helper, safe workspace clamp, OnRobot RG2 gripper,
    grasp verification, force reaction detection, handoff targeting을 소유합니다.
  - MoveIt/그리퍼/힘 감지 같은 하드웨어 adapter 세부 구현은 이 패키지에 둡니다.

- `src/macgyvbot_config`
  - ROS topic 이름, robot frame/link/joint pose, model filename, pick/return/handoff
    threshold, grasp 검증 값, timing, UI, VLM 설정 등 Python runtime constant를
    소유합니다.
  - launch composition이나 YAML은 `macgyvbot_bringup` 책임입니다.

- `src/macgyvbot_domain`
  - package 간 공유되는 in-process Python dataclass를 소유합니다.
  - 현재 `PickTarget`, `PickMotionPlan`을 제공합니다.
  - ROS transport 계약은 여기가 아니라 `macgyvbot_interfaces`에 둡니다.

- `src/macgyvbot_resources`
  - `calibration/`, `weights/`, `weights/vlm/` asset 설치를 소유합니다.
  - 대용량 모델 파일은 Git에 넣지 않고 이 패키지의 source tree나 설치된 package
    share 아래에 배치합니다.

- `src/macgyvbot_interfaces`
  - typed ROS message migration target을 소유합니다.
  - 현재 runtime은 데모 호환성을 위해 JSON over `std_msgs/String`을 유지합니다.
  - `ToolCommand`, `CommandFeedback`, `RobotTaskStatus`, `HumanGraspResult`,
    `ToolMaskLock` 메시지는 양쪽 publisher/subscriber를 함께 옮길 때 사용할
    다음 단계 계약입니다.

- `src/macgyvbot_ui`
  - 독립 UI boundary를 위한 예약 패키지입니다.
  - 현재 command와 강하게 결합된 GUI는 `macgyvbot_command`에 남아 있습니다.

## Executables And Launch

기본 실행:

```bash
ros2 launch macgyvbot_bringup macgyvbot.launch.py
```

이 launch가 띄우는 executable:

```text
macgyvbot_task        macgyvbot
macgyvbot_perception  hand_grasp_detection
macgyvbot_command     command_input_node
```

주요 launch argument:

```text
yolo_model
grasp_model
sam_checkpoint
grasp_point_mode       # vlm 또는 center
sam_enabled
use_voice_command
use_stt
use_tts
parser_mode            # llm_primary 또는 hybrid
llm_model
force_torque_topic
```

`use_stt:=false`는 마이크만 끕니다. GUI와 명령 해석 노드는
`use_voice_command:=true`인 한 계속 실행됩니다.

## Runtime Flow

```text
macgyvbot_command.command_input_node
  -> GUI 입력 또는 마이크 STT 수집
  -> local parser / Ollama LLM parser로 자연어 명령 해석
  -> /tool_command, /command_feedback 발행
  -> stop 계열 제어는 /robot_task_control 발행

macgyvbot_task.macgyvbot_main_node
  -> /tool_command 또는 수동 /target_label 수신
  -> RealSense color/depth 기반 YOLO 탐지
  -> center 또는 VLM 기반 grasp point 선택
  -> depth pixel을 robot base 좌표로 투영
  -> TaskControlCoordinator가 pick/return steps를 worker thread에서 실행
  -> /robot_task_status 발행

macgyvbot_perception.hand_grasp_detection_node
  -> 카메라 image/depth와 /robot_task_status 수신
  -> grasp_success 시점의 tool ROI/SAM mask lock
  -> ML grasp, depth contact, locked mask contact를 합쳐 사용자 grasp 판단
  -> /human_grasped_tool, /hand_grasp_detection/tool_mask_lock 발행
```

## Pick Flow

1. command node가 `bring` 명령을 `/tool_command`로 발행합니다.
2. task node가 target tool label을 설정하고 `/robot_task_status`에
   `accepted/searching` 상태를 냅니다.
3. 최신 camera frame에서 YOLO bbox를 찾습니다.
4. `grasp_point_mode:=vlm`이면 VLM이 crop 안의 grasp point와 yaw를 고르고,
   실패하면 bbox center로 fallback합니다.
5. depth와 `T_gripper2camera.npy`를 사용해 grasp pixel을 robot base 좌표로
   투영합니다.
6. `PickSequenceRunner`가 gripper open, 안전 높이 이동, XY 이동, 접근, 하강,
   gripper close를 task queue step으로 실행합니다.
7. `GraspVerifier`가 OnRobot RG 상태와 gripper 폭으로 로봇 grasp 성공을
   확인합니다.
8. `grasp_success` 상태가 발행되면 hand grasp node가 최신 SAM mask 또는 bbox
   ROI를 lock하고 `/hand_grasp_detection/tool_mask_lock` ack를 보냅니다.
9. task node는 mask lock ack 이후에만 lift와 handoff 이동을 계속합니다.
10. 사용자가 공구를 잡았다고 인식되면 gripper를 열고 Home joint pose로
    복귀한 뒤 `done`을 발행합니다.

## Return Flow

1. command node가 `return` 명령을 `/tool_command`로 발행합니다.
2. task node가 `ReturnSequenceRunner`를 시작하고 gripper를 엽니다.
3. 로봇이 관찰 자세로 이동한 뒤 hand/tool grasp perception 결과에서 사용자가
   들고 있는 공구 위치를 찾습니다.
4. 탐색된 손/공구 위치로 이동해 gripper close를 수행하고 grasp 성공을 확인합니다.
5. Home joint pose로 이동합니다.
6. force feedback으로 Z 하강 중 접촉을 감지하면 하강을 멈춥니다.
7. gripper를 열어 공구를 놓고 Home으로 복귀한 뒤 `done`을 발행합니다.

## Task Control

`macgyvbot_task.application.task_control`은 pick/return workflow를 step queue로
실행합니다. `/robot_task_control`은 JSON 또는 plain string으로 처리됩니다.

```json
{"action":"stop","reason":"operator_request"}
{"action":"pause","reason":"operator_request"}
{"action":"resume","reason":"operator_request"}
```

- `stop`: stop event를 set하고 motion cancel client에 cancel을 요청한 뒤 queue를
  비웁니다.
- `pause`: pause event를 set하고 가능한 motion cancel을 요청합니다. queue runner는
  pause 중 step 실행을 보류합니다.
- `resume`: pause event를 clear하고 `resumed` 상태를 발행합니다.

현재 자연어 parser는 명확한 정지 표현을 `stop`으로 해석합니다. `pause/resume`은
task control topic을 통한 제어 경로가 준비되어 있습니다.

## Main Topics

| Topic | Direction | Type | Owner |
| --- | --- | --- | --- |
| `/tool_command` | command -> task | `std_msgs/String` JSON | `macgyvbot_command` |
| `/robot_task_control` | command/manual -> task | `std_msgs/String` JSON/plain | `macgyvbot_command`, manual |
| `/robot_task_status` | task -> command/perception | `std_msgs/String` JSON | `macgyvbot_task` |
| `/target_label` | manual -> task | `std_msgs/String` | manual compatibility |
| `/human_grasped_tool` | perception -> task | `std_msgs/String` JSON | `macgyvbot_perception` |
| `/hand_grasp_detection/tool_mask_lock` | perception -> task | `std_msgs/String` JSON | `macgyvbot_perception` |
| `/hand_grasp_detection/annotated_image` | perception -> debug/task | `sensor_msgs/Image` | `macgyvbot_perception` |
| `/command_feedback` | command -> GUI | `std_msgs/String` JSON | `macgyvbot_command` |
| `/stt_text` | command -> command | `std_msgs/String` | `macgyvbot_command` |
| `/stt_result` | command -> legacy consumers | `std_msgs/String` | compatibility |

Camera topics are currently wired to the RealSense defaults:

```text
/camera/camera/color/image_raw
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/color/camera_info
```

## Assets

- `src/macgyvbot_resources/weights/`
  - YOLO `.pt`, hand grasp `.pkl`, SAM checkpoint를 둘 수 있는 source-tree asset
    폴더입니다.
  - 설치 ownership은 `macgyvbot_resources`가 가집니다.

- `src/macgyvbot_resources/weights/vlm/`
  - VLM local weight directory입니다.
  - 기본 VLM 모델 디렉터리 이름은
    `HuggingFaceTB__SmolVLM2-2.2B-Instruct` 형식입니다.

- `src/macgyvbot_resources/calibration/`
  - `T_gripper2camera.npy` 등 calibration asset을 보관합니다.
  - 설치 ownership은 `macgyvbot_resources`가 가집니다.

## Where To Change Things

- 새 launch argument나 node composition: `macgyvbot_bringup/launch/`
- topic 이름 또는 threshold 조정: `macgyvbot_config/macgyvbot_config/`
- 자연어 공구/동작 vocabulary: `macgyvbot_command/input_mapping/`
- GUI 표시/채팅 UI: `macgyvbot_command/ui/`
- YOLO/depth/VLM pick target 로직: `macgyvbot_perception/`
- 사용자 hand-tool grasp 판단: `macgyvbot_perception/hand_tool_grasp/`
- MoveIt, 그리퍼, force, safe workspace: `macgyvbot_manipulation/`
- pick/return 순서와 상태 발행: `macgyvbot_task/application/`
- ROS typed message 전환: `macgyvbot_interfaces/msg/`

## Validation

문서만 바꿀 때도 구조 변경이 섞였는지 확인하려면 최소한 아래를 확인합니다.

```bash
python3 -m compileall -q src
python3 -m pytest -q \
  src/macgyvbot_manipulation/test/test_handover_targeting.py \
  src/macgyvbot_manipulation/test/test_gripper_grasp.py \
  src/macgyvbot_perception/test/test_hand_grasp_ml_mask.py \
  src/macgyvbot_task/test/test_hand_grasp_result_adapter.py
```

ROS package discovery와 launch는 ROS 2 Humble 환경에서 확인합니다.

```bash
source /opt/ros/humble/setup.bash
colcon build
ros2 launch macgyvbot_bringup macgyvbot.launch.py
```
