# MacGyvBot File Roles

이 문서는 migrated MacGyvBot 패키지들의 역할을 빠르게 파악하기 위한 설명서입니다.

루트 `macgyvbot` Python 구현 패키지는 M7에서 제거되었습니다. 저장소 루트는
ROS 패키지가 아니라 colcon workspace root이며, 실행 entrypoint는
`macgyvbot_bringup`이 소유합니다.

## Runtime Packages

- `src/macgyvbot_bringup`
  - 기본 launch entrypoint를 소유합니다.
  - `macgyvbot.launch.py`는 `macgyvbot_task`, `macgyvbot_perception`,
    `macgyvbot_command` executable을 함께 실행합니다.
  - `moveit_py.yaml` 같은 launch/runtime YAML을 설치합니다.

- `src/macgyvbot_task`
  - `macgyvbot_main_node.py`와 pick/return application workflow를 소유합니다.
  - `/tool_command`, `/robot_task_control`, 카메라 입력, hand grasp 결과를 받아
    task queue 기반 로봇 작업 시퀀스를 실행합니다.
  - `/robot_task_status`와 `/tool_drop_detected`를 발행합니다.

- `src/macgyvbot_command`
  - headless command input node, command parser, STT/TTS helper를 소유합니다.
  - `/stt_text`를 입력으로 받아 `/tool_command`, `/robot_task_control`,
    `/command_feedback`를 발행합니다.

- `src/macgyvbot_perception`
  - YOLO detection, depth projection, pick target resolution, grasp point selection,
    hand grasp detection node와 hand/tool grasp helper를 소유합니다.
  - model path lookup은 `macgyvbot_resources`를 기준으로 수행합니다.
  - `grasp_point/`는 로봇이 잡을 image-space grasp point 선택을 담당합니다.
    기본 모드는 단일 호출 기반 `vlm_only_qwen3b`이며, 기존 grid 기반 `vlm`,
    Gemini API 기반 `api`, bbox 중심 `center` 모드를 함께 지원합니다.
  - `grasp_point/vlm/`은 local VLM 모델 호출, 응답 parsing, inference history
    기록처럼 VLM 자체에 가까운 공통 기능을 소유합니다.
  - `grasp_point/vlm_method/`, `grasp_point/vlm_only_method/`,
    `grasp_point/api_method/`는 각각의 grasp point 선정 방식을 소유합니다.
  - `hand_tool_grasp/`는 사용자 손이 공구를 잡았는지 판단하는 hand landmark,
    tool ROI/mask contact, ML hand grasp classification을 담당합니다.
  - ROS-facing 이름인 `hand_grasp_detection_node`,
    `/hand_grasp_detection/annotated_image`, `hand_grasp_model.pkl`은 명시적
    interface migration 없이는 유지합니다.

- `src/macgyvbot_manipulation`
  - MoveIt controller, robot pose helper, safe workspace clamp, OnRobot gripper,
    grasp verification, force reaction detection, handoff targeting을 소유합니다.

- `src/macgyvbot_config`
  - ROS topic 이름, robot frame/link, model filename, pick/return/handoff/grasp
    threshold, timing, UI, VLM 설정 등 Python runtime constant를 소유합니다.

- `src/macgyvbot_domain`
  - package 간 공유되는 in-process Python dataclass를 소유합니다.
  - 현재 `PickTarget`, `PickMotionPlan`을 제공합니다.

- `src/macgyvbot_resources`
  - 패키지 내부 `calibration/`, `weights/`, `weights/vlm/` asset 설치를 소유합니다.

- `src/macgyvbot_interfaces`
  - typed ROS message migration target을 소유합니다.
  - 현재 runtime은 호환성을 위해 JSON over `std_msgs/String`을 유지합니다.

- `src/macgyvbot_ui`
  - operator-facing GUI boundary를 소유합니다.
  - 사용자 입력을 `/stt_text`로 발행하고 `/command_feedback`,
    `/robot_task_status`, detector image topic을 구독해 표시합니다.

## Main Flow

```text
macgyvbot_command.command_input_node
  -> /stt_text 또는 마이크 STT 입력 수집
  -> command parser로 자연어 명령 해석
  -> bring/return은 /tool_command로 발행
  -> stop/pause/resume은 /robot_task_control로 발행
  -> 최신 exit 요청은 /robot_task_control의 exit action으로 발행
  -> /command_feedback 발행

macgyvbot_ui.operator_ui_node
  -> 사용자 입력을 /stt_text로 발행
  -> /command_feedback, /robot_task_status, detector image topic을 구독
  -> operator GUI에 채팅, 상태, detector view 표시

macgyvbot_task.macgyvbot_main_node
  -> /tool_command, /robot_task_control 또는 수동 /target_label 수신
  -> RealSense color/depth 기반 YOLO 탐지
  -> grasp point 선택
  -> depth pixel을 robot base 좌표로 투영
  -> PickSequenceRunner 또는 ReturnSequenceRunner가 TaskStep queue 구성
  -> TaskControlCoordinator가 queue를 순차 실행
  -> /robot_task_status 발행

macgyvbot_perception.hand_grasp_detection_node
  -> 카메라 image/depth 수신
  -> hand landmark와 tool ROI를 비교
  -> 사람이 공구를 잡았는지 /human_grasped_tool로 발행
```

## Task Queue and Safety Control

- `bring`과 `return` 명령은 즉시 긴 blocking flow를 직접 실행하지 않고,
  `TaskStep` 목록으로 변환된 뒤 `TaskControlCoordinator`의 worker thread에서
  순차 실행됩니다.
- `/robot_task_control`은 실행 중인 queue에 `stop`, `pause`, `resume`을 적용합니다.
  `stop`은 현재 MoveIt trajectory goal을 cancel하고 대기 중인 step queue를 비웁니다.
- 최신 구현에서는 `exit` action이 task queue를 종료하고 MoveIt goal을 cancel한 뒤
  Home joint pose로 복귀하며, 복귀 성공 후 OnRobot RG2 그리퍼를 open합니다.
- `pause`는 현재 MoveIt trajectory goal을 cancel하지만 대기 중인 queue는 유지합니다.
  pause로 중단된 retry 가능 step은 queue 앞에 다시 들어가고, `resume` 이후 계속됩니다.
- `/tool_drop_detected`의 `event=tool_dropped` payload는 자동 `stop`으로 해석합니다.
  이때 queue clear와 MoveIt goal cancel은 수행하되 `/robot_task_status`의
  `tool_dropped` 상태가 `cancelled`로 덮이지 않도록 처리합니다.
- 최신 구현에서는 `tool_dropped`도 내부적으로 `exit` 제어 흐름을 사용합니다.

## Assets

- `src/macgyvbot_resources/weights/`
  - YOLO `.pt`, hand grasp `.pkl`, SAM checkpoint, VLM local weight를 둘 수 있는
    source-tree asset 폴더입니다.
  - 설치 ownership은 `macgyvbot_resources`가 가집니다.

- `src/macgyvbot_resources/calibration/`
  - gripper-camera transform 등 calibration asset을 보관합니다.
  - 설치 ownership은 `macgyvbot_resources`가 가집니다.
