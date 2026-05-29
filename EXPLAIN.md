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
  - `macgyvbot_main_node.py`는 `/tool_command`를 `/task_request`로 넘기는
    경량 command router를 소유합니다.
  - `task_coordinator_node.py`는 pick/return application workflow, MoveIt/RG2
    실행 리소스, task queue, `/task_control` 처리를 소유합니다.
  - `/task_request`, `/task_control`, 카메라 입력, hand grasp 결과를 받아
    task queue 기반 로봇 작업 시퀀스를 실행합니다.
  - `/robot_task_status`와 `/tool_drop_detected`를 발행합니다.

- `src/macgyvbot_command`
  - headless command input node, command parser, STT/TTS helper를 소유합니다.
  - `/stt_text`를 입력으로 받아 `/tool_command`, `/task_control`,
    `/command_feedback`를 발행합니다.

- `src/macgyvbot_perception`
  - YOLO detection, depth projection, pick target resolution, grasp point selection,
    hand grasp detection node와 hand/tool grasp helper를 소유합니다.
  - model path lookup은 `macgyvbot_resources`를 기준으로 수행합니다.
  - `grasp_point/`는 로봇이 잡을 image-space grasp point 선택과 depth
    refinement를 담당합니다.
  - `hand_tool_grasp/`는 사용자 손이 공구를 잡았는지 판단하는 hand landmark,
    tool ROI/mask contact, ML hand grasp classification을 담당합니다.
  - ROS-facing 이름인 `hand_grasp_detection_node`,
    `/hand_grasp_detection/annotated_image`, `hand_grasp_model.pkl`은 명시적
    interface migration 없이는 유지합니다.

- `src/macgyvbot_manipulation`
  - MoveIt controller, robot pose helper, safe workspace clamp, OnRobot gripper,
    grasp verification, force reaction detection, handoff targeting을 소유합니다.
  - `moveit_controller.py`는 pose goal을 MoveIt에 바로 넘기기 전에 현재 joint
    state 기준 IK seed sampling을 수행합니다. 후보 IK 해는 현재 joint와 가장
    가까운 동치 각도로 보정하고, joint delta가 큰 후보는 planning 전에
    거부합니다.

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
  -> stop/pause/resume은 /task_control로 발행
  -> 최신 exit 요청은 /task_control의 exit action으로 발행
  -> /command_feedback 발행

macgyvbot_ui.operator_ui_node
  -> 사용자 입력을 /stt_text로 발행
  -> /command_feedback, /robot_task_status, detector image topic을 구독
  -> operator GUI에 채팅, 상태, detector view 표시

macgyvbot_task.macgyvbot_main_node
  -> /tool_command 또는 수동 /target_label 수신
  -> bring/return/release/home 요청을 /task_request로 발행
  -> /robot_task_status를 구독해 task busy 상태 추적

macgyvbot_task.task_coordinator_node
  -> /task_request와 /task_control 수신
  -> RealSense color/depth 기반 YOLO 탐지
  -> grasp point 선택
  -> depth pixel을 robot base 좌표로 투영
  -> PickSequenceRunner 또는 ReturnSequenceRunner가 TaskStep queue 구성
  -> 내부 queue에서 TaskStep을 하나씩 꺼내 worker thread로 순차 실행
  -> pause/resume/exit, MoveIt goal cancel, exit 후 Home 복귀 처리
  -> /robot_task_status 발행

macgyvbot_perception.hand_grasp_detection_node
  -> 카메라 image/depth 수신
  -> hand landmark와 tool ROI를 비교
  -> 사람이 공구를 잡았는지 /human_grasped_tool로 발행
```

## Task Queue and Safety Control

- `bring`과 `return` 명령은 즉시 긴 blocking flow를 직접 실행하지 않고,
  `/task_request`로 `task_coordinator_node.py`에 전달됩니다.
- `task_coordinator_node.py`는 요청을 받아 `TaskStep` 목록을 구성하고, 내부 queue에
  적재한 뒤 worker thread에서 한 step씩 순차 실행합니다.
- `/task_control`은 실행 중인 queue에 `pause`, `resume`, `exit`을 적용합니다.
- `exit` action은 task queue를 종료하고 MoveIt goal을 cancel한 뒤 Home joint pose로
  복귀하며, 복귀 성공 후 OnRobot RG2 그리퍼를 open합니다.
- `pause`는 현재 MoveIt trajectory goal을 cancel하지만 대기 중인 queue는 유지합니다.
  pause로 중단된 retry 가능 step은 queue 앞에 다시 들어가고, `resume` 이후 계속됩니다.
- `/tool_drop_detected`의 `event=tool_dropped` payload는 자동 `exit`으로 해석합니다.
  이때 queue clear와 MoveIt goal cancel은 수행하되 `/robot_task_status`의
  `tool_dropped` 상태가 `cancelled`로 덮이지 않도록 처리합니다.
- pick flow의 `build_steps()`는 queue에 넣을 step 목록만 구성합니다. target planning,
  VLM refine, MoveIt motion, gripper 동작은 각 `TaskStep` 실행 시점에 수행됩니다.
- pose goal 기반 MoveIt 이동은 `moveit_controller.py`에서 현재 joint state를 읽고
  여러 IK seed를 시도한 뒤, `q_curr` 기준 가장 가까운 joint goal을 선택합니다.
  선택된 IK 후보의 joint별 `raw`/`short` delta를 로그로 남기며, 최대 joint delta가
  안전 한계를 넘으면 기존 pose fallback으로 우회하지 않고 planning을 중단합니다.

## Assets

- `src/macgyvbot_resources/weights/`
  - YOLO `.pt`, hand grasp `.pkl`, SAM checkpoint, VLM local weight를 둘 수 있는
    source-tree asset 폴더입니다.
  - 설치 ownership은 `macgyvbot_resources`가 가집니다.

- `src/macgyvbot_resources/calibration/`
  - gripper-camera transform 등 calibration asset을 보관합니다.
  - 설치 ownership은 `macgyvbot_resources`가 가집니다.
