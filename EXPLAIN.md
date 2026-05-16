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
  - `/tool_command`, 카메라 입력, hand grasp 결과를 받아 로봇 작업 시퀀스를 실행합니다.
  - `/robot_task_status`를 발행합니다.

- `src/macgyvbot_command`
  - command input node, command parser, STT/TTS helper, command-coupled GUI를 소유합니다.
  - `/tool_command`, `/command_feedback`, `/stt_text`를 발행하고
    `/robot_task_status`를 구독합니다.

- `src/macgyvbot_perception`
  - YOLO detection, depth projection, pick target resolution, grasp point selection,
    hand grasp detection node와 hand/tool grasp helper를 소유합니다.
  - model path lookup은 `macgyvbot_resources`를 기준으로 수행합니다.

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
  - 독립 UI boundary를 위한 패키지입니다. 현재 command-coupled GUI는
    `macgyvbot_command`에 남아 있습니다.

## Main Flow

```text
macgyvbot_command.command_input_node
  -> GUI 입력 또는 마이크 STT 수집
  -> command parser로 자연어 명령 해석
  -> /tool_command, /command_feedback 발행

macgyvbot_task.macgyvbot_main_node
  -> /tool_command 또는 수동 /target_label 수신
  -> RealSense color/depth 기반 YOLO 탐지
  -> grasp point 선택
  -> depth pixel을 robot base 좌표로 투영
  -> PickSequenceRunner 또는 ReturnSequenceRunner로 작업 시퀀스 실행
  -> /robot_task_status 발행

macgyvbot_perception.hand_grasp_detection_node
  -> 카메라 image/depth 수신
  -> hand landmark와 tool ROI를 비교
  -> 사람이 공구를 잡았는지 /human_grasped_tool로 발행
```

## Assets

- `src/macgyvbot_resources/weights/`
  - YOLO `.pt`, hand grasp `.pkl`, SAM checkpoint, VLM local weight를 둘 수 있는
    source-tree asset 폴더입니다.
  - 설치 ownership은 `macgyvbot_resources`가 가집니다.

- `src/macgyvbot_resources/calibration/`
  - gripper-camera transform 등 calibration asset을 보관합니다.
  - 설치 ownership은 `macgyvbot_resources`가 가집니다.
