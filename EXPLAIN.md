# Current Drop Recovery Note

`/tool_drop_detected`의 `event=tool_dropped` payload는 현재 자동 exit 대신
first-version recovery로 처리합니다. 현재 MoveIt goal을 cancel하고 남은
`TaskStep` queue를 비운 뒤 `recovery_mode`를 켜고, 기존 inspection pose,
YOLO target resolver, grasp verifier, drawer flow, home motion을 orchestration합니다.
자세한 흐름은 `docs/drop_recovery.md`를 참고합니다.

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
  - `joint_velocity_config.py`는 launch 시점에 combined URDF와 MoveIt config dict에
    joint velocity 설정을 반영하는 patch helper를 소유합니다.

- `src/macgyvbot_task`
  - `macgyvbot_main_node.py`는 `/tool_command`를 typed `/task_request`로 넘기는
    경량 command router를 소유합니다.
  - `task_coordinator_node.py`는 pick/return application workflow, MoveIt/RG2
    실행 리소스, task queue, `/task_control` 처리를 소유합니다.
  - `/task_request`, `/task_control`, 카메라 입력, hand grasp 결과를 받아
    task queue 기반 로봇 작업 시퀀스를 실행합니다.
  - `/robot_task_status`와 `/tool_drop_detected`를 발행합니다.

- `src/macgyvbot_command`
  - headless command input node, command parser, STT/TTS helper를 소유합니다.
  - typed `/stt_text`를 입력으로 받아 `/tool_command`, `/task_control`,
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
  - `moveit_controller.py`는 pose goal을 MoveIt에 바로 넘기기 전에 현재 joint
    state 기준 IK seed sampling을 수행합니다. 후보 IK 해는 현재 joint와 가장
    가까운 동치 각도로 보정하고, joint delta가 큰 후보는 planning 전에
    거부합니다.
  - 모든 로봇팔 motion은 `MoveItController.plan_and_execute()`를 공통 관문으로
    지나며, 이 함수가 drawer collision scene과 RG2 self-collision ACM 준비
    상태를 planning 직전에 확인합니다.
  - `drawer_collision_scene.py`는 MoveIt planning scene에 drawer keep-out
    collision object를 추가합니다. 현재 drawer collision scene은 정적인
    `drawer_only` profile만 사용합니다.
  - `gripper_collision_scene.py`는 RG2 내부 링크끼리만 허용할 self-collision
    ACM patch를 적용합니다. 이 ACM은 drawer 같은 외부 collision object와의
    충돌을 허용하지 않습니다.

- `src/macgyvbot_config`
  - ROS topic 이름, robot frame/link, model filename, pick/return/handoff/grasp
    threshold, timing, UI, VLM 설정 등 Python runtime constant를 소유합니다.
  - `drawer.py`는 drawer handle pose, drawer/tool mapping, drawer collision
    box 좌표, collision profile, motion key별 profile routing을 소유합니다.
  - `joint_velocity.py`는 M0609 `joint_1`~`joint_6`의 사용자 조절용
    velocity limit과 전역 MoveIt velocity scaling 값을 소유합니다.

- `src/macgyvbot_domain`
  - package 간 공유되는 in-process Python dataclass를 소유합니다.
  - 현재 `PickTarget`, `PickMotionPlan`을 제공합니다.

- `src/macgyvbot_resources`
  - 패키지 내부 `calibration/`, `weights/`, `weights/vlm/` asset 설치를 소유합니다.

- `src/macgyvbot_interfaces`
  - package-boundary typed ROS message contracts를 소유합니다.
  - command, task request, task control, status, grasp, mask, drop event는
    `macgyvbot_interfaces/msg/*` 타입으로 주고받습니다.

- `src/macgyvbot_ui`
  - operator-facing GUI boundary를 소유합니다.
  - 사용자 입력을 typed `/stt_text`로 발행하고 `/command_feedback`,
    `/robot_task_status`, detector image topic을 구독해 표시합니다.

## Main Flow

```text
macgyvbot_command.command_input_node
  -> typed /stt_text 또는 마이크 STT 입력 수집
  -> command parser로 자연어 명령 해석
  -> bring/return은 /tool_command로 발행
  -> stop/pause/resume은 /task_control로 발행
  -> 최신 exit 요청은 /task_control의 exit action으로 발행
  -> /command_feedback 발행

macgyvbot_ui.operator_ui_node
  -> 사용자 입력을 typed /stt_text로 발행
  -> /command_feedback, /robot_task_status, detector image topic을 구독
  -> operator GUI에 채팅, 상태, detector view 표시

macgyvbot_task.macgyvbot_main_node
  -> /tool_command 수신
  -> bring/return/release/home 요청을 typed /task_request로 발행
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
  typed `/task_request`로 `task_coordinator_node.py`에 전달됩니다.
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
- task coordinator는 MoveItPy `PlanRequestParameters`를 전역 Pilz Industrial
  Motion Planner `PTP`로 설정합니다.
- `moveit_py.yaml`에는 `ompl`, `pilz_industrial_motion_planner`, `chomp`,
  `ompl_rrt_star` pipeline 목록과 profile별 request parameter가 정의되어 있습니다.
  Python runtime에서 실제로 사용하는 기본값은 `task_coordinator_node.py`의
  `self.planning_params`입니다.
- robot description은 `macgyvbot.launch.py`에서 `macgyvbot_resources`의
  `m0609_onrobot_rg2_combined.urdf`를 읽어 `robot_description`에 주입합니다.
  그 직후 `macgyvbot_bringup.joint_velocity_config`의
  `apply_joint_velocity_limits_to_moveit_config()`가
  `macgyvbot_config.joint_velocity`의 값을 URDF `<limit velocity="...">`와
  `robot_description_planning.joint_limits.*.max_velocity`에 함께 적용합니다.
  따라서 source URDF 파일 자체의 velocity 값은 기본 모델 값으로 남아 있고,
  MacGyvBot runtime에서 쓰는 조인트 속도 제한은 launch-time patch 결과입니다.
- 현재 joint velocity 기본값은 `joint_1`/`joint_2` 30 deg/s, `joint_3`
  36 deg/s, `joint_4`/`joint_5` 45 deg/s, `joint_6` 90 deg/s입니다.
  `task_coordinator_node.py`는 `PlanRequestParameters.max_velocity_scaling_factor`에
  `joint_velocity.py`의 `MOTION_VELOCITY_SCALING_FACTOR`를 넣습니다.
  acceleration scaling은 `moveit_py.yaml`의 기본
  `plan_request_params.max_acceleration_scaling_factor`가 소유하며, 실제 로봇
  테스트 기준으로 현재 `0.12`를 사용합니다.
- drawer collision scene은 task node 초기화 시점에 한 번 적용되는 정적인
  `drawer_only` profile입니다.
  `MoveItController.plan_and_execute(..., collision_scene_key=...)`가 호출될 때
  `DrawerCollisionSceneManager`는 key를 조회할 수 있는 구조를 유지하지만,
  현재 등록된 key가 없으므로 모든 motion은 기본 `drawer_only`로 처리됩니다.
- drawer scene manager는 초기화 때 MoveItPy local scene과 RViz/move_group 쪽 topic
  publish, `/apply_planning_scene` service update를 요청합니다. 이후 planning
  precondition에서는 scene 준비 상태만 확인하고 collision object를 다시 적용하지
  않습니다.
- `/apply_planning_scene` service가 실패해도 local MoveItPy planning scene에 object
  적용이 성공하면 task node 내부 planning은 계속 진행할 수 있습니다. RViz 표시와
  외부 planning scene 동기화가 필요한 경우 로그의 `apply_service` 결과를 확인합니다.
- drawer collision object가 목표 pose 자체 또는 손잡이 접촉 공간을 덮으면 planner가
  경로를 만들 수 없습니다. 이런 경우에는 key를 더 세분화하거나 drawer collision
  geometry를 손잡이 접근 corridor와 겹치지 않게 조정해야 합니다.

## Assets

- `src/macgyvbot_resources/weights/`
  - YOLO `.pt`, hand grasp `.pkl`, SAM checkpoint, VLM local weight를 둘 수 있는
    source-tree asset 폴더입니다.
  - 설치 ownership은 `macgyvbot_resources`가 가집니다.

- `src/macgyvbot_resources/calibration/`
  - gripper-camera transform 등 calibration asset을 보관합니다.
  - 설치 ownership은 `macgyvbot_resources`가 가집니다.
