# MacGyvBot

MacGyvBot은 음성 또는 operator 명령으로 공구를 가져오고 반납하는 ROS 2
기반 로봇팔 어시스턴트입니다. RealSense 카메라, YOLO/VLM perception,
MoveItPy 기반 Doosan M0609 제어, OnRobot RG2 그리퍼, hand-tool grasp
detection, operator UI를 하나의 데모 파이프라인으로 묶습니다.

저장소 루트는 colcon workspace이며, 실제 runtime 패키지는 `src/` 아래에
있습니다.

## 패키지 개요

```text
src/
├── macgyvbot_bringup/        # launch/config wiring
├── macgyvbot_task/           # main task node, pick/return workflow
├── macgyvbot_command/        # headless command pipeline, parser, STT/TTS
├── macgyvbot_ui/             # operator-facing GUI node and UI presenters
├── macgyvbot_perception/     # YOLO, VLM, depth, hand grasp perception
├── macgyvbot_manipulation/   # MoveIt, gripper, force, pose, safe workspace
├── macgyvbot_config/         # shared Python runtime constants
├── macgyvbot_domain/         # shared dataclasses
├── macgyvbot_resources/      # calibration and model assets
└── macgyvbot_interfaces/     # typed msg/srv/action migration target
```

작업자와 에이전트 공통 지침은 [AGENTS.md](./AGENTS.md)를 참고합니다.
상세 runtime 구조는 [EXPLAIN.md](./EXPLAIN.md)를 참고합니다.
ROS topic 소유권과 계약은
[docs/architecture/topics.md](./docs/architecture/topics.md)를 참고합니다.

## 실행 환경

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- Doosan Robotics M0609
- OnRobot RG2
- Intel RealSense D435I

로봇 실행 전 Doosan ROS 2 Humble 설치를 먼저 완료해야 합니다.

## 설치와 빌드

워크스페이스 clone:

```bash
cd ~
git clone https://github.com/MacGyvBot/macgyvbot.git
cd ~/macgyvbot
```

Python/runtime 의존성 설치:

```bash
sudo apt update
sudo apt install -y portaudio19-dev ffmpeg && python3 -m pip install -r requirements.txt
```

LLM 기반 명령 해석을 사용하려면 Ollama와 기본 모델을 준비합니다.

```bash
curl -fsSL https://ollama.com/install.sh | sh
ollama pull gemma3:1b
ollama serve
```

빌드:

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

브랜치를 바꾸거나 패키지를 다시 빌드한 뒤에는 새 터미널에서 workspace를 다시
source합니다.

## 모델과 리소스

모델과 calibration 파일은 Git에 포함하지 않습니다. 기본 위치는
`macgyvbot_resources` 패키지입니다.

```text
src/macgyvbot_resources/calibration/T_gripper2camera.npy
src/macgyvbot_resources/weights/yolov11_best.pt
src/macgyvbot_resources/weights/hand_grasp_model.pkl
src/macgyvbot_resources/weights/mobile_sam.pt
src/macgyvbot_resources/weights/vlm/
```

설치 후 asset은 아래 경로에서 사용됩니다.

```text
install/macgyvbot_resources/share/macgyvbot_resources/
```

지원되는 모델 asset은 `src/macgyvbot_resources/weights/` 아래의 다운로드
스크립트를 사용합니다.

## 실행

각 터미널은 ROS와 workspace를 source합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/macgyvbot/install/setup.bash
```

로봇/MoveIt stack 실행:

```bash
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py \
  mode:=real \
  model:=m0609 \
  host:=192.168.1.100
```

RealSense camera 실행:

```bash
ros2 launch realsense2_camera rs_align_depth_launch.py \
  depth_module.depth_profile:=640x480x30 \
  rgb_camera.color_profile:=640x480x30 \
  initial_reset:=true \
  align_depth.enable:=true
```

MacGyvBot 실행:

```bash
ros2 launch macgyvbot_bringup macgyvbot.launch.py
```

## Grasp Point Mode

개발 중 일반적으로 바꿔 쓰는 launch argument는 `grasp_point_mode`입니다.
기본 launch 값은 `vlm`입니다. 관련 mode 상수는 `macgyvbot_config.vlm`에서 관리합니다.

예시:

```bash
ros2 launch macgyvbot_bringup macgyvbot.launch.py grasp_point_mode:=vlm
ros2 launch macgyvbot_bringup macgyvbot.launch.py grasp_point_mode:=vlm_only
ros2 launch macgyvbot_bringup macgyvbot.launch.py grasp_point_mode:=vlm_only_qwen3b
ros2 launch macgyvbot_bringup macgyvbot.launch.py grasp_point_mode:=vlm_only_qwen7b
ros2 launch macgyvbot_bringup macgyvbot.launch.py grasp_point_mode:=center
ros2 launch macgyvbot_bringup macgyvbot.launch.py grasp_point_mode:=api
```

지원 mode는 `src/macgyvbot_config/macgyvbot_config/vlm.py`에서 관리합니다.

그 외 모델 경로, threshold, topic, runtime 상수는 `macgyvbot_config`와 관련
launch 파일이 소유합니다. README에는 긴 parameter 목록을 두지 않고, 운영
계약은 `EXPLAIN.md` 또는 `docs/architecture/`에 정리합니다.

## 명령 입력

대표 명령 예시:

```text
드라이버 가져다줘
플라이어 가져와
망치 줘
이거 정리해
드라이버 정리해
멈춰
재개
취소
홈위치로 가
복귀해
종료
```

command pipeline은 tool/task command를 발행하고, operator UI는 ROS topic을
통해 command feedback, robot status, detector image를 표시합니다.
GUI의 `복귀` 버튼과 Home 복귀 표현은 `/tool_command`의 `action=home`으로
전달되며, 로봇이 대기 중일 때 Home joint pose 이동과 gripper open을
실행합니다. 정지 이후 채팅에 표시되는 `취소`는 `/robot_task_control`의
`action=cancel`로 전달되어 현재 motion과 task queue만 정리하고, Home 이동이나
GUI 종료 없이 다음 명령을 기다립니다. `종료` 버튼과 종료 표현은 `/robot_task_control`의
`action=exit`으로 전달되며, 실행 중 작업을 종료하고 Home 복귀가 완료된 뒤
GUI와 command node를 종료합니다.

GUI 창 닫기는 로봇 작업 종료 명령을 대신하지 않습니다. 창만 닫으면
`/command_shutdown`으로 headless command node도 정리되며, 작업 중 로봇을
안전하게 Home으로 복귀시킨 뒤 종료하려면 GUI의 `종료` 버튼을 사용합니다.

## 테스트

빠른 문법 검사:

```bash
python3 -m compileall -q src
```

핵심 단위 테스트:

```bash
python3 -m pytest -q \
  src/macgyvbot_manipulation/test/test_handover_targeting.py \
  src/macgyvbot_manipulation/test/test_gripper_grasp.py \
  src/macgyvbot_perception/test/test_hand_grasp_ml_mask.py \
  src/macgyvbot_task/test
```

ROS 환경이 준비된 경우 workspace 테스트:

```bash
colcon test
colcon test-result --verbose
```

로봇, MoveIt, 카메라, 모델 의존 검증은 실제 장비 환경에서 낮은 속도와 충분한
작업 공간을 확보한 뒤 수행합니다.

## 기여

브랜치, 커밋, PR, 이슈, 리뷰 규칙은 [CONTRIBUTING.md](./CONTRIBUTING.md)를
따릅니다. 일반 작업자/에이전트 지침은 [AGENTS.md](./AGENTS.md)를 참고합니다.
