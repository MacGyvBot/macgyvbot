# MacGyvBot

MacGyvBot은 음성 명령 기반 공구 서랍 관리 로봇팔 어시스턴트를 위한 ROS 2 패키지입니다.

현재 저장소는 RealSense 카메라, YOLO 객체 인식, MoveItPy 기반 로봇팔 이동, OnRobot RG2 그리퍼 제어를 함께 다룹니다. 또한 사람이 로봇이 들고 있는 공구를 잡았는지 판단하는 hand-tool grasp detection 노드를 포함합니다.

## 패키지 구조

메인 pick 파이프라인은 ROS wiring과 기능별 책임을 분리해 구성합니다.

```text
macgyvbot/
├── macgyvbot.py                         # 기존 호환용 wrapper entrypoint
├── nodes/
│   ├── macgyvbot_node.py                # ROS wiring, parameter, frame loop
│   ├── stt_node.py                      # Google STT 기반 /stt_text 발행
│   ├── llm_command_node.py              # STT text -> tool command, /target_label
│   ├── voice_command_ui_node.py         # 터미널 기반 음성 명령 UI
│   └── voice_command_gui_node.py        # PyQt 기반 음성 명령 GUI
├── core/
│   ├── config.py                        # topic, frame, safety offset, grasp mode
│   └── pick_sequence.py                 # pick, handoff, 원위치 반환 시퀀스
├── perception/
│   ├── yolo_detector.py                 # YOLO 모델 경로 해석 및 추론 wrapper
│   ├── grasp_point_selector.py          # center/VLM grasp pixel 선택
│   ├── graspnet_pose_selector.py        # GraspNet pose buffer 및 base frame 변환
│   └── depth_projection.py              # depth pixel -> camera/base 좌표 투영
├── motion/
│   ├── moveit_controller.py             # MoveIt planning 실행 및 J6 yaw 회전
│   └── pose_utils.py                    # pose 생성, EE transform/orientation helper
├── ui/
│   └── debug_windows.py                 # OpenCV debug window 출력
└── voice_command/
    └── command_parser.py                # LLM fallback 전 alias/fuzzy parser
```

기존 executable 이름은 `macgyvbot`으로 유지됩니다. 호환성을 위해 `macgyvbot/macgyvbot.py`는 새 노드의 wrapper entrypoint로 남겨 둡니다.

## 주요 기능

- RealSense color/depth 이미지 구독
- YOLO 기반 공구 및 대상 객체 인식
- VLM 또는 GraspNet 기반 grasp pose 보정
- hand landmark와 tool ROI 기반 잡기 상태 판단
- depth 기반 손-공구 접촉 신호 기본 사용
- MoveItPy 기반 Doosan M0609 경로 계획 및 실행
- OnRobot RG2 그리퍼 open/close 제어
- 안전 작업 영역 클램프 기반 pick sequence 제한

## 실행 환경

- OS: `Ubuntu 22.04`
- ROS 2: `Humble`
- Python: `3.10`
- 로봇팔: `Doosan-Robotics-M0609`
- 그리퍼: `OnRobot RG2`
- 카메라: `Intel RealSense Depth Camera D435I`

## 설치 및 빌드

[Doosan ROS 2 Manual(Humble)](https://doosanrobotics.github.io/doosan-robotics-ros-manual/humble/installation.html)

MacGyvBot은 두산로보틱스 ROS 2 패키지와 함께 사용합니다. `macgyvbot` 패키지는 두산로보틱스 워크스페이스의 `doosan-robot2` 아래에 clone한 뒤, `ros2_ws/src` 기준에서 빌드합니다.

```bash
cd ~/ros2_ws/src/doosan-robot2/
git clone https://github.com/MacGyvBot/macgyvbot.git
```

Python 패키지 설치:

```bash
pip install -r requirements.txt
```

전체 워크스페이스 빌드:

```bash
cd ~/ros2_ws
colcon build
```

`macgyvbot` 패키지만 빌드:

```bash
cd ~/ros2_ws
colcon build --packages-select macgyvbot
```

빌드 후 source:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

## 전체 파이프라인 실행

각 터미널은 새로 열 때마다 ROS 2, `ros2_ws`, Doosan MoveIt 환경을 source한 뒤 실행합니다.

### Terminal 1: Doosan M0609 + MoveIt 실행

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py \
  mode:=real \
  model:=m0609 \
  host:=192.168.1.100
```

### Terminal 2: RealSense 카메라 실행

기본 실행은 YOLO bounding box 중심점을 grasp point로 사용합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch realsense2_camera rs_align_depth_launch.py \
  depth_module.depth_profile:=640x480x30 \
  rgb_camera.color_profile:=640x480x30 \
  initial_reset:=true \
  align_depth.enable:=true
```

### Terminal 3: MacGyvBot 메인 파이프라인 실행

기본 실행은 `center` grasp point mode를 사용합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch macgyvbot macgyvbot.launch.py
```

명시적으로 중심점 모드를 사용할 경우:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch macgyvbot macgyvbot.launch.py grasp_point_mode:=center
```

VLM 기반 grasp point selection을 사용할 경우:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch macgyvbot macgyvbot.launch.py grasp_point_mode:=vlm
```

VLM 모드는 YOLO가 검출한 객체 crop에서 grid 기반 grasp region을 선택한 뒤 depth로 grasp pixel을 보정합니다. VLM 추론 또는 depth 보정이 실패하면 기존 중심점 방식으로 fallback합니다.

GraspNet 통합 모드를 사용할 경우:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch macgyvbot macgyvbot.launch.py grasp_point_mode:=graspnet
```

`grasp_point_mode:=graspnet`은 YOLO + depth로 target position을 잡고, GraspNet pose에서 orientation을 받아 보정합니다. GraspNet position은 기본 이동 목표로 사용하지 않습니다.

GraspNet 통합 모드는 기본 경로의 실제 baseline/checkpoint를 사용합니다. baseline 코드는 `~/third_party/graspnet-baseline`, checkpoint는 `~/models/graspnet/checkpoint-rs.tar`에 둡니다. baseline 코드와 checkpoint 파일은 저장소에 커밋하지 않습니다.

GraspNet baseline 설치:

```bash
mkdir -p ~/third_party
cd ~/third_party
git clone https://github.com/graspnet/graspnet-baseline.git
```

GraspNet Python 의존성 설치:

```bash
cd ~/ros2_ws/src/doosan-robot2/macgyvbot
pip install -r requirements-graspnet.txt

cd ~/third_party/graspnet-baseline
pip install -r requirements.txt
```

CUDA extension 빌드:

```bash
cd ~/third_party/graspnet-baseline/pointnet2
python setup.py install

cd ~/third_party/graspnet-baseline/knn
python setup.py install
```

필요하면 `graspnetAPI`를 source 설치로 덮어씁니다.

```bash
cd ~/third_party
git clone https://github.com/graspnet/graspnetAPI.git
cd graspnetAPI
pip install .
```

checkpoint 준비:

```bash
mkdir -p ~/models/graspnet
# 공식 GraspNet baseline pretrained weights에서 RealSense용 checkpoint-rs.tar를 받아
# ~/models/graspnet/checkpoint-rs.tar 로 둡니다.
```

`macgyvbot.launch.py`는 로봇 메인 노드, hand grasp detection, STT, LLM command node를 함께 실행합니다. CLI UI는 로그와 입력이 섞이지 않도록 별도 터미널에서 실행합니다.

### Terminal 4: Ollama 서버 실행

LLM fallback을 사용하려면 Ollama 서버와 모델이 필요합니다. 이미 서버가 실행 중이면 이 터미널은 생략할 수 있습니다.

최초 설치:

```bash
curl -fsSL https://ollama.com/install.sh | sh
```

```bash
ollama pull qwen2.5:0.5b
ollama serve
```

### Terminal 5: 음성 명령 CLI UI 실행

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run macgyvbot voice_command_ui_node
```

CLI UI는 `/stt_text`, `/command_feedback`, `/tool_command`, `/target_label`을 확인하며 사용자 입력도 `/stt_text`로 발행합니다.

GUI 채팅창을 사용할 경우:

```bash
ros2 run macgyvbot voice_command_gui_node
```

GUI 실행에 PyQt5가 필요합니다.

```bash
sudo apt install python3-pyqt5
```

예:

```text
You > 드라이버 가져다줘
You > 그 조이는 거 가져와
You > 망치 줘
```

흐름:

```text
voice_command_ui_node 또는 stt_node
  -> /stt_text
  -> llm_command_node
  -> /tool_command
  -> /target_label
  -> macgyvbot
```

마이크 STT 없이 CLI 입력만 테스트하려면 Terminal 3에서 STT를 끄고 실행합니다.

```bash
ros2 launch macgyvbot macgyvbot.launch.py use_stt:=false
```

## 수동 대상 공구 요청

음성 명령 파이프라인을 거치지 않고 기존 방식으로 대상 공구를 직접 요청할 수도 있습니다.

```bash
ros2 topic pub --once /target_label std_msgs/msg/String "{data: screwdriver}"
```

사용 가능한 공구 label은 학습한 YOLO 모델의 class 이름과 같아야 합니다. 현재 예시는 `hammer`, `screwdriver`, `pliers`, `tape_measure`를 기준으로 합니다.

## 음성 명령 입력만 테스트

마이크 STT 없이 CLI 입력만 확인할 때는 `macgyvbot.launch.py`에서 STT를 끄고 실행한 뒤, CLI UI를 별도 터미널에서 실행합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch macgyvbot macgyvbot.launch.py use_stt:=false
```

다른 터미널:

```bash
ros2 run macgyvbot voice_command_ui_node
```

## 잡기 인식 노드 실행

`macgyvbot.launch.py`는 hand grasp detection 노드를 함께 실행합니다. 잡기 인식 노드만 단독으로 확인할 때는 아래 명령을 사용합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch macgyvbot hand_grasp_detection.launch.py
```

기본 구독 토픽:

- `/camera/camera/color/image_raw`
- `/camera/camera/aligned_depth_to_color/image_raw`

기본 발행 토픽:

- `/human_grasped_tool`: `std_msgs/msg/String` JSON 결과
- `/hand_grasp_detection/annotated_image`: annotation 이미지

커스텀 YOLO 모델을 사용할 경우 launch 파라미터 `yolo_model`에 모델 경로를 지정합니다. 모델 파일은 저장소에 커밋하지 않습니다.

예:

```bash
ros2 launch macgyvbot hand_grasp_detection.launch.py yolo_model:=/path/to/yolov11_best.pt
```

## 테스트

```bash
colcon test --packages-select macgyvbot
colcon test-result --verbose
```

## 기여

브랜치, 커밋, PR, 이슈, 안전 규칙은 [CONTRIBUTING.md](./CONTRIBUTING.md)를 따릅니다.
