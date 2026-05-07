# MacGyvBot

MacGyvBot은 음성 명령 기반 공구 서랍 관리 로봇팔 어시스턴트를 위한 ROS 2 패키지입니다.

현재 저장소는 RealSense 카메라, YOLO 객체 인식, MoveItPy 기반 로봇팔 이동, OnRobot RG2 그리퍼 제어를 함께 다룹니다. 또한 사람이 로봇이 들고 있는 공구를 잡았는지 판단하는 hand-tool grasp detection 노드를 포함합니다.

## 주요 기능

- RealSense color/depth 이미지 구독
- YOLO 기반 공구 및 대상 객체 인식
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
source ~/ros2_ws/src/doosan-robot2/install/setup.bash
```

## 전체 파이프라인 실행

각 터미널은 새로 열 때마다 ROS 2, `ros2_ws`, Doosan MoveIt 환경을 source한 뒤 실행합니다.

### Terminal 1: Doosan M0609 + MoveIt 실행

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws/src/doosan-robot2/install/setup.bash

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
source ~/ros2_ws/src/doosan-robot2/install/setup.bash

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
source ~/ros2_ws/src/doosan-robot2/install/setup.bash

ros2 launch macgyvbot macgyvbot.launch.py
```

명시적으로 중심점 모드를 사용할 경우:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws/src/doosan-robot2/install/setup.bash

ros2 launch macgyvbot macgyvbot.launch.py grasp_point_mode:=center
```

VLM 기반 grasp point selection을 사용할 경우:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws/src/doosan-robot2/install/setup.bash

ros2 launch macgyvbot macgyvbot.launch.py grasp_point_mode:=vlm
```

VLM 모드는 YOLO가 검출한 객체 crop에서 grid 기반 grasp region을 선택한 뒤 depth로 grasp pixel을 보정합니다. VLM 추론 또는 depth 보정이 실패하면 기존 중심점 방식으로 fallback합니다.

GraspNet 기반 orientation selection을 사용할 경우:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws/src/doosan-robot2/install/setup.bash

ros2 launch macgyvbot macgyvbot.launch.py \
  grasp_point_mode:=graspnet
```

`grasp_point_mode:=graspnet`은 YOLO + depth로 target position을 잡고, GraspNet pose에서 orientation을 받아 보정합니다. GraspNet position은 기본 이동 목표로 사용하지 않습니다.

## 실제 GraspNet inference 설정

GraspNet은 항상 실제 baseline/checkpoint를 사용합니다. `macgyvbot` 저장소에 baseline 코드나 checkpoint를 commit하지 않습니다. 공식 baseline 저장소와 checkpoint는 아래 기본 경로에 둡니다.

공식 GraspNet baseline:

- https://github.com/graspnet/graspnet-baseline
- https://github.com/graspnet/graspnetAPI

### 1. GraspNet baseline clone

```bash
mkdir -p ~/third_party
cd ~/third_party
git clone https://github.com/graspnet/graspnet-baseline.git
```

### 2. GraspNet Python 의존성 설치

ROS 2 Humble에서 사용하는 Python 환경과 같은 환경에 설치해야 합니다.

```bash
cd ~/ros2_ws/src/doosan-robot2/macgyvbot
pip install -r requirements-graspnet.txt
```

baseline 자체 의존성도 설치합니다.

```bash
cd ~/third_party/graspnet-baseline
pip install -r requirements.txt
```

CUDA extension을 빌드합니다. CUDA/PyTorch 버전이 맞지 않으면 여기서 실패하므로, 먼저 `python -c "import torch; print(torch.cuda.is_available())"`로 GPU 인식을 확인합니다.

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

### 3. checkpoint 다운로드

공식 baseline README의 pretrained weights에서 RealSense용 `checkpoint-rs.tar`를 받습니다. D435i/RealSense 입력을 쓰므로 우선 `checkpoint-rs.tar`를 권장합니다.

```bash
mkdir -p ~/models/graspnet
# checkpoint-rs.tar 파일을 ~/models/graspnet/checkpoint-rs.tar 로 둡니다.
```

checkpoint 파일은 크기가 크므로 git에 추가하지 않습니다.

### 4. GraspNet mode 실행

`macgyvbot.launch.py`에서 `grasp_point_mode:=graspnet`을 주면 GraspNet inference node도 함께 실행됩니다. 실제 inference는 RGB-D point cloud를 camera frame에서 만들고, MacGyvBot main node는 camera frame pose를 hand-eye calibration으로 `base_link` pose로 변환한 뒤 사용합니다.

GraspNet pose topic만 단독 확인할 경우:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws/src/doosan-robot2/install/setup.bash

ros2 launch macgyvbot graspnet_inference.launch.py
```

다른 터미널에서 pose를 확인합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws/src/doosan-robot2/install/setup.bash

ros2 topic echo /graspnet/target_pose
```

MacGyvBot과 함께 통합 실행할 경우:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws/src/doosan-robot2/install/setup.bash

ros2 launch macgyvbot macgyvbot.launch.py \
  grasp_point_mode:=graspnet
```

GraspNet mode에서는 position은 YOLO + depth 기준을 유지하고 orientation만 GraspNet pose를 적용합니다. GraspNet position을 실제 로봇 목표 좌표로 쓰려면 hand-eye calibration, depth scale, camera frame 이름, 안전영역 clamp가 모두 맞는지 확인한 뒤 별도 변경으로 분리합니다.

### Terminal 4: 대상 공구 요청

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws/src/doosan-robot2/install/setup.bash

ros2 topic pub --once /target_label std_msgs/msg/String "{data: screwdriver}"
```

사용 가능한 공구 label은 학습한 YOLO 모델의 class 이름과 같아야 합니다. 현재 예시는 `hammer`, `screwdriver`, `pliers`, `tape_measure`를 기준으로 합니다.

## 잡기 인식 노드 실행

`macgyvbot.launch.py`는 hand grasp detection 노드를 함께 실행합니다. 잡기 인식 노드만 단독으로 확인할 때는 아래 명령을 사용합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws/src/doosan-robot2/install/setup.bash

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
