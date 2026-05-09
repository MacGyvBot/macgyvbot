# MacGyvBot File Roles

이 문서는 `macgyvbot/` 패키지 내부 파일들과 `weights/` 폴더의 역할을 빠르게 파악하기 위한 설명서입니다.

## 전체 흐름

```text
command_input_node
  -> GUI 입력 또는 마이크 STT 수집
  -> command parser로 자연어 명령 해석
  -> /tool_command, /command_feedback 발행

macgyvbot_main_node
  -> /tool_command 또는 수동 /target_label 수신
  -> RealSense color/depth 기반 YOLO 탐지
  -> grasp point 선택
  -> depth pixel을 robot base 좌표로 투영
  -> PickSequenceRunner로 pick, handoff, 반환 시퀀스 실행
  -> /robot_task_status 발행

hand_grasp_detection_node
  -> 카메라 image/depth 수신
  -> hand landmark와 tool ROI를 비교
  -> 사람이 공구를 잡았는지 /human_grasped_tool로 발행
```

## `macgyvbot/`

### Package Markers

- `macgyvbot/__init__.py`
  - Python 패키지 인식용 파일입니다.

### `macgyvbot/config/`

- `macgyvbot/config/__init__.py`
  - `config` 하위 패키지 인식용 파일입니다.

- `macgyvbot/config/config.py`
  - 런타임 공통 설정을 Python 상수로 보관합니다.
  - MoveIt group/link 이름, home joint, 안전 Z offset, hand grasp topic, debug window 이름, grasp point mode 등을 제공합니다.
  - 여러 util과 node에서 import하는 중앙 설정 파일입니다.

### `macgyvbot/nodes/`

- `macgyvbot/nodes/__init__.py`
  - ROS node entrypoint 패키지 인식용 파일입니다.

- `macgyvbot/nodes/command_input_node.py`
  - GUI 입력과 마이크 STT 입력을 하나의 명령 입력 노드로 통합합니다.
  - `SpeechToTextService`로 음성을 텍스트로 바꾸고, `CommandLlmParser`로 자연어 명령을 해석합니다.
  - `/tool_command`에 JSON 명령을 발행하고, `/command_feedback`으로 해석 결과 피드백을 냅니다.
  - `/robot_task_status`를 구독해 로봇 실행 상태를 GUI 채팅창에 표시합니다.

- `macgyvbot/nodes/macgyvbot_main_node.py`
  - 메인 로봇 실행 노드입니다.
  - RealSense color/depth/camera_info, `/tool_command`, `/target_label`, `/human_grasped_tool`을 구독합니다.
  - YOLO 탐지, grasp point 선택, depth projection, MoveIt 실행 시퀀스 시작을 조율합니다.
  - `/robot_task_status`를 발행해 command input GUI와 상태를 동기화합니다.
  - OpenCV debug window 출력도 이 노드 내부 helper로 처리합니다.

- `macgyvbot/nodes/hand_grasp_detection_node.py`
  - 사람이 공구를 잡았는지 판단하는 별도 ROS 노드입니다.
  - color/depth image를 받아 hand landmark, tool ROI, depth contact를 계산합니다.
  - 결과 JSON을 `/human_grasped_tool`로 발행하고, overlay 이미지를 `/hand_grasp_detection/annotated_image`로 발행합니다.

### `macgyvbot/ui/`

- `macgyvbot/ui/__init__.py`
  - UI 하위 패키지 인식용 파일입니다.

- `macgyvbot/ui/voice_command_window.py`
  - PyQt 기반 command input GUI 창입니다.
  - 사용자 입력창, 전송 버튼, 채팅 로그, 상태 라벨을 구성합니다.
  - ROS 로직은 직접 갖지 않고, `command_input_node`의 메서드를 호출해 입력을 전달합니다.

### `macgyvbot/util/`

- `macgyvbot/util/__init__.py`
  - util 하위 패키지 인식용 파일입니다.

## `macgyvbot/util/input_mapping/`

- `macgyvbot/util/input_mapping/__init__.py`
  - 명령 해석 유틸 패키지 인식용 파일입니다.

- `macgyvbot/util/input_mapping/command_hard_parser.py`
  - LLM 호출 전에 먼저 실행되는 빠른 local parser입니다.
  - 공구 별칭, 한국어 키워드, fuzzy matching, action keyword를 사용합니다.
  - 명확한 명령은 이 단계에서 바로 `tool_name`, `action`, `confidence`로 변환됩니다.

- `macgyvbot/util/input_mapping/command_llm_parser.py`
  - local parser 실패 시 Ollama LLM fallback을 수행하는 parser입니다.
  - JSON command 형식을 검증하고, 낮은 confidence는 확인 질문 상태로 넘깁니다.
  - `bring`, `release`, `stop` 같은 action과 허용된 tool 목록을 관리합니다.

## `macgyvbot/util/stt/`

- `macgyvbot/util/stt/__init__.py`
  - STT 유틸 패키지 인식용 파일입니다.

- `macgyvbot/util/stt/speech_to_text.py`
  - `speech_recognition` 기반 마이크 STT wrapper입니다.
  - 마이크 목록 출력, 주변 소음 보정, Google STT background listen을 담당합니다.
  - ROS 의존성 없이 텍스트 callback만 호출하도록 분리되어 있습니다.

## `macgyvbot/util/perception/`

- `macgyvbot/util/perception/__init__.py`
  - perception 유틸 패키지 인식용 파일입니다.

- `macgyvbot/util/perception/yolo_detector.py`
  - Ultralytics YOLO 모델 로딩과 추론을 감싼 wrapper입니다.
  - ROS install share의 `weights/`, 현재 작업 디렉터리의 `weights/` 등을 순서대로 찾아 모델 경로를 해석합니다.

- `macgyvbot/util/perception/grasp_point_selector.py`
  - 탐지된 bbox에서 실제 grasp pixel을 선택합니다.
  - 기본은 bbox 중심점이고, 설정이 `vlm`이면 `grasp_by_vlm.py`를 lazy import해 VLM grasp point를 시도합니다.
  - VLM 실패 시 bbox 중심점으로 fallback합니다.

- `macgyvbot/util/perception/depth_projection.py`
  - image pixel과 depth 값을 robot base 좌표로 변환합니다.
  - camera intrinsics와 gripper-to-camera calibration, 현재 end-effector transform을 사용합니다.

## `macgyvbot/util/grasp_mechanism/`

- `macgyvbot/util/grasp_mechanism/__init__.py`
  - grasp mechanism 패키지 인식용 파일입니다.

- `macgyvbot/util/grasp_mechanism/grasp_by_vlm.py`
  - VLM 기반 grasp point 선택 로직입니다.
  - object crop을 grid로 나누고, VLM 응답에서 적절한 grasp region과 yaw/orientation 후보를 추출합니다.
  - depth refinement로 선택 pixel을 보정할 수 있습니다.
  - 무거운 model dependency가 있어 `grasp_point_selector.py`에서 필요할 때 lazy load됩니다.

## `macgyvbot/util/model_control/`

- `macgyvbot/util/model_control/__init__.py`
  - robot control 유틸 패키지 인식용 파일입니다.

- `macgyvbot/util/model_control/moveit_controller.py`
  - MoveItPy planning과 execution helper입니다.
  - pose goal/state goal 실행, safe workspace clamp, VLM yaw 기반 wrist joint 회전을 담당합니다.

- `macgyvbot/util/model_control/robot_pose.py`
  - `PoseStamped` 생성, 현재 end-effector transform 조회, orientation 변환, angle normalize helper를 제공합니다.

- `macgyvbot/util/model_control/robot_safezone.py`
  - robot workspace 안전 범위 clamp를 담당합니다.
  - 목표 pose가 안전 영역 밖이면 최소/최대 경계로 보정합니다.

- `macgyvbot/util/model_control/onrobot_gripper.py`
  - OnRobot RG gripper 제어 클래스입니다.
  - gripper open/close, TCP/Modbus 계열 통신 처리를 담당합니다.

## `macgyvbot/util/task_pipeline/`

- `macgyvbot/util/task_pipeline/__init__.py`
  - task pipeline 패키지 인식용 파일입니다.

- `macgyvbot/util/task_pipeline/task_pipeline.py`
  - pick, lift, home 복귀, 사용자 handoff 대기, 실패 시 원위치 반환까지의 실행 시퀀스를 담당합니다.
  - `macgyvbot_main_node`에서 시작되지만, 실제 단계별 MoveIt/gripper 실행 흐름은 이 파일에 분리되어 있습니다.
  - 주요 성공/실패 상태를 `/robot_task_status`로 보고할 수 있도록 node state를 사용합니다.

## `macgyvbot/util/hand_grasp/`

- `macgyvbot/util/hand_grasp/__init__.py`
  - hand grasp 유틸 패키지 인식용 파일입니다.

- `macgyvbot/util/hand_grasp/hand_detector.py`
  - MediaPipe Hands wrapper입니다.
  - 손 landmark, palm center, handedness, hand bbox를 계산합니다.
  - 중복 검출된 손을 IoU와 palm distance 기준으로 정리합니다.

- `macgyvbot/util/hand_grasp/tool_detector.py`
  - hand grasp 판단용 YOLO tool ROI detector입니다.
  - target class 중 가장 confidence가 높은 tool bbox를 반환합니다.
  - 기본 target class는 `drill`, `hammer`, `pliers`, `screwdriver`, `tape_measure`, `wrench`입니다.

- `macgyvbot/util/hand_grasp/grasp_detector.py`
  - 손과 공구가 실제로 잡힘 상태인지 판단하는 heuristic state machine입니다.
  - landmark proximity, palm/tool distance, hand-tool overlap, pinch distance, depth contact를 종합합니다.
  - 일정 frame 이상 grasp candidate가 유지되면 `HUMAN_GRASPED_TOOL`로 확정합니다.

- `macgyvbot/util/hand_grasp_detection/hand_grasp/calculations.py`
  - hand grasp 판단에 필요한 geometry/depth helper 모음입니다.
  - rectangle distance, IoU, depth median, depth contact 정보 생성, active hand 선택, text drawing 등을 제공합니다.

- `macgyvbot/util/hand_grasp/visualization.py`
  - hand grasp detection overlay drawing helper입니다.
  - tool bbox, hand landmarks, active hand, state text를 camera frame 위에 그립니다.

## `weights/`

- `weights/`
  - YOLO `.pt` weight 파일과 VLM local weight를 둘 수 있는 모델 저장 폴더입니다.
  - `setup.py`에서 `weights/*.pt`, `weights/*.py`, `weights/vlm/**`를 ROS install share로 배포하도록 구성되어 있습니다.
  - 현재 저장소에는 큰 모델 파일이 직접 포함되어 있지 않습니다.

- `weights/.gitkeep`
  - 실제 weight 파일이 없어도 Git이 `weights/` 폴더를 유지하도록 하는 자리표시 파일입니다.

- `weights/download_vlm_weights.py`
  - Hugging Face의 `HuggingFaceTB/SmolVLM2-2.2B-Instruct` 모델 snapshot을 `weights/vlm/` 아래로 다운로드하는 스크립트입니다.
  - `grasp_by_vlm.py`는 ROS install share 또는 source tree의 `weights/vlm/`에서 local VLM weight를 찾습니다.

### Expected Weight Layout

```text
weights/
├── yolov11_best.pt
├── download_vlm_weights.py
└── vlm/
    └── HuggingFaceTB__SmolVLM2-2.2B-Instruct/
        ├── config.json
        ├── model files...
        └── tokenizer / processor files...
```

`yolov11_best.pt`는 메인 객체 탐지와 hand grasp tool ROI 탐지에서 기본 YOLO 모델명으로 사용됩니다. VLM weight는 `grasp_point_mode:=vlm`일 때 grasp point 후보를 고르는 데 사용됩니다.
