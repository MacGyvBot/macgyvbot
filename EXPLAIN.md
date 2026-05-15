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
  -> PickSequenceRunner 또는 ReturnSequenceRunner로 작업 시퀀스 실행
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
  - 기존 `macgyvbot.config.config` import 호환을 위한 facade입니다.
  - 신규 코드는 역할별 config 모듈을 직접 import합니다.

- `macgyvbot/config/robot.py`
  - MoveIt group, base/end-effector frame, Home/observation joint pose를 제공합니다.

- `macgyvbot/config/topics.py`
  - ROS topic 이름을 제공합니다.

- `macgyvbot/config/models.py`
  - YOLO, hand grasp ML, SAM weight 파일명을 제공합니다.

- `macgyvbot/config/pick.py`
  - pick target planning용 Z offset과 높이 보정값을 제공합니다.

- `macgyvbot/config/grasp.py`
  - gripper grasp 검증 timeout, polling, retry 값을 제공합니다.

- `macgyvbot/config/hand_grasp.py`
  - human grasp 판단 timeout과 ML confidence 기준을 제공합니다.

- `macgyvbot/config/handoff.py`
  - handoff 위치 탐색, 안정화, offset, replan 설정을 제공합니다.

- `macgyvbot/config/return_flow.py`
  - return workflow의 force-guided Z 하강 설정을 제공합니다.

- `macgyvbot/config/timing.py`
  - sequence polling 주기를 제공합니다.

- `macgyvbot/config/ui.py`
  - OpenCV debug window 이름을 제공합니다.

- `macgyvbot/config/vlm.py`
  - grasp point selection mode와 VLM grid 설정을 제공합니다.

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
  - `/robot_task_status`가 `accepted/searching/picking/grasping`인 구간에서만 최신 YOLO ROI와 선택적 SAM mask를 갱신합니다.
  - `/robot_task_status`의 `grasp_success`를 lock trigger로 사용하고, 직전 mask/ROI를 고정한 뒤 `/hand_grasp_detection/tool_mask_lock`으로 응답합니다.
  - main pick sequence는 이 lock 응답을 받은 뒤에만 lift와 handoff 이동을 시작합니다.
  - handoff pose에서는 depth contact, `.pkl` ML classifier grasp, locked ROI/SAM mask contact가 모두 참일 때 handoff 성공을 인정합니다.
  - 각 신호는 `/human_grasped_tool` payload에 진단값으로 함께 발행합니다.
  - 결과 JSON을 `/human_grasped_tool`로 발행하고, overlay 이미지를 `/hand_grasp_detection/annotated_image`로 발행합니다.

### `macgyvbot/ui/`

- `macgyvbot/ui/__init__.py`
  - UI 하위 패키지 인식용 파일입니다.

- `macgyvbot/ui/voice_command_window.py`
  - PyQt 기반 command input GUI 창입니다.
  - 사용자 입력창, 전송 버튼, 채팅 로그, 상태 라벨을 구성합니다.
  - ROS 로직은 직접 갖지 않고, `command_input_node`의 메서드를 호출해 입력을 전달합니다.

### `macgyvbot/domain/`

- `macgyvbot/domain/target_models.py`
  - YOLO box, grasp point, depth projection으로 생성한 pick target 결과 타입을 제공합니다.

- `macgyvbot/domain/pick_models.py`
  - perception 결과를 pick motion waypoint로 바꾼 결과 타입을 제공합니다.

### `macgyvbot/application/`

- `macgyvbot/application/tool_command_controller.py`
  - parsed command를 bring/return/release/stop 실행 경로로 라우팅합니다.
  - ROS message 타입을 직접 알지 않고 callback과 status publisher를 통해 동작합니다.

- `macgyvbot/application/robot_status_publisher.py`
  - `/robot_task_status` payload shape을 한 곳에서 조립합니다.
  - ROS publish 자체는 node adapter에 위임합니다.

- `macgyvbot/application/robot_home_initializer.py`
  - 초기 Home 이동과 Home pose/orientation 저장을 담당합니다.

- `macgyvbot/application/pick_frame_processor.py`
  - 카메라 프레임의 YOLO 추론, target detection, grasp marker 렌더링, searching status 발행을 담당합니다.

- `macgyvbot/application/pick_target_planner.py`
  - object base 좌표와 depth 보정값을 안전 workspace 내부 pick waypoint로 변환합니다.

- `macgyvbot/application/pick_grasp_flow.py`
  - 초기 gripper grasp 검증을 담당합니다.

- `macgyvbot/application/pick_handoff_flow.py`
  - 사용자 손 위치 탐색, handoff 이동, hand grasp 대기, 실패 시 원위치 반환을 담당합니다.

- `macgyvbot/application/return_handoff_flow.py`
  - 반납할 사용자-held 공구 위치 탐색, 수령 위치 이동, gripper grasp를 담당합니다.

- `macgyvbot/application/return_home_placement_flow.py`
  - 반납 공구를 Home 위치로 이동시키고 force-guided 하강 후 release합니다.

- `macgyvbot/application/return_status_reporter.py`
  - return workflow 전용 status payload를 조립하고 발행합니다.

- `macgyvbot/application/pick_sequence.py`
  - pick, lift, handoff 대기, 실패 시 원위치 반환까지의 실행 시퀀스를 담당합니다.
  - target planning, 초기 grasp 검증, handoff 세부 절차는 하위 workflow에 위임합니다.

- `macgyvbot/application/return_sequence.py`
  - `return` 명령에서 사용자 handoff를 기다린 뒤 공구를 받아 Home 기준 반납 위치에 내려놓습니다.
  - 수령, 배치, status reporting 세부 절차는 하위 workflow에 위임합니다.

### `macgyvbot/perception/`

- `macgyvbot/perception/model_paths.py`
  - YOLO, SAM, ML 모델 weight 경로 해석을 공통으로 처리합니다.

- `macgyvbot/perception/yolo_detector.py`
  - Ultralytics YOLO 모델 로딩과 추론을 감싼 wrapper입니다.

- `macgyvbot/perception/pick_target_resolver.py`
  - YOLO box 선택, grasp point 선택, depth projection을 묶어 pick target을 생성합니다.
  - `macgyvbot_main_node`의 target detection callback 성격 로직을 node 밖으로 분리합니다.

- `macgyvbot/perception/depth_projection.py`
  - image pixel과 depth 값을 camera/base 좌표로 변환합니다.
  - 로봇 객체를 직접 알지 않고 `base_to_camera` 행렬 provider만 주입받습니다.

### `macgyvbot/perception/grasp_mechanism/`

- `macgyvbot/perception/grasp_mechanism/grasp_point_selector.py`
  - 탐지된 bbox에서 실제 grasp pixel을 선택합니다.
  - 기본은 bbox 중심점이고, 설정이 `vlm`이면 VLM grasp point를 시도합니다.

- `macgyvbot/perception/grasp_mechanism/grasp_by_vlm.py`
  - VLM 기반 grasp point 선택과 depth refinement를 담당합니다.

### `macgyvbot/control/`

- `macgyvbot/control/moveit_controller.py`
  - MoveItPy planning과 execution helper입니다.

- `macgyvbot/control/robot_pose.py`
  - `PoseStamped` 생성, 현재 end-effector transform 조회, orientation 변환 helper를 제공합니다.

- `macgyvbot/control/robot_safezone.py`
  - robot workspace 안전 범위 clamp를 담당합니다.

- `macgyvbot/control/onrobot_gripper.py`
  - OnRobot RG gripper 제어 클래스입니다.

- `macgyvbot/control/grasp_verifier.py`
  - 초기 grasp 성공 여부를 OnRobot RG 상태로 검증합니다.

- `macgyvbot/control/force_detection.py`
  - force/torque 입력 기반 Z 하강을 수행합니다.

### `macgyvbot/recovery/`

- `macgyvbot/recovery/__init__.py`
  - recovery 전용 객체를 위한 자리입니다. 현재 리팩토링 범위에서는 동작을 추가하지 않습니다.

### `macgyvbot/command_input/`

- `macgyvbot/command_input/input_mapping/command_hard_parser.py`
  - LLM 호출 전에 먼저 실행되는 빠른 local parser입니다.

- `macgyvbot/command_input/input_mapping/command_llm_parser.py`
  - local parser 실패 시 Ollama LLM fallback을 수행하는 parser입니다.

- `macgyvbot/command_input/input_mapping/command_vocabulary.py`
  - tool alias, action keyword, LLM 허용 schema, confirmation vocabulary를 보관합니다.

- `macgyvbot/command_input/stt/speech_to_text.py`
  - `speech_recognition` 기반 마이크 STT wrapper입니다.

- `macgyvbot/command_input/tts/tts_service.py`
  - 음성 안내 출력을 담당합니다.

### `macgyvbot/perception/hand_grasp/`

- `macgyvbot/perception/hand_grasp/hand_detector.py`
  - MediaPipe Hands wrapper입니다.

- `macgyvbot/perception/hand_grasp/tool_detector.py`
  - hand grasp 판단용 YOLO tool ROI detector입니다.

- `macgyvbot/perception/hand_grasp/ml_grasp_classifier.py`
  - ML 기반 손 grasp classifier를 로드하고, raw/stable grasp 상태를 판정합니다.

- `macgyvbot/perception/hand_grasp/sam_tool_mask.py`
  - 선택적 SAM mask 생성과 locked mask contact 계산을 담당합니다.

- `macgyvbot/perception/hand_grasp/calculations.py`
  - hand grasp 판단에 필요한 geometry/depth helper 모음입니다.

- `macgyvbot/perception/hand_grasp/visualization.py`
  - hand grasp detection overlay drawing helper입니다.

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
