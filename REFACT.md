# 리팩토링 기록

## 1. Weight 디렉터리 정리

- 저장소의 weight 보관 디렉터리를 `models/`에서 `weights/`로 변경했다.
- VLM 다운로드 스크립트를 `scripts/download_vlm_weights.py`에서 `weights/download_vlm_weights.py`로 옮겼다.
- VLM 다운로드 위치를 `weights/vlm/` 기준으로 변경했다.
- 패키지 설치 시 weight 파일이 `share/macgyvbot/weights` 아래에 설치되도록 수정했다.
- YOLO와 VLM의 런타임 경로 탐색 기준을 `weights/`로 변경했다.
- 더 이상 사용하지 않는 `scripts/` 디렉터리를 제거했다.

확인:
- 런타임 경로에서 `models/`와 제거된 `scripts/` 디렉터리 참조가 남지 않은 것을 확인했다.
- 변경된 Python 파일의 문법 검사를 수행했다.
- `setup.py --name`으로 패키지 이름이 정상적으로 확인되는 것을 확인했다.

## 2. 중복 launch 파일 정리

- `hand_grasp_detection.launch.py`의 설정이 `macgyvbot.launch.py`에 동일하게 포함되어 있는 것을 확인했다.
- 별도 launch 파일인 `launch/hand_grasp_detection.launch.py`를 제거했다.
- README의 hand grasp detection 실행 예시를 `macgyvbot.launch.py` 기준으로 수정했다.

확인:
- 코드와 README에서 제거된 launch 파일을 실행 경로로 참조하지 않는 것을 확인했다.

## 3. 문서 폴더 추가

- 브랜치별 작업 설명을 담기 위한 `docs/` 폴더를 추가했다.
- `docs/README.md`에 문서 작성 용도와 간단한 작성 기준을 정리했다.

## 4. Hand grasp detection 경계 정리

- hand grasp detection 노드를 `macgyvbot/nodes/` 아래로 이동했다.
- 루트의 `hand_grasp_detection_node.py` wrapper는 사용처가 없어 제거했다.
- depth 변환과 active hand 선택 로직을 `grasp_detection/calculations.py`로 옮겼다.
- overlay drawing 로직을 `grasp_detection/visualization.py`로 분리했다.
- YOLO 모델 경로 해석은 `ToolDetector`가 담당하도록 중복 resolver를 제거했다.

확인:
- 변경된 hand grasp detection 관련 Python 파일의 문법 검사를 수행했다.
- `setup.py --name`으로 패키지 이름이 정상적으로 확인되는 것을 확인했다.

## 5. Grasp mechanism 모듈 분리

- VLM 기반 grasp point 선택 파일을 `grasp_point_detection.py`에서 `grasp_mechanism/grasp_by_vlm.py`로 변경했다.
- bbox 중심점 기반 grasp point 선택을 `grasp_mechanism/grasp_by_bbox_center.py`로 분리했다.
- `grasp_mechanism/` 패키지 아래에 grasp point 선택 방식들을 묶었다.
- VLM crop, 추론, depth refinement 절차를 `VLMGraspMechanism`으로 옮겼다.
- `GraspPointSelector`는 선택 모드에 따라 grasp mechanism을 호출하는 역할만 하도록 줄였다.
- VLM 파일 이동에 맞춰 source tree 기준 weight fallback 경로를 조정했다.

확인:
- 변경된 grasp mechanism 관련 Python 파일의 문법 검사를 수행했다.
- 이전 `grasp_point_detection` import 참조가 남지 않은 것을 확인했다.

## 6. Main node wrapper 제거

- 루트의 `macgyvbot.py` wrapper가 실제 entrypoint에서 사용되지 않는 것을 확인했다.
- `setup.py`는 이미 `macgyvbot.nodes.macgyvbot_main_node:main`을 직접 사용하고 있어 wrapper를 제거했다.
- README의 패키지 구조와 executable 설명을 새 구조에 맞게 수정했다.

## 7. Util 폴더 정리

- 그리퍼 연결/제어 파일을 `onrobot.py`에서 `util/onrobot_gripper.py`로 옮겼다.
- 로봇 작업공간 제한 파일을 `safety.py`에서 `util/robot_safezone.py`로 옮겼다.
- 새 파일 위치에 맞춰 import 경로를 수정했다.
- README의 패키지 구조에 `util/` 항목을 추가했다.

확인:
- 변경된 util 관련 Python 파일의 문법 검사를 수행했다.
- 이전 `macgyvbot.onrobot`, `macgyvbot.safety` import 참조가 남지 않은 것을 확인했다.

## 8. Util 하위 기능 폴더 정리

- `grasp_detection/` 폴더를 hand grasp 전용 의미가 드러나도록 `util/hand_grasp/`로 옮겼다.
- `grasp_mechanism/` 폴더를 `util/grasp_mechanism/` 아래로 옮겼다.
- 새 위치에 맞춰 hand grasp node와 grasp point selector의 import 경로를 수정했다.
- README의 패키지 구조를 새 util 하위 구조에 맞게 수정했다.

확인:
- 변경된 hand grasp와 grasp mechanism 관련 Python 파일의 문법 검사를 수행했다.
- `find_packages()`에서 `macgyvbot.util.hand_grasp`와 `macgyvbot.util.grasp_mechanism`이 포함되는 것을 확인했다.

## 9. Model control 폴더 추가

- `util/model_control/` 폴더를 추가했다.
- `util/onrobot_gripper.py`와 `util/robot_safezone.py`를 `util/model_control/` 아래로 옮겼다.
- 새 위치에 맞춰 gripper와 safezone import 경로를 수정했다.
- README의 패키지 구조를 새 model control 하위 구조에 맞게 수정했다.

확인:
- 변경된 model control 관련 Python 파일의 문법 검사를 수행했다.
- `find_packages()`에서 `macgyvbot.util.model_control`이 포함되는 것을 확인했다.

## 10. Perception 폴더 정리

- YOLO detector만 `util/perception/yolo_detector.py`로 이동했다.
- `depth_projection.py`의 depth pixel to base 변환 로직을 `macgyvbot_main_node.py`로 흡수했다.
- `grasp_point_selector.py`의 grasp point 선택 흐름을 `macgyvbot_main_node.py`로 흡수했다.
- 기존 `perception/` 패키지를 제거하고 README의 패키지 구조를 수정했다.

확인:
- 변경된 main node와 YOLO detector 파일의 문법 검사를 수행했다.
- 이전 `macgyvbot.perception` import 참조가 남지 않은 것을 확인했다.
- `find_packages()`에서 `macgyvbot.util.perception`이 포함되는 것을 확인했다.

## 11. Config와 task pipeline 위치 정리

- `core/config.py`를 `config/config.py`로 옮겼다.
- `core/pick_sequence.py`를 루트 패키지의 `task_pipeline.py`로 옮겼다.
- 새 위치에 맞춰 config와 `PickSequenceRunner` import 경로를 수정했다.
- README의 패키지 구조를 새 위치에 맞게 수정했다.

확인:
- 변경된 config, task pipeline, 관련 import 파일의 문법 검사를 수행했다.
- 이전 `macgyvbot.core.config`, `macgyvbot.core.pick_sequence` import 참조가 남지 않은 것을 확인했다.
- 빈 `core/` 패키지를 제거했다.
- `setup.py --name`으로 패키지 이름이 정상적으로 확인되는 것을 확인했다.

## 12. Motion 폴더 제거

- `motion/moveit_controller.py`를 `util/model_control/moveit_controller.py`로 옮겼다.
- `motion/pose_utils.py`를 `util/model_control/robot_pose.py`로 이름을 바꿔 옮겼다.
- 새 위치에 맞춰 MoveIt controller와 robot pose import 경로를 수정했다.
- `motion/` 패키지를 제거했다.
- README의 패키지 구조를 새 model control 구조에 맞게 수정했다.

확인:
- 변경된 model control, task pipeline, main node 파일의 문법 검사를 수행했다.
- 이전 `macgyvbot.motion` import 참조가 남지 않은 것을 확인했다.
- `find_packages()`에서 `macgyvbot.motion`이 제외되는 것을 확인했다.

## 13. Input mapping 폴더 추가

- `voice_command/command_parser.py`를 `util/input_mapping/command_hard_parser.py`로 옮겼다.
- 새 위치에 맞춰 LLM command node의 parser import 경로를 수정했다.
- 빈 `voice_command/` 패키지를 제거했다.
- README의 패키지 구조를 새 input mapping 위치에 맞게 수정했다.

확인:
- 변경된 input mapping parser와 LLM command node의 문법 검사를 수행했다.
- `find_packages()`에서 `macgyvbot.util.input_mapping`이 포함되는 것을 확인했다.

## 14. Task pipeline 폴더 이동

- 루트 패키지의 `task_pipeline.py`를 `util/task_pipeline/task_pipeline.py`로 옮겼다.
- 새 위치에 맞춰 main node의 `PickSequenceRunner` import 경로를 수정했다.
- README의 패키지 구조를 새 task pipeline 위치에 맞게 수정했다.

확인:
- 변경된 task pipeline 파일과 main node의 문법 검사를 수행했다.
- 루트 패키지의 `task_pipeline.py`가 제거된 것을 확인했다.
- `find_packages()`에서 `macgyvbot.util.task_pipeline`이 포함되는 것을 확인했다.

## 15. Review 반영

- task pipeline 오타가 실제 패키지 경로로 굳어지지 않도록 `task_pipeline/task_pipeline.py`로 정정했다.
- `macgyvbot_main_node.py`에 흡수됐던 grasp point 선택 흐름을 `util/perception/grasp_point_selector.py`로 다시 분리했다.
- depth pixel to base 변환 로직을 `util/perception/depth_projection.py`로 다시 분리했다.
- main node는 ROS wiring, 상태 수신, frame loop, pick 시작 요청 중심으로 축소했다.

확인:
- `python3 -m py_compile`로 변경된 Python 파일의 문법 검사를 수행했다.
- `rg`로 task pipeline 오타 경로가 남지 않은 것을 확인했다.
- `__init__.py` 파일 기준으로 `macgyvbot.util.task_pipeline`과 perception 하위 모듈이 패키지로 포함되는 것을 확인했다.

## 16. Voice command UI 파일 정리

- `voice_command_ui_node.py`와 `voice_command_gui_node.py`를 비교해 GUI 노드가 기존 토픽 처리 기능을 포함하면서 UI만 확장한 것을 확인했다.
- 중복 파일인 `macgyvbot/nodes/voice_command_ui_node.py`를 제거했다.
- 실행 호환성을 위해 `setup.py`의 `voice_command_ui_node` 엔트리포인트를 `macgyvbot.nodes.voice_command_gui_node:main`으로 변경했다.
- README의 패키지 구조/실행 설명에서 삭제된 UI 파일 관련 문구를 정리했다.

확인:
- `rg`로 삭제된 `voice_command_ui_node.py` 파일 경로 참조가 남지 않은 것을 확인했다.
- `macgyvbot/nodes`에 `voice_command_gui_node.py`만 남아 있는 것을 확인했다.

## 17. GUI UI 코드 의존성 분리

- `voice_command_gui_node.py`에서 `VoiceCommandGuiWindow` UI 클래스를 분리해 `macgyvbot/ui/voice_command_window.py`를 추가했다.
- 노드 파일은 ROS 노드 로직 중심으로 유지하고, UI 관련 의존성은 `macgyvbot.ui.voice_command_window` import로 연결했다.
- PyQt5 미설치 처리 로직은 UI 모듈과 노드 진입점에서 안전하게 동작하도록 유지했다.

확인:
- `python3 -m py_compile macgyvbot/nodes/voice_command_gui_node.py macgyvbot/ui/voice_command_window.py`로 문법 검사를 통과했다.

## 18. STT 단일 노드로 명령 해석 통합

- `llm_command_node.py`의 하이브리드 해석 로직을 `util/input_mapping/command_llm_parser.py`로 분리했다.
- `stt_node.py`가 STT 발행과 함께 명령 해석(`local parser -> LLM fallback`)을 직접 수행하도록 통합했다.
- `stt_node`가 `/stt_text`를 구독해 GUI 키보드 입력도 동일 파이프라인으로 처리하도록 구성했다.
- STT 노드 자기 발행 메시지 루프를 피하기 위해 self-published 메시지 필터를 추가했다.
- `launch/macgyvbot.launch.py`에서 `llm_command_node` 실행을 제거하고 `stt_node` 하나만 실행하도록 정리했다.
- `use_stt:=false`일 때도 명령 해석은 가능하도록 `stt_node`는 유지하고 `enable_microphone` 파라미터로 마이크만 끄도록 변경했다.
- `setup.py`에서 `llm_command_node` 엔트리포인트를 제거했다.
- `macgyvbot/nodes/llm_command_node.py` 파일을 제거했다.
- README의 노드 구조/흐름 설명을 통합 구조에 맞게 수정했다.

확인:
- `python3 -m py_compile macgyvbot/nodes/stt_node.py macgyvbot/util/input_mapping/command_llm_parser.py`로 문법 검사를 통과했다.

## 19. GUI + STT 완전 통합 및 STT 유틸 분리

- `stt_node.py`와 `voice_command_gui_node.py` 역할을 하나로 합쳐 음성 명령 관련 노드를 `stt_node` 하나로 통합했다.
- 통합 노드가 GUI 키보드 입력과 마이크 STT 입력을 동일한 채팅 흐름으로 처리하도록 구성했다.
- STT 변환(마이크 열기, 주변소음 보정, Google STT background listen) 코드를 `util/stt/speech_to_text.py`로 분리했다.
- 새 STT 유틸 패키지 `util/stt/`와 `__init__.py`를 추가했다.
- 명령 해석은 기존 `util/input_mapping/command_hard_parser.py`와 `util/input_mapping/command_llm_parser.py`를 그대로 재사용하도록 유지했다.
- `macgyvbot/nodes/voice_command_gui_node.py`를 제거했다.
- `setup.py`에서 voice command GUI 별도 엔트리포인트를 제거하고 `stt_node` 엔트리포인트만 남겼다.
- `launch/macgyvbot.launch.py`는 `stt_node`에 `use_gui=true`를 전달해 통합 GUI를 기본 실행하도록 수정했다.
- README의 노드 구조/실행 흐름을 통합 구조에 맞게 갱신했다.

확인:
- `python3 -m py_compile macgyvbot/nodes/stt_node.py macgyvbot/util/stt/speech_to_text.py macgyvbot/util/input_mapping/command_llm_parser.py macgyvbot/ui/voice_command_window.py`로 문법 검사를 통과했다.

## 20. STT 노드를 command input 노드로 이름 변경

- 통합 입력 노드 파일명을 `macgyvbot/nodes/stt_node.py`에서 `macgyvbot/nodes/command_input_node.py`로 변경했다.
- ROS 노드 클래스와 노드 이름을 `CommandInputNode`, `command_input_node`로 변경했다.
- `setup.py` console script를 `command_input_node = macgyvbot.nodes.command_input_node:main`으로 갱신했다.
- `launch/macgyvbot.launch.py`에서 실행 파일과 노드 이름을 `command_input_node`로 갱신했다.
- README의 노드 구조/실행 명령/흐름 이름을 새 이름에 맞게 수정했다.

확인:
- `python3 -m py_compile macgyvbot/nodes/command_input_node.py`로 문법 검사를 통과했다.

## 21. Python config를 YAML 설정으로 전환 (철회됨)

> 이 변경은 22번 항목에서 되돌렸다. 현재 설정은 `macgyvbot/config/config.py`를 유지한다.

- `macgyvbot/config/config.py` 값을 `config/config.yaml`로 옮겨 ROS install share에 함께 배포되도록 정리했다.
- `macgyvbot/util/runtime_config.py`를 추가해 YAML 값을 기존 상수 형태로 로드하도록 구성했다.
- 기존 `macgyvbot.config.config` import를 모두 `macgyvbot.util.runtime_config`로 변경했다.
- `macgyvbot/config/__init__.py`와 `macgyvbot/config/config.py`를 제거했다.
- YAML 로딩 의존성으로 `PyYAML`/`python3-yaml`을 추가했다.

확인:
- `runtime_config` import로 주요 값과 radians 변환을 확인했다.
- `python3 -m py_compile`로 변경된 Python 파일 문법 검사를 통과했다.

## 22. Config rollback 및 명령/상태 파이프라인 정리

- YAML 설정 전환을 되돌리고 `macgyvbot/config/config.py` Python 상수 구성을 유지하도록 복원했다.
- `macgyvbot/util/runtime_config.py`, `config/config.yaml`, YAML 의존성(`PyYAML`, `python3-yaml`)을 제거했다.
- `grasp_by_bbox_center.py`의 단일 bbox center 선택 함수를 `util/perception/grasp_point_selector.py` 내부 helper로 합치고 파일을 제거했다.
- `ui/debug_windows.py`의 작은 OpenCV window wrapper를 `macgyvbot_main_node.py` 내부 private helper로 합치고 파일을 제거했다.
- `command_input_node`는 `/tool_command`와 `/command_feedback`만 발행하도록 정리하고, `/target_label`은 수동 호환 입력 경로로 남겼다.
- `macgyvbot_main_node`가 `/tool_command`를 직접 구독해 `bring`, `release`, `stop` 명령을 처리하도록 연결했다.
- `macgyvbot_main_node`가 `/robot_task_status`를 발행해 GUI가 실제 로봇 실행 상태(`accepted`, `searching`, `picking`, `waiting_handoff`, `done`, `failed`, `busy`, `returned`, `cancelled`)를 받을 수 있게 했다.
- `PickSequenceRunner`가 주요 실패/완료/핸드오프 대기/원위치 반환 상태를 `/robot_task_status`로 보고하도록 연결했다.
- command parser와 hand grasp tool class 목록을 `drill`, `hammer`, `pliers`, `screwdriver`, `tape_measure`, `wrench` 기준으로 맞췄다.
- README의 구조도와 명령 흐름을 `/tool_command` 중심 파이프라인으로 갱신했다.

확인:
- `rg`로 삭제된 `runtime_config`, `debug_windows`, `grasp_by_bbox_center` 실행 참조가 남지 않았는지 확인했다.
- `python3 -m py_compile`로 변경된 Python 파일 문법 검사를 수행했다.
