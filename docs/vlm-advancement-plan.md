# VLM 고도화 계획

이 문서는 MacGyvBot의 grasp point selection에서 VLM을 더 안정적으로 쓰기 위한
단계별 작업 계획이다.

## 목표

- YOLO bbox center 기반으로 먼저 x, y 위치를 맞춘 뒤 VLM을 개입시켜, VLM 입력 시점의 시야와 depth 품질을 높인다.
- VLM은 객체 상단에서 실제로 잡을 point와 gripper yaw를 선택하는 역할로 제한한다.
- 최종 z 이동은 VLM이 아니라 depth로 확인한 metric depth를 기준으로 수행한다.
- 큰 VLM 모델, SAM mask, VLM 호출 타이밍 조정, prompt/tuning을 통해 grasp point 품질을 개선한다.
- launch 시작 시 VLM 가중치를 미리 로드해 첫 pick 동작에서 발생하는 지연을 줄인다.

## 변경하려는 알고리즘

기존 방식:

```text
YOLO detection
-> 탐지된 frame의 bbox crop을 VLM에 바로 전달
-> VLM grasp point/yaw 추론
-> depth로 grasp point의 z 확인
-> pick sequence 시작
```

변경하려는 방식:

```text
YOLO detection
-> bbox center pixel을 depth로 base x, y, z 후보로 변환
-> robot이 bbox center 기준 x, y로 먼저 이동
-> 객체 상단 관찰 frame을 다시 확보
-> VLM이 상단 view에서 잡을 point와 gripper yaw 선택
-> 선택 point의 z를 depth로 확인
-> gripper yaw 조절
-> depth로 확인한 z까지 하강
-> gripper close
```

핵심 원칙:

- [x] x, y 정렬은 YOLO bbox center 기반으로 먼저 수행한다.
- [x] VLM은 초기 탐지 frame이 아니라 center 이동 이후의 상단 관찰 frame을 사용한다.
- [x] VLM은 grasp point와 gripper angle 판단에 집중한다.
- [x] z는 VLM 추론값이 아니라 RealSense depth projection 결과를 사용한다.
- [x] VLM 실패 시 기존 bbox center 기반 grasp fallback을 유지한다.

## 1단계: 현재 VLM 동작 기준선 정리

- [ ] 현재 `grasp_point_mode:=vlm` 실행 흐름을 코드 기준으로 정리한다.
- [ ] YOLO bbox 선택부터 VLM 호출, depth 보정, bbox center fallback까지의 호출 순서를 문서화한다.
- [ ] VLM 추론 시간, 첫 로드 시간, depth 보정 실패 횟수, bbox center fallback 발생 조건을 로그로 확인한다.
- [ ] 현재 기본 모델과 weight 경로를 정리한다.
- [ ] 실패 사례 이미지를 수집한다.
  - [ ] tool이 bbox 안에 작게 잡힌 경우
  - [ ] depth hole이 있는 경우
  - [ ] VLM grasp point가 tool 바깥으로 나간 경우
  - [ ] yaw가 부정확한 경우

완료 기준:

- [ ] VLM 개선 전 baseline 로그와 실패 케이스가 정리되어 있다.

## 2단계: VLM 개입 타이밍 수정

목표 흐름:

```text
YOLO detection
-> bbox center 기반 x,y 이동
-> 상단 관찰 frame 갱신
-> VLM grasp point/yaw 추론
-> VLM point의 depth 확인
-> gripper yaw 조절
-> depth z까지 하강 후 grasp
```

TODO:

- [x] pick flow에서 VLM 호출 위치를 현재보다 뒤로 미룰 수 있는 지점을 찾는다.
- [x] YOLO bbox center pixel을 depth projection으로 base 좌표 후보로 변환한다.
- [x] bbox center 기반 x, y 이동 단계를 pick sequence 앞단에 추가한다.
- [x] x, y 이동 후 frame을 다시 받아 YOLO bbox, color image, depth image를 갱신한다.
- [x] VLM 입력은 이동 이후 갱신된 상단 관찰 frame만 사용하도록 제어한다.
- [x] VLM이 선택한 point의 depth를 다시 확인해 최종 z를 계산한다.
- [x] VLM yaw 결과를 gripper wrist 회전에 연결한다.
- [x] x, y 이동 또는 VLM 추론 실패 시 기존 bbox center grasp 또는 기존 fallback 흐름으로 복귀한다.
- [ ] 보정 이동 거리 제한을 둔다.
- [ ] 작업 공간 safezone을 벗어나는 보정 목표는 폐기한다.
- [ ] 각 단계별 상태 로그를 추가한다.

완료 기준:

- [ ] VLM이 로봇 시야 보정 이후의 최신 frame으로 호출된다.
- [ ] YOLO center 보정 실패 시 전체 pick flow가 멈추지 않고 fallback한다.

## 3단계: VLM 성능 개선

### 3.1 큰 VLM 모델 적용

- [ ] 후보 모델 목록을 정리한다.
- [ ] GPU/VRAM 요구사항을 확인한다.
- [ ] 현재 `weights/vlm/` 구조에 맞는 모델 저장 규칙을 정한다.
- [ ] launch argument로 VLM 모델 경로 또는 모델 이름을 바꿀 수 있게 한다.
- [ ] 작은 모델과 큰 모델의 첫 로드 시간, 평균 추론 시간, grasp 성공률을 비교한다.
- [ ] 큰 모델 사용 시 timeout 값을 별도로 설정할 수 있게 한다.

### 3.2 SAM과 VLM 결합

- [ ] YOLO bbox를 SAM prompt로 사용해 tool mask를 얻는 흐름을 정리한다.
- [ ] VLM 입력 prompt에 mask 기반 제약을 추가할지 검토한다.
- [ ] VLM 결과 point가 SAM mask 내부인지 검증한다.
- [ ] VLM point가 mask 밖이면 mask 내부의 가장 가까운 valid point로 보정한다.
- [ ] mask 내부 depth 후보만 사용하도록 depth refinement를 제한할지 검토한다.
- [ ] SAM 실패 시 기존 bbox 기반 VLM 흐름으로 fallback한다.

### 3.3 VLM prompt 및 출력 튜닝

- [ ] 현재 prompt와 JSON schema를 정리한다.
- [ ] tool별 grasp rule을 prompt에 넣을지 검토한다.
- [ ] 긴 tool, 손잡이 있는 tool, 금속 반사 tool 등 케이스별 지시문을 분리한다.
- [ ] yaw 출력 범위와 gripper 접근 방향 제약을 명확히 한다.
- [ ] VLM 응답 파싱 실패 로그를 남긴다.
- [ ] 잘못된 응답이 오면 재시도할지, 즉시 fallback할지 기준을 정한다.

### 3.4 평가

- [ ] 동일 scene에서 center mode와 VLM mode를 비교한다.
- [ ] 작은 VLM, 큰 VLM, SAM+VLM 결과를 비교한다.
- [ ] tool별 grasp point 오차를 기록한다.
- [ ] VLM 추론 시간과 전체 pick 시간 증가량을 기록한다.
- [ ] 실제 robot grasp 성공률을 기록한다.

완료 기준:

- [ ] 개선 전후의 VLM grasp point 품질과 추론 시간이 비교되어 있다.
- [ ] 실패 시 fallback이 유지된다.

## 4단계: Launch 시작 시 VLM 가중치 사전 로드

- [x] 현재 VLM 객체가 언제 생성되고 weight가 언제 로드되는지 확인한다.
- [x] `grasp_point_mode:=vlm` 또는 `api` fallback용 VLM이 필요한 경우 launch 시작 시 로드되게 한다.
- [x] `macgyvbot_main_node` 초기화 중 `GraspPointSelector.preload_vlm_if_needed()`를 호출한다.
- [x] `VLMGraspPointSelector.preload()`에서 기존 `_ensure_model_loaded()` 경로를 재사용한다.
- [x] VLM preload 실패 시 launch를 중단하지 않고 warning만 남긴다.
- [ ] 로드 완료 전 pick 명령이 들어오면 대기하거나 명확한 상태를 발행한다.
- [ ] GUI/TTS에 VLM loading 상태를 표시할지 검토한다.
- [x] VLM 로드 실패 시 `center` fallback으로 계속 실행할지, launch 실패로 처리할지 정책을 정한다.
- [ ] 모델 로드 timeout과 에러 메시지를 추가한다.
- [ ] 첫 pick에서 VLM cold start 지연이 사라졌는지 측정한다.

완료 기준:

- [ ] launch 이후 첫 VLM pick에서 모델 로드 지연이 발생하지 않는다.
- [x] VLM preload 실패 시 warning log를 남기고 노드는 계속 실행된다.
- [ ] VLM 로드 실패 시 사용자와 로그가 원인을 충분히 알 수 있다.

## 5단계: 통합 테스트

- [x] `python3 -m compileall -q src`
- [ ] VLM weight가 없는 환경에서 launch fallback 확인
- [ ] VLM weight가 있는 환경에서 launch preload 확인
- [ ] RealSense 입력으로 YOLO bbox center 기반 x, y 이동 확인
- [ ] center 이동 이후의 최신 frame으로 VLM이 호출되는지 확인
- [ ] VLM 추론 결과가 최신 frame 기준인지 확인
- [ ] SAM enabled/disabled 양쪽에서 pick flow 확인
- [ ] `grasp_point_mode:=center` 기존 동작 회귀 확인
- [ ] `grasp_point_mode:=api` 실패 시 VLM fallback 확인
- [ ] 실제 로봇 저속 pick 테스트

## 우선순위

1. [ ] 현재 흐름과 baseline 정리
2. [x] launch 시작 시 VLM preload
3. [x] VLM 호출 타이밍을 bbox center x, y 이동 이후로 이동
4. [ ] SAM mask 기반 VLM 결과 검증
5. [ ] 큰 VLM 모델 비교
6. [ ] prompt/tuning 및 평가 자동화

## 주의사항

- [ ] VLM 개선 중에도 bbox center fallback은 유지한다.
- [ ] VLM/SAM/ML weight는 Git에 포함하지 않는다.
- [ ] 실제 로봇 테스트는 낮은 속도와 충분한 작업 공간에서 수행한다.
- [ ] VLM 성능 개선이 전체 pick latency를 과도하게 늘리지 않는지 함께 확인한다.
- [ ] API mode fallback 경로와 local VLM 경로가 서로 충돌하지 않도록 한다.
