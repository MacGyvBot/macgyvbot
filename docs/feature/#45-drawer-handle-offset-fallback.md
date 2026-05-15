# #45 Drawer Handle Offset Fallback

## 목적

`feature/#45-drawer` 브랜치는 원래 `drawer`를 탐지한 뒤 `drawer_handle`을 YOLO로 다시 탐지해 서랍을 여는 구조다.

현재는 `drawer_handle` YOLO 모델이 아직 준비되지 않았기 때문에, 임시로 `drawer`의 base 좌표에 고정 offset을 더해 손잡이 좌표를 만든다. 이 문서는 오늘 테스트용 fallback이 어디에 들어갔고, 나중에 `drawer_handle` 탐지가 준비되면 어떻게 되돌릴지 설명한다.

## 변경 요약

### 설정값

파일: `macgyvbot/config/config.py`

추가된 값:

```python
USE_DRAWER_HANDLE_OFFSET_FALLBACK = True
DRAWER_HANDLE_OFFSET_X = 0.0
DRAWER_HANDLE_OFFSET_Y = 0.0
DRAWER_HANDLE_OFFSET_Z = -0.08
```

- `USE_DRAWER_HANDLE_OFFSET_FALLBACK=True`이면 `drawer_handle` YOLO 탐지를 기다리지 않는다.
- `drawer` 검출 좌표에 `DRAWER_HANDLE_OFFSET_*`을 더해 임시 손잡이 target을 만든다.
- offset은 camera pixel 기준이 아니라 robot `base_link` 기준 좌표다.

### fallback helper

파일: `macgyvbot/util/macgyvbot_main/task_pipeline/drawer_sequence.py`

추가된 함수:

```python
handle_target_from_drawer_offset(drawer_target, logger)
```

역할:

```text
DetectedTarget(label="drawer", x, y, z)
-> offset 적용
-> DetectedTarget(label="drawer_handle", source="drawer_offset_fallback")
```

로그에는 아래 정보가 출력된다.

```text
drawer=(...)
offset=(...)
handle=(...)
```

실제 로봇에서 offset을 튜닝할 때 이 로그를 기준으로 `DRAWER_HANDLE_OFFSET_X/Y/Z`를 조정하면 된다.

### pick/return sequence 연결

파일:

- `macgyvbot/util/macgyvbot_main/task_pipeline/pick_sequence.py`
- `macgyvbot/util/macgyvbot_main/task_pipeline/return_sequence.py`

기존 구조:

```text
drawer 탐지
-> drawer 위치로 이동
-> drawer_handle YOLO 탐지
-> open_drawer(handle_target)
```

현재 fallback 구조:

```text
drawer 탐지
-> drawer 위치로 이동
-> USE_DRAWER_HANDLE_OFFSET_FALLBACK=True이면 drawer 기준 offset handle 사용
-> USE_DRAWER_HANDLE_OFFSET_FALLBACK=False이면 기존 drawer_handle YOLO 탐지
-> open_drawer(handle_target)
```

즉 기존 `drawer_handle` YOLO 경로는 삭제하지 않았다. 나중에 perception이 준비되면 설정값만 끄면 원래 경로를 탄다.

## 나중에 drawer_handle YOLO를 붙이는 방법

현재 YOLO 모델은 공구용과 서랍용을 분리해서 사용한다.

- `yolo_model`: 기존 공구 탐지 모델
- `drawer_yolo_model`: 서랍/손잡이 탐지 모델

1. drawer YOLO weight에 아래 class가 포함되어 있는지 확인한다.

```text
drawer
drawer_handle
```

2. class 이름이 코드의 설정과 정확히 같아야 한다.

```python
DRAWER_LABEL = "drawer"
DRAWER_HANDLE_LABEL = "drawer_handle"
```

3. 모델 경로를 launch argument로 넘긴다.

```bash
ros2 launch macgyvbot macgyvbot.launch.py \
  yolo_model:=/path/to/tool_model.pt \
  drawer_yolo_model:=/path/to/drawer_model.pt
```

4. `macgyvbot/config/config.py`에서 fallback을 끈다.

```python
USE_DRAWER_HANDLE_OFFSET_FALLBACK = False
```

5. 실행 로그에서 `drawer_handle` depth projection이 나오는지 확인한다.

정상이라면 fallback 로그인 `서랍 손잡이 offset fallback 적용`은 더 이상 나오지 않고, `drawer_handle` 검출 로그가 나와야 한다.

## 오늘 테스트할 때 볼 것

1. `drawer`가 YOLO로 잡히는지 확인한다.
2. 로그의 `drawer=(x, y, z)`와 `handle=(x, y, z)`를 비교한다.
3. 손잡이보다 너무 높거나 낮으면 `DRAWER_HANDLE_OFFSET_Z`를 먼저 조정한다.
4. 좌우가 틀어지면 `DRAWER_HANDLE_OFFSET_Y`를 조정한다.
5. 서랍 앞뒤 방향이 틀어지면 `DRAWER_HANDLE_OFFSET_X`를 조정한다.
6. `DRAWER_OPEN_DIRECTION_X`와 `DRAWER_PULL_DISTANCE_M`이 실제 서랍 열림 방향/거리와 맞는지 확인한다.

초기값은 실험용이다.

```python
DRAWER_HANDLE_OFFSET_X = 0.0
DRAWER_HANDLE_OFFSET_Y = 0.0
DRAWER_HANDLE_OFFSET_Z = -0.08
```

## 주의

- 이 fallback은 손잡이를 실제로 인식하지 않는다.
- 서랍 위치가 바뀌거나 손잡이가 drawer bbox 중심에서 많이 벗어나면 offset을 다시 맞춰야 한다.
- 나중에 `drawer_handle` YOLO가 안정화되면 fallback은 끄는 것이 맞다.
- TTS 관련 파일과 launch 파라미터는 #45 브랜치에서 삭제되어 있었으나 현재 작업에서 복구했다. drawer 작업 중 TTS 파일을 다시 삭제하지 않도록 주의한다.
