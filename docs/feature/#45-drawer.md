# feature/#45-drawer

이 브랜치는 기존의 바닥/테이블 위 공구 pick/return 흐름을 서랍 기반 흐름으로 바꾼다.

## 전제

- YOLO는 향후 `drawer`, `drawer_handle`, 그리고 기존 공구 label을 검출한다.
- 현재는 `drawer`와 `drawer_handle` 학습이 완료되지 않았으므로, 해당 객체와 depth 좌표가 정상 검출된다고 가정한다.
- `drawer`와 `drawer_handle`은 bbox 중심 픽셀을 depth로 base 좌표에 투영한다.
- 열린 서랍 내부 공구는 기존처럼 `GraspPointSelector`를 사용한다. 따라서 `grasp_point_mode=vlm`이면 공구 grasp point는 VLM 경로를 계속 탈 수 있다.
- 서랍을 여는 방향은 base frame 기준 +X 방향으로 가정한다. 거리와 방향은 `config.py`의 `DRAWER_PULL_DISTANCE_M`, `DRAWER_OPEN_DIRECTION_X`에서 조정한다.
- 단일 그리퍼 제약 때문에 return sequence는 서랍을 먼저 연 뒤 사용자에게서 반납 공구를 받는다.

## Pick Sequence

1. `main_node.py`가 `bring` 명령을 받으면 공구 자체가 아니라 `drawer`를 먼저 찾는다.
2. `drawer`가 보이면 `moving_to_drawer` 상태를 발행하고, drawer 좌표를 `PickSequenceRunner`로 넘긴다.
3. `PickSequenceRunner`는 공구함 상단으로 이동한 뒤 YOLO에서 `drawer_handle`을 찾는다.
4. 손잡이를 파지하고 +X 방향으로 `DRAWER_PULL_DISTANCE_M`만큼 당겨 서랍을 연다.
5. 열린 서랍 안에서 사용자가 요청한 공구 label을 YOLO로 찾는다.
6. 공구 좌표가 확인되면 기존 pick 흐름처럼 접근, grasp, lift, 사용자 전달 위치 이동을 수행한다.
7. 사용자가 공구를 잡은 것이 확인되면 그리퍼를 열어 전달한다.
8. 빈 그리퍼로 다시 서랍 손잡이 위치에 접근해 서랍 문을 닫는다.
9. 서랍을 닫은 뒤 Home 위치로 복귀하고 `done` 상태를 발행한다.

실패 처리:

- 서랍은 열렸지만 공구를 찾지 못했거나, 공구 grasp 전 단계에서 실패하면 가능한 경우 서랍을 닫고 실패 상태를 발행한다.
- 공구를 이미 잡은 뒤 lift/handoff 이동에 실패한 경우에는 공구를 들고 있을 수 있으므로 자동 서랍 닫기는 시도하지 않는다.
- 사용자 handoff timeout이 발생하면 공구를 열린 서랍의 원래 위치로 되돌린 뒤 서랍을 닫는다.

## Return Sequence

1. `return` 명령을 받으면 `ReturnSequenceRunner`가 `drawer`를 찾는다.
2. 공구함으로 이동한 뒤 YOLO에서 `drawer_handle`을 찾고 서랍을 연다.
3. 열린 서랍을 유지한 상태로 사용자 반납 감지 위치(Home 기준 전방 20cm)로 이동한다.
4. hand-tool grasp 결과로 사용자가 들고 있는 공구를 확인한다.
5. 로봇 그리퍼가 반납 공구를 grasp한다.
6. 열린 서랍 내부 위치로 이동해 공구를 내려놓고 그리퍼를 연다.
7. 빈 그리퍼로 서랍 손잡이에 접근해 서랍 문을 닫는다.
8. Home 위치로 복귀하고 `done` 상태를 발행한다.

실패 처리:

- 로봇이 공구를 잡기 전 실패하면 가능한 경우 열린 서랍을 닫는다.
- 로봇이 반납 공구를 잡은 뒤 서랍 내부 배치에 실패하면 공구를 들고 있을 수 있으므로 서랍 닫기보다 실패 상태를 우선한다.

## 상태 값

- `searching_drawer`: 공구함 찾기
- `moving_to_drawer`: 공구함으로 이동 중
- `searching_drawer_handle`: 서랍 손잡이 찾기
- `opening_drawer`: 서랍 열기
- `searching`: 열린 서랍 내부에서 요청 공구 찾기
- `waiting_handoff`: pick sequence에서 사용자 전달 대기
- `waiting_return_handoff`: return sequence에서 사용자 반납 공구 수신 위치 이동/대기
- `placing_return_tool`: 반납 공구를 열린 서랍 내부에 배치
- `closing_drawer`: 서랍 닫기

## 실제 테스트 시 조정할 값과 확인 지점

아래 항목은 코드가 실행되더라도 실제 로봇/서랍 환경에서 테스트하며 맞춰야 한다.

### YOLO와 Depth

- YOLO class 이름이 `drawer`, `drawer_handle`, 기존 공구 label과 정확히 일치하는지 확인한다.
- `drawer` bbox 중심 픽셀의 depth가 안정적으로 나오는지 확인한다.
- `drawer_handle` bbox 중심 픽셀이 실제 손잡이 중앙 또는 그리퍼가 잡기 좋은 위치인지 확인한다.
- 손잡이 중심 픽셀의 depth가 0으로 자주 나오면 bbox 중심 대신 손잡이 내부의 다른 픽셀을 쓰도록 보정이 필요하다.
- 열린 서랍 안의 공구가 YOLO에서 충분히 보이는지 확인한다.
- 열린 서랍 안 공구의 depth가 주변 서랍 바닥이나 벽 depth와 섞이지 않는지 확인한다.

### 서랍 열기/닫기 Motion

- `DRAWER_OPEN_DIRECTION_X`가 실제 서랍이 열리는 base frame 방향과 맞는지 확인한다.
- `DRAWER_PULL_DISTANCE_M`가 서랍을 충분히 열지만 과하게 당기지 않는 거리인지 확인한다.
- `DRAWER_HANDLE_APPROACH_Z_OFFSET`가 손잡이 상단 접근 높이로 충분한지 확인한다.
- `DRAWER_HANDLE_GRASP_Z_OFFSET`가 손잡이를 실제로 잡을 수 있는 높이인지 확인한다.
- `DRAWER_APPROACH_Z_OFFSET`가 서랍 주변 장애물과 충돌하지 않는 안전 이동 높이인지 확인한다.
- 손잡이를 잡은 뒤 당길 때 gripper가 미끄러지지 않는지 확인한다.
- 손잡이를 닫는 방향으로 밀 때 서랍이 끝까지 닫히는지 확인한다.
- 서랍 닫기 후 gripper를 열고 안전 높이로 올라갈 때 손잡이에 걸리지 않는지 확인한다.

### 공구 Pick

- 열린 서랍 안 공구 좌표가 실제 공구 위치와 맞는지 확인한다.
- `GRASP_Z_OFFSET`, `APPROACH_Z_OFFSET`, `COLLISION_MARGIN` 조합이 서랍 내부에서 충돌 없이 접근 가능한지 확인한다.
- `grasp_point_mode=vlm`을 사용할 경우 VLM이 서랍 내부 공구의 잡기 좋은 영역을 고르는지 확인한다.
- gripper가 공구를 잡은 뒤 lift할 때 서랍 앞판, 손잡이, 서랍 내부 벽에 닿지 않는지 확인한다.
- 사용자가 공구를 잡은 뒤 서랍을 닫으러 돌아가는 경로가 사람과 충돌하지 않는지 확인한다.

### 공구 Return

- return sequence에서 서랍을 먼저 열고 사용자 반납 위치로 이동하는 순서가 실제 사용 흐름에 맞는지 확인한다.
- `GRASP_ADVANCE_DISTANCE_M`가 사용자가 공구를 건네기 좋은 거리인지 확인한다.
- hand-tool grasp detection이 사용자가 들고 있는 반납 공구를 안정적으로 인식하는지 확인한다.
- 로봇이 사용자에게서 공구를 grasp할 때 gripper 폭과 힘이 적절한지 확인한다.
- `DRAWER_TOOL_PLACE_APPROACH_Z_OFFSET`가 열린 서랍 내부 상단 접근 높이로 충분한지 확인한다.
- `DRAWER_TOOL_PLACE_Z_OFFSET`가 공구를 서랍 안에 놓기 좋은 높이인지 확인한다.
- 현재 return placement는 `drawer_target.x + DRAWER_OPEN_DIRECTION_X * DRAWER_PULL_DISTANCE_M * 0.5` 위치에 놓는 가정이다. 실제 서랍 깊이와 내부 공간에 맞게 조정한다.
- 공구를 놓은 뒤 gripper가 열릴 때 공구가 서랍 밖으로 밀리지 않는지 확인한다.
- 공구를 놓고 안전 높이로 복귀할 때 공구나 서랍에 걸리지 않는지 확인한다.

### 안전과 실패 처리

- 서랍 열기 중 실패했을 때 로봇이 손잡이 근처에서 멈춰도 안전한지 확인한다.
- 공구를 이미 잡은 뒤 실패하면 자동으로 서랍을 닫지 않는 경로가 맞는지 확인한다.
- 사용자 handoff timeout이 발생했을 때 공구를 원래 위치로 되돌리는 동작이 서랍 내부에서 안전한지 확인한다.
- MoveIt planning 실패가 잦은 좌표가 있으면 safe zone, approach 높이, drawer offset을 조정한다.
- 실제 테스트 초반에는 속도 scaling을 낮게 유지하고, 서랍/손잡이 주변 emergency stop 접근성을 확보한다.

## 정상 로그 출력 순서

아래 순서는 실패 없이 정상 동작했을 때의 대표적인 출력 흐름이다. `tool_name`, `raw_text`, 좌표, depth, score 등은 실제 실행 시점의 값으로 치환된다. MoveIt 내부 planning 로그와 그리퍼 드라이버의 `Start opening gripper.`, `Start closing gripper.` 출력은 각 motion과 gripper 명령 사이에 추가로 섞일 수 있다.

### Bring / Pick

1. `타겟 객체 설정: tool_name (/tool_command)`
2. 상태 발행: `searching_drawer`
3. 상태 메시지: `tool_name를 꺼낼 공구함 찾기를 시작합니다.`
4. `drawer` depth projection 로그: `'drawer' 검출: source=drawer_center, pixel=(drawer_pixel_u, drawer_pixel_v), camera=(drawer_camera_x, drawer_camera_y, drawer_depth_m), base=(drawer_base_x, drawer_base_y, drawer_base_z)`
5. 상태 발행: `moving_to_drawer`
6. 상태 메시지: `공구함으로 이동 중입니다.`
7. `공구함 기준 pick 시퀀스 시작: tool=tool_name, drawer=(drawer_base_x, drawer_base_y, drawer_base_z)`
8. 상태 발행: `moving_to_drawer`
9. 상태 메시지: `공구함으로 이동 중입니다.`
10. `공구함 접근 1단계: 현재 위치에서 안전 높이 확보 z=drawer_travel_z`
11. `공구함 접근 2단계: 공구함 상단으로 이동 x=drawer_base_x, y=drawer_base_y, z=drawer_travel_z`
12. 상태 발행: `searching_drawer_handle`
13. 상태 메시지: `서랍 손잡이를 찾는 중입니다.`
14. `drawer_handle` depth projection 로그: `'drawer_handle' 검출: source=center, pixel=(handle_pixel_u, handle_pixel_v), camera=(handle_camera_x, handle_camera_y, handle_depth_m), base=(handle_base_x, handle_base_y, handle_base_z)`
15. 상태 발행: `opening_drawer`
16. 상태 메시지: `서랍 손잡이를 당겨 여는 중입니다.`
17. `서랍 열기 1단계: 손잡이 상단 접근 x=handle_closed_x, y=handle_y, z=handle_approach_z`
18. `서랍 열기 2단계: 손잡이 파지 높이로 하강`
19. `서랍 열기 3단계: 손잡이 파지`
20. `서랍 열기 4단계: 손잡이를 당겨 서랍 열기 x=handle_closed_x->handle_open_x`
21. `서랍 열기 5단계: 열린 위치에서 안전 높이로 복귀`
22. 상태 발행: `searching`
23. 상태 메시지: `열린 서랍 안에서 tool_name 탐색 중입니다.`
24. `tool_name` depth projection 로그: `'tool_name' 검출: source=tool_grasp_source, pixel=(tool_pixel_u, tool_pixel_v), camera=(tool_camera_x, tool_camera_y, tool_depth_m), base=(tool_base_x, tool_base_y, tool_base_z)`
25. `열린 서랍 내부 공구 pick 시작: Target(tool_target_x, tool_target_y), depth=tool_depth_m, travel_z=tool_travel_z, approach_z=tool_approach_z, grasp_z=tool_grasp_z`
26. `1단계: 안전 이동 높이 확보`
27. `2단계: 안전 높이에서 XY 수평 이동`
28. VLM yaw가 있을 때만 출력: `2-1단계: VLM yaw를 J6에 적용 (yaw_offset=tool_yaw_degdeg, j6=current_j6_degdeg -> target_j6_degdeg)`
29. `3단계: 타겟 상단 접근`
30. `4단계: 파지 높이 하강`
31. `5단계: 공구 grasp 시도`
32. 상태 발행: `grasping`
33. 상태 메시지: `공구 grasp 시도 1/5`
34. `그리퍼 grasp 시도 1/5`
35. `그리퍼 grip detected 신호와 폭으로 grasp 성공을 확인했습니다: width=gripper_width_mm`
36. 상태 발행: `grasp_success`
37. 상태 메시지: `공구 grasp에 성공했습니다.`
38. `6단계: 안전 높이 복귀`
39. `7단계: 사용자 전달 위치 이동 Home 기준 전방 20cm x=home_x->handoff_x, y=handoff_y, z=handoff_z`
40. `8단계: 사용자 잡기 인식 대기`
41. 상태 발행: `waiting_handoff`
42. 상태 메시지: `사용자 잡기 인식을 기다립니다.`
43. `사용자가 공구를 잡은 것으로 확인됨`
44. `9단계: 사용자 잡기 확인 후 그리퍼 오픈(놓기)`
45. 상태 발행: `closing_drawer`
46. 상태 메시지: `서랍 문을 닫는 중입니다.`
47. `서랍 닫기 1단계: 열린 손잡이 상단 접근`
48. `서랍 닫기 2단계: 손잡이 파지 높이로 하강`
49. `서랍 닫기 3단계: 손잡이 파지`
50. `서랍 닫기 4단계: 손잡이를 밀어 서랍 닫기 x=handle_open_x->handle_closed_x`
51. `서랍 닫기 5단계: 손잡이를 놓고 안전 높이로 복귀`
52. `10단계: 서랍 닫은 후 Home 위치로 복귀`
53. `Pick 시퀀스 완료`
54. 상태 발행: `done`
55. 상태 메시지: `공구 전달, 서랍 닫기, Home 복귀까지 완료되었습니다.`

참고: `4단계`에서 `approach_z`와 `grasp_z`가 같으면 30번 대신 `4단계: approach_z와 grasp_z가 같아 추가 하강 생략`이 출력된다.

### Return

1. `반납 시퀀스 시작: tool=tool_name`
2. 상태 발행: `searching_drawer`
3. 상태 메시지: `반납 공구를 넣을 공구함 찾기 중입니다.`
4. `반납 명령 수신: tool=tool_name, raw_text='raw_text'. 서랍을 먼저 연 뒤 사용자 hand-tool grasp 인식을 기다립니다.`
5. `drawer` depth projection 로그: `'drawer' 검출: source=center, pixel=(drawer_pixel_u, drawer_pixel_v), camera=(drawer_camera_x, drawer_camera_y, drawer_depth_m), base=(drawer_base_x, drawer_base_y, drawer_base_z)`
6. 상태 발행: `moving_to_drawer`
7. 상태 메시지: `공구함으로 이동 중입니다.`
8. `공구함 접근 1단계: 현재 위치에서 안전 높이 확보 z=drawer_travel_z`
9. `공구함 접근 2단계: 공구함 상단으로 이동 x=drawer_base_x, y=drawer_base_y, z=drawer_travel_z`
10. 상태 발행: `searching_drawer_handle`
11. 상태 메시지: `서랍 손잡이를 찾는 중입니다.`
12. `drawer_handle` depth projection 로그: `'drawer_handle' 검출: source=center, pixel=(handle_pixel_u, handle_pixel_v), camera=(handle_camera_x, handle_camera_y, handle_depth_m), base=(handle_base_x, handle_base_y, handle_base_z)`
13. 상태 발행: `opening_drawer`
14. 상태 메시지: `서랍 손잡이를 당겨 여는 중입니다.`
15. `서랍 열기 1단계: 손잡이 상단 접근 x=handle_closed_x, y=handle_y, z=handle_approach_z`
16. `서랍 열기 2단계: 손잡이 파지 높이로 하강`
17. `서랍 열기 3단계: 손잡이 파지`
18. `서랍 열기 4단계: 손잡이를 당겨 서랍 열기 x=handle_closed_x->handle_open_x`
19. `서랍 열기 5단계: 열린 위치에서 안전 높이로 복귀`
20. 상태 발행: `waiting_return_handoff`
21. 상태 메시지: `사용자 반납 공구를 받을 위치로 이동합니다.`
22. 상태 발행: `moving_return_grasp_pose`
23. 상태 메시지: `반납 공구를 감지하기 위해 전방 20cm 위치로 이동합니다.`
24. `반납 1단계: 공구 감지 전 전방 20cm 전진 x=home_x->return_detection_x, y=return_detection_y, z=return_detection_z`
25. `사용자 hand-tool grasp 확인: requested_tool=tool_name, detected_tool=detected_tool_label, state=hand_grasp_state, score=hand_grasp_score`
26. `사용자 hand-tool grasp 확인. 공구 grasp를 시도합니다.`
27. 상태 발행: `grasping`
28. 상태 메시지: `반납 공구 grasp 시도 1/5`
29. `반납 공구 grasp 시도 1/5`
30. `그리퍼 grip detected 신호와 폭으로 grasp 성공을 확인했습니다: width=gripper_width_mm`
31. 상태 발행: `grasp_success`
32. 상태 메시지: `반납 공구 grasp에 성공했습니다.`
33. 상태 발행: `placing_return_tool`
34. 상태 메시지: `tool_name 반납 공구를 열린 서랍 안에 넣는 중입니다.`
35. `서랍 보관 1단계: 열린 서랍 내부 상단으로 이동 x=drawer_place_x, y=drawer_place_y, z=drawer_place_approach_z`
36. `서랍 보관 2단계: 공구 배치 높이로 하강`
37. `서랍 보관 3단계: 공구 놓기`
38. `서랍 보관 4단계: 공구를 놓은 뒤 안전 높이로 복귀`
39. 상태 발행: `closing_drawer`
40. 상태 메시지: `공구 보관 후 서랍 문을 닫는 중입니다.`
41. `서랍 닫기 1단계: 열린 손잡이 상단 접근`
42. `서랍 닫기 2단계: 손잡이 파지 높이로 하강`
43. `서랍 닫기 3단계: 손잡이 파지`
44. `서랍 닫기 4단계: 손잡이를 밀어 서랍 닫기 x=handle_open_x->handle_closed_x`
45. `서랍 닫기 5단계: 손잡이를 놓고 안전 높이로 복귀`
46. `반납 마무리: 서랍을 닫은 뒤 Home으로 복귀`
47. 상태 발행: `done`
48. 상태 메시지: `tool_name 반납 공구를 서랍에 넣고 서랍을 닫았습니다.`

### Drawer Sequence

`drawer_sequence.py`는 `pick_sequence.py`와 `return_sequence.py`에서 공통으로 호출하는 drawer helper이다. 아래 순서는 각 helper method가 정상적으로 호출됐을 때 method 내부에서 출력하는 로그 순서이다. YOLO 탐지 자체는 `wait_for_target()`과 `detect_target()`에서 수행되며, 좌표 변환 성공 시 `DepthProjector.pixel_to_base_target()`의 검출 로그가 출력된다.

#### `wait_for_target(label, logger, timeout_sec, use_grasp_selector)`

1. 정상 검출된 경우 `wait_for_target()` 자체의 성공 로그는 출력하지 않는다.
2. `detect_target()`이 label에 해당하는 YOLO box를 찾는다.
3. depth projection 성공 시 검출 로그가 출력된다: `'label' 검출: source=target_source, pixel=(target_pixel_u, target_pixel_v), camera=(target_camera_x, target_camera_y, target_depth_m), base=(target_base_x, target_base_y, target_base_z)`
4. `DetectedTarget(label=label, x=target_base_x, y=target_base_y, z=target_base_z, depth_m=target_depth_m, pixel_u=target_pixel_u, pixel_v=target_pixel_v, source=target_source, yaw_deg=target_yaw_deg)`가 반환된다.

#### `move_to_drawer_view(drawer_target, logger)`

1. `공구함 접근 1단계: 현재 위치에서 안전 높이 확보 z=drawer_travel_z`
2. MoveIt plan and execute가 현재 end-effector x/y에서 `drawer_travel_z`로 이동한다.
3. `공구함 접근 2단계: 공구함 상단으로 이동 x=drawer_base_x, y=drawer_base_y, z=drawer_travel_z`
4. MoveIt plan and execute가 `drawer_base_x`, `drawer_base_y`, `drawer_travel_z`로 이동한다.
5. 성공하면 `True`를 반환한다.

#### `open_drawer(handle_target, logger)`

1. `서랍 열기 1단계: 손잡이 상단 접근 x=handle_closed_x, y=handle_y, z=handle_approach_z`
2. 그리퍼 open 명령이 실행된다.
3. MoveIt plan and execute가 `handle_closed_x`, `handle_y`, `handle_approach_z`로 이동한다.
4. `서랍 열기 2단계: 손잡이 파지 높이로 하강`
5. MoveIt plan and execute가 `handle_closed_x`, `handle_y`, `handle_grasp_z`로 이동한다.
6. `서랍 열기 3단계: 손잡이 파지`
7. 그리퍼 close 명령이 실행된다.
8. `서랍 열기 4단계: 손잡이를 당겨 서랍 열기 x=handle_closed_x->handle_open_x`
9. MoveIt plan and execute가 `handle_open_x`, `handle_y`, `handle_grasp_z`로 이동한다.
10. `서랍 열기 5단계: 열린 위치에서 안전 높이로 복귀`
11. MoveIt plan and execute가 `handle_open_x`, `handle_y`, `handle_travel_z`로 이동한다.
12. 그리퍼 open 명령이 실행된다.
13. 성공하면 `DrawerHandleMotion(closed_x=handle_closed_x, open_x=handle_open_x, y=handle_y, travel_z=handle_travel_z, approach_z=handle_approach_z, grasp_z=handle_grasp_z, ori=home_orientation)`을 반환한다.

#### `close_drawer(motion, logger)`

1. `서랍 닫기 1단계: 열린 손잡이 상단 접근`
2. 그리퍼 open 명령이 실행된다.
3. MoveIt plan and execute가 `handle_open_x`, `handle_y`, `handle_approach_z`로 이동한다.
4. `서랍 닫기 2단계: 손잡이 파지 높이로 하강`
5. MoveIt plan and execute가 `handle_open_x`, `handle_y`, `handle_grasp_z`로 이동한다.
6. `서랍 닫기 3단계: 손잡이 파지`
7. 그리퍼 close 명령이 실행된다.
8. `서랍 닫기 4단계: 손잡이를 밀어 서랍 닫기 x=handle_open_x->handle_closed_x`
9. MoveIt plan and execute가 `handle_closed_x`, `handle_y`, `handle_grasp_z`로 이동한다.
10. `서랍 닫기 5단계: 손잡이를 놓고 안전 높이로 복귀`
11. 그리퍼 open 명령이 실행된다.
12. MoveIt plan and execute가 `handle_closed_x`, `handle_y`, `handle_travel_z`로 이동한다.
13. 성공하면 `True`를 반환한다.

#### `place_tool_in_open_drawer(drawer_target, logger)`

1. `서랍 보관 1단계: 열린 서랍 내부 상단으로 이동 x=drawer_place_x, y=drawer_place_y, z=drawer_place_approach_z`
2. MoveIt plan and execute가 `drawer_place_x`, `drawer_place_y`, `drawer_place_approach_z`로 이동한다.
3. `서랍 보관 2단계: 공구 배치 높이로 하강`
4. MoveIt plan and execute가 `drawer_place_x`, `drawer_place_y`, `drawer_place_z`로 이동한다.
5. `서랍 보관 3단계: 공구 놓기`
6. 그리퍼 open 명령이 실행된다.
7. `서랍 보관 4단계: 공구를 놓은 뒤 안전 높이로 복귀`
8. MoveIt plan and execute가 `drawer_place_x`, `drawer_place_y`, `drawer_place_approach_z`로 이동한다.
9. 성공하면 `True`를 반환한다.

## 관련 파일

- `macgyvbot/config/config.py`: drawer label, handle label, 서랍 열림 거리/방향, placement offset 설정
- `macgyvbot/nodes/macgyvbot_main_node.py`: bring 명령 시 drawer 탐색과 상태 발행
- `macgyvbot/nodes/command_input_node.py`: drawer 관련 진행 상태를 GUI 상태 패널에 반영
- `macgyvbot/util/macgyvbot_main/task_pipeline/drawer_sequence.py`: drawer/handle 탐지, 열기, 닫기, 열린 서랍 내부 배치 helper
- `macgyvbot/util/macgyvbot_main/task_pipeline/pick_sequence.py`: drawer 기반 pick 및 handoff 이후 서랍 닫기
- `macgyvbot/util/macgyvbot_main/task_pipeline/return_sequence.py`: drawer 기반 return 보관 흐름
