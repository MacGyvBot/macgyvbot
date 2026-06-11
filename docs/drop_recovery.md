# Drop Recovery

이 문서는 gripper grasp 이후 공구 drop이 감지됐을 때 실행하는 1차 복구 흐름을 설명합니다.

## 범위

- 대상 패키지: `src/macgyvbot_task`
- 구현 위치: `macgyvbot_task.application.recovery`
- 진입점:
  - `run_pick_recovery()`
  - `run_return_recovery()`
- ROS node 연결: `task_coordinator_node.py`의 `/tool_drop_detected` callback

이번 버전은 interrupt/resume을 구현하지 않습니다. drop이 감지되면 남은 task queue를 비우고, recovery mode를 켠 뒤 복구 오케스트레이션을 수행합니다. 중단된 task를 재개하는 정책은 후속 작업에서 추가합니다.

## 재사용하는 기존 기능

Recovery 코드는 새 perception, grasp, drawer, motion 알고리즘을 만들지 않고 기존 구현을 호출합니다.

- drop detection: `macgyvbot_manipulation.tool_drop_monitor.ToolDropMonitor`
- drop status forwarding: `ToolDropStatusReporter`
- task queue clear/cancel: `TaskCoordinatorNode`의 queue와 MoveIt goal cancel 경로
- inspection pose: `macgyvbot_manipulation.handover_targeting.move_to_observation_pose`
- YOLO/depth target resolution: `PickTargetResolver.target_from_boxes`
- grasp planning: `PickTargetPlanner`
- grasp verification: `GraspVerifier`
- drawer open/close: `DrawerMotionFlow`
- return drawer placement: `ReturnDrawerPlacementFlow.place_tool_at_marker`
- home movement: `MoveItController.move_to_home_joints`
- flow-level logging: `macgyvbot_task.application.logging_utils`

`recovery_utils.py`는 위 기능을 thin wrapper로 묶습니다. 기존 인터페이스가 더 명확해지면 wrapper 내부 TODO 지점을 더 구체적인 adapter로 교체할 수 있습니다.

## 공통 흐름

```text
drop detected
-> cancel current MoveIt goal
-> clear remaining task queue
-> set recovery_mode = True
-> clear locked SAM/depth mask state
-> move to inspection pose
-> open gripper after inspection pose arrival
-> detect selected target tool
-> recompute yaw PCA from the current bbox/depth view
-> check graspability through existing planner
-> rotate wrist by fresh yaw PCA
-> align XY at drawer-wall clearance height
-> descend Z to grasp height
-> close gripper and verify grasp
-> align wrist with the return drawer-store posture before drawer observation
-> close drawer if needed
-> return home
-> set recovery_mode = False
```

실패 시에도 가능한 정리 절차를 수행합니다.

```text
log failure reason
-> close open drawer if known
-> return home
-> unset recovery mode
-> publish failed status
```

## Pick Recovery

Pick/delivery 중 공구를 떨어뜨린 경우 `run_pick_recovery()`가 호출됩니다.

대상 선택:

1. `status.target_tool`
2. `status.target_label`
3. `config.command["tool_name"]`
4. `"unknown"`

흐름:

1. 남은 queue를 비웁니다.
2. `status.recovery_mode = True`로 설정합니다.
3. 기존 hand inspection pose로 이동합니다.
4. 기존 YOLO/depth target resolver로 선택된 target을 찾습니다.
5. `PickTargetPlanner`로 grasp 가능성을 확인합니다.
6. `GraspVerifier` 기반 기존 gripper grasp 정책으로 grasp를 재시도합니다.
7. 성공하면 `status.held_tool`과 `status.gripper_holding`을 갱신합니다.
8. 열린 drawer가 있으면 닫고 home으로 복귀합니다.

이번 버전은 성공 후 원래 delivery task를 재개하지 않습니다. 후속 interrupt/resume 작업에서는 drop 직전 step과 queue snapshot을 저장해 재개 정책을 선택해야 합니다.

추가 pick recovery 세부 동작:

- pick target은 이전 `bring` 요청에서 저장된 `status.target_tool` 또는 `status.target_label`을 우선 사용합니다. 그래도 없을 때만 `config.command["tool_name"]`을 확인하고, 마지막 fallback만 `"unknown"`입니다.
- inspection pose에서는 먼저 bbox center/depth로 base 위치를 얻습니다.
- 그 base 위치 기준으로 기존 pick observe offset처럼 target 상대 위 관찰 자세로 이동합니다.
- target observe pose에서 YOLO grasp point를 우선 사용하고, 실패하면 기존처럼 bbox center fallback을 씁니다.
- target observe pose에서 기존 SAM/PCA yaw 계산 경로를 다시 호출해 `yaw_deg`를 반영합니다.
- `yaw_deg`가 있으면 기존 wrist yaw 회전 경로를 적용한 뒤 recovery grasp를 수행합니다.
- recovery grasp 성공 뒤에는 원래 공구 서랍 mapping을 찾아 기존 return drawer placement 방식으로 서랍 marker에 넣고 서랍을 닫습니다.
- recovery grasp 성공 뒤 drop monitor를 다시 켜므로, recovery mode 중 다시 drop이 감지되면 현재 recovery motion을 cancel하고 pending recovery로 저장한 다음 새 recovery를 다시 시작합니다.

## Return Recovery

Return 중 사용자에게 받은 공구 또는 staging된 공구를 떨어뜨린 경우 `run_return_recovery()`가 호출됩니다.

대상 선택:

1. `status.held_tool`
2. `status.target_tool`
3. `status.target_label`
4. `"unknown"`

흐름:

1. 남은 queue를 비웁니다.
2. `status.recovery_mode = True`로 설정합니다.
3. 기존 hand inspection pose로 이동합니다.
4. 기존 YOLO/depth target resolver로 선택된 target을 찾습니다.
5. `PickTargetPlanner`로 grasp 가능성을 확인합니다.
6. `DrawerMotionFlow.drawer_id_for_tool()`로 대상 서랍을 결정합니다.
7. 서랍을 열고 recovery grasp를 수행합니다.
8. 기존 `ReturnDrawerPlacementFlow.place_tool_at_marker()`로 drawer marker 위치에 배치합니다.
9. 서랍을 닫고 home으로 복귀합니다.
10. 성공하면 `held_tool`, `gripper_holding`, `drawer_open`, `opened_drawer_id`를 정리합니다.

`"unknown"` target은 기존 drawer mapping에 없으면 실패로 처리합니다. 최고 confidence detection을 임의 target으로 선택하지 않습니다.

추가 return recovery 세부 동작:

- inspection pose에서는 먼저 현재 target label 또는 supported fallback label로 공구 base 위치를 얻습니다.
- 그 base 위치 기준으로 기존 pick observe offset과 같은 target 관찰 자세로 이동합니다.
- target observe pose에서 실제로 보이는 supported tool label을 다시 확인합니다.
- 관찰된 label이 기존 target과 다르면 `status.held_tool`과 recovery target을 관찰 label로 업데이트합니다.
- 업데이트된 label 기준으로 원래 서랍 mapping을 찾고, 해당 서랍을 열어 marker target을 확보합니다.
- 해당 서랍이 이미 열린 상태라면 서랍 열기/marker 확보를 먼저 하러 가지 않고, 바로 공구 target observe/grasp 흐름으로 넘어갑니다. 이 경우 grasp 이후에 열린 서랍을 observe하고 marker target을 확보합니다.
- 서랍 marker를 확보한 뒤 다시 target observe pose로 돌아갑니다.
- target observe pose에서 YOLO grasp point를 우선 사용하고, 실패하면 기존 center fallback을 씁니다.
- 같은 view에서 SAM/PCA yaw를 다시 계산하고, `yaw_deg`가 있으면 wrist yaw를 적용한 뒤 grasp합니다.
- grasp 이후에는 기존 return drawer placement 방식으로 marker 위치에 넣고 서랍을 닫습니다.

## Return Tool Tracking

Return handoff에서는 gripper close 직전에 기존 `HumanGraspResult` payload를 이용해 최소 tracking fallback을 저장합니다.

```python
status.held_tool = detected_tool_name or fallback_tool_name or "unknown"
status.last_known_tool_bbox = result.get("tool_roi")
status.last_known_tool_confidence = result.get("tool_confidence")
```

새 tracking 시스템은 만들지 않았습니다. 후속 작업에서는 여러 frame의 `tool_label`과 `tool_confidence`를 누적해 가장 안정적인 tool을 선택할 수 있습니다.

## 주요 실패 사유

Recovery는 다음 실패를 별도 reason으로 기록합니다.

- `target_detection_failed`
- `graspability_check_failed`
- `motion_planning_failed`
- `grasp_execution_failed`
- `drawer_open_failed`
- `drawer_close_failed`
- `unknown_drawer_mapping`
- `drop_recovery_queue_shutdown_timeout`

모든 실패는 drawer close와 home 복귀를 가능한 범위에서 시도합니다.

## 나중에 추가할 작업

- drop 직전 queue snapshot 저장
- 중단된 `TaskStep` 재개 정책
- return recovery 중 drawer open 순서 세분화
- recovery 전용 retry limit parameter화
- recovery timeout의 ROS parameter화
- `"unknown"` 공구에 대한 operator confirmation 또는 staging fallback

## 현재 제한

이번 first-version recovery는 공구가 바닥이나 작업대처럼 비교적 평평한 표면 위에 떨어져 있고, YOLO bbox와 depth projection으로 안전한 pick target을 만들 수 있는 경우만 대상으로 합니다.

Inspection pose에서 공구를 찾더라도 target observe pose 또는 grasp pose planning이 실패하면, recovery는 공구를 억지로 잡으려 하지 않습니다. `GRASPABILITY_CHECK_FAILED` 로그로 "잡을 수 없음"을 남기고, 열린 서랍 close와 home 복귀를 시도한 뒤 실패로 종료합니다.

## 추후 작업

- Interrupt/resume 방식: drop 직전 `TaskStep`, queue, drawer state, held tool, marker target을 snapshot으로 저장하고 recovery 성공 뒤 재개할지 종료할지 선택합니다.
- 어려운 recovery case: 공구가 열린 서랍 턱에 걸쳐 있는 경우, 서랍 안쪽에 떨어진 경우, 서랍 벽/손잡이와 충돌 가능성이 높은 경우를 별도 recovery policy로 분리합니다.
- 서랍 내부 recovery: drawer collision scene과 marker/depth를 함께 사용해 서랍 내부 공구를 안전하게 관찰하고, 서랍 벽 clearance를 보장하는 접근 pose를 추가합니다.
- 충돌 위험 판단: 공구 bbox/depth가 drawer boundary, handle, wall clearance 영역과 겹치는지 확인하고, 위험하면 operator confirmation 또는 manual recovery 상태로 전환합니다.
- Graspability 강화: 현재는 기존 planner 중심의 1차 check이며, 추후에는 VLM/SAM mask, force, depth quality, reachable workspace를 함께 사용해 "잡을 수 있음/없음"을 더 명확히 판단합니다.
