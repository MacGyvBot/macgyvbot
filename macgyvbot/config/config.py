"""Shared MacGyvBot runtime configuration."""

import math

# MoveIt planning group 이름. dsr_moveit_config의 SRDF 그룹명과 맞아야 한다.
GROUP_NAME = "manipulator"

# 로봇 좌표계 기준 frame. 카메라/depth로 얻은 3D 좌표는 이 frame으로 변환해 planning에 사용한다.
BASE_FRAME = "base_link"

# 엔드이펙터 link 이름. 현재 EE pose 조회와 pose goal의 pose_link로 사용한다.
EE_LINK = "link_6"

# 기본 Home joint pose. 작업 시작 전 Home 저장/복귀 기준 자세로 사용한다.
# 단위는 radian이며, 사람이 읽기 쉽게 degree 값을 math.radians()로 변환한다.
HOME_JOINTS = {
    "joint_1": math.radians(0.0),
    "joint_2": math.radians(0.0),
    "joint_3": math.radians(90.0),
    "joint_4": math.radians(0.0),
    "joint_5": math.radians(90.0),
    "joint_6": math.radians(90.0),
}

# VLM이 예측한 in-plane yaw를 손목 회전으로 반영할 때 조작하는 joint 이름.
WRIST_JOINT_NAME = "joint_6"

# Pick/복귀 중 XY 이동에 사용하는 기본 안전 이동 높이.
# 물체를 집은 뒤 들어 올리거나 Home으로 이동할 때 최소 이 높이를 확보한다.
SAFE_Z = 0.40

# Pick 시 물체 바로 위 approach pose 높이.
# approach_z = detected_object_z + APPROACH_Z_OFFSET 로 계산된다.
APPROACH_Z_OFFSET = 0.18

# Pick 시 실제 파지 pose를 물체 검출 z보다 얼마나 낮게 잡을지 결정하는 보정값.
# grasp_z = corrected_object_z - GRASP_Z_OFFSET 로 계산되며,
# 최종적으로 robot_safezone.SAFE_Z_MIN 아래로는 내려가지 않는다.
GRASP_Z_OFFSET = 0.03

# 바닥 기준 물체 높이 측정값이 캘리브레이션 기준보다 높게 나오는 편차.
# 현재 depth/base 변환 결과가 실제보다 0.03m 높으므로 pick z 계산 전에 이 값을 뺀다.
OBJECT_Z_HEIGHT_BIAS_M = 0.03

# 메인 YOLO 객체 검출 모델 파일명. 패키지 share 또는 weights 경로에서 로드된다.
YOLO_MODEL_NAME = "yolov11_best.pt"

# 서랍 YOLO 객체 검출 모델 파일명. 패키지 share 또는 weights 경로에서 로드된다.
DRAWER_YOLO_MODEL_NAME = "yolov11n_drawer.pt"
DRAWER_LABEL = "drawer"
DRAWER_HANDLE_LABEL = "drawer_handle"

# 사용자 명령 입력 노드가 로봇 메인 노드로 공구/동작 명령을 보내는 topic.
TOOL_COMMAND_TOPIC = "/tool_command"

# 로봇 작업 상태를 GUI/상위 노드에 알리는 topic.
ROBOT_STATUS_TOPIC = "/robot_task_status"

# 손-공구 grasp 인식 결과(JSON String)를 publish/subscribe 하는 topic.
HAND_GRASP_TOPIC = "/human_grasped_tool"

# 손-공구 grasp 인식 시각화 이미지를 publish/subscribe 하는 topic.
HAND_GRASP_IMAGE_TOPIC = "/hand_grasp_detection/annotated_image"

# handoff 단계에서 사용자가 공구를 잡았는지 기다리는 최대 시간.
HAND_GRASP_TIMEOUT_SEC = 20.0

# 손 위치 탐색 전체 제한 시간. 별도 search helper의 기본 timeout으로 사용한다.
HANDOVER_SEARCH_TIMEOUT_SEC = 45.0

# pick 후 관찰 자세에서 안정적인 손 위치 후보를 기다리는 시간 제한.
OBSERVATION_TIMEOUT_SEC = 20.0

# 손 위치가 우연히 1프레임만 잡힌 좌표로 이동하지 않도록 요구하는 안정 유지 시간.
# 카메라가 30fps로 들어올 때 약 30프레임에 해당한다.
# 이 시간 동안 새 프레임의 손 위치가 허용 오차 안에 계속 들어와야 handoff 목표로 확정한다.
HAND_POSE_WAIT_AFTER_DETECTION_SEC = 1.0

# 손 위치 안정 판정 허용 오차. x/y/z 중 하나라도 이 범위를 벗어나면 안정화 timer를 다시 시작한다.
HAND_POSE_STABLE_TOLERANCE_M = 0.03

# 손 3D 좌표가 없을 때 탐색 vertex 좌표로 handoff를 진행할지 여부.
# False면 실제 손 좌표가 없을 때 handoff를 실패 처리한다.
ALLOW_SEARCH_VERTEX_FALLBACK_FOR_HANDOVER = False

# 손 위치로 전달하러 갈 때 x 방향으로 적용하는 보정값.
# 양/음 방향은 BASE_FRAME 기준이며, 현재는 보정 없음.
HANDOVER_HAND_X_OFFSET_M = -0.00

# 손 위치로 전달하러 갈 때 손 z보다 위로 띄우는 여유 높이.
# 실제 handoff 목표 z는 max(SAFE_Z_MIN + 0.15, hand_z + 이 값)이다.
HANDOVER_HAND_Z_OFFSET_M = 0.08

# handoff 목표 pose planning이 실패할 때 재시도할 총 횟수.
# 첫 시도 포함 값이며, 실패할수록 x는 줄이고 y는 0에 가깝게 보정한다.
HANDOVER_REPLAN_MAX_ATTEMPTS = 10

# handoff 재플래닝에서 매 시도마다 x를 줄이는 간격.
HANDOVER_REPLAN_X_STEP_M = 0.03

# 로봇 그리퍼가 물체를 제대로 잡았는지 확인하는 최대 시간.
GRASP_VERIFY_TIMEOUT_SEC = 3.0

# 그리퍼 상태를 polling하는 주기.
GRASP_VERIFY_POLL_SEC = 0.2

# grasp 성공 상태가 연속으로 몇 번 확인되어야 실제 성공으로 볼지 결정한다.
GRASP_VERIFY_STABLE_COUNT = 3

# 그리퍼 grasp 실패 시 전진/재시도하는 최대 횟수.
GRASP_RETRY_LIMIT = 5

# grasp 재시도 시 물체 방향으로 전진하는 거리.
GRASP_ADVANCE_DISTANCE_M = 0.20

# 사용자 손을 관찰하기 위한 고정 joint pose.
# pick 이후 이 자세로 이동해서 손 위치 인식이 안정될 때까지 대기한다.
OBSERVATION_JOINTS = {
    "joint_1": math.radians(0.0),
    "joint_2": math.radians(-40.0),
    "joint_3": math.radians(55.0),
    "joint_4": math.radians(0.0),
    "joint_5": math.radians(120.0),
    "joint_6": math.radians(90.0),
}

# 그리퍼가 거의 완전히 닫힌 상태면 물체를 못 잡은 것으로 보기 위한 폭 기준.
GRIPPER_CLOSED_WIDTH_THRESHOLD_MM = 5.0

# 반납 시 Z 하강 중 반력을 읽는 force/torque sensor topic.
FORCE_TORQUE_TOPIC = "/force_torque_sensor_broadcaster/wrench"

# 반납 Z 하강 시 한 번에 낮추는 거리.
RETURN_HOME_DESCENT_STEP_M = 0.01

# 반납 Z 하강 중 이 힘 이상이 감지되면 바닥/접촉으로 판단하고 하강을 멈춘다.
RETURN_HOME_FORCE_THRESHOLD_N = 8.0

# 시퀀스 내부 cooperative wait polling 주기.
# rclpy 상태를 자주 확인하면서 sleep하기 위한 짧은 대기 간격이다.
SEQUENCE_WAIT_POLL_SEC = 0.03

# OpenCV로 표시하는 메인 pick 카메라 창 이름.
ROBOT_WINDOW_NAME = "YOLO Robot Pick"

# OpenCV로 표시하는 손 grasp 인식 시각화 창 이름.
HAND_GRASP_WINDOW_NAME = "Hand Grasp Detection"
DRAWER_DETECTION_TIMEOUT_SEC = 8.0
DRAWER_APPROACH_Z_OFFSET = 0.18
DRAWER_HANDLE_APPROACH_Z_OFFSET = 0.08
DRAWER_HANDLE_GRASP_Z_OFFSET = 0.02
USE_DRAWER_HANDLE_OFFSET_FALLBACK = True
DRAWER_HANDLE_OFFSET_X = 0.0
DRAWER_HANDLE_OFFSET_Y = 0.0
DRAWER_HANDLE_OFFSET_Z = -0.08
DRAWER_PULL_DISTANCE_M = 0.18
DRAWER_OPEN_DIRECTION_X = 1.0
DRAWER_TOOL_PLACE_APPROACH_Z_OFFSET = 0.16
DRAWER_TOOL_PLACE_Z_OFFSET = 0.04

# bbox 중심을 grasp point로 쓰는 단순 모드.
GRASP_POINT_MODE_CENTER = "center"

# VLM이 이미지에서 grasp point/yaw를 추론하는 모드.
GRASP_POINT_MODE_VLM = "vlm"

# launch parameter가 없을 때 사용할 기본 grasp point 선택 방식.
DEFAULT_GRASP_POINT_MODE = GRASP_POINT_MODE_CENTER

# VLM grasp 후보를 고를 때 이미지를 나누는 grid 크기 후보들.
# 작은 grid는 거친 위치 선택, 큰 grid는 더 세밀한 위치 선택에 사용된다.
VLM_GRASP_GRID_SIZES = ((3, 3), (4, 4))
