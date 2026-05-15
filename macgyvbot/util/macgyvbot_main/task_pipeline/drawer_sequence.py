"""Drawer perception and motion helpers used by task pipelines."""

from __future__ import annotations

from dataclasses import dataclass
import time

import rclpy

from macgyvbot.config.config import (
    DRAWER_APPROACH_Z_OFFSET,
    DRAWER_DETECTION_TIMEOUT_SEC,
    DRAWER_LABEL,
    DRAWER_HANDLE_APPROACH_Z_OFFSET,
    DRAWER_HANDLE_GRASP_Z_OFFSET,
    DRAWER_HANDLE_LABEL,
    DRAWER_HANDLE_OFFSET_X,
    DRAWER_HANDLE_OFFSET_Y,
    DRAWER_HANDLE_OFFSET_Z,
    DRAWER_OPEN_DIRECTION_X,
    DRAWER_PULL_DISTANCE_M,
    DRAWER_TOOL_PLACE_APPROACH_Z_OFFSET,
    DRAWER_TOOL_PLACE_Z_OFFSET,
    SAFE_Z,
    SEQUENCE_WAIT_POLL_SEC,
)
from macgyvbot.util.macgyvbot_main.model_control.robot_pose import (
    get_ee_matrix,
    make_safe_pose,
)
from macgyvbot.util.macgyvbot_main.model_control.robot_safezone import SAFE_Z_MIN


@dataclass(frozen=True)
class DetectedTarget:
    """Projected YOLO target in robot base coordinates."""

    label: str
    x: float
    y: float
    z: float
    depth_m: float
    pixel_u: int | None = None
    pixel_v: int | None = None
    source: str = "center"
    yaw_deg: float | None = None


@dataclass(frozen=True)
class DrawerHandleMotion:
    """Motion waypoints for closing a drawer that was already opened."""

    closed_x: float
    open_x: float
    y: float
    travel_z: float
    approach_z: float
    grasp_z: float
    ori: dict


class DrawerInteraction:
    """Shared drawer detection, opening, closing, and placement motions."""

    def __init__(self, robot, motion_controller, gripper, state, wait_fn=None):
        self.robot = robot
        self.motion = motion_controller
        self.gripper = gripper
        self.state = state
        self.wait = wait_fn or cooperative_wait

    def wait_for_target(
        self,
        label,
        logger,
        timeout_sec=DRAWER_DETECTION_TIMEOUT_SEC,
        use_grasp_selector=False,
    ):
        start_time = time.monotonic()

        while rclpy.ok():
            target = self.detect_target(
                label,
                logger,
                use_grasp_selector=use_grasp_selector,
            )
            if target is not None:
                return target

            if time.monotonic() - start_time >= timeout_sec:
                logger.warn(
                    f"{timeout_sec:.1f}초 동안 YOLO에서 '{label}'을 찾지 못했습니다."
                )
                return None

            self.wait(0.2)

        return None

    def detect_target(self, label, logger, use_grasp_selector=False):
        if not self._has_camera_state():
            logger.warn("카메라 상태가 준비되지 않아 YOLO 탐지를 수행할 수 없습니다.")
            return None

        detector = self._detector_for_label(label)
        results = detector.detect(self.state.color_image)
        best_box = self._best_box_for_label(results[0].boxes, label, detector.names)
        if best_box is None:
            return None

        u, v, source, vlm_rpy_deg = self._select_pixel(
            best_box,
            label,
            use_grasp_selector,
        )
        target = self.state.depth_projector.pixel_to_base_target(
            u,
            v,
            label,
            "center",
            self.state.depth_image,
            self.state.intrinsics,
            logger,
            vlm_rpy_deg,
        )
        if target is None:
            return None

        bx, by, bz, z_m, _ = target
        return DetectedTarget(
            label=label,
            x=bx,
            y=by,
            z=bz,
            depth_m=z_m,
            pixel_u=u,
            pixel_v=v,
            source=source,
            yaw_deg=self._extract_yaw(vlm_rpy_deg, logger),
        )

    def handle_target_from_drawer_offset(self, drawer_target, logger):
        handle_target = DetectedTarget(
            label=DRAWER_HANDLE_LABEL,
            x=drawer_target.x + DRAWER_HANDLE_OFFSET_X,
            y=drawer_target.y + DRAWER_HANDLE_OFFSET_Y,
            z=drawer_target.z + DRAWER_HANDLE_OFFSET_Z,
            depth_m=drawer_target.depth_m,
            pixel_u=drawer_target.pixel_u,
            pixel_v=drawer_target.pixel_v,
            source="drawer_offset_fallback",
        )
        logger.info(
            "서랍 손잡이 offset fallback 적용: "
            f"drawer=({drawer_target.x:.3f}, {drawer_target.y:.3f}, {drawer_target.z:.3f}) "
            f"offset=({DRAWER_HANDLE_OFFSET_X:.3f}, {DRAWER_HANDLE_OFFSET_Y:.3f}, "
            f"{DRAWER_HANDLE_OFFSET_Z:.3f}) "
            f"handle=({handle_target.x:.3f}, {handle_target.y:.3f}, "
            f"{handle_target.z:.3f})"
        )
        return handle_target

    def move_to_drawer_view(self, drawer_target, logger):
        travel_z = self._travel_z(drawer_target.z)
        current_pose = get_ee_matrix(self.robot)
        current_x = current_pose[0, 3]
        current_y = current_pose[1, 3]
        ori = self.state.home_ori

        logger.info(
            "공구함 접근 1단계: 현재 위치에서 안전 높이 확보 "
            f"z={travel_z:.3f}"
        )
        if not self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(current_x, current_y, travel_z, ori, logger),
        ):
            logger.error("공구함 접근 안전 높이 확보 실패")
            return False

        logger.info(
            "공구함 접근 2단계: 공구함 상단으로 이동 "
            f"x={drawer_target.x:.3f}, y={drawer_target.y:.3f}, z={travel_z:.3f}"
        )
        return self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(
                drawer_target.x,
                drawer_target.y,
                travel_z,
                ori,
                logger,
            ),
        )

    def open_drawer(self, handle_target, logger):
        motion = self._handle_motion(handle_target)

        logger.info(
            "서랍 열기 1단계: 손잡이 상단 접근 "
            f"x={motion.closed_x:.3f}, y={motion.y:.3f}, z={motion.approach_z:.3f}"
        )
        self.gripper.open_gripper()
        self.wait(0.5)
        if not self._move_handle_pose(
            motion.closed_x,
            motion.y,
            motion.approach_z,
            motion.ori,
            logger,
        ):
            logger.error("서랍 손잡이 상단 접근 실패")
            return None

        logger.info("서랍 열기 2단계: 손잡이 파지 높이로 하강")
        if not self._move_handle_pose(
            motion.closed_x,
            motion.y,
            motion.grasp_z,
            motion.ori,
            logger,
        ):
            logger.error("서랍 손잡이 하강 실패")
            return None

        logger.info("서랍 열기 3단계: 손잡이 파지")
        self.gripper.close_gripper()
        self.wait(0.8)

        logger.info(
            "서랍 열기 4단계: 손잡이를 당겨 서랍 열기 "
            f"x={motion.closed_x:.3f}->{motion.open_x:.3f}"
        )
        if not self._move_handle_pose(
            motion.open_x,
            motion.y,
            motion.grasp_z,
            motion.ori,
            logger,
        ):
            logger.error("서랍 손잡이 당기기 실패")
            return None

        logger.info("서랍 열기 5단계: 열린 위치에서 안전 높이로 복귀")
        if not self._move_handle_pose(
            motion.open_x,
            motion.y,
            motion.travel_z,
            motion.ori,
            logger,
        ):
            logger.error("서랍 열린 위치 안전 높이 복귀 실패")
            return None

        self.gripper.open_gripper()
        self.wait(0.5)
        return motion

    def close_drawer(self, motion, logger):
        logger.info("서랍 닫기 1단계: 열린 손잡이 상단 접근")
        self.gripper.open_gripper()
        self.wait(0.5)
        if not self._move_handle_pose(
            motion.open_x,
            motion.y,
            motion.approach_z,
            motion.ori,
            logger,
        ):
            logger.error("열린 손잡이 상단 접근 실패")
            return False

        logger.info("서랍 닫기 2단계: 손잡이 파지 높이로 하강")
        if not self._move_handle_pose(
            motion.open_x,
            motion.y,
            motion.grasp_z,
            motion.ori,
            logger,
        ):
            logger.error("열린 손잡이 하강 실패")
            return False

        logger.info("서랍 닫기 3단계: 손잡이 파지")
        self.gripper.close_gripper()
        self.wait(0.8)

        logger.info(
            "서랍 닫기 4단계: 손잡이를 밀어 서랍 닫기 "
            f"x={motion.open_x:.3f}->{motion.closed_x:.3f}"
        )
        if not self._move_handle_pose(
            motion.closed_x,
            motion.y,
            motion.grasp_z,
            motion.ori,
            logger,
        ):
            logger.error("서랍 닫기 밀기 실패")
            return False

        logger.info("서랍 닫기 5단계: 손잡이를 놓고 안전 높이로 복귀")
        self.gripper.open_gripper()
        self.wait(0.5)
        return self._move_handle_pose(
            motion.closed_x,
            motion.y,
            motion.travel_z,
            motion.ori,
            logger,
        )

    def place_tool_in_open_drawer(self, drawer_target, logger):
        ori = self.state.home_ori
        place_x = drawer_target.x + (
            DRAWER_OPEN_DIRECTION_X * DRAWER_PULL_DISTANCE_M * 0.5
        )
        place_y = drawer_target.y
        place_z = max(drawer_target.z + DRAWER_TOOL_PLACE_Z_OFFSET, SAFE_Z_MIN)
        approach_z = max(
            place_z + DRAWER_TOOL_PLACE_APPROACH_Z_OFFSET,
            SAFE_Z,
            self.state.home_xyz[2],
        )

        logger.info(
            "서랍 보관 1단계: 열린 서랍 내부 상단으로 이동 "
            f"x={place_x:.3f}, y={place_y:.3f}, z={approach_z:.3f}"
        )
        if not self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(place_x, place_y, approach_z, ori, logger),
        ):
            logger.error("열린 서랍 내부 상단 이동 실패")
            return False

        logger.info("서랍 보관 2단계: 공구 배치 높이로 하강")
        if not self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(place_x, place_y, place_z, ori, logger),
        ):
            logger.error("열린 서랍 내부 하강 실패")
            return False

        logger.info("서랍 보관 3단계: 공구 놓기")
        self.gripper.open_gripper()
        self.wait(0.8)

        logger.info("서랍 보관 4단계: 공구를 놓은 뒤 안전 높이로 복귀")
        return self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(place_x, place_y, approach_z, ori, logger),
        )

    def _handle_motion(self, handle_target):
        travel_z = self._travel_z(handle_target.z)
        approach_z = max(
            handle_target.z + DRAWER_HANDLE_APPROACH_Z_OFFSET,
            SAFE_Z_MIN,
        )
        grasp_z = max(handle_target.z + DRAWER_HANDLE_GRASP_Z_OFFSET, SAFE_Z_MIN)
        if approach_z < grasp_z:
            approach_z = grasp_z

        open_x = handle_target.x + DRAWER_OPEN_DIRECTION_X * DRAWER_PULL_DISTANCE_M
        return DrawerHandleMotion(
            closed_x=handle_target.x,
            open_x=open_x,
            y=handle_target.y,
            travel_z=travel_z,
            approach_z=approach_z,
            grasp_z=grasp_z,
            ori=self.state.home_ori,
        )

    def _move_handle_pose(self, x, y, z, ori, logger):
        return self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(x, y, z, ori, logger),
        )

    def _travel_z(self, target_z):
        return max(
            target_z + DRAWER_APPROACH_Z_OFFSET,
            self.state.home_xyz[2],
            SAFE_Z,
        )

    def _has_camera_state(self):
        return (
            self.state.color_image is not None
            and self.state.depth_image is not None
            and self.state.intrinsics is not None
        )

    def _detector_for_label(self, label):
        if label in (DRAWER_LABEL, DRAWER_HANDLE_LABEL):
            return getattr(self.state, "drawer_detector", self.state.detector)
        return self.state.detector

    def _best_box_for_label(self, boxes, label, names):
        best_box = None
        best_conf = -1.0

        for box in boxes:
            detected_label = names[int(box.cls)]
            if detected_label != label:
                continue

            confidence = self._box_confidence(box)
            if confidence > best_conf:
                best_conf = confidence
                best_box = box

        return best_box

    @staticmethod
    def _box_center_pixel(box):
        bbox = box.xyxy[0].cpu().numpy()
        u = int((bbox[0] + bbox[2]) / 2)
        v = int((bbox[1] + bbox[3]) / 2)
        return u, v

    def _select_pixel(self, box, label, use_grasp_selector):
        selector = getattr(self.state, "grasp_point_selector", None)
        if not use_grasp_selector or selector is None:
            u, v = self._box_center_pixel(box)
            return u, v, "center", None

        return selector.select(
            box,
            label,
            self.state.color_image,
            self.state.depth_image,
            self.state.intrinsics,
            label,
        )

    @staticmethod
    def _extract_yaw(vlm_rpy_deg, logger):
        if vlm_rpy_deg is None or len(vlm_rpy_deg) < 3:
            return None

        try:
            return float(vlm_rpy_deg[2])
        except (TypeError, ValueError):
            logger.warn(f"VLM yaw 파싱 실패: {vlm_rpy_deg}")
            return None

    @staticmethod
    def _box_confidence(box):
        try:
            return float(box.conf[0].item())
        except (AttributeError, IndexError, TypeError, ValueError):
            try:
                return float(box.conf[0])
            except (AttributeError, IndexError, TypeError, ValueError):
                return 0.0


def cooperative_wait(duration_sec):
    end_time = time.monotonic() + max(0.0, float(duration_sec))
    while rclpy.ok() and time.monotonic() < end_time:
        remaining = end_time - time.monotonic()
        time.sleep(min(SEQUENCE_WAIT_POLL_SEC, max(0.0, remaining)))
