"""Move a staged return tool into its drawer marker target."""

from __future__ import annotations

import time

from macgyvbot_config.grasp import (
    GRASP_VERIFY_POLL_SEC,
    PREGRASP_MEASUREMENT_SETTLE_TIMEOUT_SEC,
)
from macgyvbot_config.drawer import (
    DRAWER_STORE_MARKER_APPROACH_Z_OFFSET_M,
    DRAWER_STORE_MARKER_RELEASE_Z_OFFSET_M,
)
from macgyvbot_config.return_flow import RETURN_TOOL_RELEASE_WAIT_SEC
from macgyvbot_manipulation.grasp_verifier import GraspVerifier
from macgyvbot_manipulation.robot_pose import (
    current_ee_orientation,
    get_ee_matrix,
    make_safe_pose,
)
from macgyvbot_manipulation.robot_safezone import safe_z_min_for_drawer
from macgyvbot_task.application.drawer_store_motion import (
    drawer_wall_clearance_z_for_drawer,
    move_to_drawer_store_exit,
    rotate_wrist_for_drawer_store,
)
from macgyvbot_task.application.pick_flow.pick_grasp_flow import (
    calculate_pregrasp_extra_descent,
)
from macgyvbot_task.application.pick_flow.pick_target_planner import PickTargetPlanner


class ReturnDrawerPlacementFlow:
    """Grasp a staged tool and place it at the drawer ArUco marker center."""

    def __init__(
        self,
        robot,
        motion_controller,
        gripper,
        state,
        reporter,
        wait_fn,
        tool_hold_monitor=None,
        interrupted=None,
    ):
        self.robot = robot
        self.motion = motion_controller
        self.gripper = gripper
        self.state = state
        self.reporter = reporter
        self.wait_fn = wait_fn
        self.tool_hold_monitor = tool_hold_monitor
        self.interrupted = interrupted or (lambda: False)
        self.target_planner = PickTargetPlanner(robot)
        self.grasp_verifier = GraspVerifier(
            gripper,
            wait_fn,
            interrupted=self.interrupted,
        )

    def grasp_staged_tool(self, target, tool_name, command, logger):
        if target is None or not target.found or target.base_xyz is None:
            self.reporter.fail(
                tool_name,
                "임시 관찰 위치에서 공구 bbox 중심을 찾지 못했습니다.",
                "return_store_tool_bbox_not_found",
                command,
                logger,
            )
            return False

        bx, by, bz = target.base_xyz
        plan = self.target_planner.plan(bx, by, bz, logger)

        if not self._apply_grasp_yaw(target, tool_name, command, logger):
            return False

        ori = current_ee_orientation(self.robot)

        self.reporter.publish(
            "grasping_return_tool",
            tool_name,
            f"{tool_name} 임시 위치 공구를 다시 파지합니다.",
            command,
        )
        if not self._move_to_pose(
            plan.target_x,
            plan.target_y,
            plan.grasp_z,
            ori,
            logger,
            tool_name,
            command,
            "return_store_tool_grasp_failed",
            "임시 공구 파지 하강에 실패했습니다.",
        ):
            return False

        if not self._pregrasp_depth_adjust(plan, ori, tool_name, command, logger):
            return False

        if not self.grasp_verifier.try_grasp(
            logger,
            failure_prefix="임시 공구 grasp",
        ):
            if self.interrupted():
                return False
            self.reporter.fail(
                tool_name,
                "임시 위치 공구 파지에 실패했습니다.",
                "return_store_tool_grasp_failed",
                command,
                logger,
            )
            return False

        if self.tool_hold_monitor is not None:
            self.tool_hold_monitor.start(tool_name, "return", command)

        return True

    def _apply_grasp_yaw(self, target, tool_name, command, logger):
        yaw_deg = getattr(target, "yaw_deg", None)
        if yaw_deg is None:
            return True

        rotate_wrist = getattr(self.motion, "rotate_wrist_by_yaw_deg", None)
        if rotate_wrist is None:
            self.reporter.fail(
                tool_name,
                "Return staged tool grasp yaw cannot be applied.",
                "return_store_tool_grasp_yaw_unavailable",
                command,
                logger,
            )
            return False

        ok = rotate_wrist(
            yaw_deg,
            logger,
            collision_scene_key="return/store_tool_grasp_yaw",
        )
        if ok:
            logger.info(
                "Return staged tool grasp yaw applied: "
                f"tool={tool_name}, yaw_deg={yaw_deg}"
            )
            return True

        if self.interrupted():
            return False

        self.reporter.fail(
            tool_name,
            "Return staged tool grasp yaw rotation failed.",
            "return_store_tool_grasp_yaw_failed",
            command,
            logger,
        )
        return False

    def _pregrasp_depth_adjust(self, plan, ori, tool_name, command, logger):
        logger.info("임시 공구 pre-grasp depth 측정")
        measurement = self._measure_pregrasp_depth(logger)
        if measurement is None:
            self.reporter.fail(
                tool_name,
                "임시 공구 pre-grasp depth 측정에 실패했습니다.",
                "return_store_tool_pregrasp_depth_unavailable",
                command,
                logger,
            )
            return False

        width_mm = measurement.get("width_mm")
        depth_mm = measurement.get("depth_mm")
        extra_descent_m = calculate_pregrasp_extra_descent(depth_mm)
        target_z = plan.grasp_z - extra_descent_m
        logger.info(
            "임시 공구 pre-grasp actual depth 기반 추가 하강 계산: "
            f"width={width_mm:.1f}mm, "
            f"depth={depth_mm:.1f}mm, "
            f"extra_descent={extra_descent_m:.3f}m, "
            f"target_z={target_z:.3f}m"
        )

        self.gripper.open_gripper()
        self.wait_fn(0.3)

        if extra_descent_m <= 0.0:
            logger.info("임시 공구 pre-grasp 추가 하강을 생략합니다.")
            return True

        return self._move_to_pose(
            plan.target_x,
            plan.target_y,
            target_z,
            ori,
            logger,
            tool_name,
            command,
            "return_store_tool_pregrasp_descent_failed",
            "임시 공구 pre-grasp 추가 하강에 실패했습니다.",
            min_z=target_z,
        )

    def _measure_pregrasp_depth(self, logger):
        """Close once and return settled width/depth in millimeters."""
        gripper = self.grasp_verifier.gripper
        gripper.close_gripper()

        start_time = time.monotonic()
        last_width_mm = None
        last_depth_mm = None
        while time.monotonic() - start_time < PREGRASP_MEASUREMENT_SETTLE_TIMEOUT_SEC:
            if self.interrupted():
                logger.info("임시 공구 pre-grasp depth 측정을 stop/pause 요청으로 중단합니다.")
                return None

            try:
                status = gripper.get_status()
                busy = bool(status[0]) if status else False
                last_width_mm = float(gripper.get_width())
                last_depth_mm = float(gripper.get_depth())
            except Exception as exc:
                logger.warn(f"임시 공구 pre-grasp 그리퍼 depth 읽기 실패: {exc}")
                return None

            if not busy:
                logger.info(
                    "임시 공구 pre-grasp gripper measurement: "
                    f"width={last_width_mm:.1f}mm, "
                    f"depth={last_depth_mm:.1f}mm"
                )
                return {
                    "width_mm": last_width_mm,
                    "depth_mm": last_depth_mm,
                }

            self.wait_fn(GRASP_VERIFY_POLL_SEC)

        logger.warn(
            "임시 공구 pre-grasp depth 측정 timeout: "
            f"last_width={last_width_mm}mm, "
            f"last_depth={last_depth_mm}mm"
        )
        if last_depth_mm is None:
            return None
        return {
            "width_mm": last_width_mm,
            "depth_mm": last_depth_mm,
        }

    def place_tool_at_marker(
        self,
        marker_target,
        tool_name,
        command,
        logger,
        drawer_id=None,
    ):
        if marker_target is None or not marker_target.found:
            self.reporter.fail(
                tool_name,
                "서랍 내부 ArUco marker 중심을 찾지 못했습니다.",
                "return_drawer_marker_not_found",
                command,
                logger,
            )
            return False
        if marker_target.base_xyz is None:
            self.reporter.fail(
                tool_name,
                "서랍 내부 ArUco marker 중심 좌표 변환에 실패했습니다.",
                "return_drawer_marker_projection_failed",
                command,
                logger,
            )
            return False

        marker_x, marker_y, marker_z = marker_target.base_xyz
        safe_z_min = safe_z_min_for_drawer(drawer_id)
        drawer_wall_clearance_z = drawer_wall_clearance_z_for_drawer(drawer_id)
        marker_approach_z = max(
            safe_z_min,
            float(marker_z) + DRAWER_STORE_MARKER_APPROACH_Z_OFFSET_M,
        )
        marker_release_z = max(
            safe_z_min,
            float(marker_z) + DRAWER_STORE_MARKER_RELEASE_Z_OFFSET_M,
        )
        ori = current_ee_orientation(self.robot)
        logger.info(
            "서랍 marker place 높이 계산: "
            f"drawer={drawer_id}, marker_z={marker_z:.3f}, "
            f"safe_z_min={safe_z_min:.3f}, "
            f"drawer_wall_clearance_z={drawer_wall_clearance_z:.3f}, "
            f"marker_approach_z={marker_approach_z:.3f}, "
            f"marker_release_z={marker_release_z:.3f}"
        )

        self.reporter.publish(
            "placing_drawer_tool",
            tool_name,
            f"{tool_name}을 서랍 내부 marker 중심으로 이동합니다.",
            command,
        )
        if not self._move_to_current_xy_drawer_wall_clearance(
            drawer_wall_clearance_z,
            ori,
            logger,
            tool_name,
            command,
            "return_drawer_marker_move_failed",
            "서랍 marker 접근 전 clearance 높이 확보에 실패했습니다.",
        ):
            return False

        if not self._rotate_wrist_for_drawer_place(tool_name, command, logger):
            return False
        ori = current_ee_orientation(self.robot)

        if not self._move_to_pose(
            marker_x,
            marker_y,
            drawer_wall_clearance_z,
            ori,
            logger,
            tool_name,
            command,
            "return_drawer_marker_move_failed",
            "서랍 marker 이동 전 안전 높이 이동에 실패했습니다.",
        ):
            return False

        if not self._move_to_pose(
            marker_x,
            marker_y,
            marker_approach_z,
            ori,
            logger,
            tool_name,
            command,
            "return_drawer_marker_move_failed",
            "서랍 marker 상단 접근에 실패했습니다.",
        ):
            return False

        if not self._move_to_pose(
            marker_x,
            marker_y,
            marker_release_z,
            ori,
            logger,
            tool_name,
            command,
            "return_drawer_place_failed",
            "서랍 marker 중심 내려놓기 위치 이동에 실패했습니다.",
        ):
            return False

        if self.interrupted():
            logger.info("서랍 내부 공구 release 전 stop/pause 요청으로 중단합니다.")
            return False

        if self.tool_hold_monitor is not None:
            self.tool_hold_monitor.stop("return_drawer_release")
        self.gripper.open_gripper()
        self.wait_fn(RETURN_TOOL_RELEASE_WAIT_SEC)

        if not self._move_to_pose(
            marker_x,
            marker_y,
            drawer_wall_clearance_z,
            ori,
            logger,
            tool_name,
            command,
            "return_drawer_place_failed",
            "서랍 내부 공구를 놓은 뒤 후퇴에 실패했습니다.",
        ):
            return False

        ok = move_to_drawer_store_exit(
            self.motion,
            logger,
            marker_x,
            marker_y,
            drawer_wall_clearance_z,
            ori,
            "서랍 marker release 후 exit offset 이동",
            "서랍 내부 공구를 놓은 뒤 y축 exit offset 이동에 실패했습니다.",
        )
        if ok:
            return True

        self.reporter.fail(
            tool_name,
            "서랍 내부 공구를 놓은 뒤 y축 exit offset 이동에 실패했습니다.",
            "return_drawer_place_failed",
            command,
            logger,
        )
        return False

    def _rotate_wrist_for_drawer_place(self, tool_name, command, logger):
        ok = rotate_wrist_for_drawer_store(
            self.motion,
            logger,
            label="return_drawer_place",
        )
        if ok:
            return True

        if self.interrupted():
            return False

        self.reporter.fail(
            tool_name,
            "서랍 내부 보관 전 J6 회전에 실패했습니다.",
            "return_drawer_place_wrist_rotation_failed",
            command,
            logger,
        )
        return False

    def _move_to_current_xy_drawer_wall_clearance(
        self,
        drawer_wall_clearance_z,
        ori,
        logger,
        tool_name,
        command,
        failure_reason,
        failure_message,
    ):
        current_pose = get_ee_matrix(self.robot)
        current_x = float(current_pose[0, 3])
        current_y = float(current_pose[1, 3])
        return self._move_to_pose(
            current_x,
            current_y,
            drawer_wall_clearance_z,
            ori,
            logger,
            tool_name,
            command,
            failure_reason,
            failure_message,
        )

    def _move_to_pose(
        self,
        x,
        y,
        z,
        ori,
        logger,
        tool_name,
        command,
        failure_reason,
        failure_message,
        min_z=None,
    ):
        if self.interrupted():
            logger.info("return drawer placement motion 전 stop/pause 요청으로 중단합니다.")
            return False

        ok = self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(x, y, z, ori, logger),
            min_z=min_z,
            collision_scene_key="return/drawer_placement",
        )
        if ok:
            return True

        if self.interrupted():
            return False

        self.reporter.fail(
            tool_name,
            failure_message,
            failure_reason,
            command,
            logger,
        )
        return False
