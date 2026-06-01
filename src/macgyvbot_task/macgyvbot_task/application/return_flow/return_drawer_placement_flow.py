"""Move a staged return tool into its drawer marker target."""

from __future__ import annotations

from macgyvbot_config.drawer import (
    DRAWER_STORE_MARKER_CLEARANCE_Z_OFFSET_M,
    DRAWER_STORE_MARKER_EXIT_OFFSET_XYZ_M,
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
            plan.travel_z,
            ori,
            logger,
            tool_name,
            command,
            "return_store_tool_grasp_failed",
            "임시 공구 파지 전 안전 높이 이동에 실패했습니다.",
        ):
            return False

        if not self._move_to_pose(
            plan.target_x,
            plan.target_y,
            plan.approach_z,
            ori,
            logger,
            tool_name,
            command,
            "return_store_tool_grasp_failed",
            "임시 공구 파지 접근에 실패했습니다.",
        ):
            return False

        if plan.should_descend_to_grasp and not self._move_to_pose(
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

        return self._move_to_pose(
            plan.target_x,
            plan.target_y,
            plan.travel_z,
            ori,
            logger,
            tool_name,
            command,
            "return_store_tool_grasp_failed",
            "임시 공구 파지 후 안전 높이 복귀에 실패했습니다.",
        )

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
        clearance_z = self._clearance_z_for_drawer(drawer_id)
        approach_z = max(
            safe_z_min,
            float(marker_z) + DRAWER_STORE_MARKER_APPROACH_Z_OFFSET_M,
        )
        release_z = max(
            safe_z_min,
            float(marker_z) + DRAWER_STORE_MARKER_RELEASE_Z_OFFSET_M,
        )
        ori = current_ee_orientation(self.robot)
        logger.info(
            "서랍 marker place 높이 계산: "
            f"drawer={drawer_id}, marker_z={marker_z:.3f}, "
            f"safe_z_min={safe_z_min:.3f}, "
            f"clearance_z={clearance_z:.3f}, "
            f"approach_z={approach_z:.3f}, release_z={release_z:.3f}"
        )

        self.reporter.publish(
            "placing_drawer_tool",
            tool_name,
            f"{tool_name}을 서랍 내부 marker 중심으로 이동합니다.",
            command,
        )
        if not self._move_to_current_xy_clearance(
            clearance_z,
            ori,
            logger,
            tool_name,
            command,
            "return_drawer_marker_move_failed",
            "서랍 marker 접근 전 clearance 높이 확보에 실패했습니다.",
        ):
            return False

        if not self._move_to_pose(
            marker_x,
            marker_y,
            clearance_z,
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
            approach_z,
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
            release_z,
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
            clearance_z,
            ori,
            logger,
            tool_name,
            command,
            "return_drawer_place_failed",
            "서랍 내부 공구를 놓은 뒤 후퇴에 실패했습니다.",
        ):
            return False

        exit_x = marker_x + DRAWER_STORE_MARKER_EXIT_OFFSET_XYZ_M[0]
        exit_y = marker_y + DRAWER_STORE_MARKER_EXIT_OFFSET_XYZ_M[1]
        exit_z = clearance_z + DRAWER_STORE_MARKER_EXIT_OFFSET_XYZ_M[2]
        logger.info(
            "서랍 marker release 후 exit offset 이동: "
            f"offset=({DRAWER_STORE_MARKER_EXIT_OFFSET_XYZ_M[0]:.3f}, "
            f"{DRAWER_STORE_MARKER_EXIT_OFFSET_XYZ_M[1]:.3f}, "
            f"{DRAWER_STORE_MARKER_EXIT_OFFSET_XYZ_M[2]:.3f}), "
            f"target=({exit_x:.3f}, {exit_y:.3f}, {exit_z:.3f})"
        )
        return self._move_to_pose(
            exit_x,
            exit_y,
            exit_z,
            ori,
            logger,
            tool_name,
            command,
            "return_drawer_place_failed",
            "서랍 내부 공구를 놓은 뒤 y축 exit offset 이동에 실패했습니다.",
        )

    @classmethod
    def _clearance_z_for_drawer(cls, drawer_id):
        return (
            safe_z_min_for_drawer(drawer_id)
            + DRAWER_STORE_MARKER_CLEARANCE_Z_OFFSET_M
        )

    def _move_to_current_xy_clearance(
        self,
        clearance_z,
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
            clearance_z,
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
    ):
        if self.interrupted():
            logger.info("return drawer placement motion 전 stop/pause 요청으로 중단합니다.")
            return False

        ok = self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(x, y, z, ori, logger),
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
