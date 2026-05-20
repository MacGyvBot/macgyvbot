"""Pick up a returned tool directly from the floor."""

from __future__ import annotations

from macgyvbot_manipulation.handover_targeting import build_replan_attempts
from macgyvbot_manipulation.robot_pose import current_ee_orientation, make_safe_pose
from macgyvbot_task.application.pick_flow.pick_grasp_flow import PickGraspFlow
from macgyvbot_task.application.pick_flow.pick_target_planner import PickTargetPlanner


class ReturnFloorPickupFlow:
    """Move to a floor target, grasp it, and retreat to safe height."""

    def __init__(
        self,
        robot,
        motion_controller,
        gripper,
        state,
        reporter,
        wait_fn,
        tool_hold_monitor=None,
    ):
        self.robot = robot
        self.motion = motion_controller
        self.gripper = gripper
        self.state = state
        self.reporter = reporter
        self.wait_fn = wait_fn
        self.tool_hold_monitor = tool_hold_monitor
        self.target_planner = PickTargetPlanner(robot)
        self.grasp = PickGraspFlow(gripper, state, wait_fn)

    def pick(self, target, command, logger):
        tool_name = target.label or command.get("tool_name", "unknown")
        bx, by, bz = target.base_xyz
        plan = self.target_planner.plan(bx, by, bz, logger)
        ori = self.state.home_ori

        self.reporter.publish(
            "picking_return_floor_tool",
            tool_name,
            f"바닥에 있는 {tool_name} 반납 공구를 집습니다.",
            command,
        )

        if not self._move(plan.current_x, plan.current_y, plan.travel_z, ori, logger):
            return self._fail(
                tool_name,
                command,
                logger,
                "return_floor_travel_z_failed",
            )

        safe_target_x, safe_target_y, safe_travel_z = self._move_with_replan(
            plan.target_x,
            plan.target_y,
            plan.travel_z,
            ori,
            logger,
            "바닥 반납 공구 안전 높이",
        )
        if safe_target_x is None:
            return self._fail(
                tool_name,
                command,
                logger,
                "return_floor_xy_move_failed",
            )

        if target.yaw_deg is not None:
            ok = self.motion.rotate_wrist_by_yaw_deg(target.yaw_deg, logger)
            if not ok:
                return self._fail(
                    tool_name,
                    command,
                    logger,
                    "return_floor_wrist_rotation_failed",
                )
            ori = current_ee_orientation(self.robot)

        safe_approach_z = max(plan.approach_z, safe_travel_z - 0.18)
        safe_grasp_z = min(plan.grasp_z, safe_approach_z)

        ok = self._move(safe_target_x, safe_target_y, safe_approach_z, ori, logger)
        if not ok:
            return self._fail(
                tool_name,
                command,
                logger,
                "return_floor_approach_failed",
            )

        if plan.should_descend_to_grasp:
            if not self._move(safe_target_x, safe_target_y, safe_grasp_z, ori, logger):
                return self._fail(
                    tool_name,
                    command,
                    logger,
                    "return_floor_grasp_descent_failed",
                )

        if not self.grasp.try_robot_grasp(logger):
            return self._fail(
                tool_name,
                command,
                logger,
                "return_floor_grasp_failed",
            )

        self.reporter.publish(
            "grasp_success",
            tool_name,
            "바닥 반납 공구 grasp에 성공했습니다.",
            command,
        )
        if self.tool_hold_monitor is not None:
            self.tool_hold_monitor.start(tool_name, "return", command)

        if not self._move(safe_target_x, safe_target_y, safe_travel_z, ori, logger):
            return self._fail(
                tool_name,
                command,
                logger,
                "return_floor_lift_failed",
            )

        return tool_name, ""

    def _move(self, x, y, z, ori, logger):
        return self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(x, y, z, ori, logger),
        )

    def _move_with_replan(self, x, y, z, ori, logger, label):
        attempts = build_replan_attempts(x, y, z)
        last_pose = None
        for attempt_index, (target_x, target_y, target_z) in enumerate(
            attempts,
            start=1,
        ):
            logger.info(
                f"{label} 플래닝 시도: {attempt_index}/{len(attempts)}, "
                f"target=({target_x:.3f},{target_y:.3f},{target_z:.3f})"
            )
            pose_goal = make_safe_pose(target_x, target_y, target_z, ori, logger)
            ok = self.motion.plan_and_execute(logger, pose_goal=pose_goal)
            last_pose = pose_goal.pose.position
            if ok:
                return (
                    float(last_pose.x),
                    float(last_pose.y),
                    float(last_pose.z),
                )

            if attempt_index < len(attempts):
                logger.warn(
                    f"{label} 플래닝 실패. "
                    "x를 줄이고 y를 0에 가깝게 보정해 재시도합니다."
                )

        return None, None, None

    def _fail(self, tool_name, command, logger, reason):
        self.reporter.fail(
            tool_name,
            "바닥 반납 공구 pickup에 실패했습니다.",
            reason,
            command,
            logger,
        )
        return None, reason
