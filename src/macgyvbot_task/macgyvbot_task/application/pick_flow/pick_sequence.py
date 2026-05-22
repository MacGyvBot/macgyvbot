"""Pick sequence step construction."""

import time

import rclpy

from macgyvbot_config.timing import SEQUENCE_WAIT_POLL_SEC
from macgyvbot_manipulation.robot_pose import (
    current_ee_orientation,
    make_safe_pose,
)
from macgyvbot_task.application.pick_flow.pick_grasp_flow import PickGraspFlow
from macgyvbot_task.application.pick_flow.pick_handoff_flow import PickHandoffFlow
from macgyvbot_task.application.pick_flow.pick_target_planner import PickTargetPlanner
from macgyvbot_task.application.task_control.task_step import TaskStep


class PickSequenceRunner:
    """Build pick workflow steps for task-queue execution."""

    def __init__(
        self,
        robot,
        motion_controller,
        gripper,
        state,
        tool_hold_monitor=None,
        refine_pick_target=None,
        control_events=None,
    ):
        self.robot = robot
        self.motion = motion_controller
        self.gripper = gripper
        self.state = state
        self.tool_hold_monitor = tool_hold_monitor
        self.refine_pick_target = refine_pick_target
        self.control_events = control_events or {}
        self.target_planner = PickTargetPlanner(robot)
        self.handoff = PickHandoffFlow(
            robot,
            motion_controller,
            gripper,
            state,
            self.cooperative_wait,
            tool_hold_monitor,
            interrupted=self._interrupted,
        )
        self.grasp = PickGraspFlow(
            gripper,
            state,
            self.cooperative_wait,
            interrupted=self._interrupted,
        )

    def build_steps(self, bx, by, bz, z_m, vlm_yaw_deg=None):
        self.state.human_grasped_tool = False
        self.state.last_grasp_result = None
        self.state.tool_mask_locked = False
        self.state.last_tool_mask_lock_result = None

        log = self.state.logger()
        plan = self.target_planner.plan(bx, by, bz, log)
        context = {"ori": self.state.home_ori}

        log.info(
            f"시퀀스 시작: Target({plan.target_x:.3f}, {plan.target_y:.3f}), "
            f"depth={z_m:.3f}, raw_bz={bz:.3f}, "
            f"corrected_bz={plan.corrected_bz:.3f}, "
            f"travel_z={plan.travel_z:.3f}, "
            f"approach_z={plan.approach_z:.3f}, "
            f"grasp_z={plan.grasp_z:.3f}"
        )

        steps = [
            TaskStep("pick/open_gripper", self._open_gripper),
            TaskStep(
                "pick/travel_z",
                lambda: self._move_to_pose(
                    "1단계: 안전 이동 높이 확보",
                    plan.current_x,
                    plan.current_y,
                    plan.travel_z,
                    context["ori"],
                    "안전 이동 높이 확보 실패. Pick 시퀀스 중단",
                    "안전 이동 높이 확보 실패",
                    "travel_z_plan_failed",
                ),
            ),
            TaskStep(
                "pick/xy_move",
                lambda: self._move_to_pose(
                    "2단계: 안전 높이에서 XY 수평 이동",
                    plan.target_x,
                    plan.target_y,
                    plan.travel_z,
                    context["ori"],
                    "XY 이동 실패. Pick 시퀀스 중단",
                    "XY 이동 실패",
                    "xy_move_failed",
                ),
            ),
        ]

        if vlm_yaw_deg is not None:
            steps.append(
                TaskStep(
                    "pick/apply_vlm_yaw",
                    lambda: self._rotate_wrist(vlm_yaw_deg, context),
                )
                return

            refined_target = self._refine_target_after_xy_move(log)
            if refined_target is not None:
                bx, by, bz = refined_target.base_xyz
                z_m = refined_target.depth_m
                vlm_yaw_deg = refined_target.yaw_deg
                plan = self.target_planner.plan(bx, by, bz, log)
                log.info(
                    "상단 view VLM 결과로 pick target 갱신: "
                    f"pixel={refined_target.pixel}, "
                    f"base=({plan.target_x:.3f}, {plan.target_y:.3f}, "
                    f"{bz:.3f}), depth={z_m:.3f}, "
                    f"yaw={vlm_yaw_deg}"
                )

            if vlm_yaw_deg is not None:
                ok = self.motion.rotate_wrist_by_yaw_deg(vlm_yaw_deg, log)
                if not ok:
                    log.error("J6 회전 실패. Pick 시퀀스 중단")
                    self.state._publish_robot_status(
                        "failed",
                        message="J6 회전 실패",
                        reason="wrist_rotation_failed",
                        command=self.state.current_command,
                    )
                    return
                ori = current_ee_orientation(self.robot)

            log.info("3단계: 타겟 상단 접근")
            ok = self.motion.plan_and_execute(
                log,
                pose_goal=make_safe_pose(
                    plan.target_x,
                    plan.target_y,
                    plan.approach_z,
                    ori,
                    log,
                ),
            )

        steps.extend(
            [
                TaskStep(
                    "pick/approach",
                    lambda: self._move_to_pose(
                        "3단계: 타겟 상단 접근",
                        plan.target_x,
                        plan.target_y,
                        plan.approach_z,
                        context["ori"],
                        "상단 접근 실패. Pick 시퀀스 중단",
                        "상단 접근 실패",
                        "approach_failed",
                    ),
                ),
                TaskStep(
                    "pick/grasp_descent",
                    lambda: self._descend_to_grasp(plan, context["ori"]),
                ),
                TaskStep("pick/grasp_tool", self._grasp_tool),
                TaskStep("pick/wait_tool_mask_lock", self._wait_tool_mask_lock),
                TaskStep(
                    "pick/lift",
                    lambda: self._move_to_pose(
                        "7단계: 안전 높이 복귀",
                        plan.target_x,
                        plan.target_y,
                        plan.travel_z,
                        context["ori"],
                        "안전 높이 복귀 실패",
                        "안전 높이 복귀 실패",
                        "lift_failed",
                    ),
                ),
                TaskStep(
                    "pick/move_to_handoff",
                    lambda: self._move_to_handoff(plan, context["ori"]),
                ),
                TaskStep(
                    "pick/wait_human_grasp",
                    lambda: self._wait_human_grasp(plan, context["ori"]),
                ),
                TaskStep("pick/release_to_human", self._release_to_human),
                TaskStep("pick/home_after_handoff", self._home_after_handoff),
                TaskStep("pick/done", self._publish_done, retry_on_pause=False),
            ]
        )
        return steps

    def _open_gripper(self):
        self.gripper.open_gripper()
        self.cooperative_wait(0.5)
        return True

    def _move_to_pose(
        self,
        log_message,
        x,
        y,
        z,
        ori,
        error_log,
        failure_message,
        failure_reason,
    ):
        log = self.state.logger()
        log.info(log_message)
        ok = self.motion.plan_and_execute(
            log,
            pose_goal=make_safe_pose(x, y, z, ori, log),
        )
        if ok:
            return True

        if self._interrupted():
            return False

        log.error(error_log)
        self.state._publish_robot_status(
            "failed",
            message=failure_message,
            reason=failure_reason,
            command=self.state.current_command,
        )
        return False

    def _rotate_wrist(self, vlm_yaw_deg, context):
        log = self.state.logger()
        ok = self.motion.rotate_wrist_by_yaw_deg(vlm_yaw_deg, log)
        if ok:
            context["ori"] = current_ee_orientation(self.robot)
            return True

        if self._interrupted():
            return False

        log.error("J6 회전 실패. Pick 시퀀스 중단")
        self.state._publish_robot_status(
            "failed",
            message="J6 회전 실패",
            reason="wrist_rotation_failed",
            command=self.state.current_command,
        )
        return False

    def _descend_to_grasp(self, plan, ori):
        if not plan.should_descend_to_grasp:
            self.state.logger().info("4단계: approach_z와 grasp_z가 같아 추가 하강 생략")
            return True

        return self._move_to_pose(
            "4단계: 파지 높이 하강",
            plan.target_x,
            plan.target_y,
            plan.grasp_z,
            ori,
            "파지 높이 하강 실패. Pick 시퀀스 중단",
            "파지 높이 하강 실패",
            "grasp_descent_failed",
        )

    def _grasp_tool(self):
        log = self.state.logger()
        log.info("5단계: 공구 grasp 시도")
        if self.grasp.try_robot_grasp(log):
            self.state._publish_robot_status(
                "grasp_success",
                message="공구 grasp에 성공했습니다.",
                command=self.state.current_command,
            )
            if self.tool_hold_monitor is not None:
                self.tool_hold_monitor.start(
                    self.state.target_label,
                    "bring",
                    self.state.current_command,
                )
            return True

        if self._interrupted():
            return False

        log.error("공구 grasp 실패. Pick 시퀀스 중단")
        self.state._publish_robot_status(
            "failed",
            message="공구 grasp에 실패했습니다.",
            reason="robot_grasp_failed",
            command=self.state.current_command,
        )
        return False

    def _wait_tool_mask_lock(self):
        log = self.state.logger()
        log.info("6단계: 공구 mask lock 완료 대기")
        if self.handoff.wait_for_tool_mask_lock(log):
            return True

        if self._interrupted():
            return False

        log.error("공구 mask lock 실패. Lift 전에 pick 시퀀스를 중단합니다.")
        self.state._publish_robot_status(
            "failed",
            message="공구 mask lock에 실패했습니다.",
            reason="tool_mask_lock_failed",
            command=self.state.current_command,
        )
        return False

    def _move_to_handoff(self, plan, ori):
        log = self.state.logger()
        handoff_pose = self.handoff.move_to_handoff_pose(ori, log)
        if handoff_pose[0] is not None:
            return True

        if self._interrupted():
            return False

        log.error("사용자 손 위치 확인 실패. 원래 공구 위치로 반환합니다.")
        returned = self.handoff.return_tool_to_original_position(
            plan.target_x,
            plan.target_y,
            plan.travel_z,
            plan.grasp_z,
            ori,
            log,
        )
        status = "returned" if returned else "failed"
        self.state._publish_robot_status(
            status,
            message=(
                "사용자 손 위치 확인 실패로 공구를 원래 위치에 반환했습니다."
                if returned
                else "사용자 손 위치 확인 실패 후 원위치 반환에도 실패했습니다."
            ),
            reason="handoff_pose_unavailable",
            command=self.state.current_command,
        )
        return False

    def _wait_human_grasp(self, plan, ori):
        log = self.state.logger()
        log.info("9단계: 사용자 잡기 인식 대기")
        self.state._publish_robot_status(
            "waiting_handoff",
            message="사용자 잡기 인식을 기다립니다.",
            command=self.state.current_command,
        )
        if self.handoff.wait_for_human_grasp(log):
            return True

        if self._interrupted():
            return False

        log.error("사용자 잡기 인식 실패. 원래 공구 위치로 반환합니다.")
        returned = self.handoff.return_tool_to_original_position(
            plan.target_x,
            plan.target_y,
            plan.travel_z,
            plan.grasp_z,
            ori,
            log,
        )
        status = "returned" if returned else "failed"
        self.state._publish_robot_status(
            status,
            message=(
                "사용자 잡기 인식 실패로 공구를 원래 위치에 반환했습니다."
                if returned
                else "사용자 잡기 인식 실패 후 원위치 반환에도 실패했습니다."
            ),
            reason="handoff_timeout",
            command=self.state.current_command,
        )
        return False

    def _release_to_human(self):
        self.state.logger().info("10단계: 사용자 잡기 확인 후 그리퍼 오픈(놓기)")
        if self.tool_hold_monitor is not None:
            self.tool_hold_monitor.stop("handoff_release")
        self.gripper.open_gripper()
        self.cooperative_wait(0.8)
        return True

    def _home_after_handoff(self):
        self.state.logger().info("10단계: 전달 후 Home 위치로 복귀")
        return self.handoff.move_home_after_handoff(self.state.logger())

    def _publish_done(self):
        self.state.logger().info("Pick 시퀀스 완료")
        self.state._publish_robot_status(
            "done",
            message="공구 전달 후 Home 복귀까지 완료되었습니다.",
            command=self.state.current_command,
        )
        return True

    def _interrupted(self):
        return any(
            event is not None and event.is_set()
            for event in (
                self.control_events.get("exit"),
                self.control_events.get("pause"),
            )
        )

    @staticmethod
    def cooperative_wait(duration_sec):
        end_time = time.monotonic() + max(0.0, float(duration_sec))
        while rclpy.ok() and time.monotonic() < end_time:
            remaining = end_time - time.monotonic()
            time.sleep(min(SEQUENCE_WAIT_POLL_SEC, max(0.0, remaining)))

    def _refine_target_after_xy_move(self, log):
        if self.refine_pick_target is None:
            return None

        target_label = self.state.target_label
        if not target_label:
            return None

        self.cooperative_wait(0.2)
        try:
            target = self.refine_pick_target(target_label)
        except Exception as exc:
            log.warn(f"상단 view VLM target 갱신 실패: {exc}")
            return None

        if target is None or not target.found:
            reason = getattr(target, "reason", "unknown") if target else "unknown"
            log.warn(
                "상단 view VLM target을 얻지 못해 bbox center plan을 유지합니다. "
                f"reason={reason}"
            )
            return None

        return target
