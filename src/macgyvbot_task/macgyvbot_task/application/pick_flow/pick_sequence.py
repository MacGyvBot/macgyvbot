"""Pick sequence step construction."""

import time

from macgyvbot_config.drawer import TOOL_OBSERVE_X_BACKOFF_M

from macgyvbot_manipulation.robot_pose import (
    current_ee_orientation,
    make_safe_pose,
)
from macgyvbot_manipulation.robot_safezone import (
    SAFE_Z_MIN,
    safe_z_min_for_drawer,
)
from macgyvbot_manipulation.timing import cooperative_wait
from macgyvbot_task.application.pick_flow.pick_grasp_flow import (
    PickGraspFlow,
    calculate_pregrasp_extra_descent,
)
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
        generate_grasp_detection_mask_images=None,
        control_events=None,
        drawer_flow=None,
    ):
        self.robot = robot
        self.motion = motion_controller
        self.gripper = gripper
        self.state = state
        self.tool_hold_monitor = tool_hold_monitor
        self.refine_pick_target = refine_pick_target
        self.generate_grasp_detection_mask_images = generate_grasp_detection_mask_images
        self.control_events = control_events or {}
        self.drawer_flow = drawer_flow
        self.target_planner = PickTargetPlanner(robot)
        self.handoff = PickHandoffFlow(
            robot,
            motion_controller,
            gripper,
            state,
            cooperative_wait,
            tool_hold_monitor,
            interrupted=self._interrupted,
        )
        self.grasp = PickGraspFlow(
            gripper,
            state,
            cooperative_wait,
            interrupted=self._interrupted,
        )

    def build_steps(self, bx, by, bz, vlm_yaw_deg=None):
        self.state.human_grasped_tool = False
        self.state.last_grasp_result = None
        self.state.tool_mask_locked = False
        self.state.last_tool_mask_lock_result = None
        self.state.grasp_detection_mask_images = None
        self.state.grasp_detection_mask_target = None

        log = self.state.logger()
        drawer_id = self._drawer_id_for_current_target()
        safe_z_min = safe_z_min_for_drawer(drawer_id)
        plan = self.target_planner.plan(
            bx,
            by,
            bz,
            log,
            safe_z_min=safe_z_min,
        )
        context = {
            "plan": plan,
            "ori": self.state.home_ori,
            "drawer_id": drawer_id,
            "safe_z_min": safe_z_min,
            "vlm_yaw_deg": vlm_yaw_deg,
        }

        log.info(
            f"시퀀스 시작: Target({plan.target_x:.3f}, {plan.target_y:.3f}), "
            f"safe_z_min={safe_z_min:.3f}, raw_bz={bz:.3f}, "
            f"drawer_wall_clearance_z={plan.drawer_wall_clearance_z:.3f}, "
            f"grasp_z={plan.grasp_z:.3f}"
        )

        steps = [
            TaskStep("pick/open_gripper", self._open_gripper),
            TaskStep(
                "pick/observe_offset_move",
                lambda: self._move_to_vlm_observe_pose(context),
            ),
            TaskStep(
                "pick/refine_target_and_apply_vlm_yaw",
                lambda: self._refine_target_and_apply_vlm_yaw_step(context),
            ),
            TaskStep(
                "pick/grasp_descent",
                lambda: self._descend_to_grasp(
                    context["plan"],
                    context["ori"],
                ),
            ),
            TaskStep(
                "pick/pregrasp_depth_adjust",
                lambda: self._pregrasp_depth_adjust(context),
            ),
            TaskStep("pick/grasp_tool", self._grasp_tool),
            TaskStep(
                "pick/lift",
                lambda: self._move_to_pose(
                    "7단계: 안전 높이 복귀",
                    context["plan"].target_x,
                    context["plan"].target_y,
                    context["plan"].drawer_wall_clearance_z,
                    context["ori"],
                    "서랍 벽 회피 높이 복귀 실패",
                    "서랍 벽 회피 높이 복귀 실패",
                    "lift_failed",
                ),
            ),
            TaskStep(
                "pick/move_to_handoff",
                lambda: self._move_to_handoff(context["plan"], context),
            ),
            TaskStep("pick/wait_tool_mask_lock", self._wait_tool_mask_lock),
            TaskStep(
                "pick/wait_human_grasp",
                lambda: self._wait_human_grasp(context["plan"], context),
            ),
            TaskStep("pick/release_to_human", self._release_to_human),
            TaskStep("pick/home_before_close_drawer", self._home_before_close_drawer),
            TaskStep(
                "pick/close_drawer",
                lambda: self._close_drawer_after_handoff(context),
            ),
            TaskStep("pick/home_after_handoff", self._home_after_handoff),
            TaskStep("pick/done", self._publish_done, retry_on_pause=False),
        ]
        return steps

    def _open_gripper(self):
        self.gripper.open_gripper()
        cooperative_wait(0.5)
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
        min_z=None,
    ):
        log = self.state.logger()
        log.info(log_message)
        ok = self.motion.plan_and_execute(
            log,
            pose_goal=make_safe_pose(x, y, z, ori, log),
            min_z=min_z,
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

    def _refine_target_and_apply_vlm_yaw_step(self, context):
        log = self.state.logger()
        target_label = self.state.target_label
        refined_target = None
        refine_started = time.monotonic()

        if self.refine_pick_target is not None and target_label:
            cooperative_wait(0.2)
            try:
                log.info(
                    "pick/refine_target_and_apply_vlm_yaw started: "
                    f"target={target_label}"
                )
                refined_target = self.refine_pick_target(target_label)
            except Exception as exc:
                log.warn(f"상단 view VLM target 갱신 실패: {exc}")
            finally:
                log.info(
                    "pick/refine_target_and_apply_vlm_yaw completed: "
                    f"elapsed_sec={time.monotonic() - refine_started:.3f}"
                )

        if refined_target is not None and refined_target.found:
            bx, by, bz = refined_target.base_xyz
            context["vlm_yaw_deg"] = refined_target.yaw_deg
            context["plan"] = self.target_planner.plan(
                bx,
                by,
                bz,
                log,
                safe_z_min=context["safe_z_min"],
            )
            plan = context["plan"]
            log.info(
                "상단 view VLM 결과로 pick target 갱신: "
                f"pixel={refined_target.pixel}, "
                f"base=({plan.target_x:.3f}, {plan.target_y:.3f}, "
                f"{bz:.3f}), "
                f"depth={getattr(refined_target, 'depth_m', None)}, "
                f"yaw={context['vlm_yaw_deg']}, "
                f"safe_z_min={context['safe_z_min']:.3f}"
            )
        elif refined_target is not None:
            reason = getattr(refined_target, "reason", "unknown")
            log.warn(
                "상단 view VLM target을 얻지 못해 bbox center plan을 유지합니다. "
                f"reason={reason}"
            )

        vlm_yaw_deg = context.get("vlm_yaw_deg")
        if vlm_yaw_deg is None:
            return True

        return self._rotate_wrist(vlm_yaw_deg, context)

    def _move_to_vlm_observe_pose(self, context):
        ok = self._move_to_pose(
            "1단계: 서랍 내부 관찰 offset 위치로 이동",
            context["plan"].target_x - TOOL_OBSERVE_X_BACKOFF_M,
            context["plan"].target_y,
            context["plan"].drawer_wall_clearance_z,
            context["ori"],
            "서랍 내부 관찰 offset 이동 실패. Pick 시퀀스 중단",
            "서랍 내부 관찰 offset 이동 실패",
            "observe_offset_move_failed",
        )
        if not ok:
            return False

        self.state._publish_robot_status(
            "observing_pick_target",
            tool_name=self.state.target_label,
            action="bring",
            message=f"{self.state.target_label} VLM 관찰 위치에서 SAM 추적을 시작합니다.",
            command=self.state.current_command,
        )
        self._generate_grasp_detection_mask_images()
        return True

    def _generate_grasp_detection_mask_images(self):
        if self.generate_grasp_detection_mask_images is None:
            return

        target_label = self.state.target_label
        if not target_label:
            return

        try:
            self.generate_grasp_detection_mask_images(target_label)
        except Exception as exc:
            self.state.logger().warn(
                f"grasp detection mask image 생성 실패: {exc}"
            )

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
        self.state._publish_robot_status(
            "grasping",
            tool_name=self.state.target_label,
            action="bring",
            message=f"{self.state.target_label} 파지를 위해 Z 하강합니다.",
            command=self.state.current_command,
        )
        if not plan.should_descend_to_grasp:
            self.state.logger().info(
                "4단계: drawer_wall_clearance_z와 grasp_z가 같아 추가 하강 생략"
            )
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

    def _pregrasp_depth_adjust(self, context):
        log = self.state.logger()
        plan = context["plan"]
        ori = context["ori"]
        drawer_safe_z_min = context.get("safe_z_min", SAFE_Z_MIN)

        log.info("4.5단계: pre-grasp depth 측정")
        measurement = self.grasp.measure_pregrasp_depth(log)
        if measurement is None:
            log.error("pre-grasp depth 측정 실패. Pick 시퀀스 중단")
            self.state._publish_robot_status(
                "failed",
                message="pre-grasp depth 측정에 실패했습니다.",
                reason="pregrasp_depth_unavailable",
                command=self.state.current_command,
            )
            return False

        width_mm = measurement.get("width_mm")
        depth_mm = measurement.get("depth_mm")
        extra_descent_m = calculate_pregrasp_extra_descent(depth_mm)
        redescend_min_z = drawer_safe_z_min - extra_descent_m
        target_z = max(plan.grasp_z - extra_descent_m, redescend_min_z)
        actual_descent_m = plan.grasp_z - target_z
        log.info(
            "pre-grasp actual depth 기반 추가 하강 계산: "
            f"width={width_mm:.1f}mm, "
            f"depth={depth_mm:.1f}mm, "
            f"extra_descent={extra_descent_m:.3f}m, "
            f"target_z={target_z:.3f}m, "
            f"drawer_safe_z_min={drawer_safe_z_min:.3f}m, "
            f"redescend_min_z={redescend_min_z:.3f}m"
        )

        self.gripper.open_gripper()
        cooperative_wait(0.3)

        if actual_descent_m <= 0.0:
            log.info("닫힌 그리퍼 기준 z 제한으로 pre-grasp 추가 하강을 생략합니다.")
            return True

        return self._move_to_pose(
            "4.5단계: pre-grasp actual depth 기반 추가 하강",
            plan.target_x,
            plan.target_y,
            target_z,
            ori,
            "pre-grasp 추가 하강 실패. Pick 시퀀스 중단",
            "pre-grasp 추가 하강 실패",
            "pregrasp_descent_failed",
            min_z=redescend_min_z,
        )

    def _wait_tool_mask_lock(self):
        log = self.state.logger()
        log.info("handoff 위치 이동 후 depth locked tool mask 완료 대기")
        if self.handoff.wait_for_tool_mask_lock(log):
            return True

        if self._interrupted():
            return False

        log.error("공구 mask lock 실패. 사용자 잡기 대기 전에 pick 시퀀스를 중단합니다.")
        self.state._publish_robot_status(
            "failed",
            message="공구 mask lock에 실패했습니다.",
            reason="tool_mask_lock_failed",
            command=self.state.current_command,
        )
        return False

    def _move_to_handoff(self, plan, context):
        log = self.state.logger()
        handoff_pose = self.handoff.move_to_handoff_pose(log)
        if handoff_pose[0] is not None:
            return True

        if self._interrupted():
            return False

        log.error("사용자 손 위치 확인 실패. 원래 공구 위치로 반환합니다.")
        returned, drawer_closed, home_ok = self._return_tool_close_drawer_home(
            plan,
            context,
            log,
            lift_from_current=False,
        )
        status = "returned" if returned and drawer_closed and home_ok else "failed"
        self.state._publish_robot_status(
            status,
            message=(
                "사용자 손 위치 확인 실패로 공구를 원래 위치에 반환하고 서랍을 닫았습니다."
                if returned and drawer_closed and home_ok
                else "사용자 손 위치 확인 실패 후 서랍을 닫았지만 Home 복귀에 실패했습니다."
                if returned and drawer_closed
                else "사용자 손 위치 확인 실패 후 공구를 반환했지만 서랍 닫기에 실패했습니다."
                if returned
                else "사용자 손 위치 확인 실패 후 원위치 반환에도 실패했습니다."
            ),
            reason=(
                "handoff_pose_unavailable"
                if returned and drawer_closed and home_ok
                else "handoff_pose_unavailable_home_after_recovery_failed"
                if returned and drawer_closed
                else "handoff_pose_unavailable_drawer_close_failed"
                if returned
                else "handoff_pose_unavailable"
            ),
            command=self.state.current_command,
        )
        return False

    def _wait_human_grasp(self, plan, context):
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
        returned, drawer_closed, home_ok = self._return_tool_close_drawer_home(
            plan,
            context,
            log,
        )
        status = "returned" if returned and drawer_closed and home_ok else "failed"
        self.state._publish_robot_status(
            status,
            message=(
                "사용자 잡기 인식 실패로 공구를 원래 위치에 반환하고 서랍을 닫았습니다."
                if returned and drawer_closed and home_ok
                else "사용자 잡기 인식 실패 후 서랍을 닫았지만 Home 복귀에 실패했습니다."
                if returned and drawer_closed
                else "사용자 잡기 인식 실패 후 공구를 반환했지만 서랍 닫기에 실패했습니다."
                if returned
                else "사용자 잡기 인식 실패 후 원위치 반환에도 실패했습니다."
            ),
            reason=(
                "handoff_timeout"
                if returned and drawer_closed and home_ok
                else "handoff_timeout_home_after_recovery_failed"
                if returned and drawer_closed
                else "handoff_timeout_drawer_close_failed"
                if returned
                else "handoff_timeout"
            ),
            command=self.state.current_command,
        )
        return False

    def _return_tool_close_drawer_home(
        self,
        plan,
        context,
        log,
        lift_from_current=True,
    ):
        drawer_id = context.get("drawer_id")
        returned = self.handoff.return_tool_to_original_position(
            plan.target_x,
            plan.target_y,
            plan.drawer_wall_clearance_z,
            plan.grasp_z,
            context["ori"],
            log,
            safe_z_min=context["safe_z_min"],
            drawer_id=drawer_id,
            move_home=False,
            lift_from_current=lift_from_current,
        )
        if not returned:
            return False, False, False

        if self.drawer_flow is None or drawer_id is None:
            home_ok = self.handoff.move_home_after_handoff(
                log,
                publish_on_failure=False,
            )
            return True, True, home_ok

        log.info(f"복구 반환 후 drawer {drawer_id}를 닫습니다.")
        self.state._publish_robot_status(
            "closing_drawer",
            tool_name=self.state.target_label,
            action="bring",
            message=f"{self.state.target_label}가 있던 서랍을 닫습니다.",
            command=self.state.current_command,
        )
        drawer_closed = self.drawer_flow.close_drawer(drawer_id, log)
        if not drawer_closed:
            return True, False, False

        home_ok = self.handoff.move_home_after_handoff(
            log,
            publish_on_failure=False,
        )
        return True, True, home_ok

    def _release_to_human(self):
        self.state.logger().info("10단계: 사용자 잡기 확인 후 그리퍼 오픈(놓기)")
        if self.tool_hold_monitor is not None:
            self.tool_hold_monitor.stop("handoff_release")
        self.gripper.open_gripper()
        cooperative_wait(0.8)
        return True

    def _home_before_close_drawer(self):
        self.state.logger().info("11단계: 서랍 닫기 전 Home 위치로 이동")
        return self.handoff.move_home_after_handoff(self.state.logger())

    def _home_after_handoff(self):
        self.state.logger().info("12단계: 서랍 닫은 후 Home 위치로 복귀")
        return self.handoff.move_home_after_handoff(self.state.logger())

    def _close_drawer_after_handoff(self, context):
        drawer_id = context.get("drawer_id")
        if self.drawer_flow is None or drawer_id is None:
            return True

        log = self.state.logger()
        log.info(f"사용자 전달 후 drawer {drawer_id}를 닫습니다.")
        self.state._publish_robot_status(
            "closing_drawer",
            tool_name=self.state.target_label,
            action="bring",
            message=f"{self.state.target_label}가 있던 서랍을 닫습니다.",
            command=self.state.current_command,
        )
        return self.drawer_flow.close_drawer(drawer_id, log)

    def _publish_done(self):
        self.state.logger().info("Pick 시퀀스 완료")
        self.state._publish_robot_status(
            "done",
            message="공구 전달 후 Home 복귀까지 완료되었습니다.",
            command=self.state.current_command,
        )
        return True

    def _drawer_id_for_current_target(self):
        if self.drawer_flow is None:
            return None
        return self.drawer_flow.drawer_id_for_tool(self.state.target_label)

    def _interrupted(self):
        return any(
            event is not None and event.is_set()
            for event in (
                self.control_events.get("exit"),
                self.control_events.get("pause"),
            )
        )
