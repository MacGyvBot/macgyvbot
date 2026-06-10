"""Pick sequence step construction."""

import time

import rclpy

from macgyvbot_config.drawer import TOOL_OBSERVE_X_BACKOFF_M
from macgyvbot_config.handoff import (
    HANDOFF_RELEASE_WAIT_SEC,
    HANDOFF_WAIT_POLL_SEC,
)
from macgyvbot_config.pick import (
    PICK_OPEN_GRIPPER_WAIT_SEC,
    PICK_PREGRASP_REOPEN_WAIT_SEC,
    PICK_REFINE_SETTLE_WAIT_SEC,
)

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
from macgyvbot_task.application.logging_utils import (
    log_error,
    log_info,
    log_warn,
)
from macgyvbot_task.application.task_control.task_step import TaskStep

HANDOFF_DECISION_POLL_SEC = HANDOFF_WAIT_POLL_SEC
HANDOFF_DECISION_PENDING_STATUS = "handoff_inspection_pending"


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
        estimate_grasp_yaw=None,
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
        self.estimate_grasp_yaw = estimate_grasp_yaw
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
        self.state.grasp_detection_yaw_deg = None
        self.state.grasp_detection_yaw_target = None

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
            "grasp_yaw_deg": vlm_yaw_deg,
            "grasp_wrist_joint_rad": None,
        }

        log_info(
            log,
            "pick plan ready",
            step="plan",
            event="ready",
            target_x=plan.target_x,
            target_y=plan.target_y,
            safe_z_min=safe_z_min,
            raw_bz=bz,
            drawer_wall_clearance_z=plan.drawer_wall_clearance_z,
            grasp_z=plan.grasp_z,
            should_descend_to_grasp=plan.should_descend_to_grasp,
        )

        steps = [
            TaskStep("pick/open_gripper", self._open_gripper),
            TaskStep(
                "pick/grasp_observe_offset_move",
                lambda: self._move_to_grasp_observe_pose(context),
            ),
            TaskStep(
                "pick/refine_target_and_apply_grasp_yaw",
                lambda: self._refine_target_and_apply_grasp_yaw_step(context),
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
            TaskStep("pick/grasp_tool", lambda: self._grasp_tool(context)),
            TaskStep(
                "pick/lift",
                lambda: self._move_to_pose(
                    "lift after grasp",
                    context["plan"].target_x,
                    context["plan"].target_y,
                    context["plan"].drawer_wall_clearance_z,
                    context["ori"],
                    "서랍 벽 회피 높이 복귀 실패",
                    "서랍 벽 회피 높이 복귀 실패",
                    "lift_failed",
                    collision_scene_key="pick/lift",
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
        cooperative_wait(PICK_OPEN_GRIPPER_WAIT_SEC)
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
        collision_scene_key=None,
    ):
        log = self.state.logger()
        log_info(log, log_message, step=failure_reason, event="start")
        ok = self.motion.plan_and_execute(
            log,
            pose_goal=make_safe_pose(x, y, z, ori, log),
            min_z=min_z,
            collision_scene_key=collision_scene_key,
        )
        if ok:
            return True

        if self._interrupted():
            return False

        log_error(log, error_log, step=failure_reason, event="fail")
        self.state._publish_robot_status(
            "failed",
            message=failure_message,
            reason=failure_reason,
            command=self.state.current_command,
        )
        return False

    def _refine_target_and_apply_grasp_yaw_step(self, context):
        log = self.state.logger()
        target_label = self.state.target_label
        refined_target = None
        refine_started = time.monotonic()

        if self.refine_pick_target is not None and target_label:
            cooperative_wait(PICK_REFINE_SETTLE_WAIT_SEC)
            try:
                log_info(
                    log,
                    "refine pick target",
                    step="refine_target",
                    event="start",
                    target=target_label,
                )
                refined_target = self.refine_pick_target(target_label)
            except Exception as exc:
                log_warn(
                    log,
                    "refine pick target failed",
                    step="refine_target",
                    event="fail",
                    reason=str(exc) or type(exc).__name__,
                )
            finally:
                log_info(
                    log,
                    "refine pick target",
                    step="refine_target",
                    event="done",
                    elapsed_sec=time.monotonic() - refine_started,
                )

        if refined_target is not None and refined_target.found:
            bx, by, bz = refined_target.base_xyz
            context["grasp_yaw_deg"] = refined_target.yaw_deg
            context["plan"] = self.target_planner.plan(
                bx,
                by,
                bz,
                log,
                safe_z_min=context["safe_z_min"],
            )
            plan = context["plan"]
            log_info(
                log,
                "pick target refined",
                step="refine_target",
                event="updated",
                pixel=refined_target.pixel,
                target_x=plan.target_x,
                target_y=plan.target_y,
                base_z=bz,
                depth_m=getattr(refined_target, "depth_m", None),
                yaw_deg=context["grasp_yaw_deg"],
                safe_z_min=context["safe_z_min"],
            )
        elif refined_target is not None:
            reason = getattr(refined_target, "reason", "unknown")
            log_warn(
                log,
                "pick target refinement unavailable",
                step="refine_target",
                event="fallback",
                reason=reason,
            )

        if context.get("grasp_yaw_deg") is None:
            context["grasp_yaw_deg"] = self._estimate_fallback_grasp_yaw(
                target_label,
                log,
            )

        grasp_yaw_deg = context.get("grasp_yaw_deg")
        if grasp_yaw_deg is None:
            return True

        return self._rotate_wrist(grasp_yaw_deg, context)

    def _estimate_fallback_grasp_yaw(self, target_label, log):
        if self.estimate_grasp_yaw is None or not target_label:
            return None
        try:
            yaw_deg = self.estimate_grasp_yaw(target_label)
        except Exception as exc:
            log_warn(
                log,
                "fallback grasp yaw unavailable",
                step="pca_yaw",
                event="fail",
                target=target_label,
                reason=str(exc) or type(exc).__name__,
            )
            return None

        if yaw_deg is None:
            return None

        log_info(
            log,
            "fallback grasp yaw applied",
            step="pca_yaw",
            event="fallback",
            target=target_label,
            yaw_deg=yaw_deg,
        )
        return yaw_deg

    def _move_to_grasp_observe_pose(self, context):
        ok = self._move_to_pose(
            "grasp point observe offset move",
            context["plan"].target_x - TOOL_OBSERVE_X_BACKOFF_M,
            context["plan"].target_y,
            context["plan"].drawer_wall_clearance_z,
            context["ori"],
            "grasp point observe offset move failed. Pick sequence aborted.",
            "grasp point observe offset move failed",
            "observe_offset_move_failed",
            collision_scene_key="pick/observe_offset_move",
        )
        if not ok:
            return False

        self.state._publish_robot_status(
            "observing_pick_target",
            tool_name=self.state.target_label,
            action="bring",
            message=(
                f"{self.state.target_label} grasp point observe pose에서 "
                "SAM 추적을 시작합니다."
            ),
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
            log_warn(
                self.state.logger(),
                "grasp mask image generation failed",
                step="grasp_mask",
                event="fail",
                target=target_label,
                reason=str(exc) or type(exc).__name__,
            )

    def _rotate_wrist(self, vlm_yaw_deg, context):
        log = self.state.logger()
        ok = self.motion.rotate_wrist_by_yaw_deg(
            vlm_yaw_deg,
            log,
            collision_scene_key="pick/apply_wrist_yaw",
        )
        if ok:
            context["ori"] = current_ee_orientation(self.robot)
            return True

        if self._interrupted():
            return False

        log_error(log, "wrist rotation failed", step="wrist", event="fail")
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
            message=f"{self.state.target_label} 파지를 위해 XY 정렬 후 Z 하강합니다.",
            command=self.state.current_command,
        )
        if not self._move_to_pose(
            "4단계: 파지 XY 위치 정렬",
            plan.target_x,
            plan.target_y,
            plan.drawer_wall_clearance_z,
            ori,
            "파지 XY 위치 정렬 실패. Pick 시퀀스 중단",
            "파지 XY 위치 정렬 실패",
            "grasp_xy_move_failed",
            collision_scene_key="pick/grasp_xy_align",
        ):
            return False

        if not plan.should_descend_to_grasp:
            log_info(
                self.state.logger(),
                "grasp descent skipped",
                step="grasp_descent",
                event="skip",
                reason="same_height",
                drawer_wall_clearance_z=plan.drawer_wall_clearance_z,
                grasp_z=plan.grasp_z,
            )
            return True

        return self._move_to_pose(
            "4단계: 파지 Z 높이 하강",
            plan.target_x,
            plan.target_y,
            plan.grasp_z,
            ori,
            "파지 높이 하강 실패. Pick 시퀀스를 중단합니다.",
            "파지 높이 하강 실패",
            "grasp_descent_failed",
            collision_scene_key="pick/grasp_descent",
        )

    def _grasp_tool(self, context):
        log = self.state.logger()
        log_info(log, "robot grasp", step="grasp", event="start")
        if self.grasp.try_robot_grasp(log):
            context["grasp_wrist_joint_rad"] = self._current_wrist_joint_rad(log)
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

        log_error(log, "robot grasp failed", step="grasp", event="fail")
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

        log_info(log, "pregrasp depth measurement", step="pregrasp", event="start")
        measurement = self.grasp.measure_pregrasp_depth(log)
        if measurement is None:
            log_error(log, "pregrasp depth unavailable", step="pregrasp", event="fail")
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
        log_info(
            log,
            "pregrasp descent calculated",
            step="pregrasp",
            event="calculated",
            width_mm=width_mm,
            depth_mm=depth_mm,
            extra_descent_m=extra_descent_m,
            target_z=target_z,
            drawer_safe_z_min=drawer_safe_z_min,
            redescend_min_z=redescend_min_z,
        )

        self.gripper.open_gripper()
        cooperative_wait(PICK_PREGRASP_REOPEN_WAIT_SEC)

        if actual_descent_m <= 0.0:
            log_info(
                log,
                "pregrasp descent skipped",
                step="pregrasp",
                event="skip",
                reason="z_limit",
            )
            return True

        return self._move_to_pose(
            "4.5단계: pre-grasp actual depth 기반 추가 하강",
            plan.target_x,
            plan.target_y,
            target_z,
            ori,
            "pre-grasp 추가 하강 실패. Pick 시퀀스를 중단합니다.",
            "pre-grasp 추가 하강 실패",
            "pregrasp_descent_failed",
            min_z=redescend_min_z,
            collision_scene_key="pick/pregrasp_depth_adjust",
        )

    def _wait_tool_mask_lock(self):
        log = self.state.logger()
        log_info(log, "wait for tool mask lock", step="tool_mask_lock", event="start")
        if self.handoff.wait_for_tool_mask_lock(log):
            return True

        if self._interrupted():
            return False

        log_error(log, "tool mask lock failed", step="tool_mask_lock", event="fail")
        self.state._publish_robot_status(
            "failed",
            message="공구 mask lock에 실패했습니다.",
            reason="tool_mask_lock_failed",
            command=self.state.current_command,
        )
        return False

    def _move_to_handoff(self, plan, context):
        log = self.state.logger()
        while True:
            handoff_pose = self.handoff.move_to_handoff_pose(log)
            if handoff_pose[0] is not None:
                return True

            if self._interrupted():
                return False

            if self.handoff.last_failure_reason != "handoff_search_failed":
                break

            decision = self._wait_for_handoff_decision(
                "handoff_search_failed",
                log,
            )
            if decision == "retry":
                log_info(
                    log,
                    "handoff pose retry requested",
                    step="handoff",
                    event="retry",
                )
                continue
            if decision == "interrupted":
                return False
            break

        log_error(log, "handoff pose unavailable", step="handoff", event="fail")
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
                "사용자 위치 확인 실패로 공구를 원래 위치에 반환하고 서랍을 닫았습니다."
                if returned and drawer_closed and home_ok
                else "사용자 위치 확인 실패 후 서랍은 닫았지만 Home 복귀는 실패했습니다."
                if returned and drawer_closed
                else "사용자 위치 확인 실패 후 공구 반환은 했지만 서랍 닫기는 실패했습니다."
                if returned
                else "사용자 위치 확인과 공구 반환 모두 실패했습니다."
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
        while True:
            log_info(log, "wait for human grasp", step="human_grasp", event="start")
            self.state._publish_robot_status(
                "waiting_handoff",
                message="사용자 잡기 인식을 기다립니다.",
                command=self.state.current_command,
            )
            if self.handoff.wait_for_human_grasp(log):
                return True

            if self._interrupted():
                return False

            if self.handoff.last_failure_reason != "handoff_timeout":
                break

            decision = self._wait_for_handoff_decision("handoff_timeout", log)
            if decision == "retry":
                log_info(
                    log,
                    "human grasp wait retry requested",
                    step="human_grasp",
                    event="retry",
                )
                continue
            if decision == "interrupted":
                return False
            break

        log_error(log, "handoff pose unavailable", step="handoff", event="fail")
        returned, drawer_closed, home_ok = self._return_tool_close_drawer_home(
            plan,
            context,
            log,
        )
        status = "returned" if returned and drawer_closed and home_ok else "failed"
        self.state._publish_robot_status(
            status,
            message=(
                "사용자 손 인식 실패로 공구를 원래 위치에 반환하고 서랍을 닫았습니다."
                if returned and drawer_closed and home_ok
                else "사용자 손 인식 실패 후 서랍은 닫았지만 Home 복귀는 실패했습니다."
                if returned and drawer_closed
                else "사용자 손 인식 실패 후 공구 반환은 했지만 서랍 닫기는 실패했습니다."
                if returned
                else "사용자 손 인식과 공구 반환 모두 실패했습니다."
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

    def _wait_for_handoff_decision(self, reason, log):
        retry_req = self.control_events.get("handoff_retry")
        fallback_req = self.control_events.get("handoff_fallback")
        pending = self.control_events.get("handoff_decision_pending")
        if retry_req is None or fallback_req is None or pending is None:
            return "fallback"

        retry_req.clear()
        fallback_req.clear()
        pending.set()
        self.state._publish_robot_status(
            HANDOFF_DECISION_PENDING_STATUS,
            action="bring",
            message="사용자의 손을 인식하지 못했습니다. 다시 인식할까요, 복귀할까요?",
            reason=reason,
            command=self.state.current_command,
        )
        log_warn(
            log,
            "handoff inspection decision pending",
            step="handoff_decision",
            event="pending",
            reason=reason,
            options="retry/fallback",
        )

        try:
            while rclpy.ok():
                if self._interrupted():
                    return "interrupted"
                if retry_req.is_set():
                    retry_req.clear()
                    return "retry"
                if fallback_req.is_set():
                    fallback_req.clear()
                    return "fallback"
                cooperative_wait(HANDOFF_DECISION_POLL_SEC)
        finally:
            pending.clear()

        return "interrupted"

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
            grasp_wrist_joint_rad=context.get("grasp_wrist_joint_rad"),
        )
        if not returned:
            return False, False, False

        if self.drawer_flow is None or drawer_id is None:
            home_ok = self.handoff.move_home_after_handoff(
                log,
                publish_on_failure=False,
            )
            return True, True, home_ok

        log_info(log, "close drawer after recovery", step="close_drawer", event="start", drawer_id=drawer_id)
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
        log_info(self.state.logger(), "release to human", step="release", event="start")
        if self.tool_hold_monitor is not None:
            self.tool_hold_monitor.stop("handoff_release")
        self.gripper.open_gripper()
        cooperative_wait(HANDOFF_RELEASE_WAIT_SEC)
        return True

    def _home_before_close_drawer(self):
        log_info(
            self.state.logger(),
            "home before close drawer",
            step="home_before_close_drawer",
            event="start",
        )
        return self.handoff.move_home_after_handoff(
            self.state.logger(),
            collision_scene_key="handoff/home_before_close_drawer",
        )

    def _home_after_handoff(self):
        log_info(self.state.logger(), "home after handoff", step="home", event="start")
        return self.handoff.move_home_after_handoff(self.state.logger())

    def _current_wrist_joint_rad(self, log):
        reader = getattr(self.motion, "current_wrist_joint_rad", None)
        if reader is None:
            return None
        return reader(log)

    def _close_drawer_after_handoff(self, context):
        drawer_id = context.get("drawer_id")
        if self.drawer_flow is None or drawer_id is None:
            return True

        log = self.state.logger()
        log_info(log, "close drawer after handoff", step="close_drawer", event="start", drawer_id=drawer_id)
        self.state._publish_robot_status(
            "closing_drawer",
            tool_name=self.state.target_label,
            action="bring",
            message=f"{self.state.target_label}가 있던 서랍을 닫습니다.",
            command=self.state.current_command,
        )
        return self.drawer_flow.close_drawer(drawer_id, log)

    def _publish_done(self):
        log_info(self.state.logger(), "pick sequence done", step="done", event="done")
        self.state._publish_robot_status(
            "done",
            message="공구 전달 및 Home 복귀까지 완료했습니다.",
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
