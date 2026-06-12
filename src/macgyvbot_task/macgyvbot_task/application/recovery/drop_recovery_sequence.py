"""Drop recovery sequence step construction."""

from __future__ import annotations

import math

from macgyvbot_manipulation.grasp_verifier import GraspVerifier
from macgyvbot_manipulation.robot_pose import (
    current_ee_orientation,
    make_safe_pose,
    normalize_angle_deg,
)
from macgyvbot_task.application.logging_utils import (
    log_error,
    log_info,
    log_warn,
)
from macgyvbot_task.application.recovery.drop_recovery import (
    _detect_target,
    _fail,
    _monitor_action,
    _redetect_target,
    _select_target_tool,
    _update_return_target_from_observed_label,
)
from macgyvbot_task.application.recovery.recovery_utils import (
    _pregrasp_depth_adjust_for_recovery,
    _set_limited_gripper_width_for_recovery,
    cleanup_after_recovery,
    clear_recovery_perception_lock,
    is_graspable,
    log_recovery_event,
    move_to_inspection_pose,
    open_gripper_after_inspection,
    publish_recovery_grasp_success,
    publish_recovery_status,
    recovery_restart_requested,
    set_recovery_mode,
    wait_for_recovery_tool_mask_lock,
)
from macgyvbot_task.application.pick_flow.pick_target_planner import PickTargetPlanner
from macgyvbot_task.application.task_control.task_step import TaskStep


class DropRecoverySequenceRunner:
    """Build drop recovery workflow steps for task-queue execution."""

    def __init__(
        self,
        *,
        status,
        motion_controller,
        gripper,
        config,
        logger,
        task_type,
    ):
        self.status = status
        self.motion_controller = motion_controller
        self.gripper = gripper
        self.config = config
        self.logger = logger
        self.task_type = task_type
        self.target_tool = _select_target_tool(status, config, task_type)
        self.detection = None
        self.grasp_plan = None
        self.grasp_ori = None
        self.grasp_wrist_target_rad = None

    def build_steps(self):
        return [
            TaskStep(
                "recovery/start",
                self._start,
                retry_on_pause=False,
            ),
            TaskStep(
                "recovery/move_to_inspection_pose",
                self._move_to_inspection_pose,
            ),
            TaskStep(
                "recovery/open_gripper_after_inspection",
                self._open_gripper_after_inspection,
            ),
            TaskStep(
                "recovery/detect_target",
                self._detect_initial_target,
            ),
            TaskStep(
                "recovery/move_to_target_observe",
                self._move_to_target_observe,
            ),
            TaskStep(
                "recovery/redetect_target",
                self._redetect_target,
            ),
            TaskStep(
                "recovery/check_graspability",
                self._check_graspability,
            ),
            TaskStep(
                "recovery/prepare_grasp_plan_and_yaw",
                self._prepare_grasp_plan_and_yaw,
            ),
            TaskStep(
                "recovery/apply_grasp_yaw",
                self._apply_grasp_yaw,
            ),
            TaskStep(
                "recovery/execute_grasp",
                self._execute_grasp,
            ),
            TaskStep(
                "recovery/wait_tool_mask_lock",
                self._wait_tool_mask_lock,
            ),
            TaskStep(
                "recovery/cleanup",
                self._cleanup,
            ),
        ]

    def _start(self):
        log_recovery_event(
            self.logger,
            "RECOVERY_STARTED",
            f"{self.task_type} recovery started",
            {"task_type": self.task_type, "target_tool": self.target_tool},
        )
        set_recovery_mode(self.status, True)
        clear_recovery_perception_lock(self.status, self.logger)
        publish_recovery_status(
            self.status,
            "recovering",
            self.target_tool,
            "drop recovery started.",
            reason="drop_recovery_started",
        )
        return True

    def _move_to_inspection_pose(self):
        publish_recovery_status(
            self.status,
            "recovering",
            self.target_tool,
            "Moving to inspection pose to find the dropped tool.",
            reason="moving_to_recovery_inspection",
        )
        return self._run_queue_step(
            "moving_to_recovery_inspection",
            lambda: move_to_inspection_pose(self.motion_controller, self.config),
            "motion_planning_failed",
            "PLANNING_FAILED",
            f"{self.task_type} recovery inspection pose move failed",
        )

    def _open_gripper_after_inspection(self):
        return self._run_queue_step(
            "inspection_gripper_open",
            lambda: open_gripper_after_inspection(
                self.gripper,
                self.config,
                self.logger,
            ),
            "inspection_gripper_open_failed",
            "RECOVERY_FAILED",
            f"{self.task_type} recovery gripper open after inspection failed",
        )

    def _detect_initial_target(self):
        publish_recovery_status(
            self.status,
            "recovering",
            self.target_tool,
            "Searching dropped tool bbox again.",
            reason="detecting_recovery_target",
        )

        def detect_initial_target():
            self.detection = _detect_target(self.config, self.target_tool)
            return self.detection is not None

        ok = self._run_queue_step(
            "detecting_recovery_target",
            detect_initial_target,
            "target_detection_failed",
            "TARGET_NOT_FOUND",
            f"{self.task_type} recovery target not found",
        )
        if not ok:
            self.detection = None
        return ok

    def _move_to_target_observe(self):
        if self.config.target_observe_fn is None:
            return True

        publish_recovery_status(
            self.status,
            "recovering",
            self.target_tool,
            "Moving to target observe pose.",
            reason="moving_to_recovery_target_observe",
        )
        return self._run_queue_step(
            "moving_to_recovery_target_observe",
            lambda: self.config.target_observe_fn(
                self.detection,
                self.target_tool,
                self.logger,
            ),
            "target_observe_move_failed",
            "PLANNING_FAILED",
            f"{self.task_type} recovery target observe pose move failed",
        )

    def _redetect_target(self):
        if self.config.target_observe_fn is None:
            return True

        if self.task_type == "return":
            self.target_tool = _update_return_target_from_observed_label(
                self.status,
                self.config,
                self.logger,
                self.target_tool,
            )

        def redetect_target():
            self.detection = _redetect_target(self.config, self.target_tool)
            return self.detection is not None

        ok = self._run_queue_step(
            "redetecting_recovery_target",
            redetect_target,
            "target_redetection_failed",
            "TARGET_NOT_FOUND",
            f"{self.task_type} recovery target not found at observe pose",
        )
        if not ok:
            self.detection = None
            return False

        publish_recovery_status(
            self.status,
            "recovering",
            self.target_tool,
            "Fresh bbox/depth target and yaw PCA were recomputed.",
            reason="recovery_target_redetected",
        )
        return True

    def _check_graspability(self):
        return self._run_queue_step(
            "checking_recovery_graspability",
            lambda: is_graspable(
                self.detection,
                self.motion_controller,
                None,
                self.config,
            ),
            "graspability_check_failed",
            "GRASPABILITY_CHECK_FAILED",
            f"{self.task_type} recovery target is not graspable",
        )

    def _prepare_grasp_plan_and_yaw(self):
        publish_recovery_status(
            self.status,
            "recovering",
            self.target_tool,
            "Preparing recovery grasp plan and yaw target.",
            reason="recovery_grasp_plan_started",
        )

        return self._run_queue_step(
            "recovery_grasp_plan_started",
            self._prepare_grasp_plan_and_yaw_action,
            "grasp_execution_failed",
            "RECOVERY_FAILED",
            f"{self.task_type} recovery grasp planning failed",
        )

    def _prepare_grasp_plan_and_yaw_action(self):
        base_xyz = getattr(self.detection, "base_xyz", None)
        if base_xyz is None:
            return False

        robot = getattr(self.config, "robot", None)
        if robot is None:
            log_error(
                self.logger,
                "recovery grasp failed",
                step="recovery",
                event="fail",
                reason="robot_missing",
            )
            return False

        self.grasp_plan = PickTargetPlanner(robot).plan(*base_xyz, self.logger)
        self.grasp_ori = current_ee_orientation(robot)
        self.grasp_wrist_target_rad = None
        yaw_deg = getattr(self.detection, "yaw_deg", None)
        if yaw_deg is None:
            return True

        try:
            yaw_deg = float(yaw_deg)
        except (TypeError, ValueError):
            log_warn(
                self.logger,
                "recovery grasp yaw invalid",
                step="recovery_wrist",
                event="prepare_fail",
                target=self.target_tool,
                yaw_deg=yaw_deg,
            )
            return False

        if not math.isfinite(yaw_deg):
            log_warn(
                self.logger,
                "recovery grasp yaw nonfinite",
                step="recovery_wrist",
                event="prepare_fail",
                target=self.target_tool,
                yaw_deg=yaw_deg,
            )
            return False

        yaw_deg = normalize_angle_deg(yaw_deg)
        if abs(yaw_deg) < 0.1:
            log_info(
                self.logger,
                "recovery grasp yaw rotation skipped",
                step="recovery_wrist",
                event="prepared",
                target=self.target_tool,
                reason="yaw_near_zero",
                yaw_deg=yaw_deg,
            )
            return True

        current_wrist_rad = self._current_wrist_joint_rad()
        if current_wrist_rad is None:
            return False

        self.grasp_wrist_target_rad = float(current_wrist_rad) + math.radians(yaw_deg)
        log_info(
            self.logger,
            "recovery grasp wrist target prepared",
            step="recovery_wrist",
            event="prepared",
            target=self.target_tool,
            yaw_deg=yaw_deg,
            current_wrist_deg=math.degrees(float(current_wrist_rad)),
            target_wrist_deg=math.degrees(self.grasp_wrist_target_rad),
        )
        return True

    def _apply_grasp_yaw(self):
        return self._run_queue_step(
            "recovery_grasp_yaw",
            self._apply_grasp_yaw_action,
            "grasp_yaw_failed",
            "RECOVERY_FAILED",
            f"{self.task_type} recovery grasp yaw failed",
        )

    def _apply_grasp_yaw_action(self):
        if self.grasp_wrist_target_rad is None:
            return True

        move_wrist = getattr(self.motion_controller, "move_wrist_to_joint_rad", None)
        if move_wrist is None:
            log_error(
                self.logger,
                "recovery wrist joint target move unavailable",
                step="recovery_wrist",
                event="fail",
                target=self.target_tool,
            )
            return False

        if not move_wrist(
            self.grasp_wrist_target_rad,
            self.logger,
            collision_scene_key="recovery/apply_wrist_yaw",
        ):
            return False

        self.grasp_ori = current_ee_orientation(getattr(self.config, "robot", None))
        return True

    def _execute_grasp(self):
        publish_recovery_status(
            self.status,
            "recovering",
            self.target_tool,
            "Attempting recovery grasp.",
            reason="recovery_grasp_started",
        )
        ok = self._run_queue_step(
            "recovery_grasp_started",
            self._execute_grasp_action,
            "grasp_execution_failed",
            "GRASP_ATTEMPT_FAILED",
            f"{self.task_type} recovery grasp failed",
        )
        if not ok:
            return False

        self.status.held_tool = self.target_tool
        self.status.gripper_holding = True
        publish_recovery_grasp_success(self.status, self.config, self.target_tool)
        return True

    def _execute_grasp_action(self):
        if self.grasp_plan is None or self.grasp_ori is None:
            return False

        if not _set_limited_gripper_width_for_recovery(
            self.gripper,
            self.config,
            self.target_tool,
            self.logger,
        ) and hasattr(self.gripper, "open_gripper"):
            self.gripper.open_gripper()

        if not self.motion_controller.plan_and_execute(
            self.logger,
            pose_goal=make_safe_pose(
                self.grasp_plan.target_x,
                self.grasp_plan.target_y,
                self.grasp_plan.drawer_wall_clearance_z,
                self.grasp_ori,
                self.logger,
            ),
            collision_scene_key="recovery/grasp_xy_align",
        ):
            return False

        if (
            self.grasp_plan.should_descend_to_grasp
            and not self.motion_controller.plan_and_execute(
                self.logger,
                pose_goal=make_safe_pose(
                    self.grasp_plan.target_x,
                    self.grasp_plan.target_y,
                    self.grasp_plan.grasp_z,
                    self.grasp_ori,
                    self.logger,
                ),
                collision_scene_key="recovery/grasp_descent",
            )
        ):
            return False

        if not _pregrasp_depth_adjust_for_recovery(
            self.grasp_plan,
            self.grasp_ori,
            self.motion_controller,
            self.gripper,
            self.config,
            self.target_tool,
            self.logger,
        ):
            return False

        verifier = GraspVerifier(
            self.gripper,
            getattr(self.config, "wait_fn", None),
            interrupted=self._paused_or_exiting,
        )
        return bool(verifier.try_grasp(self.logger, failure_prefix="recovery grasp"))

    def _current_wrist_joint_rad(self):
        current_wrist = getattr(self.motion_controller, "current_wrist_joint_rad", None)
        if current_wrist is None:
            log_error(
                self.logger,
                "recovery current wrist joint unavailable",
                step="recovery_wrist",
                event="prepare_fail",
                target=self.target_tool,
            )
            return None
        return current_wrist(self.logger)

    def _wait_tool_mask_lock(self):
        ok = self._run_queue_step(
            "recovery_tool_mask_lock",
            lambda: wait_for_recovery_tool_mask_lock(
                self.config,
                self.logger,
                self.target_tool,
            ),
            "tool_mask_lock_failed",
            "RECOVERY_FAILED",
            f"{self.task_type} recovery tool mask lock failed after grasp",
        )
        if not ok:
            return False

        if self.config.tool_hold_monitor is not None:
            self.config.tool_hold_monitor.start(
                self.target_tool,
                _monitor_action(self.task_type),
                self.config.command,
            )
        return True

    def _cleanup(self):
        if recovery_restart_requested(self.config) or self._paused_or_exiting():
            return False

        cleanup_ok = cleanup_after_recovery(
            self.status,
            self.motion_controller,
            self.config,
            self.logger,
            self.task_type,
            self.target_tool,
            reason="recovery_succeeded",
            finish_recovery_mode=False,
        )
        if recovery_restart_requested(self.config) or self._paused_or_exiting():
            return False

        log_recovery_event(
            self.logger,
            "RECOVERY_SUCCEEDED" if cleanup_ok else "RECOVERY_FAILED",
            f"{self.task_type} recovery finished",
            {
                "task_type": self.task_type,
                "target_tool": self.target_tool,
                "cleanup_ok": cleanup_ok,
            },
        )
        return bool(cleanup_ok)

    def _run_queue_step(self, reason, step_fn, failure_reason, event_type, message):
        if recovery_restart_requested(self.config) or self._paused_or_exiting():
            return False

        step_ok = bool(step_fn())
        if recovery_restart_requested(self.config) or self._paused_or_exiting():
            return False
        if step_ok:
            return True

        return _fail(
            self.status,
            self.motion_controller,
            self.config,
            self.logger,
            self.task_type,
            self.target_tool,
            failure_reason,
            event_type,
            message,
        )

    def _paused_or_exiting(self):
        return _event_is_set(self.config.pause_event) or _event_is_set(
            self.config.exit_event
        )


def build_drop_recovery_steps(
    status,
    motion_controller,
    gripper,
    config,
    logger,
    task_type,
):
    runner = DropRecoverySequenceRunner(
        status=status,
        motion_controller=motion_controller,
        gripper=gripper,
        config=config,
        logger=logger,
        task_type=task_type,
    )
    return runner.build_steps()


def _event_is_set(event):
    return event is not None and bool(event.is_set())
