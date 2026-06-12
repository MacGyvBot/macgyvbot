"""Bring sequence step construction."""

import time

from macgyvbot_config.timing import CAMERA_LOOP_IDLE_SLEEP_SEC
from macgyvbot_task.application.task_control.task_step import TaskStep


class BringSequenceRunner:
    """Build bring workflow steps for task-queue execution."""

    def __init__(
        self,
        *,
        state,
        drawer_flow,
        return_perception,
        frame_processor,
        detector,
        pick_target_resolver,
        pick_runner,
        publish_robot_status,
        task_log,
        interrupted,
        append_task_steps,
        has_queued_task_steps,
        recover_after_drawer_validation_failure,
    ):
        self.state = state
        self.drawer_flow = drawer_flow
        self.return_perception = return_perception
        self.frame_processor = frame_processor
        self.detector = detector
        self.pick_target_resolver = pick_target_resolver
        self.pick_runner = pick_runner
        self.publish_robot_status = publish_robot_status
        self.task_log = task_log
        self.interrupted = interrupted
        self.append_task_steps = append_task_steps
        self.has_queued_task_steps = has_queued_task_steps
        self.recover_after_drawer_validation_failure = (
            recover_after_drawer_validation_failure
        )

    def build_steps(self, tool_name):
        drawer_id = self.drawer_flow.drawer_id_for_tool(tool_name)
        steps = []
        if drawer_id is not None:
            steps.extend(self._drawer_prepare_steps(tool_name, drawer_id))

        steps.append(
            TaskStep(
                "bring/search_target",
                lambda tool=tool_name: self._search_target_and_queue_pick(tool),
            )
        )
        return steps

    def _drawer_prepare_steps(self, tool_name, drawer_id):
        return [
            TaskStep(
                "bring/drawer_prepare_handle_target",
                lambda tool=tool_name, drawer=drawer_id: (
                    self._prepare_drawer_handle_target_step(tool, drawer)
                ),
            ),
            TaskStep(
                "bring/drawer_handle_preapproach",
                lambda tool=tool_name, drawer=drawer_id: (
                    self._drawer_prepare_motion_step(
                        tool,
                        drawer,
                        self.drawer_flow.move_to_open_handle_preapproach,
                    )
                ),
            ),
            TaskStep(
                "bring/drawer_handle_pose",
                lambda tool=tool_name, drawer=drawer_id: (
                    self._drawer_prepare_motion_step(
                        tool,
                        drawer,
                        self.drawer_flow.move_to_open_handle_pose,
                    )
                ),
            ),
            TaskStep(
                "bring/drawer_grip_handle",
                lambda tool=tool_name, drawer=drawer_id: (
                    self._drawer_prepare_motion_step(
                        tool,
                        drawer,
                        self.drawer_flow.grip_open_handle,
                    )
                ),
            ),
            TaskStep(
                "bring/drawer_pull_open",
                lambda tool=tool_name, drawer=drawer_id: (
                    self._drawer_pull_open_step(tool, drawer)
                ),
            ),
            TaskStep(
                "bring/drawer_release_handle",
                lambda tool=tool_name, drawer=drawer_id: (
                    self._drawer_prepare_motion_step(
                        tool,
                        drawer,
                        self.drawer_flow.release_open_handle,
                    )
                ),
            ),
            TaskStep(
                "bring/drawer_observe",
                lambda tool=tool_name, drawer=drawer_id: (
                    self._observe_pick_drawer_step(tool, drawer)
                ),
            ),
            TaskStep(
                "bring/drawer_validate_contents",
                lambda tool=tool_name, drawer=drawer_id: (
                    self._validate_pick_drawer_step(tool, drawer)
                ),
            ),
        ]

    def _prepare_drawer_handle_target_step(self, target_label, drawer_id):
        self.state.target_label = target_label
        self.state.target_tool = target_label
        self.state.drawer_preparing_tool = target_label
        self.publish_robot_status(
            "opening_drawer",
            tool_name=target_label,
            action="bring",
            message=f"{target_label}가 들어있는 서랍을 엽니다.",
            command=self.state.current_command,
        )
        log = self.task_log("drawer", quiet_info=True)
        log.info(
            "preparing drawer before target search",
            step="drawer_prepare",
            event="start",
            target=target_label,
            drawer=drawer_id,
        )
        return self._drawer_prepare_motion_step(
            target_label,
            drawer_id,
            self.drawer_flow.prepare_open_handle_target,
        )

    def _drawer_prepare_motion_step(self, target_label, drawer_id, motion_step):
        log = self.task_log("drawer", quiet_info=True)
        ok = motion_step(drawer_id, log)
        if not ok and not self.interrupted():
            self.publish_robot_status(
                "failed",
                tool_name=target_label,
                action="bring",
                message=f"{target_label} 탐색 전 서랍 준비에 실패했습니다.",
                reason="drawer_prepare_failed",
                command=self.state.current_command,
            )
        return ok

    def _drawer_pull_open_step(self, target_label, drawer_id):
        ok = self._drawer_prepare_motion_step(
            target_label,
            drawer_id,
            self.drawer_flow.pull_open_drawer,
        )
        if ok:
            self.state.drawer_open = True
            self.state.opened_drawer_id = drawer_id
        return ok

    def _observe_pick_drawer_step(self, target_label, drawer_id):
        self.publish_robot_status(
            "observing_drawer",
            tool_name=target_label,
            action="bring",
            message=f"{target_label} 탐색을 위해 서랍 내부를 관찰합니다.",
            command=self.state.current_command,
        )
        log = self.task_log("drawer", quiet_info=True)
        ok = self.drawer_flow.observe_drawer(drawer_id, log)
        if ok or self.interrupted():
            return ok

        self.publish_robot_status(
            "failed",
            tool_name=target_label,
            action="bring",
            message=f"{target_label} 탐색 전 서랍 내부 관찰에 실패했습니다.",
            reason="pick_drawer_observe_failed",
            command=self.state.current_command,
        )
        self.recover_after_drawer_validation_failure(
            target_label,
            drawer_id,
            log,
            "pick_drawer_observe_failed",
        )
        return False

    def _validate_pick_drawer_step(self, target_label, drawer_id):
        log = self.task_log("drawer", quiet_info=True)
        if not self._validate_pick_drawer_ownership(target_label, drawer_id, log):
            return False

        self.state.target_label = target_label
        self.state.target_tool = target_label
        self.state.drawer_prepared_tool = target_label
        self.state.drawer_preparing_tool = None
        self._publish_searching_status(target_label)
        return True

    def _validate_pick_drawer_ownership(self, target_label, drawer_id, log):
        observed_tools = self.return_perception.detect_drawer_tool_labels()
        if observed_tools is None:
            self.publish_robot_status(
                "failed",
                tool_name=target_label,
                action="bring",
                message=f"{target_label} 서랍 내부 공구 상태를 확인하지 못했습니다.",
                reason="pick_drawer_occupancy_unknown",
                command=self.state.current_command,
            )
            self.recover_after_drawer_validation_failure(
                target_label,
                drawer_id,
                log,
                "pick_drawer_occupancy_unknown",
            )
            return False

        conflicting_tools = [
            observed_tool
            for observed_tool in observed_tools
            if observed_tool != target_label
        ]
        if target_label in observed_tools and not conflicting_tools:
            log.info(
                "pick drawer ownership validated",
                step="drawer_ownership",
                event="valid",
                target=target_label,
                drawer=drawer_id,
                observed_tools=observed_tools,
            )
            return True

        reason = "pick_drawer_tool_not_found"
        if conflicting_tools:
            reason = "pick_drawer_tool_mismatch"
        observed_text = ", ".join(observed_tools) if observed_tools else "none"
        log.warn(
            "pick drawer ownership mismatch",
            step="drawer_ownership",
            event="fail",
            expected_tool=target_label,
            observed_tool=observed_text,
            drawer_id=drawer_id,
            reason=reason,
        )
        self.publish_robot_status(
            "failed",
            tool_name=target_label,
            action="bring",
            message=(
                f"{target_label} 서랍에서 기대 공구를 찾지 못했습니다. "
                f"감지된 공구: {observed_text}"
            ),
            reason=reason,
            command=self.state.current_command,
        )
        self.recover_after_drawer_validation_failure(
            target_label,
            drawer_id,
            log,
            reason,
        )
        return False

    def _publish_searching_status(self, target_label):
        self.state._last_search_status_target = target_label
        self.publish_robot_status(
            "searching",
            tool_name=target_label,
            action="bring",
            message=f"{target_label} 탐색을 시작합니다.",
            command=self.state.current_command,
        )

    def _search_target_and_queue_pick(self, target_label):
        self.state.target_label = target_label
        self.state.target_tool = target_label
        if self.state._last_search_status_target != target_label:
            self._publish_searching_status(target_label)

        while not self.interrupted():
            if self.has_queued_task_steps("pick"):
                return True
            if not self.frame_processor.has_camera_state():
                time.sleep(CAMERA_LOOP_IDLE_SLEEP_SEC)
                continue

            color_image = self.state.color_image.copy()
            depth_image = self.state.depth_image.copy()
            intrinsics = dict(self.state.intrinsics)
            results = self.detector.detect(color_image)
            boxes = results[0].boxes if results else None
            target = self.pick_target_resolver.target_from_boxes(
                boxes,
                target_label,
                color_image,
                depth_image,
                intrinsics,
                use_bbox_center=(
                    self.pick_target_resolver.should_refine_grasp_point_at_top_view()
                ),
            )
            if target.found:
                if self.interrupted():
                    return False
                bx, by, bz = target.base_xyz
                self._append_pick_steps_for_target(
                    target_label,
                    bx,
                    by,
                    bz,
                    target.yaw_deg,
                )
                return True

            if target.reason == "target_not_found":
                self.task_log("pick", quiet_info=True).debug(
                    "target search waiting",
                    step="search",
                    event="wait",
                    target=target_label,
                    reason=target.reason,
                )
            time.sleep(CAMERA_LOOP_IDLE_SLEEP_SEC)

        return False

    def _append_pick_steps_for_target(
        self,
        target_label,
        bx,
        by,
        bz,
        vlm_yaw_deg=None,
    ):
        self.state.target_label = target_label
        self.state.target_tool = target_label
        if self.has_queued_task_steps("pick"):
            return True

        steps = self.pick_runner.build_steps(bx, by, bz, vlm_yaw_deg)
        if self.has_queued_task_steps("pick"):
            return True
        self.append_task_steps("pick", steps)
        self.publish_robot_status(
            "picking",
            tool_name=target_label,
            action="bring",
            message=f"{target_label} pick 동작을 시작합니다.",
            command=self.state.current_command,
        )
        return True
