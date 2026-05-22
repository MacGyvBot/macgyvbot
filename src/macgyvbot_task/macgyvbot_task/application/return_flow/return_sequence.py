"""Return sequence step construction for receiving and storing a user-held tool."""

import time

import rclpy

from macgyvbot_config.timing import SEQUENCE_WAIT_POLL_SEC
from macgyvbot_manipulation.handover_targeting import move_to_observation_pose
from macgyvbot_task.application.return_flow.return_handoff_flow import (
    ReturnHandoffFlow,
)
from macgyvbot_task.application.return_flow.return_home_placement_flow import (
    ReturnHomePlacementFlow,
)
from macgyvbot_task.application.return_flow.return_target_resolver import (
    RETURN_SOURCE_HAND,
    ReturnTargetResolver,
)
from macgyvbot_task.application.status.return_status_reporter import (
    ReturnStatusReporter,
)
from macgyvbot_task.application.task_control.task_step import TaskStep


class ReturnSequenceRunner:
    """Build return workflow steps for camera-gated hand return targets."""

    def __init__(
        self,
        robot,
        motion_controller,
        gripper,
        state,
        tool_hold_monitor=None,
        control_events=None,
        drawer_flow=None,
        detect_home_tool_label=None,
    ):
        self.robot = robot
        self.motion = motion_controller
        self.gripper = gripper
        self.state = state
        self.tool_hold_monitor = tool_hold_monitor
        self.control_events = control_events or {}
        self.drawer_flow = drawer_flow
        self.detect_home_tool_label = detect_home_tool_label or (lambda: None)
        self.reporter = ReturnStatusReporter(state)
        self.target_resolver = ReturnTargetResolver(
            state,
            self._cooperative_wait,
        )
        self.handoff = ReturnHandoffFlow(
            robot,
            motion_controller,
            gripper,
            state,
            self.reporter,
            self._cooperative_wait,
            tool_hold_monitor,
            interrupted=self._interrupted,
        )
        self.placement = ReturnHomePlacementFlow(
            robot,
            motion_controller,
            gripper,
            state,
            self.reporter,
            self._cooperative_wait,
            tool_hold_monitor,
            interrupted=self._interrupted,
        )

    def build_steps(self, command):
        context = {
            "command": command,
            "requested_tool": command.get("tool_name", "unknown"),
            "tool_name": None,
            "observed_tool": None,
            "drawer_id": None,
            "ori": self.state.home_ori,
        }
        return [
            TaskStep("return/prepare", lambda: self._prepare(context)),
            TaskStep("return/receive_tool", lambda: self._receive_tool(context)),
            TaskStep("return/place_home", lambda: self._place_home(context)),
            TaskStep(
                "return/resolve_home_observed_drawer",
                lambda: self._resolve_home_observed_drawer(context),
            ),
            TaskStep(
                "return/open_close_home_observed_drawer",
                lambda: self._open_close_home_observed_drawer(context),
            ),
            TaskStep(
                "return/done",
                lambda: self._publish_done(context),
                retry_on_pause=False,
            ),
        ]

    def _prepare(self, context):
        command = context["command"]
        requested_tool = context["requested_tool"]
        raw_text = command.get("raw_text", "")

        self.state.human_grasped_tool = False
        self.state.last_grasp_result = None
        self.reporter.publish(
            "waiting_return_handoff",
            requested_tool,
            "사용자 반납 공구를 받을 준비를 시작합니다.",
            command,
        )
        self.state.logger().info(
            f"반납 명령 수신: tool={requested_tool}, raw_text='{raw_text}'. "
            "그리퍼를 열고 사용자 hand-tool grasp 인식을 기다립니다."
        )
        self.gripper.open_gripper()
        self._cooperative_wait(0.5)
        return True

    def _receive_tool(self, context):
        command = context["command"]
        requested_tool = context["requested_tool"]
        log = self.state.logger()

        if not self._move_to_observation_pose(requested_tool, command, log):
            return False

        self.state.human_grasped_tool = False
        self.state.last_grasp_result = None
        self.reporter.publish(
            "checking_return_target",
            requested_tool,
            "카메라 기준 반납 공구 위치를 확인합니다.",
            command,
        )
        target = self.target_resolver.resolve(requested_tool, log)
        if target.tool_name and target.tool_name != "unknown":
            self.state.target_label = target.tool_name

        if target.source == RETURN_SOURCE_HAND:
            tool_name, failure_reason = self._handle_hand_target(
                target,
                requested_tool,
                command,
                log,
            )
        else:
            log.info(
                "손 위치 목표를 찾지 못했습니다. "
                "현재 카메라 ROI/depth 조건으로 grasp 가능 여부를 확인합니다."
            )
            self.reporter.publish(
                "checking_return_target",
                requested_tool,
                "카메라 ROI와 거리 조건 안에 공구가 들어오는지 확인합니다.",
                command,
                reason=target.reason or "hand_target_not_found",
            )
            tool_name, failure_reason = self.handoff.grasp_at_current_position(
                requested_tool,
                command,
                log,
            )

        if tool_name is None:
            if self._interrupted() or failure_reason in {
                "interrupted",
                "return_grasp_failed",
            }:
                return False

            self._recover_to_home(
                requested_tool,
                command,
                log,
                reason=failure_reason,
            )
            return False

        context["tool_name"] = tool_name
        return True

    def _place_home(self, context):
        tool_name = context["tool_name"]
        if tool_name is None:
            return False

        return self.placement.place_at_robot_home(
            tool_name,
            context["ori"],
            context["command"],
            self.state.logger(),
        )

    def _publish_done(self, context):
        tool_name = context["tool_name"] or context["requested_tool"]
        self.reporter.publish(
            "done",
            tool_name,
            f"{tool_name} 반납 공구를 Home에 배치했습니다.",
            context["command"],
        )
        return True

    def _resolve_home_observed_drawer(self, context):
        if self.drawer_flow is None:
            return True

        observed_tool = self.detect_home_tool_label()
        if observed_tool is None:
            self.reporter.fail(
                context["tool_name"] or context["requested_tool"],
                "Home 위치 관찰에서 반납 공구 라벨을 찾지 못했습니다.",
                "return_home_observed_tool_not_found",
                context["command"],
                self.state.logger(),
            )
            return False

        drawer_id = self.drawer_flow.drawer_id_for_tool(observed_tool)
        if drawer_id is None:
            self.reporter.fail(
                observed_tool,
                f"{observed_tool}에 매핑된 서랍이 없습니다.",
                "return_home_observed_tool_drawer_unmapped",
                context["command"],
                self.state.logger(),
            )
            return False

        context["observed_tool"] = observed_tool
        context["drawer_id"] = drawer_id
        self.state.logger().info(
            f"Home 관찰 라벨 기준 drawer 선택: tool={observed_tool}, "
            f"drawer={drawer_id}"
        )
        return True

    def _open_close_home_observed_drawer(self, context):
        drawer_id = context.get("drawer_id")
        observed_tool = context.get("observed_tool")
        if self.drawer_flow is None or drawer_id is None:
            return True

        self.reporter.publish(
            "opening_drawer",
            observed_tool,
            f"{observed_tool} 보관 서랍을 열고 닫습니다.",
            context["command"],
        )
        return self.drawer_flow.open_and_close(drawer_id, self.state.logger())

    def _handle_hand_target(self, target, requested_tool, command, logger):
        self.reporter.publish(
            "return_hand_detected",
            target.tool_name,
            "사용자 손 위치로 먼저 이동합니다.",
            command,
        )
        moved, failure_reason = self.handoff.move_to_candidate(
            target.tool_name,
            target.hand_candidate,
            command,
            logger,
        )
        if not moved:
            return None, failure_reason

        self.state.human_grasped_tool = False
        self.state.last_grasp_result = None
        self.reporter.publish(
            "checking_return_target",
            target.tool_name,
            "카메라 ROI와 거리 조건 안에 공구가 들어오는지 확인합니다.",
            command,
        )
        return self.handoff.grasp_at_current_position(
            target.tool_name,
            command,
            logger,
        )

    def _move_to_observation_pose(self, tool_name, command, logger):
        self.reporter.publish(
            "moving_return_grasp_pose",
            tool_name,
            "반납 공구를 감지하기 위해 관찰 자세로 이동합니다.",
            command,
        )
        ok, start_pose = move_to_observation_pose(self.motion, self.robot, logger)
        logger.info(
            "반납 1단계: 공구 감지 전 관찰 자세 이동 "
            f"pose=({start_pose.x:.3f},{start_pose.y:.3f},{start_pose.z:.3f})"
        )
        if ok:
            return True

        self.reporter.fail(
            tool_name,
            "반납 공구 감지 전 관찰 자세 이동에 실패했습니다.",
            "return_detection_observation_failed",
            command,
            logger,
        )
        return False

    def _recover_to_home(self, tool_name, command, logger, reason):
        self.reporter.publish(
            "recovering",
            tool_name,
            "실패 후 Home 복귀를 시도합니다.",
            command,
            reason=reason,
        )
        ok = self.motion.move_to_home_joints(logger)
        if ok:
            self.reporter.publish(
                "returned",
                tool_name,
                "실패 후 Home으로 복귀했습니다.",
                command,
                reason=reason,
            )
            return True

        if self._interrupted():
            logger.info("실패 후 Home 복귀 중 stop/pause 요청으로 중단합니다.")
            return False

        self.reporter.publish(
            "failed",
            tool_name,
            "실패 후 Home 복귀에도 실패했습니다.",
            command,
            reason=f"{reason}_recovery_failed",
        )
        return False

    def _interrupted(self):
        return any(
            event is not None and event.is_set()
            for event in (
                self.control_events.get("exit"),
                self.control_events.get("pause"),
            )
        )

    @staticmethod
    def _cooperative_wait(duration_sec):
        end_time = time.monotonic() + max(0.0, float(duration_sec))
        while rclpy.ok() and time.monotonic() < end_time:
            remaining = end_time - time.monotonic()
            time.sleep(min(SEQUENCE_WAIT_POLL_SEC, max(0.0, remaining)))
