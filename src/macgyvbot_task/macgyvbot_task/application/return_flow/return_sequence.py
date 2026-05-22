"""Return sequence step construction for receiving and storing a user-held tool."""

import time

import rclpy

from macgyvbot_config.timing import SEQUENCE_WAIT_POLL_SEC
from macgyvbot_manipulation.handover_targeting import move_to_observation_pose
from macgyvbot_task.application.return_flow.return_floor_pickup_flow import (
    ReturnFloorPickupFlow,
)
from macgyvbot_task.application.return_flow.return_handoff_flow import (
    ReturnHandoffFlow,
)
from macgyvbot_task.application.return_flow.return_home_placement_flow import (
    ReturnHomePlacementFlow,
)
from macgyvbot_task.application.return_flow.return_target_resolver import (
    RETURN_SOURCE_FLOOR,
    RETURN_SOURCE_HAND,
    ReturnTargetResolver,
)
from macgyvbot_task.application.status.return_status_reporter import (
    ReturnStatusReporter,
)
from macgyvbot_task.application.task_control.task_step import TaskStep


class ReturnSequenceRunner:
    """Build return workflow steps for hand or floor return targets."""

    def __init__(
        self,
        robot,
        motion_controller,
        gripper,
        state,
        pick_target_resolver,
        tool_hold_monitor=None,
        control_events=None,
    ):
        self.robot = robot
        self.motion = motion_controller
        self.gripper = gripper
        self.state = state
        self.tool_hold_monitor = tool_hold_monitor
        self.control_events = control_events or {}
        self.reporter = ReturnStatusReporter(state)
        self.target_resolver = ReturnTargetResolver(
            state,
            pick_target_resolver,
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
        self.floor_pickup = ReturnFloorPickupFlow(
            robot,
            motion_controller,
            gripper,
            state,
            self.reporter,
            self._cooperative_wait,
            tool_hold_monitor,
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
            "ori": self.state.home_ori,
        }
        return [
            TaskStep("return/prepare", lambda: self._prepare(context)),
            TaskStep("return/receive_tool", lambda: self._receive_tool(context)),
            TaskStep("return/place_home", lambda: self._place_home(context)),
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
            "반납 공구가 손에 있는지 바닥에 있는지 확인합니다.",
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
        elif target.source == RETURN_SOURCE_FLOOR:
            self.reporter.publish(
                "return_floor_detected",
                target.tool_name,
                "바닥에 있는 반납 공구를 집습니다.",
                command,
            )
            tool_name, failure_reason = self.floor_pickup.pick(
                target.floor_target,
                command,
                log,
            )
        else:
            failure_reason = target.reason or "return_target_not_found"
            self.reporter.fail(
                requested_tool,
                "반납받을 손 또는 바닥 공구를 찾지 못했습니다.",
                failure_reason,
                command,
                log,
            )
            self._recover_to_home(
                requested_tool,
                command,
                log,
                reason=failure_reason,
            )
            return False

        if tool_name is None:
            if self._interrupted() or failure_reason in {
                "interrupted",
                "return_grasp_failed",
                "return_floor_grasp_failed",
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
            "이동한 위치에서 손과 바닥 공구를 다시 확인합니다.",
            command,
        )
        local_target = self.target_resolver.resolve(requested_tool, logger)
        if local_target.tool_name and local_target.tool_name != "unknown":
            self.state.target_label = local_target.tool_name

        if local_target.source == RETURN_SOURCE_HAND:
            return self.handoff.grasp_at_current_position(
                local_target.tool_name,
                command,
                logger,
            )

        if local_target.source == RETURN_SOURCE_FLOOR:
            self.reporter.publish(
                "return_floor_detected",
                local_target.tool_name,
                "이동한 위치에서 바닥 반납 공구를 확인했습니다.",
                command,
            )
            return self.floor_pickup.pick(
                local_target.floor_target,
                command,
                logger,
            )

        self.reporter.fail(
            target.tool_name,
            "이동한 위치에서 손 또는 바닥 공구를 찾지 못했습니다.",
            local_target.reason or "return_target_not_found_after_move",
            command,
            logger,
        )
        return None, local_target.reason or "return_target_not_found_after_move"

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
