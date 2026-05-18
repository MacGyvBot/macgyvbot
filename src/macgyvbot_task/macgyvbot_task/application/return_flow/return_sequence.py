"""Return sequence step construction for receiving and storing a user-held tool."""

import time

import rclpy

from macgyvbot_config.timing import SEQUENCE_WAIT_POLL_SEC
from macgyvbot_task.application.return_flow.return_handoff_flow import (
    ReturnHandoffFlow,
)
from macgyvbot_task.application.return_flow.return_home_placement_flow import (
    ReturnHomePlacementFlow,
)
from macgyvbot_task.application.status.return_status_reporter import (
    ReturnStatusReporter,
)
from macgyvbot_task.application.task_control.task_step import TaskStep


class ReturnSequenceRunner:
    """Build return workflow steps for task-queue execution."""

    def __init__(
        self,
        robot,
        motion_controller,
        gripper,
        state,
        control_events=None,
    ):
        self.robot = robot
        self.motion = motion_controller
        self.gripper = gripper
        self.state = state
        self.control_events = control_events or {}
        self.reporter = ReturnStatusReporter(state)
        self.handoff = ReturnHandoffFlow(
            robot,
            motion_controller,
            gripper,
            state,
            self.reporter,
            self._cooperative_wait,
            interrupted=self._interrupted,
        )
        self.placement = ReturnHomePlacementFlow(
            robot,
            motion_controller,
            gripper,
            state,
            self.reporter,
            self._cooperative_wait,
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
            TaskStep("return/done", lambda: self._publish_done(context), retry_on_pause=False),
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

        tool_name, receive_failure_reason = self.handoff.receive(
            requested_tool,
            command,
            log,
        )
        if tool_name is not None:
            context["tool_name"] = tool_name
            return True

        if self._interrupted() or receive_failure_reason == "return_grasp_failed":
            return False

        self._recover_to_home(
            requested_tool,
            command,
            log,
            reason=receive_failure_reason,
        )
        return False

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
                self.control_events.get("stop"),
                self.control_events.get("pause"),
            )
        )

    @staticmethod
    def _cooperative_wait(duration_sec):
        end_time = time.monotonic() + max(0.0, float(duration_sec))
        while rclpy.ok() and time.monotonic() < end_time:
            remaining = end_time - time.monotonic()
            time.sleep(min(SEQUENCE_WAIT_POLL_SEC, max(0.0, remaining)))
