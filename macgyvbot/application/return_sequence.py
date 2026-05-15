"""Return sequence orchestration for receiving and storing a user-held tool."""

import time

import rclpy

from macgyvbot.config.timing import SEQUENCE_WAIT_POLL_SEC
from macgyvbot.application.return_handoff_flow import ReturnHandoffFlow
from macgyvbot.application.return_home_placement_flow import ReturnHomePlacementFlow
from macgyvbot.application.return_status_reporter import ReturnStatusReporter


class ReturnSequenceRunner:
    """Receive a user-held tool and place it in its configured home pose."""

    def __init__(self, robot, motion_controller, gripper, state):
        self.robot = robot
        self.motion = motion_controller
        self.gripper = gripper
        self.state = state
        self.reporter = ReturnStatusReporter(state)
        self.handoff = ReturnHandoffFlow(
            robot,
            motion_controller,
            gripper,
            state,
            self.reporter,
            self._cooperative_wait,
        )
        self.placement = ReturnHomePlacementFlow(
            robot,
            motion_controller,
            gripper,
            state,
            self.reporter,
            self._cooperative_wait,
        )

    def run(self, command):
        log = self.state.logger()
        requested_tool = command.get("tool_name", "unknown")
        raw_text = command.get("raw_text", "")

        try:
            self.state.human_grasped_tool = False
            self.state.last_grasp_result = None

            self.reporter.publish(
                "waiting_return_handoff",
                requested_tool,
                "사용자 반납 공구를 받을 준비를 시작합니다.",
                command,
            )
            log.info(
                f"반납 명령 수신: tool={requested_tool}, raw_text='{raw_text}'. "
                "그리퍼를 열고 사용자 hand-tool grasp 인식을 기다립니다."
            )

            ori = self.state.home_ori
            self.gripper.open_gripper()
            self._cooperative_wait(0.5)

            tool_name, receive_failure_reason = self.handoff.receive(
                requested_tool,
                command,
                log,
            )
            if tool_name is None:
                if receive_failure_reason == "return_grasp_failed":
                    return

                self._recover_to_home(
                    requested_tool,
                    command,
                    log,
                    reason=receive_failure_reason,
                )
                return

            if not self.placement.place_at_robot_home(
                tool_name,
                ori,
                command,
                log,
            ):
                return

            self.reporter.publish(
                "done",
                tool_name,
                f"{tool_name} 반납 공구를 Home에 배치했습니다.",
                command,
            )

        finally:
            self._clear_state()

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

        self.reporter.publish(
            "failed",
            tool_name,
            "실패 후 Home 복귀에도 실패했습니다.",
            command,
            reason=f"{reason}_recovery_failed",
        )
        return False

    def _clear_state(self):
        self.state.picking = False
        self.state.target_label = None
        self.state.human_grasped_tool = False
        self.state.current_command = None

    @staticmethod
    def _cooperative_wait(duration_sec):
        end_time = time.monotonic() + max(0.0, float(duration_sec))
        while rclpy.ok() and time.monotonic() < end_time:
            remaining = end_time - time.monotonic()
            time.sleep(min(SEQUENCE_WAIT_POLL_SEC, max(0.0, remaining)))
