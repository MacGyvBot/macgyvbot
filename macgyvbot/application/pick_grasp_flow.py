"""Initial gripper grasp verification workflow."""

from __future__ import annotations

from macgyvbot.control.grasp_verifier import GraspVerifier


class PickGraspFlow:
    """Handle initial robot grasp verification for pick sequences."""

    def __init__(
        self,
        gripper,
        state,
        wait_fn,
    ):
        self.state = state
        self.grasp_verifier = GraspVerifier(gripper, wait_fn)

    def try_robot_grasp(self, logger):
        def publish_attempt(attempt, retry_limit):
            self.state._publish_robot_status(
                "grasping",
                message=f"공구 grasp 시도 {attempt}/{retry_limit}",
                command=self.state.current_command,
            )

        return self.grasp_verifier.try_grasp(
            logger,
            publish_attempt=publish_attempt,
            failure_prefix="그리퍼 grasp",
        )
