"""Best-effort MoveIt action goal cancellation client."""

from __future__ import annotations

try:
    from action_msgs.srv import CancelGoal
    from unique_identifier_msgs.msg import UUID
except Exception:  # pragma: no cover - ROS message import depends on runtime setup.
    CancelGoal = None
    UUID = None


class MotionCancelClient:
    """Send CancelGoal requests to a MoveIt action endpoint."""

    def __init__(
        self,
        node,
        action_name="/execute_trajectory",
        timeout_sec=0.2,
    ):
        self.node = node
        self.action_name = action_name.rstrip("/")
        self.timeout_sec = timeout_sec
        self._client = None

        if CancelGoal is None or UUID is None:
            self.node.get_logger().warn(
                "action_msgs.srv.CancelGoal import 실패. motion cancel 비활성화."
            )
            return

        service_name = f"{self.action_name}/_action/cancel_goal"
        self._client = node.create_client(CancelGoal, service_name)

    def cancel_all_goals(self, reason=""):
        if self._client is None:
            self.node.get_logger().warn("motion cancel client가 초기화되지 않았습니다.")
            return False

        if not self._client.service_is_ready():
            self.node.get_logger().warn(
                f"MoveIt cancel service가 준비되지 않았습니다: {self._client.srv_name}"
            )
            return False

        request = CancelGoal.Request()
        request.goal_info.goal_id = UUID(uuid=[0] * 16)
        request.goal_info.stamp.sec = 0
        request.goal_info.stamp.nanosec = 0
        self._client.call_async(request)
        self.node.get_logger().info(
            f"MoveIt goal cancel 요청 전송: action={self.action_name}, reason={reason}"
        )
        return True
