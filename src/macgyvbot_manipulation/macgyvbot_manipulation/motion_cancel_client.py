"""Best-effort motion action goal cancellation clients."""

from __future__ import annotations

from dataclasses import dataclass

try:
    from action_msgs.srv import CancelGoal
    from unique_identifier_msgs.msg import UUID
except Exception:  # pragma: no cover - ROS message import depends on runtime setup.
    CancelGoal = None
    UUID = None


DEFAULT_ACTION_NAMES = (
    "/dsr_moveit_controller/follow_joint_trajectory",
)

MOTION_ACTION_TYPE_SUFFIXES = (
    "/FollowJointTrajectory",
    "/ExecuteTrajectory",
    "/MoveGroup",
)


@dataclass
class _CancelEndpoint:
    action_name: str
    client: object


class MotionCancelClient:
    """Send CancelGoal requests to known and discovered motion action endpoints."""

    def __init__(
        self,
        node,
        action_name=None,
        action_names=None,
        timeout_sec=0.2,
    ):
        self.node = node
        self.action_names = self._normalize_action_names(action_name, action_names)
        self.timeout_sec = timeout_sec
        self._endpoints = {}

        if CancelGoal is None or UUID is None:
            self.node.get_logger().warn(
                "action_msgs.srv.CancelGoal import 실패. motion cancel 비활성화."
            )
            return

        for name in self.action_names:
            self._ensure_endpoint(name)

    def cancel_all_goals(self, reason=""):
        if CancelGoal is None or UUID is None:
            self.node.get_logger().warn("motion cancel client가 초기화되지 않았습니다.")
            return False

        self.refresh_action_graph()

        ready_endpoints = [
            endpoint
            for endpoint in self._endpoints.values()
            if endpoint.client.service_is_ready()
        ]
        if not ready_endpoints:
            self.node.get_logger().warn(
                "motion cancel service가 준비되지 않았습니다. "
                f"known_actions={sorted(self._endpoints)}"
            )
            return False

        for endpoint in ready_endpoints:
            endpoint.client.call_async(self._cancel_all_request())

        self.node.get_logger().info(
            "motion goal cancel 요청 전송: "
            f"actions={[endpoint.action_name for endpoint in ready_endpoints]}, "
            f"reason={reason}"
        )
        return True

    def refresh_action_graph(self):
        """Discover active motion action names from the ROS graph."""
        get_action_names = getattr(self.node, "get_action_names_and_types", None)
        if get_action_names is None:
            return

        try:
            action_names_and_types = get_action_names()
        except Exception as exc:
            self.node.get_logger().warn(f"action graph 조회 실패: {exc}")
            return

        for action_name, action_types in action_names_and_types:
            if self._is_motion_action(action_types):
                self._ensure_endpoint(action_name)

    def _ensure_endpoint(self, action_name):
        action_name = self._normalize_action_name(action_name)
        if not action_name or action_name in self._endpoints:
            return

        service_name = f"{action_name}/_action/cancel_goal"
        self._endpoints[action_name] = _CancelEndpoint(
            action_name=action_name,
            client=self.node.create_client(CancelGoal, service_name),
        )

    @staticmethod
    def _normalize_action_names(action_name, action_names):
        names = []
        if action_names is not None:
            names.extend(action_names)
        elif action_name is not None:
            names.append(action_name)
        else:
            names.extend(DEFAULT_ACTION_NAMES)

        normalized = []
        for name in names:
            normalized_name = MotionCancelClient._normalize_action_name(name)
            if normalized_name and normalized_name not in normalized:
                normalized.append(normalized_name)
        return tuple(normalized)

    @staticmethod
    def _normalize_action_name(action_name):
        return str(action_name or "").strip().rstrip("/")

    @staticmethod
    def _is_motion_action(action_types):
        return any(
            str(action_type).endswith(suffix)
            for action_type in action_types
            for suffix in MOTION_ACTION_TYPE_SUFFIXES
        )

    @staticmethod
    def _cancel_all_request():
        request = CancelGoal.Request()
        request.goal_info.goal_id = UUID(uuid=[0] * 16)
        request.goal_info.stamp.sec = 0
        request.goal_info.stamp.nanosec = 0
        return request
