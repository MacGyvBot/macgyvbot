import sys
import types
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
for package_path in (
    REPO_ROOT / "src" / "macgyvbot_task",
    REPO_ROOT / "src" / "macgyvbot_config",
    REPO_ROOT / "src" / "macgyvbot_domain",
    REPO_ROOT / "src" / "macgyvbot_manipulation",
):
    sys.path.insert(0, str(package_path))


class DummyNode:
    pass


class DummyToolCommand:
    def __init__(self):
        self.action = "unknown"
        self.tool_name = "unknown"
        self.target_mode = "unknown"
        self.raw_text = ""
        self.match_method = "unknown"
        self.confidence = 0.0


class DummyRobotTaskStatus:
    def __init__(self):
        self.status = "unknown"
        self.task = ""
        self.tool_name = "unknown"
        self.action = "unknown"
        self.message = ""
        self.reason = ""
        self.command = DummyToolCommand()


class DummyTaskRequest:
    pass


rclpy_module = types.ModuleType("rclpy")
rclpy_node_module = types.ModuleType("rclpy.node")
rclpy_node_module.Node = DummyNode
rclpy_module.ok = lambda: True
rclpy_module.node = rclpy_node_module
sys.modules.setdefault("rclpy", rclpy_module)
sys.modules.setdefault("rclpy.node", rclpy_node_module)

interfaces_module = types.ModuleType("macgyvbot_interfaces")
interfaces_msg_module = types.ModuleType("macgyvbot_interfaces.msg")
interfaces_msg_module.RobotTaskStatus = DummyRobotTaskStatus
interfaces_msg_module.TaskRequest = DummyTaskRequest
interfaces_msg_module.ToolCommand = DummyToolCommand
interfaces_module.msg = interfaces_msg_module
sys.modules.setdefault("macgyvbot_interfaces", interfaces_module)
sys.modules.setdefault("macgyvbot_interfaces.msg", interfaces_msg_module)

from macgyvbot_task.macgyvbot_main_node import MacGyvBotNode


def test_returned_status_clears_router_busy_state():
    node = object.__new__(MacGyvBotNode)
    node._task_active = True
    node._target_label = "screwdriver"
    node._current_command = {"action": "bring", "tool_name": "screwdriver"}

    msg = DummyRobotTaskStatus()
    msg.status = "returned"
    msg.action = "bring"
    msg.tool_name = "screwdriver"
    msg.command.action = "bring"
    msg.command.tool_name = "screwdriver"

    node._robot_status_cb(msg)

    assert node._task_active is False
    assert node._target_label is None
    assert node._current_command is None
