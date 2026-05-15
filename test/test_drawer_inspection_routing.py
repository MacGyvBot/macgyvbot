import ast
from pathlib import Path
import unittest


NODE_PATH = (
    Path(__file__).resolve().parents[1]
    / "macgyvbot"
    / "nodes"
    / "macgyvbot_main_node.py"
)


def _method_node(class_node, method_name):
    for node in class_node.body:
        if isinstance(node, ast.FunctionDef) and node.name == method_name:
            return node
    raise AssertionError(f"{method_name} not found")


def _method_calls(method):
    calls = set()
    for node in ast.walk(method):
        if isinstance(node, ast.Call):
            func = node.func
            if isinstance(func, ast.Attribute):
                calls.add(func.attr)
            elif isinstance(func, ast.Name):
                calls.add(func.id)
    return calls


class TestDrawerInspectionRouting(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        tree = ast.parse(NODE_PATH.read_text(encoding="utf-8"))
        cls.node_class = next(
            node
            for node in tree.body
            if isinstance(node, ast.ClassDef) and node.name == "MacGyvBotNode"
        )

    def test_bring_requests_start_drawer_inspection_thread(self):
        method = _method_node(self.node_class, "_set_target_label")

        self.assertIn("start_drawer_pick_sequence", _method_calls(method))

    def test_frame_loop_does_not_start_legacy_pick_detection(self):
        method = _method_node(self.node_class, "_process_frames")
        calls = _method_calls(method)

        self.assertNotIn("_handle_target_detection", calls)
        self.assertNotIn("start_pick_sequence", calls)

    def test_drawer_pick_thread_runs_inspection_before_drawer_detection(self):
        method = _method_node(self.node_class, "start_drawer_pick_sequence")
        calls = _method_calls(method)

        self.assertIn("Thread", calls)
        self.assertNotIn("start_pick_sequence", calls)


if __name__ == "__main__":
    unittest.main()
