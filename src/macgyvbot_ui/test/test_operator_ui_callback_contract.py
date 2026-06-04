import ast
import unittest
from pathlib import Path


OPERATOR_UI_NODE = (
    Path(__file__).resolve().parents[1]
    / "macgyvbot_ui"
    / "operator_ui_node.py"
)


class OperatorUiCallbackContractTest(unittest.TestCase):
    def setUp(self):
        self.tree = ast.parse(OPERATOR_UI_NODE.read_text(encoding="utf-8"))

    def _operator_class(self):
        for node in self.tree.body:
            if isinstance(node, ast.ClassDef) and node.name == "OperatorUiNode":
                return node
        self.fail("OperatorUiNode class was not found")

    def test_public_user_text_callback_exists(self):
        method_names = {
            node.name
            for node in self._operator_class().body
            if isinstance(node, ast.FunctionDef)
        }

        self.assertIn("publish_user_text", method_names)

    def test_window_uses_public_user_text_callback(self):
        source = OPERATOR_UI_NODE.read_text(encoding="utf-8")

        self.assertIn("on_user_text=node.publish_user_text", source)
        self.assertNotIn("on_user_text=node._push_user_text", source)


if __name__ == "__main__":
    unittest.main()
