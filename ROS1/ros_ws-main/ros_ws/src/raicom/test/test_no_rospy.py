import ast
import os
import unittest


class NoRospyImportTest(unittest.TestCase):
    def test_core_modules_do_not_import_rospy(self):
        root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src", "raicom_core"))
        offenders = []
        for name in os.listdir(root):
            if not name.endswith(".py"):
                continue
            path = os.path.join(root, name)
            with open(path, "r") as handle:
                tree = ast.parse(handle.read(), filename=path)
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    for alias in node.names:
                        if alias.name == "rospy" or alias.name.startswith("rospy."):
                            offenders.append(path)
                if isinstance(node, ast.ImportFrom) and (node.module or "").startswith("rospy"):
                    offenders.append(path)
        self.assertEqual(offenders, [])


if __name__ == "__main__":
    unittest.main()
