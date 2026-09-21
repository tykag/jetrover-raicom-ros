import os
import unittest
import xml.etree.ElementTree as ET


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


class InterfaceFilesTest(unittest.TestCase):
    def test_required_interface_files_exist(self):
        required = [
            "msg/Detection2D.msg",
            "msg/DetectionArray.msg",
            "msg/ParkTarget.msg",
            "msg/TaskState.msg",
            "action/AlignTarget.action",
            "action/ArmTask.action",
            "action/RunMission.action",
            "srv/SavePose.srv",
        ]
        missing = [name for name in required if not os.path.isfile(os.path.join(ROOT, name))]
        self.assertEqual(missing, [])

    def test_canary_launch_not_included_by_main(self):
        with open(os.path.join(ROOT, "launch", "raicom.launch"), "r", encoding="utf-8") as handle:
            main = handle.read()
        with open(os.path.join(ROOT, "launch", "nodes.launch"), "r", encoding="utf-8") as handle:
            nodes = handle.read()
        self.assertNotIn("nodes_active_vision.launch", main)
        self.assertNotIn("nodes_active_vision.launch", nodes)

    def test_package_xml_has_message_generation(self):
        tree = ET.parse(os.path.join(ROOT, "package.xml"))
        root = tree.getroot()
        names = [child.tag.split("}")[-1] + ":" + (child.text or "") for child in root]
        joined = " ".join(names)
        self.assertIn("message_generation", joined)
        self.assertIn("message_runtime", joined)
        self.assertIn("actionlib_msgs", joined)

    def test_default_config_disables_motion(self):
        with open(os.path.join(ROOT, "config", "active_vision.yaml"), "r", encoding="utf-8") as handle:
            text = handle.read()
        self.assertIn("enable_base_motion: false", text)
        self.assertIn("enable_arm_motion: false", text)
        self.assertIn("allow_high: false", text)
        self.assertIn("grasp_mode: fixed", text)
        self.assertIn("aim_calibrated: false", text)


if __name__ == "__main__":
    unittest.main()
