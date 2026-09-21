import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
sys.path.insert(0, os.path.dirname(__file__))
from fakes import FakeArm, FakeBackends, FakeBase, gear_frame
from raicom_core.arm_runner import ArmTaskRunner
from raicom_core.config import refuse_reason
from raicom_core.inventory import Inventory
from raicom_core.mission import SingleLowMission


def calibrated_cfg(**kwargs):
    cfg = {
        "enable_base_motion": True,
        "enable_arm_motion": True,
        "allow_high": False,
        "grasp_mode": "fixed",
        "aim_calibrated": True,
        "roi_calibrated": True,
        "closed_gripper": 350,
        "search_lateral_m": 0.12,
        "aims": {
            "front_bottom_aim": {"nx": 0.25, "ny": 0.5, "x_sign": 1.0, "y_sign": 1.0},
        },
    }
    cfg.update(kwargs)
    return cfg


class ConfigRefuseTest(unittest.TestCase):
    def test_missing_calibration_refuses(self):
        reason = refuse_reason(
            {
                "grasp_mode": "fixed",
                "allow_high": False,
                "aim_calibrated": False,
                "roi_calibrated": True,
            }
        )
        self.assertEqual(reason, "aim not calibrated")


class ArmRunnerTest(unittest.TestCase):
    def test_preserve_gripper_and_servo_failure_stops_base(self):
        backends = FakeBackends()
        ok = ArmTaskRunner().run("look", "verify_front", "low", True, 350, backends)
        self.assertTrue(ok["success"])
        self.assertEqual(backends.arm.calls[0][0], "go_preserve_gripper")
        backends.arm.fail_on_pick = True
        bad = ArmTaskRunner().run("pick", "", "low", False, 350, backends)
        self.assertFalse(bad["success"])
        self.assertTrue(any(reason == "arm_error" for reason in backends.base.stopped))


class MissionIntegrationTest(unittest.TestCase):
    def _mission(self, frames, **kwargs):
        backends = FakeBackends(frames=frames, **kwargs)
        mission = SingleLowMission(calibrated_cfg(), Inventory())
        mission.inventory.set_mapping("gear", "bolt")
        result = mission.run(backends, slot="front_left.bottom")
        return mission, backends, result

    def test_success_marks_picked(self):
        see = [gear_frame(stamp=i * 0.1) for i in range(12)]
        verify = gear_frame(nx=0.55, ny=0.92, stamp=2.0, grasp=True)
        mission, backends, result = self._mission(see, post_pick=verify)
        self.assertTrue(result["success"], result)
        self.assertEqual(result["status"], "grasp_verified")
        self.assertEqual(mission.inventory.picked_count(), 1)
        self.assertEqual(backends.cmds_stop_last(), "mission_end")

    def test_lost_target_stops_and_does_not_pick(self):
        frames = [gear_frame(stamp=0.0)] + [[]] * 8
        mission, backends, result = self._mission(frames)
        self.assertFalse(result["success"])
        self.assertEqual(mission.inventory.picked_count(), 0)
        self.assertTrue(backends.base.stopped)

    def test_bad_depth_stops_and_does_not_pick(self):
        frames = [gear_frame(stamp=i * 0.1) for i in range(8)]
        mission, backends, result = self._mission(frames, depth_ok=False)
        self.assertEqual(result["reason"], "depth_invalid")
        self.assertEqual(mission.inventory.picked_count(), 0)
        self.assertTrue(any(reason == "bad_depth" for reason in backends.base.stopped))

    def test_cancel_aborts_unverified(self):
        frames = [gear_frame(stamp=i * 0.1) for i in range(8)]
        mission, backends, result = self._mission(frames, cancel_after=1)
        self.assertEqual(result["status"], "aborted")
        self.assertEqual(mission.inventory.slot("front_left.bottom").state, "unverified")
        self.assertEqual(mission.inventory.picked_count(), 0)

    def test_servo_failure_does_not_pick(self):
        frames = [gear_frame(stamp=i * 0.1) for i in range(12)]
        mission, backends, result = self._mission(frames, fail_on_pick=True)
        self.assertFalse(result["success"])
        self.assertEqual(mission.inventory.picked_count(), 0)

    def test_unverified_when_view_ambiguous(self):
        see = [gear_frame(stamp=i * 0.1) for i in range(12)]
        mission, backends, result = self._mission(see, post_pick=[])
        self.assertEqual(result["status"], "unverified")
        self.assertEqual(mission.inventory.picked_count(), 0)
        self.assertEqual(mission.inventory.slot("front_left.bottom").state, "unverified")


def _patch_stop_last():
    def cmds_stop_last(self):
        return self.base.stopped[-1] if self.base.stopped else None

    FakeBackends.cmds_stop_last = cmds_stop_last


_patch_stop_last()


if __name__ == "__main__":
    unittest.main()
