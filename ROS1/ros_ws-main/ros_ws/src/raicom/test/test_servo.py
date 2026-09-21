import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
from raicom_core.depth_filter import filter_depth
from raicom_core.search_policy import SearchPolicy
from raicom_core.visual_servo import ServoConfig, VisualServo


class DepthFilterTest(unittest.TestCase):
    def test_median_of_near_surface(self):
        patch = [
            0.40, 0.41, 0.42, 0.43, 0.80, 0.0, 2.0,
            0.40, 0.41, 0.39, 0.42, 0.41, 0.40, 0.41,
        ]
        sample = filter_depth(patch, min_valid=8, max_spread_m=0.035)
        self.assertTrue(sample.ok)
        self.assertGreater(sample.valid_count, 7)
        self.assertAlmostEqual(sample.depth_m, 0.41, delta=0.03)

    def test_invalid_when_too_few_or_spread(self):
        few = filter_depth([0.2, 0.0, 0.0], min_valid=8, max_spread_m=0.035)
        self.assertFalse(few.ok)
        spread = filter_depth(
            [0.12, 0.13, 0.40, 0.41, 0.42, 0.43, 0.44, 0.45, 0.46, 0.47, 0.48, 0.49],
            min_valid=8,
            max_spread_m=0.035,
        )
        self.assertFalse(spread.ok)

    def test_empty_patch_fails(self):
        sample = filter_depth([], min_valid=8, max_spread_m=0.035)
        self.assertFalse(sample.ok)
        self.assertEqual(sample.depth_m, 0.0)


class SearchPolicyTest(unittest.TestCase):
    def test_poses_then_limited_lateral_then_home(self):
        policy = SearchPolicy(lateral_m=0.12)
        kinds = [step.kind for step in policy.steps()]
        self.assertEqual(kinds[:3], ["pose", "pose", "pose"])
        self.assertEqual([step.name for step in policy.steps()[:3]], ["look_high", "look_mid", "look_low"])
        laterals = [step for step in policy.steps() if step.kind == "translate"]
        self.assertEqual(len(laterals), 4)
        self.assertTrue(all(abs(step.dx) <= 0.15 for step in laterals))
        self.assertEqual(policy.steps()[-1].kind, "home")
        self.assertFalse(policy.can_continue_after(policy.steps()))

    def test_does_not_loop(self):
        policy = SearchPolicy()
        steps = policy.steps()
        self.assertEqual(len(steps), len(set((s.kind, s.name, s.dx) for s in steps)))


class VisualServoTest(unittest.TestCase):
    def _cfg(self, **kwargs):
        values = dict(
            aim_nx=0.5,
            aim_ny=0.4,
            x_sign=1.0,
            y_sign=1.0,
            coarse_max_v=0.07,
            fine_max_v=0.025,
            need_stable=3,
            timeout_s=2.0,
            max_travel_m=0.2,
            stale_s=0.3,
        )
        values.update(kwargs)
        return ServoConfig(**values)

    def test_signs_are_configurable(self):
        pos = VisualServo(self._cfg(x_sign=1.0, y_sign=1.0)).command(
            nx=0.7, ny=0.2, stamp=0.0, now=0.0, dt=0.1
        )
        neg = VisualServo(self._cfg(x_sign=-1.0, y_sign=-1.0)).command(
            nx=0.7, ny=0.2, stamp=0.0, now=0.0, dt=0.1
        )
        self.assertNotEqual(pos.vx, 0.0)
        self.assertNotEqual(pos.vy, 0.0)
        self.assertAlmostEqual(pos.vx, -neg.vx)
        self.assertAlmostEqual(pos.vy, -neg.vy)

    def test_lost_or_stale_target_is_zero(self):
        servo = VisualServo(self._cfg())
        lost = servo.command(nx=None, ny=None, stamp=0.0, now=0.1, dt=0.1)
        self.assertEqual((lost.vx, lost.vy), (0.0, 0.0))
        servo.command(nx=0.8, ny=0.8, stamp=0.0, now=0.0, dt=0.1)
        stale = servo.command(nx=0.8, ny=0.8, stamp=0.0, now=1.0, dt=0.1)
        self.assertEqual((stale.vx, stale.vy), (0.0, 0.0))

    def test_timeout_and_travel_stop(self):
        servo = VisualServo(self._cfg(timeout_s=0.2, max_travel_m=0.05))
        first = servo.command(nx=0.9, ny=0.9, stamp=0.0, now=0.0, dt=0.1)
        self.assertNotEqual((first.vx, first.vy), (0.0, 0.0))
        timed = servo.command(nx=0.9, ny=0.9, stamp=0.3, now=0.3, dt=0.1)
        self.assertEqual((timed.vx, timed.vy), (0.0, 0.0))
        self.assertIn(timed.reason, ("timeout", "travel_limit"))

    def test_stable_in_deadzone(self):
        servo = VisualServo(self._cfg(need_stable=3))
        last = None
        for i in range(4):
            last = servo.command(nx=0.5, ny=0.4, stamp=i * 0.1, now=i * 0.1, dt=0.1)
        self.assertEqual((last.vx, last.vy), (0.0, 0.0))
        self.assertTrue(servo.aligned)


if __name__ == "__main__":
    unittest.main()
