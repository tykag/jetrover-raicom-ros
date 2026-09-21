import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
from raicom_core.slot_assignment import assign_slot
from raicom_core.target_tracker import TargetTracker
from raicom_core.types import Detection


def det(**kwargs):
    defaults = dict(
        class_name="gear",
        confidence=0.8,
        nx=0.25,
        ny=0.5,
        x1=0.1,
        y1=0.4,
        x2=0.4,
        y2=0.6,
        track_id=-1,
        stamp=0.0,
    )
    defaults.update(kwargs)
    return Detection(**defaults)


class SlotAssignmentTest(unittest.TestCase):
    def test_front_splits_left_and_right(self):
        self.assertEqual(assign_slot("front", det(nx=0.2)), "front_left")
        self.assertEqual(assign_slot("front", det(nx=0.8)), "front_right")

    def test_side_faces_only_allow_matching_inner_stack(self):
        self.assertEqual(assign_slot("left", det(nx=0.5)), "inner_left")
        self.assertEqual(assign_slot("right", det(nx=0.5)), "inner_right")
        self.assertIsNone(assign_slot("left", det(nx=0.95)))
        self.assertIsNone(assign_slot("right", det(nx=0.05)))


class TargetTrackerTest(unittest.TestCase):
    def test_stable_after_three_close_frames(self):
        tracker = TargetTracker(need_stable=3, max_lost=2)
        d1 = det(stamp=0.0)
        d2 = det(nx=0.26, stamp=0.1)
        d3 = det(nx=0.24, stamp=0.2)
        self.assertIsNone(tracker.update([d1], face="front"))
        self.assertIsNone(tracker.update([d2], face="front"))
        locked = tracker.update([d3], face="front")
        self.assertIsNotNone(locked)
        self.assertEqual(locked.slot, "front_left")
        self.assertGreaterEqual(locked.track_id, 0)

    def test_brief_dropout_keeps_same_id(self):
        tracker = TargetTracker(need_stable=2, max_lost=2)
        first = tracker.update([det(stamp=0.0)], face="front")
        self.assertIsNone(first)
        locked = tracker.update([det(stamp=0.1)], face="front")
        track_id = locked.track_id
        self.assertIsNone(tracker.update([], face="front"))
        again = tracker.update([det(nx=0.27, stamp=0.3)], face="front")
        self.assertEqual(again.track_id, track_id)

    def test_lock_does_not_switch_to_neighbor(self):
        tracker = TargetTracker(need_stable=2, max_lost=2)
        tracker.update([det(nx=0.22, stamp=0.0)], face="front")
        locked = tracker.update([det(nx=0.22, stamp=0.1)], face="front")
        neighbor = det(nx=0.78, confidence=0.99, stamp=0.2)
        same = det(nx=0.23, confidence=0.6, stamp=0.2)
        kept = tracker.update([neighbor, same], face="front")
        self.assertEqual(kept.track_id, locked.track_id)
        self.assertEqual(kept.slot, "front_left")

    def test_wrong_class_does_not_steal_lock(self):
        tracker = TargetTracker(need_stable=2, max_lost=1)
        tracker.update([det(class_name="gear", stamp=0.0)], face="front")
        locked = tracker.update([det(class_name="gear", stamp=0.1)], face="front")
        stolen = tracker.update(
            [det(class_name="bolt", nx=0.22, confidence=0.99, stamp=0.2)],
            face="front",
        )
        self.assertIsNone(stolen)
        self.assertEqual(tracker.locked.track_id, locked.track_id)

    def test_empty_scene_clears_after_max_lost(self):
        tracker = TargetTracker(need_stable=1, max_lost=1)
        tracker.update([det(stamp=0.0)], face="front")
        tracker.update([], face="front")
        self.assertIsNone(tracker.update([], face="front"))
        self.assertIsNone(tracker.locked)


if __name__ == "__main__":
    unittest.main()
