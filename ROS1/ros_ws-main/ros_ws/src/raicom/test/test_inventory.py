import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
from raicom_core.inventory import Inventory


class InventoryTest(unittest.TestCase):
    def setUp(self):
        self.inv = Inventory()
        self.inv.set_mapping("gear", "bolt")

    def test_eight_slots_in_face_order(self):
        self.assertEqual(
            self.inv.order(),
            (
                "front_left.top",
                "front_left.bottom",
                "front_right.top",
                "front_right.bottom",
                "inner_left.top",
                "inner_left.bottom",
                "inner_right.top",
                "inner_right.bottom",
            ),
        )

    def test_bottom_blocked_until_top_picked_when_high_allowed(self):
        self.assertEqual(self.inv.next_slot(allow_high=True), "front_left.top")
        self.inv.mark_visible("front_left.top", "gear", 0.9)
        self.inv.lock("front_left.top", track_id=1)
        self.assertIsNone(self.inv.next_slot(allow_high=True))
        self.assertEqual(self.inv.slot("front_left.bottom").state, "unknown")
        self.inv.mark_picked("front_left.top")
        self.assertEqual(self.inv.next_slot(allow_high=True), "front_left.bottom")

    def test_single_low_mode_selects_exposed_bottom(self):
        self.assertEqual(self.inv.next_slot(allow_high=False), "front_left.bottom")

    def test_failed_and_unverified_do_not_count_as_picked(self):
        self.inv.mark_visible("front_left.bottom", "gear", 0.8)
        self.inv.lock("front_left.bottom", track_id=2)
        self.inv.mark_failed("front_left.bottom")
        self.assertEqual(self.inv.picked_count(), 0)
        self.inv.mark_visible("front_right.bottom", "bolt", 0.8)
        self.inv.lock("front_right.bottom", track_id=3)
        self.inv.mark_unverified("front_right.bottom")
        self.assertEqual(self.inv.picked_count(), 0)
        self.assertEqual(self.inv.next_slot(allow_high=False), "inner_left.bottom")

    def test_slot_cannot_be_consumed_twice(self):
        self.inv.mark_visible("front_left.bottom", "gear", 0.9)
        self.inv.lock("front_left.bottom", track_id=4)
        self.inv.mark_picked("front_left.bottom")
        with self.assertRaises(ValueError):
            self.inv.lock("front_left.bottom", track_id=5)
        with self.assertRaises(ValueError):
            self.inv.mark_picked("front_left.bottom")
        self.assertEqual(self.inv.picked_count(), 1)

    def test_class_maps_to_park(self):
        self.assertEqual(self.inv.park_for("gear"), "P1")
        self.assertEqual(self.inv.park_for("bolt"), "P2")
        self.assertEqual(self.inv.park_for("unknown"), "unknown")

    def test_abort_keeps_unverified_and_does_not_pick(self):
        self.inv.mark_visible("front_left.bottom", "gear", 0.85)
        self.inv.lock("front_left.bottom", track_id=7)
        self.inv.abort("front_left.bottom")
        slot = self.inv.slot("front_left.bottom")
        self.assertEqual(slot.state, "unverified")
        self.assertEqual(self.inv.picked_count(), 0)

    def test_park_slots_assign_in_order_without_duplicate(self):
        first = self.inv.next_park_slot("P1", allow_high=False)
        self.assertEqual(first, "P1.stack_a.bottom")
        self.inv.occupy_park(first)
        with self.assertRaises(ValueError):
            self.inv.occupy_park(first)
        self.assertIsNone(self.inv.next_park_slot("P1", allow_high=False))


if __name__ == "__main__":
    unittest.main()
