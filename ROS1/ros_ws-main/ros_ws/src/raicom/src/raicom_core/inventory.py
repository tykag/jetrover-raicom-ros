SLOT_ORDER = (
    "front_left.top",
    "front_left.bottom",
    "front_right.top",
    "front_right.bottom",
    "inner_left.top",
    "inner_left.bottom",
    "inner_right.top",
    "inner_right.bottom",
)

FACE_OF = {
    "front_left": "front",
    "front_right": "front",
    "inner_left": "left",
    "inner_right": "right",
}

PARK_ORDER = ("stack_a.bottom", "stack_a.top", "stack_b.bottom", "stack_b.top")
CONSUMED = ("picked", "failed", "unverified")


class Slot(object):
    def __init__(self, slot_id):
        stack, layer = slot_id.split(".")
        self.slot_id = slot_id
        self.stack = stack
        self.layer = layer
        self.face = FACE_OF[stack]
        self.state = "unknown"
        self.class_name = "unknown"
        self.target_park = "unknown"
        self.confidence = 0.0
        self.attempts = 0
        self.track_id = None


class Inventory(object):
    def __init__(self):
        self.slots = dict((name, Slot(name)) for name in SLOT_ORDER)
        self.p1 = ""
        self.p2 = ""
        self.parks = {
            "P1": dict((name, "empty") for name in PARK_ORDER),
            "P2": dict((name, "empty") for name in PARK_ORDER),
        }

    def order(self):
        return SLOT_ORDER

    def slot(self, slot_id):
        return self.slots[slot_id]

    def set_mapping(self, p1, p2):
        self.p1 = p1
        self.p2 = p2

    def park_for(self, class_name):
        if class_name and class_name == self.p1:
            return "P1"
        if class_name and class_name == self.p2:
            return "P2"
        return "unknown"

    def _top_id(self, slot_id):
        return slot_id.rsplit(".", 1)[0] + ".top"

    def _selectable(self, slot, allow_high, skip_locked):
        if slot.state in CONSUMED:
            return False
        if skip_locked and slot.state == "locked":
            return False
        if slot.layer == "top":
            return bool(allow_high)
        if not allow_high:
            return True
        return self.slots[self._top_id(slot.slot_id)].state == "picked"

    def next_slot(self, allow_high=False, skip_locked=True):
        if any(slot.state == "locked" for slot in self.slots.values()):
            return None
        for name in SLOT_ORDER:
            if self._selectable(self.slots[name], allow_high, skip_locked):
                return name
        return None

    def mark_visible(self, slot_id, class_name, confidence):
        slot = self._require_open(slot_id)
        slot.state = "visible"
        slot.class_name = class_name
        slot.confidence = float(confidence)
        slot.target_park = self.park_for(class_name)

    def lock(self, slot_id, track_id):
        slot = self._require_open(slot_id)
        slot.state = "locked"
        slot.track_id = int(track_id)
        slot.attempts += 1

    def mark_picked(self, slot_id):
        slot = self.slots[slot_id]
        if slot.state == "picked":
            raise ValueError("slot already picked: %s" % slot_id)
        if slot.state != "locked":
            raise ValueError("pick requires lock: %s" % slot_id)
        slot.state = "picked"

    def mark_failed(self, slot_id):
        self.slots[slot_id].state = "failed"

    def mark_unverified(self, slot_id):
        self.slots[slot_id].state = "unverified"

    def abort(self, slot_id):
        self.mark_unverified(slot_id)

    def picked_count(self):
        return sum(1 for slot in self.slots.values() if slot.state == "picked")

    def next_park_slot(self, park, allow_high=False):
        names = PARK_ORDER if allow_high else ("stack_a.bottom",)
        for name in names:
            layer = name.split(".")[1]
            if layer == "top" and not allow_high:
                continue
            if self.parks[park][name] == "empty":
                if layer == "top" and self.parks[park][name.replace(".top", ".bottom")] != "occupied":
                    continue
                return "%s.%s" % (park, name)
        return None

    def occupy_park(self, park_slot):
        park, stack, layer = park_slot.split(".")
        name = "%s.%s" % (stack, layer)
        if self.parks[park][name] != "empty":
            raise ValueError("park slot occupied: %s" % park_slot)
        self.parks[park][name] = "occupied"

    def _require_open(self, slot_id):
        slot = self.slots[slot_id]
        if slot.state in CONSUMED:
            raise ValueError("slot already consumed: %s" % slot_id)
        return slot
