from raicom_core.slot_assignment import assign_slot
from raicom_core.types import Detection


def _iou(a, b):
    x1 = max(a.x1, b.x1)
    y1 = max(a.y1, b.y1)
    x2 = min(a.x2, b.x2)
    y2 = min(a.y2, b.y2)
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    area_a = max(0.0, a.x2 - a.x1) * max(0.0, a.y2 - a.y1)
    area_b = max(0.0, b.x2 - b.x1) * max(0.0, b.y2 - b.y1)
    denom = area_a + area_b - inter
    if denom <= 1e-9:
        return 0.0
    return inter / denom


def _center_dist(a, b):
    return ((a.nx - b.nx) ** 2 + (a.ny - b.ny) ** 2) ** 0.5


class TrackedTarget(object):
    def __init__(self, detection, slot, track_id):
        self.detection = detection
        self.slot = slot
        self.track_id = track_id
        self.hits = 1
        self.lost = 0


class TargetTracker(object):
    def __init__(self, need_stable=3, max_lost=2, max_dist=0.18, min_iou=0.1):
        self.need_stable = int(need_stable)
        self.max_lost = int(max_lost)
        self.max_dist = float(max_dist)
        self.min_iou = float(min_iou)
        self._next_id = 1
        self.locked = None

    def update(self, detections, face):
        cands = []
        for det in detections:
            slot = assign_slot(face, det)
            if slot is None:
                continue
            cands.append((det, slot))

        if self.locked is None:
            return self._acquire(cands)

        match = self._match(self.locked, cands)
        if match is None:
            self.locked.lost += 1
            if self.locked.lost > self.max_lost:
                self.locked = None
            return None

        det, slot = match
        self.locked.detection = det
        self.locked.slot = slot
        self.locked.lost = 0
        self.locked.hits += 1
        if self.locked.hits >= self.need_stable:
            return self.locked
        return None

    def _acquire(self, cands):
        if not cands:
            return None
        det, slot = cands[0]
        track = TrackedTarget(det, slot, self._next_id)
        self._next_id += 1
        self.locked = track
        if track.hits >= self.need_stable:
            return track
        return None

    def _match(self, locked, cands):
        best = None
        best_score = 1e9
        for det, slot in cands:
            if det.class_name != locked.detection.class_name:
                continue
            if slot != locked.slot:
                continue
            dist = _center_dist(locked.detection, det)
            iou = _iou(locked.detection, det)
            if dist > self.max_dist and iou < self.min_iou:
                continue
            score = dist - iou
            if score < best_score:
                best_score = score
                best = (det, slot)
        return best
