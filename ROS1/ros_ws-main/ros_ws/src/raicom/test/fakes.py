from raicom_core.types import Detection


class FakeBase(object):
    def __init__(self):
        self.cmds = []
        self.stopped = []

    def drive(self, vx, vy, reason):
        self.cmds.append((vx, vy, reason))

    def translate(self, dx):
        self.cmds.append(("translate", dx))

    def back(self, dist):
        self.cmds.append(("back", dist))

    def stop(self, reason):
        self.stopped.append(reason)
        self.cmds.append((0.0, 0.0, reason))


class FakeArm(object):
    def __init__(self, fail_on_pick=False):
        self.calls = []
        self.fail_on_pick = fail_on_pick

    def go(self, name, preserve_gripper=False):
        self.calls.append(("go", name, preserve_gripper))

    def go_preserve_gripper(self, name, gripper):
        self.calls.append(("go_preserve_gripper", name, gripper))

    def pick(self, layer):
        self.calls.append(("pick", layer))
        if self.fail_on_pick:
            raise RuntimeError("servo error")

    def place(self, layer):
        self.calls.append(("place", layer))


class FakeBackends(object):
    def __init__(self, frames=None, post_pick=None, depth_ok=True, cancel_after=None, fail_on_pick=False):
        self.base = FakeBase()
        self.arm = FakeArm(fail_on_pick=fail_on_pick)
        self.frames = list(frames or [])
        self.post_pick = post_pick
        self.depth_ok = depth_ok
        self.cancel_after = cancel_after
        self.steps = 0
        self._now = 0.0

    def detections(self):
        self.steps += 1
        if any(call[0] == "pick" for call in self.arm.calls) and self.post_pick is not None:
            return list(self.post_pick)
        if not self.frames:
            return []
        idx = min(self.steps - 1, len(self.frames) - 1)
        return list(self.frames[idx])

    def depth_values(self, detection):
        if not self.depth_ok:
            return [0.0] * 4
        return [0.40 + 0.001 * i for i in range(16)]

    def clock(self):
        return self._now

    def cancelled(self):
        return self.cancel_after is not None and self.steps >= self.cancel_after


def gear_frame(nx=0.25, ny=0.5, stamp=0.0, grasp=False):
    return [
        Detection(
            class_name="gear",
            confidence=0.9,
            nx=nx,
            ny=0.92 if grasp else ny,
            x1=nx - 0.1,
            y1=(0.92 if grasp else ny) - 0.1,
            x2=nx + 0.1,
            y2=(0.92 if grasp else ny) + 0.1,
            track_id=-1,
            stamp=stamp,
        )
    ]
