from raicom_core.depth_filter import filter_depth
from raicom_core.search_policy import SearchPolicy
from raicom_core.target_tracker import TargetTracker
from raicom_core.visual_servo import ServoConfig, VisualServo


class AlignmentController(object):
    def __init__(self, config, search=None, tracker=None):
        self.config = config
        self.search = search or SearchPolicy(config.get("search_lateral_m", 0.12))
        self.tracker = tracker or TargetTracker(need_stable=3, max_lost=2)

    def align(self, face, slot, layer, aim_key, backends):
        if not self.config.get("enable_base_motion", False):
            backends.base.stop("motion_disabled")
        aim = (self.config.get("aims") or {}).get(aim_key)
        if not aim:
            backends.base.stop("missing_aim")
            return {"success": False, "status": "failed", "reason": "missing_aim"}

        locked = None
        for step in self.search.steps():
            if backends.cancelled():
                backends.base.stop("aborted")
                return {"success": False, "status": "aborted", "reason": "preempted"}
            try:
                self._do_step(step, backends)
            except Exception as exc:
                backends.base.stop("arm_error")
                return {"success": False, "status": "failed", "reason": str(exc)}
            locked = self._observe(face, backends)
            if locked is not None:
                break
        if locked is None:
            backends.base.stop("search_failed")
            return {"success": False, "status": "failed", "reason": "no_target"}

        servo = VisualServo(
            ServoConfig(
                aim_nx=float(aim["nx"]),
                aim_ny=float(aim["ny"]),
                x_sign=float(aim.get("x_sign", 1.0)),
                y_sign=float(aim.get("y_sign", 1.0)),
            )
        )
        now = backends.clock()
        dt = 0.1
        while True:
            if backends.cancelled():
                backends.base.stop("aborted")
                return {"success": False, "status": "aborted", "reason": "preempted"}
            dets = backends.detections()
            tracked = self.tracker.update(dets, face=face)
            if tracked is None:
                backends.base.stop("lost")
                return {"success": False, "status": "failed", "reason": "lost_target"}
            depth = filter_depth(backends.depth_values(tracked.detection))
            if not depth.ok:
                backends.base.stop("bad_depth")
                return {"success": False, "status": "failed", "reason": "depth_invalid"}
            cmd = servo.command(
                tracked.detection.nx,
                tracked.detection.ny,
                tracked.detection.stamp,
                now,
                dt,
            )
            if self.config.get("enable_base_motion", False):
                backends.base.drive(cmd.vx, cmd.vy, cmd.reason)
            else:
                backends.base.stop("motion_disabled")
            if cmd.reason in ("timeout", "travel_limit"):
                backends.base.stop(cmd.reason)
                return {"success": False, "status": "failed", "reason": cmd.reason}
            if servo.aligned:
                backends.base.stop("aligned")
                return {
                    "success": True,
                    "status": "aligned",
                    "slot": "%s.%s" % (tracked.slot, layer),
                    "class_name": tracked.detection.class_name,
                    "track_id": tracked.track_id,
                    "reason": "aligned",
                }
            now += dt

    def _do_step(self, step, backends):
        if step.kind == "pose":
            backends.arm.go(step.name, preserve_gripper=False)
        elif step.kind == "translate":
            if self.config.get("enable_base_motion", False):
                backends.base.translate(step.dx)
            else:
                backends.base.stop("motion_disabled")
        elif step.kind == "home":
            backends.arm.go("look_low", preserve_gripper=False)
            backends.base.stop("home")

    def _observe(self, face, backends):
        for _ in range(3):
            tracked = self.tracker.update(backends.detections(), face=face)
            if tracked is not None:
                return tracked
        return None
