from raicom_core.alignment import AlignmentController
from raicom_core.arm_runner import ArmTaskRunner
from raicom_core.config import refuse_reason
from raicom_core.inventory import Inventory
from raicom_core.slot_assignment import assign_slot


class SingleLowMission(object):
    def __init__(self, config, inventory=None):
        self.config = config
        self.inventory = inventory or Inventory()
        self.aligner = AlignmentController(config)
        self.arm_tasks = ArmTaskRunner()
        self.result = "idle"
        self.busy = False

    def run(self, backends, slot=None):
        self.busy = True
        reason = refuse_reason(self.config)
        if reason:
            self.busy = False
            self.result = "failed"
            backends.base.stop(reason)
            return {"success": False, "status": "failed", "reason": reason}

        slot_id = slot or self.inventory.next_slot(allow_high=False)
        if not slot_id:
            self.busy = False
            self.result = "failed"
            backends.base.stop("no_slot")
            return {"success": False, "status": "failed", "reason": "no_slot"}

        stack, layer = slot_id.split(".")
        face = self.inventory.slot(slot_id).face
        aim_key = "%s_%s_aim" % (face, "bottom" if layer == "bottom" else "top")
        try:
            aligned = self.aligner.align(face, stack, layer, aim_key, backends)
            if not aligned.get("success"):
                if aligned.get("status") == "aborted":
                    self.inventory.abort(slot_id)
                    self.result = "aborted"
                else:
                    self.inventory.mark_failed(slot_id)
                    self.result = "failed"
                backends.base.stop(aligned.get("reason", "align_failed"))
                return {
                    "success": False,
                    "status": self.result,
                    "slot": slot_id,
                    "reason": aligned.get("reason"),
                }

            self.inventory.mark_visible(
                slot_id, aligned["class_name"], 0.9
            )
            self.inventory.lock(slot_id, aligned["track_id"])
            if not self.config.get("enable_arm_motion", False):
                backends.base.stop("arm_motion_disabled")
                pick = {"success": True, "reason": "arm_motion_disabled"}
            else:
                pick = self.arm_tasks.run(
                    "pick", "", "low", False, int(self.config.get("closed_gripper", 350)), backends
                )
            if not pick.get("success"):
                status = "aborted" if pick.get("status") == "aborted" else "failed"
                if status == "aborted":
                    self.inventory.abort(slot_id)
                else:
                    self.inventory.mark_failed(slot_id)
                self.result = status
                backends.base.stop(pick.get("reason", "arm_failed"))
                return {"success": False, "status": status, "slot": slot_id, "reason": pick.get("reason")}

            verify = self._verify(slot_id, aligned["class_name"], backends)
            if verify == "picked":
                self.inventory.mark_picked(slot_id)
                self.result = "grasp_verified"
                return {
                    "success": True,
                    "status": "grasp_verified",
                    "slot": slot_id,
                    "class_name": aligned["class_name"],
                    "reason": "verified",
                }
            if verify == "failed":
                self.inventory.mark_failed(slot_id)
                self.result = "failed"
                backends.base.stop("grasp_failed")
                return {"success": False, "status": "failed", "slot": slot_id, "reason": "grasp_failed"}
            self.inventory.mark_unverified(slot_id)
            self.result = "unverified"
            backends.base.stop("unverified")
            return {"success": False, "status": "unverified", "slot": slot_id, "reason": "unverified"}
        finally:
            self.busy = False
            backends.base.stop("mission_end")

    def abort(self, backends, slot_id=None):
        backends.base.stop("aborted")
        if slot_id:
            self.inventory.abort(slot_id)
        self.result = "aborted"

    def _verify(self, slot_id, class_name, backends):
        closed = int(self.config.get("closed_gripper", 350))
        face = self.inventory.slot(slot_id).face
        pose = "verify_%s" % face
        if self.config.get("enable_arm_motion", False):
            backends.arm.go_preserve_gripper(pose, closed)
        backends.base.back(0.10)
        detections = backends.detections()
        stack = slot_id.split(".")[0]
        in_slot = [
            det for det in detections
            if assign_slot(face, det) == stack
        ]
        in_grasp = [
            det for det in detections
            if getattr(det, "slot", "") == "grasp" or det.ny > 0.85
        ]
        if not in_slot and any(det.class_name == class_name for det in in_grasp):
            return "picked"
        if in_slot:
            return "failed"
        return "unverified"
