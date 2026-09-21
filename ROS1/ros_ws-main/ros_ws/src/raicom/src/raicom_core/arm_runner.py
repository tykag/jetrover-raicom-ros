class ArmTaskRunner(object):
    def run(self, command, pose_name, layer, preserve_gripper, gripper, backends):
        if backends.cancelled():
            return {"success": False, "status": "aborted", "reason": "preempted"}
        try:
            if command == "pick":
                backends.arm.pick(layer or "low")
            elif command == "place":
                backends.arm.place(layer or "low")
            elif preserve_gripper:
                backends.arm.go_preserve_gripper(pose_name, gripper)
            else:
                backends.arm.go(pose_name)
            if backends.cancelled():
                return {"success": False, "status": "aborted", "reason": "preempted"}
            return {"success": True, "status": "done", "reason": "ok"}
        except Exception as exc:
            backends.base.stop("arm_error")
            return {"success": False, "status": "failed", "reason": str(exc)}
