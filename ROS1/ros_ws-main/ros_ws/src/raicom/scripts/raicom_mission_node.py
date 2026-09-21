#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Single-low mission action plus legacy start/status/abort services."""
from __future__ import print_function

import os
import sys
import threading

import rospy
import actionlib
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, EmptyResponse, Trigger, TriggerResponse

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from raicom_core.config import load_config
from raicom_core.inventory import Inventory
from raicom_core.mission import SingleLowMission
from raicom_core.types import Detection
from raicom.msg import RunMissionAction, RunMissionResult
from raicom.srv import SavePose, SavePoseResponse


class NullArm(object):
    def go(self, name, preserve_gripper=False):
        rospy.loginfo("mission pose %s", name)

    def go_preserve_gripper(self, name, gripper):
        rospy.loginfo("mission preserve %s gripper=%s", name, gripper)

    def pick(self, layer):
        rospy.loginfo("mission pick %s", layer)

    def place(self, layer):
        rospy.loginfo("mission place %s", layer)


class RosBase(object):
    def __init__(self, pub):
        self.pub = pub

    def drive(self, vx, vy, reason):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        self.pub.publish(msg)

    def translate(self, dx):
        self.stop("translate_skipped")

    def back(self, dist):
        self.stop("back_skipped")

    def stop(self, reason):
        self.pub.publish(Twist())


class MissionNode(object):
    def __init__(self):
        rospy.init_node("raicom_mission")
        self.config = load_config(rospy.get_param("~config", ""))
        self.inventory = Inventory()
        self.mission = SingleLowMission(self.config, self.inventory)
        self.abort_flag = False
        self.last_dets = []
        self.cmd_pub = rospy.Publisher("/hiwonder_controller/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/raicom/start_auto", Bool, self._on_start_auto, queue_size=1)
        rospy.Subscriber("/raicom/target", String, self._on_target, queue_size=1)
        rospy.Subscriber("/raicom/save_pose", String, self._on_save_topic, queue_size=1)
        rospy.Service("/raicom/start", Trigger, self._srv_start)
        rospy.Service("/raicom/status", Trigger, self._srv_status)
        rospy.Service("/raicom/abort", Empty, self._srv_abort)
        rospy.Service("/raicom/save_pose", SavePose, self._srv_save)
        self.server = actionlib.SimpleActionServer(
            "/raicom/run_mission",
            RunMissionAction,
            execute_cb=self._execute,
            auto_start=False,
        )
        self.server.start()
        rospy.loginfo("raicom_mission ready")

    def _backends(self):
        class Backends(object):
            pass

        backends = Backends()
        backends.base = RosBase(self.cmd_pub)
        backends.arm = NullArm()
        backends.detections = lambda: list(self.last_dets)
        backends.depth_values = lambda det: [0.40] * 16
        backends.clock = rospy.get_time
        backends.cancelled = lambda: self.abort_flag or self.server.is_preempt_requested()
        return backends

    def _on_target(self, msg):
        parts = (msg.data or "").split(",")
        if len(parts) < 4:
            return
        self.last_dets = [
            Detection(parts[0], float(parts[3]), float(parts[1]), float(parts[2]), 0, 0, 1, 1, -1, rospy.get_time())
        ]

    def _on_start_auto(self, msg):
        if msg.data:
            self._kick()

    def _kick(self):
        if self.mission.busy:
            return
        threading.Thread(target=lambda: self.mission.run(self._backends()), daemon=True).start()

    def _srv_start(self, _req):
        self._kick()
        return TriggerResponse(success=True, message="started")

    def _srv_status(self, _req):
        return TriggerResponse(success=not self.mission.busy, message=self.mission.result)

    def _srv_abort(self, _req):
        self.abort_flag = True
        self.mission.abort(self._backends())
        return EmptyResponse()

    def _on_save_topic(self, msg):
        rospy.logwarn("legacy /raicom/save_pose topic; use SavePose service")

    def _srv_save(self, req):
        if not (req.name or "").strip():
            return SavePoseResponse(success=False, message="empty name")
        return SavePoseResponse(success=True, message="accepted %s" % req.name)

    def _execute(self, goal):
        self.abort_flag = False
        result = self.mission.run(self._backends(), slot=goal.slot or None)
        msg = RunMissionResult(
            success=bool(result.get("success")),
            status=str(result.get("status", "")),
            slot=str(result.get("slot", "")),
            class_name=str(result.get("class_name", "")),
            reason=str(result.get("reason", "")),
        )
        if result.get("status") == "aborted":
            self.server.set_preempted(msg)
        elif result.get("success"):
            self.server.set_succeeded(msg)
        else:
            self.server.set_aborted(msg)


def main():
    MissionNode()
    rospy.spin()


if __name__ == "__main__":
    main()
