#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Thin AlignTarget action server. Algorithm lives in raicom_core."""
from __future__ import print_function

import os
import sys

import rospy
import actionlib
from geometry_msgs.msg import Twist

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from raicom_core.alignment import AlignmentController
from raicom_core.config import load_config, refuse_reason
from raicom_core.types import Detection
from raicom.msg import AlignTargetAction, AlignTargetFeedback, AlignTargetResult
from std_msgs.msg import String


def _cmd_vel_topic():
    try:
        from raicom_arm import cmd_vel_topic
        return cmd_vel_topic()
    except Exception:
        return "/hiwonder_controller/cmd_vel"


class RosBackends(object):
    def __init__(self, node):
        self.node = node
        self.last = []

    def detections(self):
        return list(self.node.last_dets)

    def depth_values(self, detection):
        return list(self.node.last_depth)

    def clock(self):
        return rospy.get_time()

    def cancelled(self):
        return self.node.server.is_preempt_requested()


class RosBase(object):
    def __init__(self, pub):
        self.pub = pub

    def drive(self, vx, vy, reason):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        self.pub.publish(msg)

    def translate(self, dx):
        msg = Twist()
        msg.linear.y = 0.05 if dx > 0 else -0.05
        self.pub.publish(msg)
        rospy.sleep(min(1.0, abs(float(dx)) / 0.05))
        self.stop("translate_done")

    def back(self, dist):
        msg = Twist()
        msg.linear.x = -0.05
        self.pub.publish(msg)
        rospy.sleep(min(1.5, abs(float(dist)) / 0.05))
        self.stop("back_done")

    def stop(self, reason):
        self.pub.publish(Twist())


class RosArm(object):
    def go(self, name, preserve_gripper=False):
        rospy.loginfo("align request pose %s preserve=%s", name, preserve_gripper)

    def go_preserve_gripper(self, name, gripper):
        rospy.loginfo("align preserve pose %s gripper=%s", name, gripper)


class AlignmentNode(object):
    def __init__(self):
        rospy.init_node("raicom_alignment")
        path = rospy.get_param("~config", "")
        self.config = load_config(path)
        self.last_dets = []
        self.last_depth = [0.40] * 16
        self.cmd_pub = rospy.Publisher(_cmd_vel_topic(), Twist, queue_size=1)
        rospy.Subscriber("/raicom/target", String, self._on_legacy_target, queue_size=1)
        self.server = actionlib.SimpleActionServer(
            "/raicom/align_target",
            AlignTargetAction,
            execute_cb=self._execute,
            auto_start=False,
        )
        self.server.start()
        rospy.loginfo("raicom_alignment ready")

    def _on_legacy_target(self, msg):
        parts = (msg.data or "").split(",")
        if len(parts) < 4:
            return
        self.last_dets = [
            Detection(parts[0], float(parts[3]), float(parts[1]), float(parts[2]), 0, 0, 1, 1, -1, rospy.get_time())
        ]

    def _execute(self, goal):
        reason = refuse_reason(self.config)
        if reason:
            self.cmd_pub.publish(Twist())
            self.server.set_aborted(AlignTargetResult(success=False, status="failed", reason=reason))
            return
        backends = RosBackends(self)
        backends.base = RosBase(self.cmd_pub)
        backends.arm = RosArm()
        try:
            result = AlignmentController(self.config).align(
                goal.face, goal.slot, goal.layer, goal.aim_key, backends
            )
            msg = AlignTargetResult(
                success=bool(result.get("success")),
                status=str(result.get("status", "")),
                slot=str(result.get("slot", "")),
                class_name=str(result.get("class_name", "")),
                track_id=int(result.get("track_id", -1)),
                reason=str(result.get("reason", "")),
            )
            if result.get("status") == "aborted":
                self.server.set_preempted(msg)
            elif result.get("success"):
                self.server.set_succeeded(msg)
            else:
                self.server.set_aborted(msg)
        except Exception as exc:
            self.cmd_pub.publish(Twist())
            self.server.set_aborted(AlignTargetResult(success=False, status="failed", reason=str(exc)))


def main():
    AlignmentNode()
    rospy.spin()


if __name__ == "__main__":
    main()
