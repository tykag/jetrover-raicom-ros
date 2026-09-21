#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Thin ArmTask action server wrapping taught poses. Do not change yaml poses here."""
from __future__ import print_function

import os
import sys

import rospy
import actionlib
from geometry_msgs.msg import Twist
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from raicom_arm import Arm, cmd_vel_topic, send_pose, servo_cmd_topic
from raicom_core.arm_runner import ArmTaskRunner
from raicom_core.config import load_config
from raicom.msg import ArmTaskAction, ArmTaskResult


class RosBase(object):
    def __init__(self, pub):
        self.pub = pub

    def stop(self, reason):
        self.pub.publish(Twist())

    def drive(self, vx, vy, reason):
        self.stop(reason)

    def translate(self, dx):
        self.stop("no_translate")

    def back(self, dist):
        self.stop("no_back")


class ArmAdapter(object):
    def __init__(self, arm):
        self.arm = arm

    def go(self, name, preserve_gripper=False):
        if preserve_gripper:
            raise RuntimeError("use go_preserve_gripper")
        self.arm.go(name)

    def go_preserve_gripper(self, name, gripper):
        pose = dict(self.arm.poses[name])
        pose["gripper"] = int(gripper)
        send_pose(self.arm.pub, pose)

    def pick(self, layer):
        self.arm.pick(layer)

    def place(self, layer):
        self.arm.place(layer)


class ArmNode(object):
    def __init__(self):
        rospy.init_node("raicom_arm_task")
        self.config = load_config(rospy.get_param("~config", ""))
        pub = rospy.Publisher(servo_cmd_topic(), MultiRawIdPosDur, queue_size=1)
        self.arm = Arm(pub)
        self.base_pub = rospy.Publisher(cmd_vel_topic(), Twist, queue_size=1)
        self.server = actionlib.SimpleActionServer(
            "/raicom/arm_task",
            ArmTaskAction,
            execute_cb=self._execute,
            auto_start=False,
        )
        self.server.start()
        rospy.loginfo("raicom_arm_task ready")

    def _execute(self, goal):
        class Backends(object):
            pass

        backends = Backends()
        backends.arm = ArmAdapter(self.arm)
        backends.base = RosBase(self.base_pub)
        backends.cancelled = self.server.is_preempt_requested
        try:
            result = ArmTaskRunner().run(
                goal.command,
                goal.pose_name,
                goal.layer,
                bool(goal.preserve_gripper),
                int(goal.gripper),
                backends,
            )
            msg = ArmTaskResult(
                success=bool(result.get("success")),
                status=str(result.get("status", "")),
                reason=str(result.get("reason", "")),
            )
            if result.get("status") == "aborted":
                self.server.set_preempted(msg)
            elif result.get("success"):
                self.server.set_succeeded(msg)
            else:
                self.server.set_aborted(msg)
        except Exception as exc:
            self.base_pub.publish(Twist())
            self.server.set_aborted(ArmTaskResult(success=False, status="failed", reason=str(exc)))


def main():
    ArmNode()
    rospy.spin()


if __name__ == "__main__":
    main()
