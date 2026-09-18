#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RAICOM 自主运行（不含语音）。

流程：
  等 /raicom/start_auto 或服务 /raicom/start
  → 导航 sort → 看板 → YOLO 锁 P1/P2 → 转回前方
  → 循环 8 次：导航 pick → 识别 → 抓 → 导航对应园区 → 放置（低/高 两层，可选第二摞）

路点教学（导航已起来且 Pose Estimate 对好）：
  rostopic pub -1 /raicom/save_pose std_msgs/String "data: sort"

参数：
  ~side         A 或 B（看板左转/右转）
  ~task         full | mapping_only
  ~cycles       抓放次数，默认 8
  ~skip_nav     true 时不走底盘（人已经把车停到位）
"""
from __future__ import print_function

import math
import os
import sys
import threading
import time

import rospy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, EmptyResponse, Trigger, TriggerResponse

import actionlib

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from raicom_arm import Arm  # noqa: E402

WP_PATH = os.path.expanduser("~/yolo_models/raicom_waypoints.yaml")
NEEDED = ("sort", "pick", "park1", "park2")
# 待派送：ny 小=更远=里列；nx 小=画面左 → pick_inner_l，大=右 → pick_inner_r
INNER_NY = 0.42
NX_MID = 0.5


def q_from_yaw(yaw):
    # Melodic 的 tf 是 py2，这里自己算，避免 python3 import tf 崩
    half = 0.5 * float(yaw)
    return (0.0, 0.0, math.sin(half), math.cos(half))


def yaw_from_q(q):
    # q: geometry_msgs Quaternion
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def valid_wp(d):
    return isinstance(d, dict) and "x" in d and "y" in d


def first_valid_wp(waypoints, names):
    for name in names:
        if valid_wp(waypoints.get(name)):
            return name
    return None


class AutoNode(object):
    def __init__(self):
        rospy.init_node("raicom_auto")
        self.side = str(rospy.get_param("~side", "A")).upper()
        self.task = str(rospy.get_param("~task", "full"))
        self.cycles = int(rospy.get_param("~cycles", 8))
        self.skip_nav = bool(rospy.get_param("~skip_nav", False))
        self.busy = False
        self.abort = False
        self.placed = {1: 0, 2: 0}
        self.last_target = None
        self._amcl_pose = None
        self.arm_pub = rospy.Publisher(
            "/servo_controllers/port_id_1/multi_id_pos_dur", MultiRawIdPosDur, queue_size=1
        )
        self.arm = Arm(self.arm_pub)
        self.mb = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.waypoints = self.load_wp()
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self._on_amcl, queue_size=1)
        rospy.Subscriber("/raicom/start_auto", Bool, self._on_start_auto, queue_size=1)
        rospy.Subscriber("/raicom/save_pose", String, self._on_save_pose, queue_size=1)
        rospy.Subscriber("/raicom/target", String, self._on_target, queue_size=1)
        rospy.Service("/raicom/start", Trigger, self._srv_start)
        rospy.Service("/raicom/abort", Empty, self._srv_abort)
        rospy.sleep(0.5)
        rospy.loginfo(
            "raicom_auto ready side=%s task=%s skip_nav=%s wp=%s",
            self.side, self.task, self.skip_nav, WP_PATH,
        )
        rospy.loginfo("start: rosservice call /raicom/start  (or pub /raicom/start_auto)")
        rospy.loginfo("save pose: rostopic pub -1 /raicom/save_pose std_msgs/String \"data: sort\"")

    def load_wp(self):
        if not os.path.isfile(WP_PATH):
            rospy.logwarn("no waypoints file %s", WP_PATH)
            return {}
        with open(WP_PATH, "r") as f:
            data = yaml.safe_load(f) or {}
        return data if isinstance(data, dict) else {}

    def write_wp(self):
        os.makedirs(os.path.dirname(WP_PATH), exist_ok=True)
        with open(WP_PATH, "w") as f:
            yaml.safe_dump(self.waypoints, f, default_flow_style=False, allow_unicode=True)

    def _on_amcl(self, msg):
        self._amcl_pose = msg

    def current_pose(self):
        # 不用 tf（Melodic+python3 会 ImportError），读 AMCL
        msg = self._amcl_pose
        if msg is None:
            msg = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped, timeout=3.0)
            self._amcl_pose = msg
        p = msg.pose.pose.position
        yaw = yaw_from_q(msg.pose.pose.orientation)
        return {"x": float(p.x), "y": float(p.y), "yaw": float(yaw)}

    def _on_save_pose(self, msg):
        name = (msg.data or "").strip()
        if not name:
            return
        try:
            pose = self.current_pose()
        except Exception as e:
            rospy.logerr("save_pose failed (need navigation + 2D Pose Estimate): %s", e)
            return
        self.waypoints[name] = pose
        self.write_wp()
        rospy.loginfo("saved waypoint %s = %s", name, pose)

    def _on_target(self, msg):
        parts = (msg.data or "").split(",")
        if len(parts) < 4:
            return
        self.last_target = {
            "name": parts[0],
            "nx": float(parts[1]),
            "ny": float(parts[2]),
            "conf": float(parts[3]),
        }

    def _on_start_auto(self, msg):
        if msg.data:
            self._kick()

    def _srv_start(self, _req):
        self._kick()
        return TriggerResponse(success=True, message="started")

    def _srv_abort(self, _req):
        self.abort = True
        try:
            self.mb.cancel_all_goals()
        except Exception:
            pass
        return EmptyResponse()

    def _kick(self):
        if self.busy:
            rospy.logwarn("already running")
            return
        threading.Thread(target=self.run, daemon=True).start()

    def kill_joystick(self):
        os.system("rosnode kill /joystick_control >/dev/null 2>&1")
        rospy.sleep(0.3)

    def set_yolo(self, mode):
        rospy.set_param("/raicom/yolo_mode", mode)

    def wait_mapping(self, timeout=25.0):
        t0 = time.time()
        while time.time() - t0 < timeout and not rospy.is_shutdown() and not self.abort:
            if rospy.get_param("/raicom/mapping_ready", False):
                p1 = rospy.get_param("/raicom/p1_class", "")
                p2 = rospy.get_param("/raicom/p2_class", "")
                if p1 and p2 and p1 != p2:
                    return p1, p2
            rospy.sleep(0.2)
        return None, None

    def wait_target(self, timeout=8.0, min_conf=0.45):
        self.last_target = None
        t0 = time.time()
        best = None
        while time.time() - t0 < timeout and not rospy.is_shutdown() and not self.abort:
            t = self.last_target
            if t and t.get("conf", 0) >= min_conf:
                best = t
                if t["conf"] >= 0.70:
                    return t
            rospy.sleep(0.15)
        return best

    def goto(self, name, timeout=90.0):
        if self.skip_nav:
            rospy.loginfo("skip_nav, assume already at %s", name)
            return True
        wp = self.waypoints.get(name)
        if not valid_wp(wp):
            rospy.logerr("waypoint '%s' empty. Drive there and: rostopic pub -1 /raicom/save_pose std_msgs/String \"data: %s\"", name, name)
            return False
        if not self.mb.wait_for_server(rospy.Duration(8.0)):
            rospy.logerr("move_base not running")
            return False
        yaw = float(wp.get("yaw", 0.0))
        q = q_from_yaw(yaw)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(wp["x"])
        goal.target_pose.pose.position.y = float(wp["y"])
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        rospy.loginfo("goto %s x=%.2f y=%.2f yaw=%.2f", name, wp["x"], wp["y"], yaw)
        self.mb.send_goal(goal)
        ok = self.mb.wait_for_result(rospy.Duration(timeout))
        if not ok:
            self.mb.cancel_goal()
            rospy.logerr("goto %s timeout", name)
            return False
        st = self.mb.get_state()
        # SUCCEEDED=3
        if int(st) != 3:
            rospy.logwarn("goto %s state=%s", name, st)
            return False
        return True

    def park_for(self, cls_name, p1, p2):
        if cls_name == p1:
            return 1
        if cls_name == p2:
            return 2
        rospy.logwarn("class %s not in mapping p1=%s p2=%s, default P1", cls_name, p1, p2)
        return 1

    def park_waypoint(self, park):
        n = self.placed[park]
        base = "park%d" % park
        alt = "park%d_b" % park
        if n >= 2 and valid_wp(self.waypoints.get(alt)):
            return alt
        return base

    def place_height(self, park):
        n = self.placed[park]
        return "high" if (n % 2) == 1 else "low"

    def pick_waypoint_for_target(self, tgt):
        """外列 pick；里列按画面左右选 pick_inner_l / pick_inner_r。"""
        if not tgt or float(tgt.get("ny", 1.0)) >= INNER_NY:
            return "pick"
        if float(tgt.get("nx", 0.5)) < NX_MID:
            return first_valid_wp(self.waypoints, ("pick_inner_l", "pick_inner", "pick"))
        return first_valid_wp(self.waypoints, ("pick_inner_r", "pick_inner", "pick"))

    def detect_at_pick(self):
        """先到外列；里列不够则按左右换 pick_inner_l / pick_inner_r。"""
        if not self.goto("pick"):
            return None
        self.set_yolo("detect")
        rospy.sleep(0.4)
        tgt = self.wait_target(8.0)
        need_inner = (not tgt) or (float(tgt.get("ny", 1.0)) < INNER_NY)
        if not need_inner:
            self.set_yolo("idle")
            return tgt

        # 有目标按 nx 选左右；没目标先左后右各试一次
        if tgt:
            order = (
                ("pick_inner_l", "pick_inner", "pick")
                if float(tgt.get("nx", 0.5)) < NX_MID
                else ("pick_inner_r", "pick_inner", "pick")
            )
            candidates = [first_valid_wp(self.waypoints, order)]
        else:
            candidates = []
            for name in ("pick_inner_l", "pick_inner_r", "pick_inner"):
                if valid_wp(self.waypoints.get(name)):
                    candidates.append(name)

        self.set_yolo("idle")
        for name in candidates:
            if not name or name == "pick":
                continue
            rospy.loginfo("switch to %s (inner columns)", name)
            if not self.goto(name):
                continue
            self.set_yolo("detect")
            rospy.sleep(0.4)
            tgt2 = self.wait_target(8.0)
            self.set_yolo("idle")
            if tgt2:
                return tgt2
        return tgt

    def run(self):
        self.busy = True
        self.abort = False
        try:
            self.kill_joystick()
            rospy.set_param("/raicom/reset_yolo", True)
            rospy.sleep(0.4)
            self.set_yolo("mapping")
            rospy.loginfo("=== SORT mapping side=%s ===", self.side)
            if not self.goto("sort"):
                if self.task == "full" and not self.skip_nav:
                    return
            self.arm.look_board(self.side)
            rospy.sleep(0.4)
            p1, p2 = self.wait_mapping(30.0)
            if not p1:
                rospy.logerr("mapping not locked")
                self.arm.look_front()
                return
            rospy.loginfo("mapping P1=%s P2=%s", p1, p2)
            self.arm.look_front()
            if self.task == "mapping_only":
                rospy.loginfo("mapping_only done")
                return

            for i in range(self.cycles):
                if self.abort or rospy.is_shutdown():
                    break
                rospy.loginfo("=== cycle %d/%d ===", i + 1, self.cycles)
                tgt = self.detect_at_pick()
                if not tgt:
                    rospy.logwarn("no cargo, skip cycle")
                    continue
                rospy.loginfo(
                    "see %s conf=%.2f nx=%.2f ny=%.2f",
                    tgt["name"], tgt["conf"], tgt.get("nx", -1), tgt.get("ny", -1),
                )
                self.arm.pick()
                park = self.park_for(tgt["name"], p1, p2)
                wp = self.park_waypoint(park)
                if not self.goto(wp):
                    break
                self.arm.place(self.place_height(park))
                self.placed[park] += 1
                rospy.loginfo("placed on P%d count=%s", park, self.placed)
            self.arm.look_front()
            rospy.loginfo("=== auto finished placed=%s ===", self.placed)
        except Exception as e:
            rospy.logerr("auto exception: %s", e)
            import traceback
            traceback.print_exc()
        finally:
            self.set_yolo("idle")
            self.busy = False


def main():
    AutoNode()
    rospy.spin()


if __name__ == "__main__":
    main()
