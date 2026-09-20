#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RAICOM 自主运行（不含语音）。

流程：
  等 /raicom/start_auto 或服务 /raicom/start
  → 导航 sort → 看板 → YOLO 锁 P1/P2 → 转回前方
  → 循环 8 次：导航 pick 附近 → YOLO 选一块 → 底盘对到 pick_aim → 抓 → 对应园区 → 放置

单步试抓（导航+YOLO+本节点已开）：
  rosservice call /raicom/test_pick

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
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, EmptyResponse, Trigger, TriggerResponse

import actionlib

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from raicom_arm import Arm, cmd_vel_topic, load_aim, load_poses, servo_cmd_topic  # noqa: E402

WP_PATH = os.path.expanduser("~/yolo_models/raicom_waypoints.yaml")
NEEDED = ("sort", "pick", "park1", "park2")


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
        self.picked = 0
        self.last_target = None
        self._amcl_pose = None
        arm_topic = servo_cmd_topic()
        vel_topic = cmd_vel_topic()
        rospy.loginfo("arm topic %s", arm_topic)
        rospy.loginfo("cmd_vel topic %s", vel_topic)
        self.arm_pub = rospy.Publisher(arm_topic, MultiRawIdPosDur, queue_size=1)
        self.cmd_vel = rospy.Publisher(vel_topic, Twist, queue_size=1)
        self.arm = Arm(self.arm_pub)
        self.mb = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.waypoints = self.load_wp()
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self._on_amcl, queue_size=1)
        rospy.Subscriber("/raicom/start_auto", Bool, self._on_start_auto, queue_size=1)
        rospy.Subscriber("/raicom/save_pose", String, self._on_save_pose, queue_size=1)
        rospy.Subscriber("/raicom/target", String, self._on_target, queue_size=1)
        rospy.Service("/raicom/start", Trigger, self._srv_start)
        rospy.Service("/raicom/abort", Empty, self._srv_abort)
        rospy.Service("/raicom/test_pick", Trigger, self._srv_test_pick)
        rospy.sleep(0.5)
        rospy.loginfo(
            "raicom_auto ready side=%s task=%s skip_nav=%s wp=%s",
            self.side, self.task, self.skip_nav, WP_PATH,
        )
        rospy.loginfo("start: rosservice call /raicom/start")
        rospy.loginfo("try one grasp: rosservice call /raicom/test_pick")
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
        self.stop_base()
        return EmptyResponse()

    def _srv_test_pick(self, _req):
        if self.busy:
            return TriggerResponse(success=False, message="busy")
        threading.Thread(target=self._run_test_pick, daemon=True).start()
        return TriggerResponse(success=True, message="test_pick started")

    def _kick(self):
        if self.busy:
            rospy.logwarn("already running")
            return
        threading.Thread(target=self.run, daemon=True).start()

    def stop_base(self):
        try:
            self.cmd_vel.publish(Twist())
        except Exception:
            pass

    def clear_lock(self):
        rospy.set_param("/raicom/lock_nx", -1.0)
        rospy.set_param("/raicom/lock_ny", -1.0)

    def lock_target(self, tgt):
        rospy.set_param("/raicom/lock_nx", float(tgt["nx"]))
        rospy.set_param("/raicom/lock_ny", float(tgt["ny"]))

    def pick_layer(self):
        return "low" if self.picked >= 4 else "high"

    def reload_arm(self):
        self.arm.poses, self.arm.path = load_poses()

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

    def align_to_aim(self, timeout=14.0):
        """麦克纳姆把当前锁定目标对到 pick_aim。成功返回最后一帧 target。"""
        try:
            self.mb.cancel_all_goals()
        except Exception:
            pass
        aim = load_aim()
        aim_nx = float(aim["nx"])
        aim_ny = float(aim["ny"])
        x_sign = float(aim.get("x_sign", 1.0))
        y_sign = float(aim.get("y_sign", 1.0))
        kp_x, kp_y = 0.35, 0.40
        max_v = 0.08
        tol_n, tol_f = 0.045, 0.055
        need_stable = 8
        rospy.loginfo("align to aim nx=%.3f ny=%.3f", aim_nx, aim_ny)
        t0 = time.time()
        stable = 0
        lost = 0
        last = None
        rate = rospy.Rate(10)
        while time.time() - t0 < timeout and not rospy.is_shutdown() and not self.abort:
            t = self.last_target
            if not t or t.get("conf", 0) < 0.40:
                lost += 1
                self.stop_base()
                if lost > 25:
                    rospy.logwarn("align lost target")
                    return None
                rate.sleep()
                continue
            lost = 0
            last = t
            self.lock_target(t)
            ex = float(t["nx"]) - aim_nx
            ey = float(t["ny"]) - aim_ny
            # ny 小=更远 → 前进；nx 大=画面右 → 右移（ROS +y 常为左，所以 y 用负号）
            vx = x_sign * (-kp_x) * ey
            vy = y_sign * (-kp_y) * ex
            vx = max(-max_v, min(max_v, vx))
            vy = max(-max_v, min(max_v, vy))
            if abs(ex) < tol_n:
                vy = 0.0
            if abs(ey) < tol_f:
                vx = 0.0
            if vx == 0.0 and vy == 0.0:
                stable += 1
            else:
                stable = 0
            tw = Twist()
            tw.linear.x = vx
            tw.linear.y = vy
            self.cmd_vel.publish(tw)
            rospy.loginfo_throttle(0.8, "align ex=%.3f ey=%.3f vx=%.3f vy=%.3f stable=%d", ex, ey, vx, vy, stable)
            if stable >= need_stable:
                self.stop_base()
                rospy.loginfo("align ok %s nx=%.3f ny=%.3f", t["name"], t["nx"], t["ny"])
                return t
            rate.sleep()
        self.stop_base()
        rospy.logwarn("align timeout")
        return last if last and stable >= 3 else None

    def detect_and_align(self):
        """开到 pick 观察位，选一块（优先近的），底盘对到示教像素。"""
        self.clear_lock()
        if not self.goto("pick"):
            return None
        self.arm.look_front()
        self.set_yolo("detect")
        rospy.sleep(0.4)
        tgt = self.wait_target(8.0)
        if not tgt:
            rospy.logwarn("no cargo in view")
            return None
        self.lock_target(tgt)
        rospy.loginfo("lock %s nx=%.3f ny=%.3f conf=%.2f", tgt["name"], tgt["nx"], tgt["ny"], tgt["conf"])
        aligned = self.align_to_aim()
        return aligned or tgt

    def _run_test_pick(self):
        self.busy = True
        self.abort = False
        try:
            self.kill_joystick()
            self.arm.look_front()
            tgt = self.detect_and_align()
            if not tgt:
                rospy.logerr("test_pick: no target")
                return
            layer = self.pick_layer()
            rospy.loginfo("test_pick %s layer=%s", tgt["name"], layer)
            self.reload_arm()
            self.arm.pick(layer)
            self.picked += 1
            self.arm.look_front()
        except Exception as e:
            rospy.logerr("test_pick: %s", e)
            import traceback
            traceback.print_exc()
        finally:
            self.clear_lock()
            self.set_yolo("idle")
            self.stop_base()
            self.busy = False

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

            self.picked = 0
            self.placed = {1: 0, 2: 0}
            for i in range(self.cycles):
                if self.abort or rospy.is_shutdown():
                    break
                rospy.loginfo("=== cycle %d/%d ===", i + 1, self.cycles)
                tgt = self.detect_and_align()
                if not tgt:
                    rospy.logwarn("no cargo, skip cycle")
                    continue
                rospy.loginfo(
                    "see %s conf=%.2f nx=%.2f ny=%.2f layer=%s",
                    tgt["name"], tgt["conf"], tgt.get("nx", -1), tgt.get("ny", -1), self.pick_layer(),
                )
                self.reload_arm()
                self.arm.pick(self.pick_layer())
                self.picked += 1
                park = self.park_for(tgt["name"], p1, p2)
                wp = self.park_waypoint(park)
                if not self.goto(wp):
                    break
                self.reload_arm()
                self.arm.place(self.place_height(park))
                self.placed[park] += 1
                rospy.loginfo("placed on P%d count=%s", park, self.placed)
            self.arm.look_front()
            rospy.loginfo("=== auto finished picked=%s placed=%s ===", self.picked, self.placed)
        except Exception as e:
            rospy.logerr("auto exception: %s", e)
            import traceback
            traceback.print_exc()
        finally:
            self.clear_lock()
            self.stop_base()
            self.set_yolo("idle")
            self.busy = False


def main():
    AutoNode()
    rospy.spin()


if __name__ == "__main__":
    main()
