#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RAICOM 自主运行（不含语音）。

流程：
  等 /raicom/start_auto 或服务 /raicom/start
  → 导航 sort（已在附近则跳过）→ 看板 → YOLO 锁 P1/P2 → 转回前方
  → 循环：导航 pick → look_cargo 高视角（顶面贴纸+正面）→ YOLO → 底盘对 pick_aim → 低层抓
  → 对应园区 approach（离台）→ 低速挪到作业位 → 低层放
  不要把 park1/park2 作业位直接当全局导航终点。

单步试抓（人已经停在台前，侧面夹，不用再示教）：
  rosservice call /raicom/test_grasp

带导航的试抓：
  rosservice call /raicom/test_pick

路点教学（导航已起来且 Pose Estimate 对好）：
  rostopic pub -1 /raicom/save_pose std_msgs/String "data: sort"

参数：
  ~side         A 或 B（看板左转/右转）
  ~task         full | mapping_only
  ~cycles       抓放次数，默认 2（高层未标定时先各园区放一块）
  ~skip_nav     true 时不走底盘（人已经把车停到位）
  ~grasp_mode   fixed 或 3d；默认 fixed，完成坐标/手眼标定后才启用 3d
  ~allow_high   true 才抓/放高层；默认 false
  ~place_settle_sec 到放置点后等待车体稳定的秒数，默认 0.6
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
from raicom_arm import Arm, cmd_vel_topic, grasp_from_view, load_aim, load_poses, measure_stable_from_view, measure_from_view, servo_cmd_topic  # noqa: E402

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
        self.cycles = int(rospy.get_param("~cycles", 2))
        self.skip_nav = bool(rospy.get_param("~skip_nav", False))
        self.grasp_mode = str(rospy.get_param("~grasp_mode", "fixed")).lower()
        self.allow_high = bool(rospy.get_param("~allow_high", False))
        self.stack_capacity = int(rospy.get_param("~stack_capacity", 2 if self.allow_high else 1))
        self.place_settle_sec = max(0.0, float(rospy.get_param("~place_settle_sec", 0.6)))
        self.approach_back = max(0.12, float(rospy.get_param("~approach_back", 0.28)))
        self.already_at_dist = max(0.15, float(rospy.get_param("~already_at_dist", 0.40)))
        self.busy = False
        self.abort = False
        self.last_result = "idle"
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
        rospy.Service("/raicom/test_grasp", Trigger, self._srv_test_grasp)
        rospy.Service("/raicom/measure_grasp", Trigger, self._srv_measure_grasp)
        rospy.Service("/raicom/look_cargo", Trigger, self._srv_look_cargo)
        rospy.Service("/raicom/look_front", Trigger, self._srv_look_front)
        rospy.Service("/raicom/status", Trigger, self._srv_status)
        rospy.sleep(0.5)
        rospy.loginfo(
            "raicom_auto ready side=%s task=%s skip_nav=%s wp=%s",
            self.side, self.task, self.skip_nav, WP_PATH,
        )
        rospy.loginfo(
            "grasp_mode=%s allow_high=%s stack_capacity=%d approach_back=%.2f",
            self.grasp_mode, self.allow_high, self.stack_capacity, self.approach_back,
        )
        rospy.loginfo("place_settle_sec=%.2f; high layer disabled until ~allow_high:=true",
                      self.place_settle_sec)
        rospy.loginfo("start: rosservice call /raicom/start")
        rospy.loginfo("look cargo: rosservice call /raicom/look_cargo")
        rospy.loginfo("side grasp here: rosservice call /raicom/test_grasp")
        rospy.loginfo("measure only: rosservice call /raicom/measure_grasp")
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

    def _srv_status(self, _req):
        return TriggerResponse(success=not self.busy, message=self.last_result)

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
        self.last_result = "test_pick_running"
        threading.Thread(target=self._run_test_pick, daemon=True).start()
        return TriggerResponse(success=True, message="test_pick started; query /raicom/status")

    def _srv_test_grasp(self, _req):
        if self.busy:
            return TriggerResponse(success=False, message="busy")
        threading.Thread(target=self._run_test_grasp, daemon=True).start()
        return TriggerResponse(success=True, message="test_grasp started")

    def _srv_look_cargo(self, _req):
        if self.busy:
            return TriggerResponse(success=False, message="busy")
        self.arm.go("look_cargo", 1.4)
        return TriggerResponse(success=True, message="look_cargo")

    def _srv_look_front(self, _req):
        if self.busy:
            return TriggerResponse(success=False, message="busy")
        self.arm.look_front()
        return TriggerResponse(success=True, message="look_front")

    def _srv_measure_grasp(self, _req):
        if self.busy:
            return TriggerResponse(success=False, message="busy")
        self.set_yolo("detect")
        self.last_target = None
        target = self.wait_target(8.0)
        self.set_yolo("idle")
        if not target:
            return TriggerResponse(success=False, message="no target")
        measured = measure_stable_from_view(target["nx"], target["ny"])
        if measured is None:
            return TriggerResponse(success=False, message="no valid depth")
        xyz = measured["xyz"]
        return TriggerResponse(
            success=True,
            message="xyz=(%.3f, %.3f, %.3f)m depth=%.3fm valid=%d spread=%.3fm" % (
                xyz[0], xyz[1], xyz[2], measured["depth"],
                measured["valid_count"], measured["spread"],
            ),
        )

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
        if not self.allow_high:
            return "low"
        return "low" if self.picked >= 4 else "high"

    def reload_arm(self):
        self.arm.poses, self.arm.path = load_poses()

    def kill_joystick(self):
        os.system("rosnode kill /joystick_control /robot_1/joystick_control >/dev/null 2>&1")
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
        return self.goto_wp(wp, timeout=timeout, name=name)

    def already_at(self, wp, dist=None):
        dist = self.already_at_dist if dist is None else dist
        try:
            pose = self.current_pose()
        except Exception:
            return False
        dx = float(pose["x"]) - float(wp["x"])
        dy = float(pose["y"]) - float(wp["y"])
        if math.hypot(dx, dy) > dist:
            return False
        if "yaw" in wp:
            yaw_error = math.atan2(
                math.sin(float(pose["yaw"]) - float(wp["yaw"])),
                math.cos(float(pose["yaw"]) - float(wp["yaw"])),
            )
            if abs(yaw_error) > math.radians(20.0):
                return False
        return True

    def back_off_wp(self, wp, dist=None):
        dist = self.approach_back if dist is None else dist
        yaw = float(wp.get("yaw", 0.0))
        return {
            "x": float(wp["x"]) - dist * math.cos(yaw),
            "y": float(wp["y"]) - dist * math.sin(yaw),
            "yaw": yaw,
        }

    def goto_wp(self, wp, timeout=90.0, name="goal"):
        if self.skip_nav:
            rospy.loginfo("skip_nav, assume already at %s", name)
            return True
        if self.already_at(wp):
            rospy.loginfo("already near %s, skip global nav", name)
            return True
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
        if int(st) != 3:
            rospy.logwarn("goto %s state=%s", name, st)
            return False
        return True

    def creep_to(self, wp, timeout=8.0, speed=0.07, done=0.06, name="place"):
        """不用 move_base，全向低速挪到作业位，避免贴台规划失败后旋转。"""
        if self.skip_nav:
            return True
        try:
            self.mb.cancel_all_goals()
        except Exception:
            pass
        t0 = time.time()
        rate = rospy.Rate(10)
        while time.time() - t0 < timeout and not rospy.is_shutdown() and not self.abort:
            pose = self.current_pose()
            dx = float(wp["x"]) - pose["x"]
            dy = float(wp["y"]) - pose["y"]
            dist = math.hypot(dx, dy)
            if dist <= done:
                self.stop_base()
                rospy.loginfo("creep %s ok dist=%.3f", name, dist)
                return True
            vx_m = speed * dx / max(dist, 1e-3)
            vy_m = speed * dy / max(dist, 1e-3)
            yaw = pose["yaw"]
            c, s = math.cos(yaw), math.sin(yaw)
            tw = Twist()
            tw.linear.x = max(-speed, min(speed, vx_m * c + vy_m * s))
            tw.linear.y = max(-speed, min(speed, -vx_m * s + vy_m * c))
            self.cmd_vel.publish(tw)
            rate.sleep()
        self.stop_base()
        try:
            pose = self.current_pose()
            dist = math.hypot(float(wp["x"]) - pose["x"], float(wp["y"]) - pose["y"])
        except Exception:
            dist = 99.0
        ok = dist <= 0.14
        rospy.logwarn("creep %s done dist=%.3f ok=%s", name, dist, ok)
        return ok

    def goto_place(self, park):
        place_name = self.park_waypoint(park)
        if not place_name:
            return False
        place = self.waypoints.get(place_name)
        if not valid_wp(place):
            rospy.logerr("place waypoint %s missing", place_name)
            return False
        approach_name = place_name + "_approach"
        approach = self.waypoints.get(approach_name)
        if not valid_wp(approach):
            approach = self.back_off_wp(place)
            rospy.logwarn("no %s, derived back-off x=%.2f y=%.2f", approach_name, approach["x"], approach["y"])
        if not self.goto_wp(approach, timeout=75.0, name=approach_name):
            return False
        return self.creep_to(place, name=place_name)

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
        total_capacity = self.stack_capacity * 2
        if n >= total_capacity:
            rospy.logerr("P%d is full: %d blocks", park, total_capacity)
            return None
        # Each base waypoint represents one two-block stack. Once full, use
        # the alternate stack waypoint for the next two blocks.
        if n >= self.stack_capacity:
            if valid_wp(self.waypoints.get(alt)):
                return alt
            rospy.logerr(
                "P%d already has %d blocks; missing alternate waypoint %s for another stack",
                park, self.stack_capacity, alt,
            )
            return None
        return base

    def settle_for_place(self):
        """Stop the base and let chassis/arm vibrations decay before release."""
        self.stop_base()
        if self.place_settle_sec > 0.0:
            rospy.sleep(self.place_settle_sec)

    def place_height(self, park):
        if not self.allow_high:
            return "low"
        n = self.placed[park] % self.stack_capacity
        return "high" if n == 1 else "low"

    def align_to_aim(self, timeout=14.0, max_v=0.08, aim_key="pick_aim"):
        """麦克纳姆把当前锁定目标对到 pick_aim。成功返回最后一帧 target。"""
        try:
            self.mb.cancel_all_goals()
        except Exception:
            pass
        aim = load_aim(key=aim_key)
        aim_nx = float(aim["nx"])
        aim_ny = float(aim["ny"])
        x_sign = float(aim.get("x_sign", 1.0))
        y_sign = float(aim.get("y_sign", 1.0))
        kp_x, kp_y = 0.35, 0.40
        max_v = float(max_v)
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
        """开到 pick 观察位，抬到 look_cargo 高视角（顶面贴纸），底盘对到 pick_aim。"""
        self.clear_lock()
        if not self.goto("pick"):
            return None
        self.arm.go("look_cargo", 1.3)
        self.set_yolo("detect")
        rospy.sleep(0.8)
        tgt = self.wait_target(8.0)
        if not tgt:
            rospy.logwarn("no cargo in view")
            return None
        self.lock_target(tgt)
        rospy.loginfo("lock %s nx=%.3f ny=%.3f conf=%.2f", tgt["name"], tgt["nx"], tgt["ny"], tgt["conf"])
        aligned = self.align_to_aim(aim_key="pick_aim")
        return aligned or tgt

    def detect_and_grasp_3d(self):
        """Detect from the calibrated top view and execute guarded 3D grasp."""
        self.clear_lock()
        if not self.goto("pick"):
            return None
        self.arm.go("look_cargo", 1.3)
        self.set_yolo("detect")
        rospy.sleep(0.8)
        tgt = self.wait_target(8.0)
        if not tgt:
            rospy.logwarn("3d grasp: no cargo in calibrated view")
            return None
        self.lock_target(tgt)
        rospy.loginfo("3d target %s conf=%.2f nx=%.3f ny=%.3f",
                      tgt["name"], tgt["conf"], tgt["nx"], tgt["ny"])
        self.set_yolo("idle")
        self.reload_arm()
        if not grasp_from_view(self.arm_pub, tgt["nx"], tgt["ny"]):
            rospy.logerr("3d grasp rejected; no navigation to park")
            return None
        return tgt

    def _run_test_pick(self):
        self.busy = True
        self.abort = False
        try:
            self.kill_joystick()
            tgt = self.detect_and_align()
            if not tgt:
                self.last_result = "test_pick_failed:no_target"
                rospy.logerr("test_pick: no target")
                return
            layer = self.pick_layer()
            rospy.loginfo("test_pick %s layer=%s", tgt["name"], layer)
            self.reload_arm()
            self.arm.pick(layer)
            self.picked += 1
            self.arm.look_front()
            self.last_result = "test_pick_motion_complete:object_not_verified"
            rospy.logwarn("test_pick motion complete; object presence is not verified")
        except Exception as e:
            self.last_result = "test_pick_failed:%s" % e
            rospy.logerr("test_pick: %s", e)
            import traceback
            traceback.print_exc()
        finally:
            self.clear_lock()
            self.set_yolo("idle")
            self.stop_base()
            self.busy = False

    def _run_test_grasp(self):
        """停在台前。先抬到能看见顶面的角度，再用深度算出方块中心，逆解侧向夹取。不用旧抓取姿势。"""
        self.busy = True
        self.abort = False
        try:
            self.kill_joystick()
            self.reload_arm()
            self.clear_lock()
            rospy.loginfo("test_grasp: look at top sticker, then side close")
            self.arm.go("look_cargo", 1.3)
            self.set_yolo("detect")
            rospy.sleep(0.8)
            tgt = self.wait_target(8.0)
            if not tgt:
                rospy.logerr("test_grasp: no target, camera must see the TOP sticker")
                return
            self.lock_target(tgt)
            rospy.loginfo("see %s nx=%.3f ny=%.3f conf=%.2f", tgt["name"], tgt["nx"], tgt["ny"], tgt["conf"])
            self.reload_arm()
            ok = grasp_from_view(self.arm_pub, tgt["nx"], tgt["ny"])
            if not ok:
                rospy.logerr("test_grasp: 3d grasp failed")
                return
        except Exception as e:
            rospy.logerr("test_grasp: %s", e)
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
                if self.grasp_mode == "3d":
                    tgt = self.detect_and_grasp_3d()
                else:
                    tgt = self.detect_and_align()
                if not tgt:
                    rospy.logwarn("no cargo, skip cycle")
                    continue
                rospy.loginfo(
                    "see %s conf=%.2f nx=%.2f ny=%.2f layer=%s",
                    tgt["name"], tgt["conf"], tgt.get("nx", -1), tgt.get("ny", -1), self.pick_layer(),
                )
                if self.grasp_mode != "3d":
                    self.reload_arm()
                    self.arm.pick(self.pick_layer())
                self.picked += 1
                park = self.park_for(tgt["name"], p1, p2)
                wp = self.park_waypoint(park)
                if not wp:
                    break
                if not self.goto_place(park):
                    break
                self.settle_for_place()
                self.reload_arm()
                height = self.place_height(park)
                rospy.loginfo("placing %s on P%d stack layer=%s count_before=%d",
                              tgt["name"], park, height, self.placed[park])
                self.arm.place(height, park=park)
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
