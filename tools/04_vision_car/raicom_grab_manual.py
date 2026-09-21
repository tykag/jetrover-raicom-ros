#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RAICOM 手动+半自动抓取（固定姿势）。

高层（两块摞着，抓上面那块）：
  1. 手柄把车开到台前（不用很准，方块在视野里就行）
  2. python3 raicom_grab_manual.py align      # YOLO 检测方块，自动把车对到 pick_aim
  3. python3 raicom_grab_manual.py ready      # 摆到高层抓取姿势（夹爪张开）
  4. python3 raicom_grab_manual.py close      # 闭合+抬起

低层（只剩一块在台面上）：
  python3 raicom_grab_manual.py align_low     # 使用低层专用停车点
  python3 raicom_grab_manual.py ready_low     # 摆到低层抓取姿势
  python3 raicom_grab_manual.py close_low     # 闭合+抬起

放置（夹爪全程夹紧，到位后最后才松，再垂直抬离）：
  python3 raicom_grab_manual.py place_low     # 放低层
  python3 raicom_grab_manual.py place_high    # 放高层（摞第二块）

其他：
  python3 raicom_grab_manual.py open          # 原地松开（不动关节）
  python3 raicom_grab_manual.py home          # 回 look_front
  python3 raicom_grab_manual.py save_low      # 手柄调好低层姿势后，记录到 yaml

姿势来自 ~/yolo_models/raicom_arm_poses.yaml。
align 需要 YOLO 在跑：python3 ~/yolo_models/raicom_yolo_trt.py _show:=false &
"""
from __future__ import print_function

import os
import sys
import time

import rospy
from geometry_msgs.msg import Twist
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from std_msgs.msg import String

sys.path.insert(0, os.path.expanduser("~/yolo_models"))
from raicom_arm import (  # noqa: E402
    Arm,
    cmd_vel_topic,
    dump_poses,
    load_aim,
    load_poses,
    poses_yaml_path,
    read_current_joints,
    send_pose,
    servo_cmd_topic,
)

JOINT_KEYS = ("joint1", "joint2", "joint3", "joint4", "joint5")


def grip_of(arm, name, default):
    """读某个姿势里的夹爪值。缺了用默认。"""
    pose = arm.poses.get(name) or {}
    return int(pose.get("gripper", default))


def go_down(arm, layer):
    """抬到 hover 再落到对应层的 down。关节用示教值，夹爪张开。"""
    down_name = "pick_down_low" if layer == "low" else "pick_down"
    if down_name not in arm.poses:
        raise KeyError("yaml 里没有 %s" % down_name)
    open_g = grip_of(arm, down_name, 100)

    hover = dict(arm.poses["pick_hover"])
    hover["gripper"] = open_g
    print("-> pick_hover (夹爪 %d 张开)" % open_g)
    send_pose(arm.pub, hover, duration=1.2)

    down = dict(arm.poses[down_name])
    down["gripper"] = open_g
    print("-> %s (夹爪 %d 张开)" % (down_name, open_g))
    send_pose(arm.pub, down, duration=1.0)


def close_and_lift(arm, layer):
    """保持对应层的 down 关节闭合，再按 hover 关节抬起。"""
    down_name = "pick_down_low" if layer == "low" else "pick_down"
    if down_name not in arm.poses:
        raise KeyError("yaml 里没有 %s" % down_name)
    close_g = grip_of(arm, "pick_close", 350)

    closed = dict(arm.poses[down_name])
    closed["gripper"] = close_g
    print("-> %s 闭合 (夹爪 %d)" % (down_name, close_g))
    send_pose(arm.pub, closed, duration=0.6)
    time.sleep(0.3)

    lift = dict(arm.poses["pick_hover"])
    lift["gripper"] = close_g
    print("-> 抬起 (夹爪 %d 保持闭合)" % close_g)
    send_pose(arm.pub, lift, duration=1.0)


def open_here(arm):
    """原地只松夹爪，不动关节。高低层通用。"""
    open_g = grip_of(arm, "pick_down", 100)
    pose = read_current_joints()
    pose["gripper"] = open_g
    print("-> 原地松开 (夹爪 %d)" % open_g)
    send_pose(arm.pub, pose, duration=0.6)


def place_and_release(arm, height):
    """放置：全程夹紧移动，到位后最后才松夹爪，再抬离。"""
    down_name = "place_down_high" if height == "high" else "place_down_low"
    if down_name not in arm.poses:
        raise KeyError("yaml 里没有 %s" % down_name)
    close_g = grip_of(arm, "pick_close", 350)
    open_g = grip_of(arm, "place_open", 100)

    hover = dict(arm.poses["pick_hover"])
    hover["gripper"] = close_g
    print("-> pick_hover (夹紧 %d 移动)" % close_g)
    send_pose(arm.pub, hover, duration=1.0)

    down = dict(arm.poses[down_name])
    down["gripper"] = close_g
    print("-> %s (还夹着，先不松)" % down_name)
    send_pose(arm.pub, down, duration=1.1)
    time.sleep(0.3)

    release = dict(arm.poses[down_name])
    release["gripper"] = open_g
    print("-> 到位，松夹爪 (%d)" % open_g)
    send_pose(arm.pub, release, duration=0.6)
    time.sleep(0.3)

    away = dict(arm.poses["pick_hover"])
    away["gripper"] = open_g
    print("-> 垂直抬离")
    send_pose(arm.pub, away, duration=1.0)


def save_low(arm):
    """把当前舵机位置记成 pick_down_low。手柄调好低层姿势后用。"""
    pose = read_current_joints()
    poses, _old = load_poses()
    pose["gripper"] = grip_of(arm, "pick_down", 100)
    poses["pick_down_low"] = pose
    path = poses_yaml_path()
    dump_poses(path, poses)
    print("SAVED pick_down_low =", pose)
    print("wrote", path)


def do_align(timeout=14.0, max_v=0.08, aim_name="pick_aim"):
    """YOLO 检测方块，麦克纳姆把方块对到指定像素位置。"""
    vel_pub = rospy.Publisher(cmd_vel_topic(), Twist, queue_size=1)

    print("杀掉手柄节点避免冲突...")
    os.system("rosnode kill /joystick_control /robot_1/joystick_control >/dev/null 2>&1")
    rospy.sleep(0.3)

    rospy.set_param("/raicom/yolo_mode", "detect")

    holder = {"t": None, "ts": 0.0}

    def _cb(msg):
        parts = (msg.data or "").split(",")
        if len(parts) >= 4:
            holder["t"] = {
                "name": parts[0],
                "nx": float(parts[1]),
                "ny": float(parts[2]),
                "conf": float(parts[3]),
            }
            holder["ts"] = time.time()

    rospy.Subscriber("/raicom/target", String, _cb, queue_size=1)
    rospy.sleep(0.5)

    aim = load_aim(key=aim_name)
    aim_nx = float(aim["nx"])
    aim_ny = float(aim["ny"])
    x_sign = float(aim.get("x_sign", 1.0))
    y_sign = float(aim.get("y_sign", 1.0))
    print("%s: nx=%.3f ny=%.3f" % (aim_name, aim_nx, aim_ny))

    kp_x, kp_y = 0.35, 0.40
    tol_n, tol_f = 0.045, 0.055
    need_stable = 8

    t0 = time.time()
    stable = 0
    lost = 0
    last = None
    rate = rospy.Rate(10)

    print("开始对位（%.1f 秒超时）..." % timeout)
    while time.time() - t0 < timeout and not rospy.is_shutdown():
        t = holder["t"]
        if not t or t.get("conf", 0) < 0.40 or (time.time() - holder["ts"]) > 0.6:
            lost += 1
            vel_pub.publish(Twist())
            if lost > 25:
                print("丢失目标，退出。YOLO 看不到方块，先用手柄把车开到能看见的位置")
                break
            rate.sleep()
            continue

        lost = 0
        last = t
        ex = float(t["nx"]) - aim_nx
        ey = float(t["ny"]) - aim_ny
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
        vel_pub.publish(tw)

        if stable > 0 and stable % 3 == 0:
            print("  对位中 ex=%.3f ey=%.3f stable=%d/%d" % (ex, ey, stable, need_stable))

        if stable >= need_stable:
            vel_pub.publish(Twist())
            print("对位完成! %s nx=%.3f ny=%.3f" % (t["name"], t["nx"], t["ny"]))
            os.system("roslaunch hiwonder_peripherals joystick_control.launch >/dev/null 2>&1 &")
            rospy.sleep(1.0)
            print("手柄已重启")
            return True

        rate.sleep()

    vel_pub.publish(Twist())
    print("对位超时" + ("，最后看到 %s" % last["name"] if last else ""))
    os.system("roslaunch hiwonder_peripherals joystick_control.launch >/dev/null 2>&1 &")
    rospy.sleep(1.0)
    print("手柄已重启")
    return False


def usage(arm):
    print("用法:")
    print("  align       YOLO 自动对位（需 YOLO 在跑）")
    print("  align_low   YOLO 使用低层专用停车点对位")
    print("  ready       高层抓取姿势（夹爪张开）")
    print("  close       高层闭合+抬起")
    print("  ready_low   低层抓取姿势（夹爪张开）")
    print("  close_low   低层闭合+抬起")
    print("  place_low   放低层（夹紧移动，最后才松）")
    print("  place_high  放高层（夹紧移动，最后才松）")
    print("  open        原地松开")
    print("  home        回 look_front")
    print("  save_low    把当前姿势记成 pick_down_low")
    print("")
    print("姿势来自:", arm.path or "(默认值)")


def main():
    rospy.init_node("grab_manual", anonymous=True)
    cmd = sys.argv[1] if len(sys.argv) > 1 else ""
    pub = rospy.Publisher(servo_cmd_topic(), MultiRawIdPosDur, queue_size=1)
    rospy.sleep(0.5)
    arm = Arm(pub)

    if cmd == "align":
        if do_align():
            print("\n对位成功，接下来: raicom_grab_manual.py ready")
        else:
            print("\n对位失败，用手柄手动调整")

    elif cmd == "align_low":
        if do_align(aim_name="pick_aim_low"):
            print("\n低层对位成功，接下来: raicom_grab_manual.py ready_low")
        else:
            print("\n低层对位失败，用手柄手动调整")

    elif cmd == "ready":
        go_down(arm, "high")
        print("就位。确认夹爪正对方块后: raicom_grab_manual.py close")

    elif cmd == "close":
        close_and_lift(arm, "high")
        print("完成。")

    elif cmd == "ready_low":
        go_down(arm, "low")
        print("就位（低层）。确认夹爪正对方块后: raicom_grab_manual.py close_low")

    elif cmd == "close_low":
        close_and_lift(arm, "low")
        print("完成。")

    elif cmd == "place_low":
        place_and_release(arm, "low")
        print("完成。")

    elif cmd == "place_high":
        place_and_release(arm, "high")
        print("完成。")

    elif cmd == "open":
        open_here(arm)
        print("已松开。")

    elif cmd == "home":
        print("-> look_front")
        arm.go("look_front", 1.5)
        print("已回 look_front。")

    elif cmd == "save_low":
        save_low(arm)

    else:
        usage(arm)
        sys.exit(1)


if __name__ == "__main__":
    main()
