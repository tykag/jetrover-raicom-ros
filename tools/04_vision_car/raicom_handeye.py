#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
手眼标定复测 / 重标（只标平移，旋转沿用厂方轴向）

为什么只标平移：
  _HAND2CAM 的旋转部分全是 0/±1，是相机安装朝向决定的纯轴向置换，装配公差
  影响不到它。平移 (-0.101, 0.011, 0.045) 是厂方标称值，每台车的相机实际位置
  差几毫米到几厘米，这才是 2026-09-20 实车三维抓取夹不准的原因
  （当天已验证深度本身是好的：0.224m，5 帧 spread 4mm，逆解也有解）。

真值从哪来：
  pick_down 是手柄调出来、已经验证能夹住方块的姿势。对它的关节做正解，得到的
  就是「夹爪成功夹住方块时所在的位置」= 方块真实坐标。不需要 ArUco 板。
  这样标出来的结果比教科书手眼标定更适合抓取：正解末端到夹爪钳口之间那段
  未知偏移会被一起吸收掉，因为逆解瞄的也是同一个末端坐标系。

用法（车上，先 source 环境，YOLO 要在跑）：
  python3 raicom_handeye.py show     # 看当前用的矩阵
  python3 raicom_handeye.py check    # 多姿势测同一块，看散布（标定前基线）
  python3 raicom_handeye.py calib    # 标定：相机测量 + 正解真值 + 解平移 + 写 yaml
  python3 raicom_handeye.py verify   # 标定后复测，看残差和散布
  python3 raicom_handeye.py reset    # 删掉标定，回厂方值

标定前必须做到（否则真值不成立）：
  1. 台面放一块方块
  2. raicom_grab_manual.py align     让车对到 pick_aim
  3. 确认此时 ready + close 能夹住（不确定就先试一次，再 open 放回原位）
  4. 之后到标定结束，车和方块都不要动

标定结果存 ~/yolo_models/raicom_handeye.yaml，raicom_arm.py 启动时自动读。
"""
from __future__ import print_function

import os
import sys
import time

import numpy as np
import rospy
import yaml
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from std_msgs.msg import String

sys.path.insert(0, os.path.expanduser("~/yolo_models"))
from raicom_arm import (  # noqa: E402
    Arm,
    HANDEYE_YAML,
    _arm_fk,
    _HAND2CAM_FACTORY,
    load_hand2cam,
    load_poses,
    measure_stable_from_view,
    send_pose,
    servo_cmd_topic,
)

JOINT_KEYS = ("joint1", "joint2", "joint3", "joint4", "joint5")
# 方块中心在顶面下方多少米。和 raicom_arm._block_in_arm 里的 0.025 必须一致。
HALF_BLOCK = 0.025
# check/verify 用的观察姿势：look_cargo 附近小幅扰动，保证方块还在画面里
CHECK_OFFSETS = (
    (0, 0),
    (-22, 0),
    (22, 0),
    (0, -16),
    (0, 16),
)


def wait_target(timeout=6.0, min_conf=0.45):
    """等 YOLO 报一个目标，返回 (nx, ny, name, conf)。"""
    holder = {"t": None}

    def _cb(msg):
        parts = (msg.data or "").split(",")
        if len(parts) >= 4 and float(parts[3]) >= min_conf:
            holder["t"] = (float(parts[1]), float(parts[2]), parts[0], float(parts[3]))

    sub = rospy.Subscriber("/raicom/target", String, _cb, queue_size=1)
    rospy.set_param("/raicom/yolo_mode", "detect")
    deadline = time.time() + timeout
    while time.time() < deadline and not rospy.is_shutdown():
        if holder["t"] is not None:
            break
        rospy.sleep(0.1)
    sub.unregister()
    return holder["t"]


def measure_center(label=""):
    """当前姿势下相机测出的方块中心（机械臂基坐标系）+ 该姿势的正解矩阵。"""
    tgt = wait_target()
    if tgt is None:
        print("  %sYOLO 看不到方块" % label)
        return None
    nx, ny, name, conf = tgt
    got = measure_stable_from_view(nx, ny)
    if got is None:
        print("  %s深度不稳，测不出来（nx=%.3f ny=%.3f）" % (label, nx, ny))
        return None
    fk = _arm_fk(got["pulses"])
    if fk is None:
        print("  %s正解失败" % label)
        return None
    return {
        "center": np.array(got["xyz"], dtype=float),
        "fk": np.array(fk, dtype=float),
        "name": name,
        "conf": conf,
        "depth": got["depth"],
        "nx": nx,
        "ny": ny,
    }


def truth_center(poses):
    """pick_down 关节的正解位置 = 能夹住时夹爪所在处 = 方块真实中心。"""
    down = poses.get("pick_down")
    if not down:
        raise KeyError("yaml 里没有 pick_down")
    pulses = [int(down[k]) for k in JOINT_KEYS]
    fk = _arm_fk(pulses)
    if fk is None:
        raise RuntimeError("pick_down 正解失败")
    return np.array([fk[0][3], fk[1][3], fk[2][3]], dtype=float), pulses


def goto_offset(arm, d_j1, d_j4, duration=1.4):
    """从 look_cargo 出发，joint1/joint4 加个偏移，用来换观察角度。"""
    base = dict(arm.poses["look_cargo"])
    base["joint1"] = int(max(0, min(1000, base["joint1"] + d_j1)))
    base["joint4"] = int(max(0, min(1000, base["joint4"] + d_j4)))
    send_pose(arm.pub, base, duration=duration)
    time.sleep(0.4)


def scatter_report(arm, tag):
    """多姿势测同一块，报散布。散布大说明旋转也不对，光标平移救不了。"""
    print("=== %s：多姿势测同一方块 ===" % tag)
    got = []
    for d_j1, d_j4 in CHECK_OFFSETS:
        goto_offset(arm, d_j1, d_j4)
        item = measure_center("  j1%+d j4%+d: " % (d_j1, d_j4))
        if item is None:
            continue
        c = item["center"]
        print("  j1%+d j4%+d -> (%.3f, %.3f, %.3f)  depth=%.3f" % (
            d_j1, d_j4, c[0], c[1], c[2], item["depth"]))
        got.append(c)

    if len(got) < 2:
        print("  有效测量只有 %d 个，测不出散布。先确认 YOLO 能稳定看到方块。" % len(got))
        return None
    arr = np.array(got)
    spread = arr.max(axis=0) - arr.min(axis=0)
    worst = float(spread.max())
    print("  %d 个姿势，xyz 极差 = (%.3f, %.3f, %.3f) m，最大 %.1f mm" % (
        len(got), spread[0], spread[1], spread[2], worst * 1000))
    return worst


def cmd_show():
    mat = load_hand2cam()
    src = "raicom_handeye.yaml" if os.path.isfile(HANDEYE_YAML) else "厂方默认值"
    print("当前 _HAND2CAM（来自 %s）:" % src)
    for row in mat:
        print("  [%7.4f %7.4f %7.4f %8.4f]" % tuple(row))
    fac = [r[3] for r in _HAND2CAM_FACTORY[:3]]
    cur = [r[3] for r in mat[:3]]
    d = [cur[i] - fac[i] for i in range(3)]
    print("平移相对厂方值偏移: (%+.4f, %+.4f, %+.4f) m" % tuple(d))
    if os.path.isfile(HANDEYE_YAML):
        data = yaml.safe_load(open(HANDEYE_YAML)) or {}
        if data.get("note"):
            print("备注:", data["note"])


def cmd_check(arm):
    worst = scatter_report(arm, "标定前基线")
    if worst is None:
        return
    print("")
    if worst > 0.02:
        print("散布 %.1f mm 偏大。可能是平移错（每个姿势的误差方向不同），"
              "也可能是旋转错。先跑 calib 标平移，再 verify 看散布有没有收下来。"
              % (worst * 1000))
    else:
        print("散布 %.1f mm，姿势间基本一致。接着跑 calib 标平移。" % (worst * 1000))


def cmd_calib(arm):
    poses, path = load_poses()
    print("姿势来自:", path or "(默认值)")

    # 真值：纯计算，不动臂，不会碰到方块
    truth, down_pulses = truth_center(poses)
    print("真值（pick_down %s 正解）= (%.3f, %.3f, %.3f)" % (
        down_pulses, truth[0], truth[1], truth[2]))

    # 测量：抬到 look_cargo 让相机看见顶面贴纸
    print("\n臂到 look_cargo 测量...")
    arm.go("look_cargo", 1.6)
    time.sleep(0.6)
    item = measure_center()
    if item is None:
        print("测量失败，标定中止。原姿势没动过，方块也没碰。")
        return
    meas = item["center"]
    print("相机测得 = (%.3f, %.3f, %.3f)  %s conf=%.2f depth=%.3f" % (
        meas[0], meas[1], meas[2], item["name"], item["conf"], item["depth"]))

    # 基坐标系下的误差，转回手眼平移的修正量
    err = truth - meas
    print("\n基坐标系误差 = (%+.4f, %+.4f, %+.4f) m，模长 %.1f mm" % (
        err[0], err[1], err[2], float(np.linalg.norm(err)) * 1000))

    rot = item["fk"][:3, :3]
    delta = rot.T.dot(err)
    old = np.array([r[3] for r in load_hand2cam()[:3]], dtype=float)
    new = old + delta
    print("手眼平移修正 = (%+.4f, %+.4f, %+.4f) m" % tuple(delta))
    print("平移: (%.4f, %.4f, %.4f) -> (%.4f, %.4f, %.4f)" % (
        old[0], old[1], old[2], new[0], new[1], new[2]))

    if float(np.linalg.norm(delta)) > 0.15:
        print("\n修正量超过 15 cm，不像装配公差。可能真值和测量不是同一块方块，"
              "或者车/方块在中途被动过。没有写文件。")
        return

    data = {
        "t": [round(float(v), 5) for v in new],
        "note": ("%s 实车标定。真值 pick_down=%s 正解，相机在 look_cargo 测量，"
                 "残余基坐标误差 %.1f mm。旋转沿用厂方轴向。"
                 % (time.strftime("%Y-%m-%d"), down_pulses,
                    float(np.linalg.norm(err)) * 1000)),
    }
    with open(HANDEYE_YAML, "w") as f:
        yaml.safe_dump(data, f, default_flow_style=False, allow_unicode=True)
    print("\n已写 %s" % HANDEYE_YAML)
    print("接着跑: python3 raicom_handeye.py verify")


def cmd_verify(arm):
    poses, _path = load_poses()
    truth, down_pulses = truth_center(poses)
    print("真值 = (%.3f, %.3f, %.3f)（pick_down %s）\n" % (
        truth[0], truth[1], truth[2], down_pulses))

    print("=== 标定后：look_cargo 残差 ===")
    arm.go("look_cargo", 1.6)
    time.sleep(0.6)
    item = measure_center()
    if item is None:
        print("测不出来")
    else:
        res = truth - item["center"]
        n = float(np.linalg.norm(res)) * 1000
        print("  测得 (%.3f, %.3f, %.3f)，残差 (%+.4f, %+.4f, %+.4f)，模长 %.1f mm"
              % (item["center"][0], item["center"][1], item["center"][2],
                 res[0], res[1], res[2], n))
        # 夹爪开口约 55mm，方块 50mm，左右只剩 2.5mm/边
        if n <= 5:
            print("  残差 %.1f mm，够抓了。" % n)
        elif n <= 12:
            print("  残差 %.1f mm，勉强。夹爪开口只比方块宽 5 mm，建议再标一次。" % n)
        else:
            print("  残差 %.1f mm，还是夹不准。" % n)

    print("")
    worst = scatter_report(arm, "标定后")
    if worst is not None and worst > 0.02:
        print("")
        print("散布还有 %.1f mm。平移已经标过了还这么散，说明旋转也不对，"
              "得做完整手眼标定（ArUco 板 + 8 个姿势解 AX=XB）。" % (worst * 1000))


def cmd_reset():
    if os.path.isfile(HANDEYE_YAML):
        os.remove(HANDEYE_YAML)
        print("已删除 %s，回到厂方标称值。" % HANDEYE_YAML)
    else:
        print("本来就没有标定文件，用的是厂方值。")


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else ""
    if cmd == "show":
        cmd_show()
        return
    if cmd == "reset":
        cmd_reset()
        return
    if cmd not in ("check", "calib", "verify"):
        print(__doc__)
        sys.exit(1)

    rospy.init_node("raicom_handeye", anonymous=True)
    pub = rospy.Publisher(servo_cmd_topic(), MultiRawIdPosDur, queue_size=1)
    rospy.sleep(0.5)
    arm = Arm(pub)
    if "look_cargo" not in arm.poses:
        print("yaml 里没有 look_cargo，没法测量")
        sys.exit(1)

    try:
        if cmd == "check":
            cmd_check(arm)
        elif cmd == "calib":
            cmd_calib(arm)
        else:
            cmd_verify(arm)
    finally:
        rospy.set_param("/raicom/yolo_mode", "idle")


if __name__ == "__main__":
    main()
