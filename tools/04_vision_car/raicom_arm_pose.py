#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RAICOM 看板 / 抓取臂姿势（.d6a 动作组）

【车上安装一次】
  python3 raicom_arm_pose.py install

【调用】
  python3 raicom_arm_pose.py look_board    # A 半场：右转约 90° 看分拣板
  python3 raicom_arm_pose.py look_front    # 转回正前方，看待派送
  python3 raicom_arm_pose.py look_board_b  # B 半场：左转约 90°

手柄微调后若 joint1 不是 875，安装时改：
  python3 raicom_arm_pose.py install --j1-right 860
"""
from __future__ import print_function

import argparse
import os
import sqlite3
import sys
import time

ACTION_DIRS = [
    os.path.expanduser("~/software/arm_pc/ActionGroups"),
    os.path.expanduser("~/share/arm_pc/ActionGroups"),
    os.path.expanduser("~/yolo_models/ActionGroups"),
]

# 平放向前（与 joystick HOME 一致）
POSE_FRONT = {
    "joint1": 500,
    "joint2": 750,
    "joint3": 0,
    "joint4": 375,
    "joint5": 500,
    "gripper": 500,
}
DURATION_MS = 1500


def pulse_90(center=500, span=500, deg=90.0, max_deg=120.0):
    return int(round(center + span * (deg / max_deg)))


def pose_row(pose, duration_ms):
    return (
        pose["joint1"],
        pose["joint2"],
        pose["joint3"],
        pose["joint4"],
        pose["joint5"],
        pose["gripper"],
        duration_ms,
    )


def write_d6a(path, frames):
    """frames: list of (j1,j2,j3,j4,j5,grip, duration_ms)"""
    if os.path.exists(path):
        os.remove(path)
    conn = sqlite3.connect(path)
    cu = conn.cursor()
    cu.execute(
        "CREATE TABLE ActionGroup("
        "[Index] INTEGER PRIMARY KEY NOT NULL,"
        "Time INT,"
        "Servo1 INT, Servo2 INT, Servo3 INT,"
        "Servo4 INT, Servo5 INT, Servo6 INT)"
    )
    for i, (j1, j2, j3, j4, j5, grip, dur) in enumerate(frames, start=1):
        cu.execute(
            "INSERT INTO ActionGroup VALUES (?,?,?,?,?,?,?,?)",
            (i, int(dur), int(j1), int(j2), int(j3), int(j4), int(j5), int(grip)),
        )
    conn.commit()
    conn.close()


def collect_dirs(create=False):
    dirs = []
    for d in ACTION_DIRS:
        if os.path.isdir(d):
            dirs.append(d)
        elif create and d.endswith("yolo_models/ActionGroups"):
            os.makedirs(d, exist_ok=True)
            dirs.append(d)
    # 至少保证 yolo_models 一份
    yolo = os.path.expanduser("~/yolo_models/ActionGroups")
    if yolo not in dirs:
        if create:
            os.makedirs(yolo, exist_ok=True)
        if os.path.isdir(yolo):
            dirs.append(yolo)
    return dirs


def install(j1_right, j1_left):
    front = dict(POSE_FRONT)
    board = dict(POSE_FRONT)
    board_b = dict(POSE_FRONT)
    board["joint1"] = j1_right
    board_b["joint1"] = j1_left

    poses = {
        "look_front": [pose_row(front, DURATION_MS)],
        "look_board": [pose_row(board, DURATION_MS)],
        "look_board_b": [pose_row(board_b, DURATION_MS)],
    }
    dirs = collect_dirs(create=True)
    if not dirs:
        print("找不到可写目录，请先 mkdir -p ~/yolo_models/ActionGroups")
        sys.exit(1)
    for name, frames in poses.items():
        for d in dirs:
            path = os.path.join(d, name + ".d6a")
            write_d6a(path, frames)
            print("wrote", path)
    print("OK  look_board joint1=%d  look_board_b joint1=%d  look_front joint1=500" % (j1_right, j1_left))


def find_d6a(name):
    for d in collect_dirs(create=False):
        path = os.path.join(d, name + ".d6a")
        if os.path.isfile(path):
            return path
    return None


def play_d6a(path):
    import rospy
    from hiwonder_servo_msgs.msg import MultiRawIdPosDur, RawIdPosDur

    rospy.init_node("raicom_arm_pose", anonymous=True)
    pub = rospy.Publisher(
        "/servo_controllers/port_id_1/multi_id_pos_dur",
        MultiRawIdPosDur,
        queue_size=1,
    )
    rospy.sleep(0.4)
    conn = sqlite3.connect(path)
    cu = conn.cursor()
    cu.execute("select * from ActionGroup")
    print("play", path)
    while True:
        act = cu.fetchone()
        if act is None:
            break
        duration = float(act[1]) / 1000.0
        positions = []
        for i in range(0, len(act) - 2, 1):
            sid = 10 if i + 1 == 6 else i + 1
            positions.append(RawIdPosDur(int(sid), int(act[2 + i]), duration))
        pub.publish(MultiRawIdPosDur(id_pos_dur_list=positions))
        time.sleep(duration)
    cu.close()
    conn.close()
    print("done")


def main():
    p = argparse.ArgumentParser(description="RAICOM arm look poses")
    p.add_argument(
        "cmd",
        choices=["install", "look_board", "look_front", "look_board_b"],
        help="install=写入动作组；其余=播放",
    )
    p.add_argument("--j1-right", type=int, default=pulse_90(), help="A半场右转 joint1，默认约875")
    p.add_argument("--j1-left", type=int, default=pulse_90(deg=-90.0), help="B半场左转 joint1，默认约125")
    args = p.parse_args()

    if args.cmd == "install":
        install(args.j1_right, args.j1_left)
        return

    path = find_d6a(args.cmd)
    if path is None:
        print("没有 %s.d6a，先运行: python3 raicom_arm_pose.py install" % args.cmd)
        sys.exit(1)
    play_d6a(path)


if __name__ == "__main__":
    main()
