#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RAICOM 机械臂：直接发总线舵机，不依赖你手写 .d6a。

姿势来自同目录 / 车上 ~/yolo_models/raicom_arm_poses.yaml，缺了用下面默认值。

命令行（车上）：
  python3 raicom_arm.py look_board
  python3 raicom_arm.py look_board_b
  python3 raicom_arm.py look_front
  python3 raicom_arm.py pick
  python3 raicom_arm.py place_low
  python3 raicom_arm.py save look_board    # 手柄对准后记录当前脉冲
  python3 raicom_arm.py save look_board_b
  python3 raicom_arm.py save look_front
"""
from __future__ import print_function

import os
import sys
import time

try:
    import yaml
except ImportError:
    yaml = None

DEFAULT_POSES = {
    "look_front": {"joint1": 500, "joint2": 750, "joint3": 0, "joint4": 375, "joint5": 500, "gripper": 500},
    "look_board": {"joint1": 143, "joint2": 518, "joint3": 96, "joint4": 222, "joint5": 536, "gripper": 500},
    "look_board_b": {"joint1": 857, "joint2": 518, "joint3": 96, "joint4": 222, "joint5": 536, "gripper": 500},
    # 抓取：先张开，伸下去，闭合，抬起。数值必须下场微调 yaml。
    "pick_hover": {"joint1": 500, "joint2": 520, "joint3": 180, "joint4": 420, "joint5": 500, "gripper": 750},
    "pick_down": {"joint1": 500, "joint2": 360, "joint3": 260, "joint4": 480, "joint5": 500, "gripper": 750},
    "pick_close": {"joint1": 500, "joint2": 360, "joint3": 260, "joint4": 480, "joint5": 500, "gripper": 250},
    "pick_lift": {"joint1": 500, "joint2": 520, "joint3": 180, "joint4": 420, "joint5": 500, "gripper": 250},
    "place_down_low": {"joint1": 500, "joint2": 380, "joint3": 240, "joint4": 470, "joint5": 500, "gripper": 250},
    "place_down_high": {"joint1": 500, "joint2": 460, "joint3": 200, "joint4": 430, "joint5": 500, "gripper": 250},
    "place_open": {"joint1": 500, "joint2": 380, "joint3": 240, "joint4": 470, "joint5": 500, "gripper": 750},
}

JOINT_IDS = (
    ("joint1", 1),
    ("joint2", 2),
    ("joint3", 3),
    ("joint4", 4),
    ("joint5", 5),
    ("gripper", 10),
)

YAML_CANDIDATES = [
    os.path.expanduser("~/yolo_models/raicom_arm_poses.yaml"),
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "raicom_arm_poses.yaml"),
]
ID_TO_JOINT = dict((sid, name) for name, sid in JOINT_IDS)


def poses_yaml_path():
    home = os.path.expanduser("~/yolo_models/raicom_arm_poses.yaml")
    if os.path.isfile(home) or os.path.isdir(os.path.dirname(home)):
        return home
    return YAML_CANDIDATES[1]


def dump_poses(path, poses):
    lines = ["# RAICOM arm poses. save look_board 会覆盖对应项。\n"]
    order = [
        "look_front", "look_board", "look_board_b",
        "pick_hover", "pick_down", "pick_close", "pick_lift",
        "place_down_low", "place_down_high", "place_open",
    ]
    names = order + [k for k in poses if k not in order]
    for name in names:
        p = poses.get(name)
        if not isinstance(p, dict) or "joint1" not in p:
            continue
        lines.append(
            "%s: {joint1: %d, joint2: %d, joint3: %d, joint4: %d, joint5: %d, gripper: %d}\n"
            % (
                name,
                int(p["joint1"]), int(p["joint2"]), int(p["joint3"]),
                int(p["joint4"]), int(p["joint5"]), int(p.get("gripper", 500)),
            )
        )
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.writelines(lines)


def read_current_joints(timeout=3.0):
    import rospy
    from hiwonder_servo_msgs.msg import ServoStateList

    msg = rospy.wait_for_message(
        "/servo_controllers/port_id_1/servo_states", ServoStateList, timeout=timeout
    )
    pose = {}
    for st in msg.servo_states:
        name = ID_TO_JOINT.get(int(st.id))
        if name:
            pose[name] = int(st.position)
    for key, _sid in JOINT_IDS:
        if key not in pose:
            raise RuntimeError("missing servo %s in feedback" % key)
    return pose


def load_poses():
    poses = dict((k, dict(v)) for k, v in DEFAULT_POSES.items())
    path = None
    for cand in YAML_CANDIDATES:
        if os.path.isfile(cand):
            path = cand
            break
    if path and yaml is not None:
        with open(path, "r") as f:
            data = yaml.safe_load(f) or {}
        extra = data.get("poses") if isinstance(data.get("poses"), dict) else data
        if isinstance(extra, dict):
            for name, vals in extra.items():
                if isinstance(vals, dict) and "joint1" in vals:
                    poses[name] = dict(vals)
    return poses, path


def send_pose(pub, pose, duration=1.5, wait=True):
    from hiwonder_servo_msgs.msg import MultiRawIdPosDur, RawIdPosDur

    items = []
    for key, sid in JOINT_IDS:
        if key in pose:
            items.append(RawIdPosDur(int(sid), int(pose[key]), float(duration)))
    pub.publish(MultiRawIdPosDur(id_pos_dur_list=items))
    if wait:
        time.sleep(float(duration) + 0.15)


class Arm(object):
    def __init__(self, pub=None):
        self.poses, self.path = load_poses()
        self.pub = pub

    def attach(self, pub):
        self.pub = pub

    def go(self, name, duration=1.5):
        if name not in self.poses:
            raise KeyError("unknown pose %s (known: %s)" % (name, ",".join(sorted(self.poses.keys()))))
        if self.pub is None:
            raise RuntimeError("arm publisher not ready")
        send_pose(self.pub, self.poses[name], duration=duration)

    def look_board(self, side="A"):
        self.go("look_board_b" if str(side).upper() == "B" else "look_board", 1.6)

    def look_front(self):
        self.go("look_front", 1.6)

    def pick(self):
        self.go("pick_hover", 1.2)
        self.go("pick_down", 1.0)
        time.sleep(0.15)
        self.go("pick_close", 0.6)
        time.sleep(0.2)
        self.go("pick_lift", 1.0)

    def place(self, height="low"):
        down = "place_down_high" if height == "high" else "place_down_low"
        self.go(down, 1.1)
        time.sleep(0.1)
        open_pose = dict(self.poses[down])
        open_pose["gripper"] = self.poses.get("place_open", {}).get("gripper", 750)
        send_pose(self.pub, open_pose, duration=0.6)
        self.look_front()


def _cli():
    import rospy
    from hiwonder_servo_msgs.msg import MultiRawIdPosDur

    cmd = sys.argv[1] if len(sys.argv) > 1 else ""
    side = "A"
    if len(sys.argv) > 2:
        side = sys.argv[2]
    rospy.init_node("raicom_arm", anonymous=True)
    if cmd == "save":
        name = "look_board"
        if len(sys.argv) > 2:
            name = sys.argv[2]
        pose = read_current_joints()
        poses, _old = load_poses()
        poses[name] = pose
        path = poses_yaml_path()
        dump_poses(path, poses)
        print("SAVED", name, pose)
        print("wrote", path)
        return

    pub = rospy.Publisher("/servo_controllers/port_id_1/multi_id_pos_dur", MultiRawIdPosDur, queue_size=1)
    rospy.sleep(0.4)
    arm = Arm(pub)
    if cmd == "look_board":
        arm.look_board(side)
    elif cmd == "look_board_b":
        arm.look_board("B")
    elif cmd == "look_front":
        arm.look_front()
    elif cmd == "pick":
        arm.pick()
    elif cmd in ("place_low", "place"):
        arm.place("low")
    elif cmd == "place_high":
        arm.place("high")
    else:
        print("usage: raicom_arm.py look_board|look_board_b|look_front|pick|place_low|place_high [A|B]")
        print("       raicom_arm.py save look_board")
        print("poses from:", arm.path or "(defaults)")
        sys.exit(1)
    print("done", cmd)


if __name__ == "__main__":
    _cli()
