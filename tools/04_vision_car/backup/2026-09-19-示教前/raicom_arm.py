#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RAICOM 机械臂：直接发总线舵机，不依赖你手写 .d6a。

姿势来自同目录 / 车上 ~/yolo_models/raicom_arm_poses.yaml，缺了用下面默认值。

命令行（车上）：
  python3 raicom_arm.py look_board
  python3 raicom_arm.py look_board_b
  python3 raicom_arm.py look_front
  python3 raicom_arm.py pick            # 上层；第二层: pick_low
  python3 raicom_arm.py place_low
  python3 raicom_arm.py save look_board    # 手柄对准后记录当前脉冲
  python3 raicom_arm.py save pick_hover
  python3 raicom_arm.py save pick_down
  python3 raicom_arm.py save pick_down_low
  python3 raicom_arm.py save_aim           # 记下当前 YOLO 框中心（对位靶子）
"""
from __future__ import print_function

import os
import sys
import time

try:
    import yaml
except ImportError:
    yaml = None

DEFAULT_AIM = {"nx": 0.50, "ny": 0.62, "x_sign": 1.0, "y_sign": 1.0}

DEFAULT_POSES = {
    "look_front": {"joint1": 500, "joint2": 750, "joint3": 0, "joint4": 375, "joint5": 500, "gripper": 500},
    "look_board": {"joint1": 143, "joint2": 518, "joint3": 96, "joint4": 222, "joint5": 536, "gripper": 500},
    "look_board_b": {"joint1": 857, "joint2": 518, "joint3": 96, "joint4": 222, "joint5": 536, "gripper": 500},
    # 抓取：先张开，伸下去，闭合，抬起。数值必须下场微调 yaml。
    "pick_hover": {"joint1": 500, "joint2": 520, "joint3": 180, "joint4": 420, "joint5": 500, "gripper": 750},
    "pick_down": {"joint1": 500, "joint2": 360, "joint3": 260, "joint4": 480, "joint5": 500, "gripper": 750},
    "pick_down_low": {"joint1": 500, "joint2": 300, "joint3": 280, "joint4": 500, "joint5": 500, "gripper": 750},
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


def topic_prefix(suffix):
    """导航在 robot_1 下时，舵机话题是 /robot_1/servo_controllers/..."""
    import rospy
    hits = []
    try:
        topics = rospy.get_published_topics()
    except Exception:
        return "/"
    for name, _typ in topics:
        if name.endswith(suffix):
            hits.append(name[: -len(suffix)])
    for prefix in hits:
        if "robot_" in prefix:
            return prefix
    if hits:
        return hits[0]
    return "/"


def servo_cmd_topic():
    prefix = topic_prefix("servo_controllers/port_id_1/multi_id_pos_dur")
    if prefix == "/":
        prefix = topic_prefix("servo_controllers/port_id_1/servo_states")
    return prefix + "servo_controllers/port_id_1/multi_id_pos_dur"


def servo_state_topic():
    return topic_prefix("servo_controllers/port_id_1/servo_states") + "servo_controllers/port_id_1/servo_states"


def cmd_vel_topic():
    return topic_prefix("hiwonder_controller/cmd_vel") + "hiwonder_controller/cmd_vel"


def nudge_to_jaws(timeout=10.0):
    """夹紧前用底盘把方块对进张开的夹爪。5cm 方块、张开约 5.5cm，不能只靠手臂。"""
    import rospy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String

    os.system("rosnode kill /joystick_control >/dev/null 2>&1")
    rospy.sleep(0.25)
    rospy.set_param("/raicom/yolo_mode", "detect")
    holder = {"t": None}
    def _cb(msg):
        parts = (msg.data or "").split(",")
        if len(parts) >= 4:
            holder["t"] = (parts[0], float(parts[1]), float(parts[2]), float(parts[3]), time.time())
    rospy.Subscriber("/raicom/target", String, _cb, queue_size=1)
    pub = rospy.Publisher(cmd_vel_topic(), Twist, queue_size=1)
    rospy.sleep(0.4)
    aim = load_aim()
    aim_nx, aim_ny = float(aim["nx"]), float(aim["ny"])
    x_sign = float(aim.get("x_sign", 1.0))
    y_sign = float(aim.get("y_sign", 1.0))
    print("nudge to jaws nx=%.3f ny=%.3f" % (aim_nx, aim_ny))
    t0 = time.time()
    stable = 0
    rate_sleep = 0.1
    while time.time() - t0 < timeout:
        t = holder["t"]
        tw = Twist()
        if not t or (time.time() - t[4]) > 0.6 or t[3] < 0.40:
            pub.publish(tw)
            stable = 0
            time.sleep(rate_sleep)
            continue
        ex = t[1] - aim_nx
        ey = t[2] - aim_ny
        vx = x_sign * (-0.22) * ey
        vy = y_sign * (-0.22) * ex
        vx = max(-0.03, min(0.03, vx))
        vy = max(-0.03, min(0.03, vy))
        if abs(ex) < 0.015:
            vy = 0.0
        if abs(ey) < 0.015:
            vx = 0.0
        tw.linear.x = vx
        tw.linear.y = vy
        pub.publish(tw)
        if vx == 0.0 and vy == 0.0:
            stable += 1
        else:
            stable = 0
        print("nudge %s ex=%.3f ey=%.3f vx=%.3f vy=%.3f" % (t[0], ex, ey, vx, vy))
        if stable >= 5:
            pub.publish(Twist())
            print("nudge ok")
            return True
        time.sleep(rate_sleep)
    pub.publish(Twist())
    print("nudge timeout, close anyway")
    return False


def poses_yaml_path():
    home = os.path.expanduser("~/yolo_models/raicom_arm_poses.yaml")
    if os.path.isfile(home) or os.path.isdir(os.path.dirname(home)):
        return home
    return YAML_CANDIDATES[1]


def _read_yaml_file(path):
    if not path or not os.path.isfile(path) or yaml is None:
        return {}
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    return data if isinstance(data, dict) else {}


def load_aim(path=None):
    aim = dict(DEFAULT_AIM)
    for cand in ([path] if path else []) + YAML_CANDIDATES:
        if not cand or not os.path.isfile(cand):
            continue
        raw = _read_yaml_file(cand).get("pick_aim")
        if isinstance(raw, dict) and "nx" in raw and "ny" in raw:
            aim["nx"] = float(raw["nx"])
            aim["ny"] = float(raw["ny"])
            aim["x_sign"] = float(raw.get("x_sign", 1.0))
            aim["y_sign"] = float(raw.get("y_sign", 1.0))
            break
    return aim


def dump_poses(path, poses, aim=None):
    if aim is None:
        aim = load_aim(path)
    lines = [
        "# RAICOM arm poses. save / save_aim 会覆盖对应项。\n",
        "# pick_aim = 货在夹爪正前方时的 YOLO 框中心；对位用。左右反了改 y_sign: -1\n",
    ]
    order = [
        "look_front", "look_board", "look_board_b",
        "pick_hover", "pick_down", "pick_down_low", "pick_close", "pick_lift",
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
    lines.append(
        "pick_aim: {nx: %.3f, ny: %.3f, x_sign: %g, y_sign: %g}\n"
        % (float(aim["nx"]), float(aim["ny"]), float(aim.get("x_sign", 1)), float(aim.get("y_sign", 1)))
    )
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.writelines(lines)


def read_current_joints(timeout=3.0):
    import rospy
    from hiwonder_servo_msgs.msg import ServoStateList

    topic = servo_state_topic()
    print("read joints from", topic)
    msg = rospy.wait_for_message(topic, ServoStateList, timeout=timeout)
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

    def pick(self, layer="high"):
        # 关节用示教的 hover/down；闭合/抬起不再读独立脉冲，避免只存了前两个就跳回默认值
        hover = dict(self.poses["pick_hover"])
        down_key = "pick_down_low" if layer == "low" else "pick_down"
        down = dict(self.poses.get(down_key) or self.poses["pick_down"])
        close_g = int(self.poses.get("pick_close", {}).get("gripper", 250))
        send_pose(self.pub, hover, duration=1.2)
        send_pose(self.pub, down, duration=1.0)
        time.sleep(0.2)
        nudge_to_jaws()
        closed = dict(down)
        closed["gripper"] = close_g
        send_pose(self.pub, closed, duration=0.6)
        time.sleep(0.2)
        lift = dict(hover)
        lift["gripper"] = close_g
        send_pose(self.pub, lift, duration=1.0)

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
    if cmd == "save_aim":
        from std_msgs.msg import String
        msg = rospy.wait_for_message("/raicom/target", String, timeout=6.0)
        parts = (msg.data or "").split(",")
        if len(parts) < 3:
            raise RuntimeError("bad /raicom/target: %s" % msg.data)
        path = poses_yaml_path()
        poses, _old = load_poses()
        aim = load_aim(path)
        aim["nx"] = float(parts[1])
        aim["ny"] = float(parts[2])
        dump_poses(path, poses, aim=aim)
        print("SAVED pick_aim", aim, "from", parts[0])
        print("wrote", path)
        return

    cmd_topic = servo_cmd_topic()
    print("servo cmd", cmd_topic)
    pub = rospy.Publisher(cmd_topic, MultiRawIdPosDur, queue_size=1)
    rospy.sleep(0.4)
    arm = Arm(pub)
    if cmd == "look_board":
        arm.look_board(side)
    elif cmd == "look_board_b":
        arm.look_board("B")
    elif cmd == "look_front":
        arm.look_front()
    elif cmd == "pick":
        arm.pick("high")
    elif cmd == "pick_low":
        arm.pick("low")
    elif cmd in ("place_low", "place"):
        arm.place("low")
    elif cmd == "place_high":
        arm.place("high")
    else:
        print("usage: raicom_arm.py look_board|look_board_b|look_front|pick|pick_low|place_low|place_high [A|B]")
        print("       raicom_arm.py save pick_hover|pick_down|pick_down_low|look_board")
        print("       raicom_arm.py save_aim")
        print("poses from:", arm.path or "(defaults)")
        sys.exit(1)
    print("done", cmd)


if __name__ == "__main__":
    _cli()
