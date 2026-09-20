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
    "look_front": {"joint1": 500, "joint2": 667, "joint3": 42, "joint4": 208, "joint5": 500, "gripper": 500},
    # 实车此刻能看见方块顶面贴纸的姿势。侧面没有图案，识别必须用这个角度。
    "look_cargo": {"joint1": 491, "joint2": 380, "joint3": 294, "joint4": 187, "joint5": 500, "gripper": 182},
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


# 厂方 track_and_grab 的手眼矩阵：相机点 -> 夹爪末端。
# 旋转是纯轴向置换（相机装的朝向），装配公差影响不到它；平移是标称值，
# 每台车不一样，用 raicom_handeye.py 标完存进 raicom_handeye.yaml。
_HAND2CAM_FACTORY = (
    (0.0, 0.0, 1.0, -0.101),
    (-1.0, 0.0, 0.0, 0.011),
    (0.0, -1.0, 0.0, 0.045),
    (0.0, 0.0, 0.0, 1.0),
)
HANDEYE_YAML = os.path.expanduser("~/yolo_models/raicom_handeye.yaml")
_hand2cam_cache = {"t": None, "mat": None}


def load_hand2cam():
    """标过就用 yaml 里的平移，否则退回厂方值。按文件 mtime 缓存。"""
    try:
        stamp = os.path.getmtime(HANDEYE_YAML)
    except OSError:
        stamp = None
    if _hand2cam_cache["mat"] is not None and _hand2cam_cache["t"] == stamp:
        return _hand2cam_cache["mat"]

    mat = [list(row) for row in _HAND2CAM_FACTORY]
    if stamp is not None:
        data = _read_yaml_file(HANDEYE_YAML)
        trans = data.get("t")
        if isinstance(trans, (list, tuple)) and len(trans) == 3:
            for i in range(3):
                mat[i][3] = float(trans[i])
    _hand2cam_cache["t"] = stamp
    _hand2cam_cache["mat"] = mat
    return mat
# 车上 /robot_1/depth_cam/rgb/camera_info ，640x360
_GRIP_OPEN = 200
_GRIP_CLOSE = 550


def _quat_to_mat(xyz, wxyz):
    import numpy as np

    w, x, y, z = wxyz
    rot = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    mat = np.eye(4)
    mat[:3, :3] = rot
    mat[:3, 3] = list(xyz)
    return mat


def _camera_intrinsics():
    import rospy
    from sensor_msgs.msg import CameraInfo

    topic = topic_prefix("depth_cam/depth/camera_info") + "depth_cam/depth/camera_info"
    msg = rospy.wait_for_message(topic, CameraInfo, timeout=2.0)
    if len(msg.K) < 6 or msg.K[0] <= 0 or msg.K[4] <= 0:
        raise RuntimeError("invalid camera intrinsics from %s" % topic)
    return float(msg.K[0]), float(msg.K[4]), float(msg.K[2]), float(msg.K[5])


def _depth_meters(nx, ny):
    import numpy as np
    import rospy
    from sensor_msgs.msg import Image

    topic = topic_prefix("depth_cam/depth/image_raw") + "depth_cam/depth/image_raw"
    msg = rospy.wait_for_message(topic, Image, timeout=2.0)
    if msg.encoding == "16UC1":
        depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        lo, hi, scale = 80, 2500, 0.001
    elif msg.encoding == "32FC1":
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        lo, hi, scale = 0.08, 2.5, 1.0
    else:
        raise RuntimeError("unsupported depth %s" % msg.encoding)
    u = int(max(0, min(msg.width - 1, round(float(nx) * (msg.width - 1)))))
    v = int(max(0, min(msg.height - 1, round(float(ny) * (msg.height - 1)))))
    radius = int(rospy.get_param("/raicom/depth_roi_radius", 14))
    patch = depth[
        max(0, v - radius):min(msg.height, v + radius + 1),
        max(0, u - radius):min(msg.width, u + radius + 1),
    ]
    valid = patch[(patch > lo) & (patch < hi)]
    min_valid = int(rospy.get_param("/raicom/depth_min_valid", 12))
    if valid.size < min_valid:
        return None, u, v, 0, None

    # A missing center pixel is common on reflective blocks. Prefer the nearer
    # part of the ROI, then use a median of that subset to reject speckles.
    near_limit = float(np.percentile(valid, 35))
    surface = valid[valid <= near_limit]
    if surface.size < max(5, min_valid // 2):
        surface = valid
    spread = float(np.percentile(surface, 90) - np.percentile(surface, 10)) * scale
    max_spread = float(rospy.get_param("/raicom/depth_max_spread_m", 0.035))
    if spread > max_spread:
        return None, u, v, int(valid.size), spread
    return float(np.median(surface)) * scale, u, v, int(valid.size), spread


def _arm_fk(pulses):
    """当前舵机 -> 末端位姿矩阵。连杆按麦克纳姆厂方长度。"""
    from hiwonder_sdk import common
    from hiwonder_kinematics.forward_kinematics import ForwardKinematics
    from hiwonder_kinematics.inverse_kinematics import set_link
    import hiwonder_kinematics.transform as transform

    fk = ForwardKinematics(debug=False)
    try:
        set_link(0.22736, 0.130, 0.130, 0.055, 0.117)
        fk.set_link(0.22736, 0.130, 0.130, 0.055, 0.117)
    except Exception:
        pass
    res = fk.get_fk(transform.pulse2angle(list(pulses[:5])))
    if not res:
        return None
    quat = res[1]
    return common.xyz_quat_to_mat(
        [float(res[0][0]), float(res[0][1]), float(res[0][2])],
        [float(quat.w), float(quat.x), float(quat.y), float(quat.z)],
    )


def _block_in_arm(nx, ny, pulses):
    """深度打在顶面。转到机械臂坐标后，再下移半个方块，得到中心。"""
    import numpy as np
    from hiwonder_sdk import common

    dist, u, v, valid_count, spread = _depth_meters(nx, ny)
    if dist is None:
        return None, None
    fx, fy, cx, cy = _camera_intrinsics()
    cam = np.array([(u - cx) * dist / fx, (v - cy) * dist / fy, dist], dtype=float)
    cam[0] -= 0.01
    endpoint = _arm_fk(pulses)
    if endpoint is None:
        return None, None
    pose_end = np.matmul(np.array(load_hand2cam(), dtype=float), common.xyz_euler_to_mat(cam, (0, 0, 0)))
    world = np.matmul(endpoint, pose_end)
    top = [float(world[0, 3]), float(world[1, 3]), float(world[2, 3])]
    center = [top[0], top[1], top[2] - 0.025]
    return center, (dist, u, v, top, valid_count, spread)


def _ik_pulses(xyz, current, pitches=(0, 15, 30, 45, 70, 85)):
    """侧向优先：俯仰 0 是水平夹，85 才是从上往下。选离当前舵机最近的解。"""
    import numpy as np
    from hiwonder_kinematics.inverse_kinematics import get_ik
    import hiwonder_kinematics.transform as transform

    cur = np.array(list(current[:5]), dtype=float)
    fallback = None
    fallback_d = 1e9
    fallback_pitch = None
    for pitch in pitches:
        try:
            sols = get_ik([float(xyz[0]), float(xyz[1]), float(xyz[2])], float(pitch), [-180, 180], 1)
        except TypeError:
            sols = get_ik([float(xyz[0]), float(xyz[1]), float(xyz[2])], float(pitch), [-180, 180])
        if not sols:
            continue
        best = None
        best_d = 1e9
        for sol in sols:
            try:
                pulse_solutions = transform.angle2pulse(sol[0])
            except Exception:
                continue
            for pulses in pulse_solutions:
                vals = []
                ok = True
                for raw in pulses:
                    val = float(raw)
                    if val < -30 or val > 1030:
                        ok = False
                        break
                    vals.append(int(max(0, min(1000, round(val)))))
                if not ok or len(vals) < 5:
                    continue
                delta = float(np.sum(np.abs(np.array(vals[:5], dtype=float) - cur)))
                if delta < best_d:
                    best_d = delta
                    best = vals[:5]
        if best is None:
            continue
        if best_d < fallback_d:
            fallback = best
            fallback_d = best_d
            fallback_pitch = pitch
        if best_d < 1400:
            return best, pitch
    return fallback, fallback_pitch


def _pose_of(pulses, gripper):
    return {
        "joint1": int(pulses[0]), "joint2": int(pulses[1]), "joint3": int(pulses[2]),
        "joint4": int(pulses[3]), "joint5": int(pulses[4]), "gripper": int(gripper),
    }


def measure_from_view(nx, ny):
    """Read a target's 3D position without moving the arm."""
    import rospy

    now = read_current_joints()
    pulses = [now[k] for k in ("joint1", "joint2", "joint3", "joint4", "joint5")]
    xyz, extra = _block_in_arm(nx, ny, pulses)
    if xyz is None:
        rospy.logerr("grasp: no depth at nx=%.3f ny=%.3f", nx, ny)
        return None
    dist, u, v, top, valid_count, spread = extra
    rospy.loginfo(
        "block center x=%.3f y=%.3f z=%.3f topz=%.3f depth=%.3f px=%d,%d valid=%d spread=%.3f",
        xyz[0], xyz[1], xyz[2], top[2], dist, u, v, valid_count, spread,
    )
    return {"xyz": xyz, "top": top, "depth": dist, "u": u, "v": v,
            "valid_count": valid_count, "spread": spread, "pulses": pulses}


def measure_stable_from_view(nx, ny, samples=5, max_delta=0.012):
    """Require several close 3D measurements before allowing a grasp."""
    import rospy

    values = []
    deadline = time.time() + float(rospy.get_param("/raicom/depth_stable_timeout", 3.0))
    while len(values) < int(samples) and time.time() < deadline and not rospy.is_shutdown():
        item = measure_from_view(nx, ny)
        if item is not None:
            values.append(item)
        time.sleep(0.08)
    if len(values) < int(samples):
        rospy.logerr("grasp: only %d/%d valid depth samples", len(values), int(samples))
        return None
    import numpy as np

    xyzs = np.array([v["xyz"] for v in values], dtype=float)
    center = np.median(xyzs, axis=0)
    deltas = np.max(np.abs(xyzs - center), axis=1)
    if float(np.max(deltas)) > float(max_delta):
        rospy.logerr("grasp: unstable 3d target max_delta=%.3fm", float(np.max(deltas)))
        return None
    result = dict(values[-1])
    result["xyz"] = [float(v) for v in center]
    result["samples"] = len(values)
    return result


def grasp_from_view(pub, nx, ny):
    """实时三维坐标 + 逆解。不使用 pick_down / pick_hover。"""
    import rospy

    measured = measure_stable_from_view(nx, ny)
    if measured is None:
        return False
    xyz = measured["xyz"]
    pulses = measured["pulses"]
    if not (0.10 < xyz[0] < 0.40 and abs(xyz[1]) < 0.22 and -0.05 < xyz[2] < 0.36):
        rospy.logerr("grasp: xyz out of reach, not moving")
        return False
    approach = [xyz[0] - 0.055, xyz[1], xyz[2]]
    pre, pitch = _ik_pulses(approach, pulses)
    if pre is None:
        rospy.logerr("grasp: no ik for approach")
        return False
    goal, pitch = _ik_pulses(xyz, pre, (pitch,))
    if goal is None:
        goal, pitch = _ik_pulses(xyz, pulses)
    if goal is None:
        rospy.logerr("grasp: no ik")
        return False
    rospy.loginfo("ik pitch=%s approach=%s goal=%s", pitch, pre, goal)
    send_pose(pub, _pose_of([pre[0], pulses[1], pulses[2], pulses[3], pulses[4]], _GRIP_OPEN), duration=0.7)
    send_pose(pub, _pose_of(pre, _GRIP_OPEN), duration=1.2)
    send_pose(pub, _pose_of(goal, _GRIP_OPEN), duration=0.7)
    send_pose(pub, _pose_of(goal, _GRIP_CLOSE), duration=0.6)
    time.sleep(0.15)
    lift, _pitch2 = _ik_pulses([xyz[0], xyz[1], xyz[2] + 0.045], goal, (pitch,))
    if lift:
        send_pose(pub, _pose_of(lift, _GRIP_CLOSE), duration=0.9)
    return True


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

    def close_and_lift(self, layer="high"):
        """已经在侧面张开姿势时闭合再抬起。不再先抬到 hover，避免相机离开贴纸。"""
        hover = dict(self.poses["pick_hover"])
        down_key = "pick_down_low" if layer == "low" else "pick_down"
        down = dict(self.poses.get(down_key) or self.poses["pick_down"])
        close_g = int(self.poses.get("pick_close", {}).get("gripper", 250))
        closed = dict(down)
        closed["gripper"] = close_g
        send_pose(self.pub, closed, duration=0.6)
        time.sleep(0.2)
        lift = dict(hover)
        lift["gripper"] = close_g
        send_pose(self.pub, lift, duration=1.0)

    def close_here_and_lift(self):
        """保持当前关节，只闭合夹爪，再按 hover 抬起。贴纸在顶面，不能先掰到正对侧面。"""
        pose = read_current_joints()
        close_g = int(self.poses.get("pick_close", {}).get("gripper", 250))
        closed = dict(pose)
        closed["gripper"] = close_g
        send_pose(self.pub, closed, duration=0.6)
        time.sleep(0.2)
        lift = dict(self.poses["pick_hover"])
        lift["joint1"] = int(pose.get("joint1", lift["joint1"]))
        lift["gripper"] = close_g
        send_pose(self.pub, lift, duration=1.0)

    def place(self, height="low", park=None):
        """放到指定层。全程夹紧移动，到位后最后才松夹爪，再抬离。"""
        suffix = "high" if height == "high" else "low"
        park_down = "park%d_place_down_%s" % (int(park), suffix) if park is not None else ""
        down = park_down if park_down in self.poses else "place_down_%s" % suffix
        close_g = int(self.poses.get("pick_close", {}).get("gripper", 350))
        open_g = int(self.poses.get("place_open", {}).get("gripper", 100))

        # 移动途中夹爪一律保持闭合，不看 yaml 里 place_down_* 的夹爪值，
        # 避免那里存了半开的旧值把货中途摔下去。
        hold = dict(self.poses[down])
        hold["gripper"] = close_g
        send_pose(self.pub, hold, duration=1.1)
        time.sleep(0.2)

        # 到位了才松
        release = dict(self.poses[down])
        release["gripper"] = open_g
        send_pose(self.pub, release, duration=0.6)
        time.sleep(0.3)

        # 先垂直抬离，再回前视姿势；直接扫回去会碰倒刚放下的货
        away = dict(self.poses["pick_hover"])
        away["gripper"] = open_g
        send_pose(self.pub, away, duration=1.0)
        self.look_front()
        return True


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
