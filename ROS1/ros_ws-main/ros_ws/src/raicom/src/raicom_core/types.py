from collections import namedtuple

Detection = namedtuple(
    "Detection",
    "class_name confidence nx ny x1 y1 x2 y2 track_id stamp",
)
Detection.__new__.__defaults__ = ("", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1, 0.0)

DepthSample = namedtuple(
    "DepthSample",
    "ok depth_m valid_count spread_m reason",
)
DepthSample.__new__.__defaults__ = (False, 0.0, 0, 0.0, "")

ServoCommand = namedtuple("ServoCommand", "vx vy reason")
ServoCommand.__new__.__defaults__ = (0.0, 0.0, "idle")
