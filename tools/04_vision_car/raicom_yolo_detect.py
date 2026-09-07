#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RAICOM bolt/gear 检测（ROS1 + YOLOv5）
用法（小车 ROS1 终端）：
  source ~/ros_ws/devel/setup.zsh
  export ROS_MASTER_URI=http://127.0.0.1:11311
  export ROS_IP=127.0.0.1
  unset ROS_HOSTNAME
  python3 ~/yolo_models/raicom_yolo_detect.py

按 q 退出。
"""

from __future__ import print_function

import os
import sys

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image

# ===== 按你车上的路径改 =====
WEIGHTS = os.path.expanduser("~/yolo_models/best_fixed.pt")
YOLOV5_DIR = os.path.expanduser("~/yolov5")  # 若用本地 yolov5 仓库
TOPIC = "/depth_cam/rgb/image_raw"
CONF = 0.55
IMG_SIZE = 640
# best_fixed.pt 名字顺序：0=gear, 1=bolt
CLASS_NAMES = {0: "gear", 1: "bolt"}


def load_model():
    import torch

    if not os.path.isfile(WEIGHTS):
        raise FileNotFoundError("找不到模型: %s" % WEIGHTS)

    # 优先本地 yolov5；没有再试 torch.hub 在线
    if os.path.isdir(YOLOV5_DIR):
        model = torch.hub.load(
            YOLOV5_DIR, "custom", path=WEIGHTS, source="local", force_reload=False
        )
    else:
        model = torch.hub.load(
            "ultralytics/yolov5", "custom", path=WEIGHTS, force_reload=False
        )

    model.conf = CONF
    model.iou = 0.45
    # 强制显示名（与 best_fixed 一致）
    try:
        model.names = CLASS_NAMES
    except Exception:
        pass
    model.eval()
    return model


class RaicomYoloDetect(object):
    def __init__(self):
        rospy.init_node("raicom_yolo_detect", anonymous=True)
        self.frame = None
        self.model = load_model()
        rospy.loginfo("model loaded: %s", WEIGHTS)
        rospy.Subscriber(TOPIC, Image, self.cb, queue_size=1)

    def cb(self, msg):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        # ROS 常见为 rgb8
        self.frame = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    def spin(self):
        rate = rospy.Rate(20)
        rospy.loginfo("waiting for %s ...", TOPIC)
        while not rospy.is_shutdown():
            if self.frame is None:
                rate.sleep()
                continue

            frame = self.frame.copy()
            results = self.model(frame, size=IMG_SIZE)
            # results.xyxy[0]: x1,y1,x2,y2,conf,cls
            dets = results.xyxy[0].cpu().numpy() if hasattr(results, "xyxy") else []

            items = []
            for d in dets:
                x1, y1, x2, y2, conf, cls_id = d[:6]
                cls_id = int(cls_id)
                name = CLASS_NAMES.get(cls_id, str(cls_id))
                cx = (x1 + x2) / 2.0
                items.append((cx, name, float(conf), int(x1), int(y1), int(x2), int(y2)))
                color = (0, 255, 0) if name == "gear" else (255, 128, 0)
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                cv2.putText(
                    frame,
                    "%s %.2f" % (name, conf),
                    (int(x1), max(20, int(y1) - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    color,
                    2,
                )

            # 分拣板：按左右判定园区（至少检出 2 个时）
            tip = "detect: %d" % len(items)
            if len(items) >= 2:
                items_sorted = sorted(items, key=lambda x: x[0])
                left = items_sorted[0]
                right = items_sorted[-1]
                tip = "园区一<- %s | %s ->园区二" % (left[1], right[1])
                rospy.loginfo_throttle(1.0, tip)

            cv2.putText(
                frame, tip, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2
            )
            cv2.imshow("raicom_yolo", frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
            rate.sleep()

        cv2.destroyAllWindows()


def main():
    try:
        node = RaicomYoloDetect()
        node.spin()
    except Exception as e:
        print("ERROR:", e)
        import traceback

        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
