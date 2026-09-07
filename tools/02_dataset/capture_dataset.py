#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
JetRover 深度相机采图脚本（ROS1 宿主机用）
用法：
  1. 确认已开机且相机在工作（bringup 自启一般已开相机）
  2. 在车上终端执行：
       source ~/ros_ws/devel/setup.zsh
       python3 ~/Desktop/capture_dataset.py
  3. 窗口里按：
       空格 / s  = 保存一张
       q / ESC   = 退出
照片默认保存在：~/Desktop/dataset_raw/
"""

from __future__ import print_function

import os
import time

import cv2
import rospy
from sensor_msgs.msg import Image

SAVE_DIR = os.path.expanduser("~/Desktop/dataset_raw")
TOPIC = "/depth_cam/rgb/image_raw"


class Capturer(object):
    def __init__(self):
        self.frame = None
        self.count = 0
        if not os.path.isdir(SAVE_DIR):
            os.makedirs(SAVE_DIR)
        # 已有张数，避免覆盖
        existing = [f for f in os.listdir(SAVE_DIR) if f.lower().endswith((".jpg", ".png"))]
        self.count = len(existing)
        rospy.Subscriber(TOPIC, Image, self.cb, queue_size=1)

    def cb(self, msg):
        # ROS Image (rgb8) -> OpenCV BGR
        import numpy as np

        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        self.frame = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    def save(self):
        if self.frame is None:
            print("还没有画面，请稍等…")
            return
        name = "img_{:04d}.jpg".format(self.count)
        path = os.path.join(SAVE_DIR, name)
        cv2.imwrite(path, self.frame)
        self.count += 1
        print("已保存: {}".format(path))


def main():
    rospy.init_node("capture_dataset", anonymous=True)
    cap = Capturer()
    print("等待相机话题 {} …".format(TOPIC))
    print("保存目录: {}".format(SAVE_DIR))
    print("按 空格/s 保存，按 q 退出")

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        if cap.frame is not None:
            show = cap.frame.copy()
            tip = "SPACE/s=save  q=quit  saved={}".format(cap.count)
            cv2.putText(show, tip, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("capture_dataset", show)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord(" "), ord("s"), ord("S")):
                cap.save()
                time.sleep(0.15)  # 防连按
            elif key in (ord("q"), ord("Q"), 27):
                break
        rate.sleep()

    cv2.destroyAllWindows()
    print("结束。共约 {} 张，目录: {}".format(cap.count, SAVE_DIR))


if __name__ == "__main__":
    main()
