#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RAICOM 分拣识别：停稳后按空格识别一次（不连续推理，避免 3 秒一帧的卡顿）

用法:
  source ~/ros_ws/devel/setup.zsh
  export ROS_MASTER_URI=http://127.0.0.1:11311
  export ROS_IP=127.0.0.1
  unset ROS_HOSTNAME
  python3 ~/yolo_models/raicom_yolo_oneshot.py

按键:
  空格 / s = 对当前画面推理一次
  q       = 退出
"""

from __future__ import print_function

import os
import sys
import time

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image

# 优先用 320 版（更快）；没有则用原来的 640
ONNX_320 = os.path.expanduser("~/yolo_models/best_fixed_320.onnx")
ONNX_640 = os.path.expanduser("~/yolo_models/best_fixed.onnx")
TOPIC = "/depth_cam/rgb/image_raw"
CONF = 0.50
IOU = 0.45
NAMES = ["gear", "bolt"]


def letterbox(im, new_shape=640, color=(114, 114, 114)):
    shape = im.shape[:2]
    r = min(new_shape / shape[0], new_shape / shape[1])
    new_unpad = (int(round(shape[1] * r)), int(round(shape[0] * r)))
    dw = (new_shape - new_unpad[0]) / 2.0
    dh = (new_shape - new_unpad[1]) / 2.0
    if shape[::-1] != new_unpad:
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return im, r, (dw, dh)


def xywh2xyxy(x):
    y = np.copy(x)
    y[:, 0] = x[:, 0] - x[:, 2] / 2
    y[:, 1] = x[:, 1] - x[:, 3] / 2
    y[:, 2] = x[:, 0] + x[:, 2] / 2
    y[:, 3] = x[:, 1] + x[:, 3] / 2
    return y


def nms(boxes, scores, iou_thres=0.45):
    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    areas = (x2 - x1) * (y2 - y1)
    order = scores.argsort()[::-1]
    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        w = np.maximum(0.0, xx2 - xx1)
        h = np.maximum(0.0, yy2 - yy1)
        inter = w * h
        ovr = inter / (areas[i] + areas[order[1:]] - inter + 1e-6)
        inds = np.where(ovr <= iou_thres)[0]
        order = order[inds + 1]
    return keep


def postprocess(pred, r, dwdh, conf_thres, iou_thres):
    if pred.ndim == 3:
        pred = pred[0]
    if pred.shape[1] < 6:
        return []
    scores = pred[:, 4:5] * pred[:, 5:]
    cls_ids = scores.argmax(1)
    confs = scores.max(1)
    mask = confs > conf_thres
    pred, confs, cls_ids = pred[mask], confs[mask], cls_ids[mask]
    if pred.shape[0] == 0:
        return []
    boxes = xywh2xyxy(pred[:, :4])
    dw, dh = dwdh
    boxes[:, [0, 2]] -= dw
    boxes[:, [1, 3]] -= dh
    boxes /= r
    keep = nms(boxes, confs, iou_thres)
    out = []
    for i in keep:
        x1, y1, x2, y2 = boxes[i]
        out.append((float(x1), float(y1), float(x2), float(y2), float(confs[i]), int(cls_ids[i])))
    return out


class OnnxYolo(object):
    def __init__(self):
        import onnxruntime as ort

        if os.path.isfile(ONNX_320):
            path = ONNX_320
            self.img_size = 320
        elif os.path.isfile(ONNX_640):
            path = ONNX_640
            self.img_size = 640
        else:
            raise FileNotFoundError("找不到 onnx，请放 best_fixed_320.onnx 或 best_fixed.onnx")

        providers = ["CPUExecutionProvider"]
        if "CUDAExecutionProvider" in ort.get_available_providers():
            providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
        self.session = ort.InferenceSession(path, providers=providers)
        self.input_name = self.session.get_inputs()[0].name
        print("ONNX:", path, "size", self.img_size, "providers", self.session.get_providers())

    def infer(self, bgr):
        img, r, dwdh = letterbox(bgr, self.img_size)
        x = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        x = x.transpose(2, 0, 1)[None]
        pred = self.session.run(None, {self.input_name: x})[0]
        return postprocess(pred, r, dwdh, CONF, IOU)


class Node(object):
    def __init__(self):
        rospy.init_node("raicom_yolo_oneshot", anonymous=True)
        self.frame = None
        self.model = OnnxYolo()
        self.last_tip = "SPACE=detect  Q=quit"
        self.last_dets = []
        rospy.Subscriber(TOPIC, Image, self.cb, queue_size=1, buff_size=2**24)
        cv2.namedWindow("raicom_oneshot", cv2.WINDOW_NORMAL)

    def cb(self, msg):
        try:
            if msg.encoding in ("rgb8", "bgr8"):
                raw = np.frombuffer(msg.data[: msg.height * msg.step], dtype=np.uint8)
                img = raw.reshape(msg.height, msg.step)[:, : msg.width * 3]
                img = img.reshape(msg.height, msg.width, 3)
                if msg.encoding == "rgb8":
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            else:
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            self.frame = img
        except Exception as e:
            rospy.logerr_throttle(2.0, str(e))

    def do_detect(self):
        if self.frame is None:
            self.last_tip = "no frame yet"
            return
        frame = self.frame.copy()
        t0 = time.time()
        dets = self.model.infer(frame)
        ms = (time.time() - t0) * 1000
        self.last_dets = dets
        items = []
        for x1, y1, x2, y2, conf, cls_id in dets:
            name = NAMES[cls_id] if 0 <= cls_id < len(NAMES) else str(cls_id)
            items.append(((x1 + x2) / 2.0, name, conf))
        if len(items) >= 2:
            items = sorted(items, key=lambda x: x[0])
            left, right = items[0], items[-1]
            mapping = {left[1]: "P1", right[1]: "P2"}
            self.last_tip = "P1<- %s | %s ->P2  %.0fms" % (left[1], right[1], ms)
            print("==== RESULT ====")
            print(self.last_tip)
            print("mapping:", mapping)
            print("===============")
            # 写入文件，方便程序读取
            with open(os.path.expanduser("~/yolo_models/last_mapping.txt"), "w") as f:
                f.write("%s->P1\n%s->P2\n" % (left[1], right[1]))
        elif len(items) == 1:
            self.last_tip = "only 1: %s  %.0fms" % (items[0][1], ms)
            print(self.last_tip)
        else:
            self.last_tip = "no detect  %.0fms" % ms
            print(self.last_tip)
        cv2.imwrite(os.path.expanduser("~/yolo_models/debug_oneshot.jpg"), self.draw(frame))

    def draw(self, frame):
        draw = frame.copy()
        for x1, y1, x2, y2, conf, cls_id in self.last_dets:
            name = NAMES[cls_id] if 0 <= cls_id < len(NAMES) else str(cls_id)
            color = (0, 255, 0) if name == "gear" else (0, 128, 255)
            cv2.rectangle(draw, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            cv2.putText(
                draw,
                "%s %.2f" % (name, conf),
                (int(x1), max(20, int(y1) - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                color,
                2,
            )
        cv2.putText(draw, self.last_tip, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        return draw

    def spin(self):
        rate = rospy.Rate(30)
        rospy.loginfo("ready: SPACE to detect once")
        while not rospy.is_shutdown():
            if self.frame is not None:
                show = self.draw(self.frame)
                show = cv2.resize(show, None, fx=0.5, fy=0.5)
                cv2.imshow("raicom_oneshot", show)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord(" "), ord("s"), ord("S")):
                print("detecting...")
                self.do_detect()
            elif key in (ord("q"), ord("Q"), 27):
                break
            rate.sleep()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        Node().spin()
    except Exception as e:
        print("ERROR:", e)
        import traceback
        traceback.print_exc()
        sys.exit(1)
