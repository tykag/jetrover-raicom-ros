#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RAICOM 检测（ONNX，适合小车 Python3.6）
依赖: pip3 install onnxruntime opencv-python numpy  （若缺什么再装）
或:  pip3 install onnxruntime-gpu

用法:
  source ~/ros_ws/devel/setup.zsh
  export ROS_MASTER_URI=http://127.0.0.1:11311
  export ROS_IP=127.0.0.1
  unset ROS_HOSTNAME
  python3 ~/yolo_models/raicom_yolo_onnx.py
"""

from __future__ import print_function

import os
import sys
import time

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image

ONNX_PATH = os.path.expanduser("~/yolo_models/best_fixed.onnx")
TOPIC = "/depth_cam/rgb/image_raw"
CONF = 0.55
IOU = 0.45
# 必须与导出 ONNX 时一致（你导出的是 640）
IMG_SIZE = 640
# 每隔几帧才推理（2 或 3 可明显降延迟）
INFER_EVERY = 3
# 送入网络前先缩小原图最长边，加快 letterbox（不影响 ONNX 输入尺寸）
PRE_MAX = 480
SHOW_SCALE = 0.5
NAMES = ["gear", "bolt"]


def letterbox(im, new_shape=640, color=(114, 114, 114)):
    shape = im.shape[:2]
    r = min(new_shape / shape[0], new_shape / shape[1])
    new_unpad = (int(round(shape[1] * r)), int(round(shape[0] * r)))
    dw = new_shape - new_unpad[0]
    dh = new_shape - new_unpad[1]
    dw /= 2
    dh /= 2
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


def postprocess(pred, r, dwdh, conf_thres=0.55, iou_thres=0.45):
    # pred: (1, num, 5+nc) or (num, 5+nc)  — YOLOv5
    if pred.ndim == 3:
        pred = pred[0]
    # obj * cls
    if pred.shape[1] < 6:
        return []
    obj = pred[:, 4:5]
    cls_scores = pred[:, 5:]
    scores = obj * cls_scores
    cls_ids = scores.argmax(1)
    confs = scores.max(1)
    mask = confs > conf_thres
    pred = pred[mask]
    confs = confs[mask]
    cls_ids = cls_ids[mask]
    if pred.shape[0] == 0:
        return []
    boxes = xywh2xyxy(pred[:, :4])
    # undo letterbox
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
    def __init__(self, path):
        import onnxruntime as ort

        if not os.path.isfile(path):
            raise FileNotFoundError("找不到 ONNX: %s" % path)
        so = ort.SessionOptions()
        providers = ["CPUExecutionProvider"]
        avail = ort.get_available_providers()
        if "CUDAExecutionProvider" in avail:
            providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
        self.session = ort.InferenceSession(path, so, providers=providers)
        self.input_name = self.session.get_inputs()[0].name
        print("ONNX loaded:", path, "providers:", self.session.get_providers())

    def infer(self, bgr):
        # 先缩小再 letterbox，减轻 CPU 负担
        h, w = bgr.shape[:2]
        m = max(h, w)
        if m > PRE_MAX:
            s = float(PRE_MAX) / m
            bgr = cv2.resize(bgr, (int(w * s), int(h * s)))
        img, r, dwdh = letterbox(bgr, IMG_SIZE)
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        x = rgb.astype(np.float32) / 255.0
        x = x.transpose(2, 0, 1)[None]
        pred = self.session.run(None, {self.input_name: x})[0]
        # 坐标映射回缩放前尺寸
        dets = postprocess(pred, r, dwdh, CONF, IOU)
        if m > PRE_MAX:
            inv = m / float(PRE_MAX)
            dets = [
                (a * inv, b * inv, c * inv, d * inv, conf, cls)
                for (a, b, c, d, conf, cls) in dets
            ]
        return dets


class Node(object):
    def __init__(self):
        rospy.init_node("raicom_yolo_onnx", anonymous=True)
        self.frame = None
        self._got = False
        self._frame_id = 0
        self._last_dets = []
        self._last_ms = 0.0
        self.model = OnnxYolo(ONNX_PATH)
        # buff_size 小一点，尽量拿最新帧，减少排队延迟
        rospy.Subscriber(TOPIC, Image, self.cb, queue_size=1, buff_size=2**24)
        cv2.namedWindow("raicom_onnx", cv2.WINDOW_NORMAL)
        rospy.loginfo(
            "speed cfg: IMG_SIZE=%d INFER_EVERY=%d", IMG_SIZE, INFER_EVERY
        )

    def cb(self, msg):
        # 兼容 rgb8 / bgr8，以及 step 对齐；只保留最新一帧
        try:
            if msg.encoding in ("rgb8", "bgr8"):
                n = msg.height * msg.step
                raw = np.frombuffer(msg.data[:n], dtype=np.uint8)
                img = raw.reshape(msg.height, msg.step)
                img = img[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
                if msg.encoding == "rgb8":
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            else:
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3
                )
                if "rgb" in msg.encoding.lower():
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            self.frame = img
            self._frame_id += 1
            if not self._got:
                self._got = True
                mean = float(img.mean())
                rospy.loginfo(
                    "got frame %dx%d enc=%s mean=%.1f",
                    msg.width,
                    msg.height,
                    msg.encoding,
                    mean,
                )
                cv2.imwrite(
                    os.path.expanduser("~/yolo_models/debug_raw.jpg"), img
                )
        except Exception as e:
            rospy.logerr_throttle(2.0, "image convert fail: %s", e)

    def spin(self):
        rate = rospy.Rate(30)
        rospy.loginfo("waiting %s", TOPIC)
        while not rospy.is_shutdown():
            if self.frame is None:
                rate.sleep()
                continue

            # 取当前最新帧，避免处理积压旧图
            frame = self.frame
            fid = self._frame_id

            # 跳帧推理
            if fid % INFER_EVERY == 0:
                t0 = time.time()
                self._last_dets = self.model.infer(frame)
                self._last_ms = (time.time() - t0) * 1000

            dets = self._last_dets
            draw = frame.copy()
            items = []
            for x1, y1, x2, y2, conf, cls_id in dets:
                name = NAMES[cls_id] if 0 <= cls_id < len(NAMES) else str(cls_id)
                cx = (x1 + x2) / 2.0
                items.append((cx, name, conf))
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

            # OpenCV 不支持中文，用英文避免 ????
            tip = "%d det | %.0fms" % (len(items), self._last_ms)
            if len(items) >= 2:
                items = sorted(items, key=lambda x: x[0])
                tip = "P1<- %s | %s ->P2  %.0fms" % (
                    items[0][1],
                    items[-1][1],
                    self._last_ms,
                )
                rospy.loginfo_throttle(1.0, tip)

            cv2.putText(
                draw, tip, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2
            )

            if SHOW_SCALE != 1.0:
                draw = cv2.resize(draw, None, fx=SHOW_SCALE, fy=SHOW_SCALE)

            cv2.imshow("raicom_onnx", draw)
            if (cv2.waitKey(1) & 0xFF) in (ord("q"), 27):
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
