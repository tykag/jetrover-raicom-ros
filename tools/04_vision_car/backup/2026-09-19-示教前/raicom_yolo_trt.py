#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RAICOM 全自动检测（TensorRT engine，连续运行）

【车上先生成 engine】（只需做一次）:
  /usr/src/tensorrt/bin/trtexec \
    --onnx=/home/hiwonder/yolo_models/best_fixed_320.onnx \
    --saveEngine=/home/hiwonder/yolo_models/best_fixed_320.engine \
    --fp16

【运行】:
  source ~/ros_ws/devel/setup.zsh
  export ROS_MASTER_URI=http://127.0.0.1:11311
  export ROS_IP=127.0.0.1
  unset ROS_HOSTNAME
  python3 ~/yolo_models/raicom_yolo_trt.py

逻辑:
  - 连续订阅相机并推理
  - 稳定检出左右两个目标后，写入 last_mapping.txt，并设置 ROS 参数
  - /raicom/mapping_ready == true 时表示园区映射已锁定，可供全自动程序读取
"""

from __future__ import print_function

import os
import sys
import time
import threading

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool

ENGINE_320 = os.path.expanduser("~/yolo_models/best_fixed_320.engine")
ENGINE_640 = os.path.expanduser("~/yolo_models/best_fixed.engine")
ONNX_320 = os.path.expanduser("~/yolo_models/best_fixed_320.onnx")
CONF = 0.50
IOU = 0.45
NAMES = ["gear", "bolt"]
# 连续多少次结果一致才锁定映射（防抖）
STABLE_NEED = 3
SHOW = True  # 比赛可改 False 略省一点
# mapping=锁P1/P2；detect=待派送单目标；idle=不推理
DEFAULT_MODE = "mapping"


def letterbox(im, new_shape=320, color=(114, 114, 114)):
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
    if pred.shape[-1] < 6 and pred.shape[0] >= 6:
        pred = pred.T
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


class TrtYolo(object):
    def __init__(self):
        import tensorrt as trt
        import pycuda.driver as cuda
        import pycuda.autoinit  # noqa: F401

        self.cuda = cuda
        self.trt = trt

        if os.path.isfile(ENGINE_320):
            path = ENGINE_320
            self.img_size = 320
        elif os.path.isfile(ENGINE_640):
            path = ENGINE_640
            self.img_size = 640
        else:
            raise FileNotFoundError(
                "找不到 .engine。请先用 trtexec 从 onnx 生成。\n"
                "期望: %s 或 %s" % (ENGINE_320, ENGINE_640)
            )

        logger = trt.Logger(trt.Logger.WARNING)
        with open(path, "rb") as f, trt.Runtime(logger) as runtime:
            self.engine = runtime.deserialize_cuda_engine(f.read())
        self.context = self.engine.create_execution_context()

        self.inputs, self.outputs, self.bindings, self.stream = [], [], [], cuda.Stream()
        for i in range(self.engine.num_bindings):
            shape = self.engine.get_binding_shape(i)
            size = trt.volume(shape)
            dtype = trt.nptype(self.engine.get_binding_dtype(i))
            host = cuda.pagelocked_empty(size, dtype)
            device = cuda.mem_alloc(host.nbytes)
            self.bindings.append(int(device))
            if self.engine.binding_is_input(i):
                self.inputs.append({"host": host, "device": device, "shape": tuple(shape)})
            else:
                self.outputs.append({"host": host, "device": device, "shape": tuple(shape)})

        print("TRT loaded:", path, "img_size", self.img_size)

    def infer(self, bgr):
        img, r, dwdh = letterbox(bgr, self.img_size)
        x = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        x = np.ascontiguousarray(x.transpose(2, 0, 1).ravel())
        np.copyto(self.inputs[0]["host"], x)
        self.cuda.memcpy_htod_async(self.inputs[0]["device"], self.inputs[0]["host"], self.stream)
        self.context.execute_async_v2(bindings=self.bindings, stream_handle=self.stream.handle)
        for out in self.outputs:
            self.cuda.memcpy_dtoh_async(out["host"], out["device"], self.stream)
        self.stream.synchronize()
        pred = self.outputs[0]["host"].reshape(self.outputs[0]["shape"])
        return postprocess(pred, r, dwdh, CONF, IOU)


class Node(object):
    def __init__(self):
        rospy.init_node("raicom_yolo_trt", anonymous=True)
        self.lock = threading.Lock()
        self.frame = None
        self.model = TrtYolo()
        self.stable = []
        self.locked = None  # (left_name, right_name)
        self.last_target = None
        self.pub_map = rospy.Publisher("/raicom/mapping", String, queue_size=1, latch=True)
        self.pub_ready = rospy.Publisher("/raicom/mapping_ready", Bool, queue_size=1, latch=True)
        self.pub_target = rospy.Publisher("/raicom/target", String, queue_size=1)
        self.pub_ready.publish(Bool(data=False))
        rospy.set_param("/raicom/mapping_ready", False)
        if not rospy.has_param("/raicom/yolo_mode"):
            rospy.set_param("/raicom/yolo_mode", DEFAULT_MODE)
        rospy.Subscriber("/depth_cam/rgb/image_raw", Image, self.cb, queue_size=1, buff_size=2**24)
        show = bool(rospy.get_param("~show", SHOW))
        self.show = show
        if show:
            cv2.namedWindow("raicom_trt", cv2.WINDOW_NORMAL)

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
            with self.lock:
                self.frame = img
        except Exception as e:
            rospy.logerr_throttle(2.0, str(e))

    def try_lock_mapping(self, items):
        if len(items) < 2:
            self.stable = []
            return
        items = sorted(items, key=lambda x: x[0])
        left, right = items[0][1], items[-1][1]
        if left == right:
            self.stable = []
            return
        key = (left, right)
        if self.stable and self.stable[-1] == key:
            self.stable.append(key)
        else:
            self.stable = [key]
        if len(self.stable) >= STABLE_NEED and self.locked != key:
            self.locked = key
            text = "%s->P1;%s->P2" % (left, right)
            path = os.path.expanduser("~/yolo_models/last_mapping.txt")
            with open(path, "w") as f:
                f.write("%s->P1\n%s->P2\n" % (left, right))
            self.pub_map.publish(String(data=text))
            self.pub_ready.publish(Bool(data=True))
            rospy.set_param("/raicom/mapping_ready", True)
            rospy.set_param("/raicom/p1_class", left)
            rospy.set_param("/raicom/p2_class", right)
            rospy.loginfo("MAPPING LOCKED: %s", text)

    def maybe_reset(self):
        if not rospy.get_param("/raicom/reset_yolo", False):
            return
        self.locked = None
        self.stable = []
        self.last_target = None
        self.pub_ready.publish(Bool(data=False))
        rospy.set_param("/raicom/mapping_ready", False)
        rospy.set_param("/raicom/reset_yolo", False)
        rospy.set_param("/raicom/lock_nx", -1.0)
        rospy.set_param("/raicom/lock_ny", -1.0)
        rospy.loginfo("YOLO mapping reset")

    def choose_item(self, items, fw, fh):
        """有 lock 则跟同一块；否则取画面里更近的（ny 大）。"""
        scored = []
        for cx, name, conf, x1, y1, x2, y2 in items:
            nx = max(0.0, min(1.0, cx / float(max(fw, 1))))
            ny = max(0.0, min(1.0, ((y1 + y2) * 0.5) / float(max(fh, 1))))
            scored.append((nx, ny, name, conf, x1, y1, x2, y2, cx))
        if not scored:
            return None
        lock_nx = float(rospy.get_param("/raicom/lock_nx", -1.0))
        if lock_nx >= 0.0:
            lock_ny = float(rospy.get_param("/raicom/lock_ny", 0.5))
            scored.sort(key=lambda s: (s[0] - lock_nx) ** 2 + (s[1] - lock_ny) ** 2)
            return scored[0]
        scored.sort(key=lambda s: -s[1])
        return scored[0]

    def publish_best_target(self, items, fw, fh):
        chosen = self.choose_item(items, fw, fh)
        if chosen is None:
            self.last_target = None
            return
        nx, ny, name, conf = chosen[0], chosen[1], chosen[2], chosen[3]
        text = "%s,%.3f,%.3f,%.3f" % (name, nx, ny, conf)
        self.last_target = (name, nx, ny, conf)
        self.pub_target.publish(String(data=text))
        rospy.set_param("/raicom/target_class", name)
        rospy.set_param("/raicom/target_conf", float(conf))

    def spin(self):
        rate = rospy.Rate(30)
        rospy.loginfo("raicom TRT auto running...")
        while not rospy.is_shutdown():
            self.maybe_reset()
            mode = rospy.get_param("/raicom/yolo_mode", DEFAULT_MODE)
            with self.lock:
                frame = None if self.frame is None else self.frame.copy()
            if frame is None or mode == "idle":
                rate.sleep()
                continue

            t0 = time.time()
            dets = self.model.infer(frame)
            ms = (time.time() - t0) * 1000

            items = []
            for x1, y1, x2, y2, conf, cls_id in dets:
                name = NAMES[cls_id] if 0 <= cls_id < len(NAMES) else str(cls_id)
                items.append(((x1 + x2) * 0.5, name, conf, x1, y1, x2, y2))

            if mode == "mapping" and self.locked is None:
                self.try_lock_mapping([(i[0], i[1]) for i in items])
            elif mode == "detect":
                h, w = frame.shape[:2]
                self.publish_best_target(items, w, h)

            tip = "%.0fms n=%d mode=%s" % (ms, len(items), mode)
            if self.locked:
                tip = "OK P1<-%s | %s->P2  %.0fms" % (self.locked[0], self.locked[1], ms)
            elif mode == "mapping" and len(items) >= 2:
                s = sorted(items, key=lambda x: x[0])
                tip = "try P1<-%s | %s->P2  %.0fms" % (s[0][1], s[-1][1], ms)
            elif mode == "detect" and self.last_target:
                tip = "det %s conf=%.2f  %.0fms" % (self.last_target[0], self.last_target[3], ms)

            rospy.loginfo_throttle(1.0, tip)

            if self.show:
                draw = frame
                for cx, name, conf, x1, y1, x2, y2 in items:
                    color = (0, 255, 0) if name == "gear" else (0, 128, 255)
                    cv2.rectangle(draw, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                    cv2.putText(
                        draw,
                        "%s %.2f" % (name, conf),
                        (int(x1), max(20, int(y1) - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        color,
                        2,
                    )
                cv2.putText(draw, tip, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
                cv2.imshow("raicom_trt", cv2.resize(draw, None, fx=0.5, fy=0.5))
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
            rate.sleep()
        if self.show:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        Node().spin()
    except Exception as e:
        print("ERROR:", e)
        import traceback

        traceback.print_exc()
        print("")
        print("若提示找不到 engine，先在小车执行:")
        print("  ls /usr/src/tensorrt/bin/trtexec")
        print("  # 电脑先导出 best_fixed_320.onnx 再拷上车，然后:")
        print(
            "  /usr/src/tensorrt/bin/trtexec --onnx=$HOME/yolo_models/best_fixed_320.onnx "
            "--saveEngine=$HOME/yolo_models/best_fixed_320.engine --fp16"
        )
        sys.exit(1)
