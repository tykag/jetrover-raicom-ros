#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RAICOM 中转区语音（ROS1 + WonderEchoPro）

赛规两项：
1) 播报「遥操作区任务已完成」→ 串口发 AA 55 FF 01 FB（模组被动播报）
2) 唤醒后口令「执行全自主运输任务」等 → 模组发 AA 55 00 01 FB → /raicom/start_auto=true

协议与固件表一致：
  项目总结/语音-WonderEchoPro/命令词播报词协议列表V3_中文_RAICOM比赛.xlsx
"""
from __future__ import print_function

import os
import threading
import time

import rospy
import serial
from serial.tools import list_ports
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, TriggerResponse

# 与固件表 / 厂方 awake.WonderEchoPro 常量对齐（发送=模组→车，接收=车→模组）
PKT_WAKE = bytes([0xAA, 0x55, 0x03, 0x00, 0xFB])   # 唤醒：小车收
PKT_SLEEP = bytes([0xAA, 0x55, 0x02, 0x00, 0xFB])  # 休眠：小车收
PKT_START = bytes([0xAA, 0x55, 0x00, 0x01, 0xFB])  # 启动口令：小车收
PKT_ANNOUNCE = bytes([0xAA, 0x55, 0xFF, 0x01, 0xFB])  # 中转区播报：小车发


def find_port(preferred=None):
    if preferred and os.path.exists(preferred):
        return preferred
    for env_key in ("WONDERECHO_PORT", "VOICE_PORT"):
        p = os.environ.get(env_key)
        if p and os.path.exists(p):
            return p
    for cand in (
        "/dev/ttyUSB0",
        "/dev/ttyCH341USB0",
        "/dev/ttyCH341USB1",
        "/dev/ttyACM0",
    ):
        if os.path.exists(cand):
            return cand
    for info in list_ports.comports():
        desc = (info.description or "").lower()
        if "ch340" in desc or "ch341" in desc or "usb serial" in desc:
            return info.device
    return None


class WonderEchoSerial(object):
    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(
            None, baud, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE, timeout=0.05
        )
        self.ser.rts = False
        self.ser.dtr = False
        self.ser.setPort(port)
        self.ser.open()
        self._buf = bytearray()
        self.lock = threading.Lock()

    def flush_input(self):
        with self.lock:
            while self.ser.in_waiting > 0:
                self.ser.read(self.ser.in_waiting)
            self._buf = bytearray()

    def write_pkt(self, pkt):
        with self.lock:
            self.ser.write(pkt)
            self.ser.flush()

    def read_pkts(self):
        """从串口拼出完整 5 字节帧 AA 55 xx xx FB。"""
        out = []
        with self.lock:
            n = self.ser.in_waiting
            if n:
                self._buf.extend(self.ser.read(n))
            while True:
                if len(self._buf) < 5:
                    break
                try:
                    i = self._buf.index(0xAA)
                except ValueError:
                    self._buf = bytearray()
                    break
                if i:
                    del self._buf[:i]
                if len(self._buf) < 5:
                    break
                if self._buf[1] != 0x55 or self._buf[4] != 0xFB:
                    del self._buf[0]
                    continue
                pkt = bytes(self._buf[:5])
                del self._buf[:5]
                out.append(pkt)
        return out

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


class RaicomVoiceNode(object):
    def __init__(self):
        rospy.init_node("raicom_voice", anonymous=False)
        self.started = False
        self.lock = threading.Lock()
        self.awake = False

        port = find_port(rospy.get_param("~port", None))
        if not port:
            raise RuntimeError(
                "找不到 WonderEchoPro 串口。插上模块后查 ls /dev/ttyUSB* /dev/ttyCH341*，"
                "或 export WONDERECHO_PORT=/dev/xxx"
            )

        self.dev = WonderEchoSerial(port)
        self.dev.flush_input()

        self.pub_start = rospy.Publisher("/raicom/start_auto", Bool, queue_size=1, latch=True)
        self.pub_start.publish(Bool(data=False))
        self.pub_words = rospy.Publisher("/raicom/voice_cmd", String, queue_size=1)
        self.pub_awake = rospy.Publisher("/raicom/voice_awake", Bool, queue_size=1, latch=True)
        self.pub_awake.publish(Bool(data=False))

        rospy.Subscriber("/raicom/voice_announce", String, self.announce_callback, queue_size=5)
        rospy.Service("/raicom/announce_teleop_done", Trigger, self.srv_announce)
        rospy.Service("/raicom/reset_start", Trigger, self.srv_reset)

        rospy.loginfo("RAICOM WonderEchoPro ready  port=%s", port)
        rospy.loginfo("唤醒: 小虎小虎 → 收 AA 55 03 00 FB")
        rospy.loginfo("口令: 执行全自主运输任务 → 收 AA 55 00 01 FB → /raicom/start_auto")
        rospy.loginfo("播报: 发 AA 55 FF 01 FB（rosservice call /raicom/announce_teleop_done \"{}\"）")

        self._stop = False
        self._th = threading.Thread(target=self._poll_loop, name="wonderecho_poll")
        self._th.daemon = True
        self._th.start()

    def announce_teleop_done(self):
        try:
            self.dev.write_pkt(PKT_ANNOUNCE)
            rospy.loginfo("sent announce PKT_ANNOUNCE (遥操作区任务已完成)")
            return True
        except Exception as e:
            rospy.logerr("announce write fail: %s", e)
            return False

    def srv_announce(self, _req):
        ok = self.announce_teleop_done()
        return TriggerResponse(success=ok, message="FF 01 sent" if ok else "serial fail")

    def srv_reset(self, _req):
        with self.lock:
            self.started = False
            self.awake = False
        self.pub_start.publish(Bool(data=False))
        self.pub_awake.publish(Bool(data=False))
        return TriggerResponse(success=True, message="reset")

    def announce_callback(self, msg):
        if msg.data in ("teleop_done", "遥操作区任务已完成", "1"):
            self.announce_teleop_done()

    def _on_start_cmd(self):
        with self.lock:
            if self.started:
                rospy.logwarn("already started, ignore")
                return
            self.started = True
        self.pub_start.publish(Bool(data=True))
        self.pub_words.publish(String(data="执行全自主运输任务"))
        rospy.loginfo(">>> RAICOM START AUTO <<<")

    def _handle_pkt(self, pkt):
        if pkt == PKT_WAKE:
            self.awake = True
            self.pub_awake.publish(Bool(data=True))
            self.pub_words.publish(String(data="唤醒成功"))
            rospy.loginfo("wakeup (小虎小虎)")
            return
        if pkt == PKT_SLEEP:
            self.awake = False
            self.pub_awake.publish(Bool(data=False))
            self.pub_words.publish(String(data="休眠"))
            rospy.loginfo("sleep")
            return
        if pkt == PKT_START:
            self._on_start_cmd()
            return
        # 其它帧忽略（比赛固件已删无关命令）
        rospy.logdebug("ignore pkt: %s", " ".join("%02X" % b for b in pkt))

    def _poll_loop(self):
        rate_hz = 50
        while not self._stop and not rospy.is_shutdown():
            try:
                for pkt in self.dev.read_pkts():
                    self._handle_pkt(pkt)
            except Exception as e:
                rospy.logerr_throttle(5.0, "serial read: %s", e)
            time.sleep(1.0 / rate_hz)

    def shutdown(self):
        self._stop = True
        if self._th.is_alive():
            self._th.join(timeout=1.0)
        self.dev.close()


if __name__ == "__main__":
    node = RaicomVoiceNode()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()
