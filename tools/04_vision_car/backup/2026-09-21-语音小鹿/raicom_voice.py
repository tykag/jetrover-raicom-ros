#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RAICOM 中转区语音（ROS1 + WonderEchoPro）

赛规两项：
1) 播报「遥操作区任务已完成」→ 播放 voice/teleop_done.wav（USB 声卡，同开机「我准备好了」）
2) 唤醒「小鹿小鹿」后口令「执行运输任务」→ 模组发 AA 55 00 01 FB → /raicom/start_auto=true

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
PKT_WAKE = bytes([0xAA, 0x55, 0x03, 0x00, 0xFB])   # 唤醒 小鹿小鹿：小车收
PKT_SLEEP = bytes([0xAA, 0x55, 0x02, 0x00, 0xFB])  # 休眠：小车收
PKT_START = bytes([0xAA, 0x55, 0x00, 0x01, 0xFB])  # 口令 执行运输任务：小车收
_VOICE_CANDIDATES = (
    os.path.expanduser("~/yolo_models/voice"),
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "voice"),
)
VOICE_DIR = next((p for p in _VOICE_CANDIDATES if os.path.isdir(p)), _VOICE_CANDIDATES[0])
WAV_WAKE = "wozai.wav"          # 我在
WAV_ACK = "ack.wav"              # 收到
WAV_ANNOUNCE = "teleop_done.wav"  # 遥操作区任务已完成


def find_port(preferred=None):
    if preferred and os.path.exists(preferred):
        return preferred
    for env_key in ("WONDERECHO_PORT", "VOICE_PORT"):
        p = os.environ.get(env_key)
        if p and os.path.exists(p):
            return p
    for cand in (
        "/dev/ring_mic",
        "/dev/ttyUSB1",
        "/dev/ttyCH341USB0",
        "/dev/ttyCH341USB1",
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
        # CH340 的 DTR 为低会把 CI1302 按在复位，串口助手能看到数据、节点却没反应
        self.ser.dtr = True
        self.ser.rts = True
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

        port = rospy.get_param("~port", None) or None
        port = find_port(port)
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

        print("RAICOM voice port=%s" % port, flush=True)
        rospy.loginfo("唤醒: 小鹿小鹿 → 收 AA 55 03 00 FB")
        rospy.loginfo("口令: 执行运输任务 → 收 AA 55 00 01 FB → /raicom/start_auto")
        rospy.loginfo("播报: rosservice call /raicom/announce_teleop_done \"{}\" → voice/teleop_done.wav")

        self._stop = False
        self._th = threading.Thread(target=self._poll_loop, name="wonderecho_poll")
        self._th.daemon = True
        self._th.start()

    def _play_wav(self, name):
        path = os.path.join(VOICE_DIR, name)
        if not os.path.isfile(path):
            rospy.logerr("wav missing: %s", path)
            return False
        # WonderEcho 的 USB 声卡是 plughw:2,0（开机「我准备好了」同路喇叭）
        # sox play 在这台车上会报 0-bit，所以先 aplay
        rc = os.system("aplay -q -D plughw:2,0 '%s'" % path)
        if rc != 0:
            rc = os.system("play -q '%s'" % path)
        ok = rc == 0
        if ok:
            rospy.loginfo("played %s", name)
        else:
            rospy.logerr("play fail %s rc=%s", name, rc)
        return ok

    def announce_teleop_done(self):
        return self._play_wav(WAV_ANNOUNCE)

    def srv_announce(self, _req):
        ok = self.announce_teleop_done()
        return TriggerResponse(success=ok, message="played teleop_done" if ok else "play fail")

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
        self.pub_words.publish(String(data="执行运输任务"))
        self._play_wav(WAV_ACK)
        rospy.loginfo(">>> RAICOM START AUTO <<<")

    def _handle_pkt(self, pkt):
        if pkt == PKT_WAKE:
            self.awake = True
            self.pub_awake.publish(Bool(data=True))
            self.pub_words.publish(String(data="小鹿小鹿"))
            rospy.loginfo("wakeup 小鹿小鹿")
            self._play_wav(WAV_WAKE)
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
        # 其它帧也打出来，方便对出厂固件实际字节
        msg = " ".join("%02X" % b for b in pkt)
        rospy.loginfo("pkt %s", msg)
        print("pkt %s" % msg, flush=True)

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
