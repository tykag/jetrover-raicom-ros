#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RAICOM 中转区语音（ROS1）

赛规两项加分：
1) 播报「遥操作区任务已完成」→ 话题 /raicom/voice_announce 或服务触发
2) 唤醒后说「执行全自主运输任务」→ 发布 /raicom/start_auto=true，供全自动状态机订阅

依赖：已单独启动 mic_init（不要塞进 bringup 的 startup_check）
"""
from __future__ import print_function

import json
import os
import threading

import rospy
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, TriggerResponse
from ros_robot_controller.msg import BuzzerState

try:
    from xf_mic_asr_offline import voice_play
except ImportError:
    voice_play = None

# 自定义播报 wav（优先）
CUSTOM_WAV = os.path.expanduser("~/yolo_models/voice/teleop_done.wav")
# 厂方 feedback 目录里的备用名（若你拷贝成同名）
FACTORY_NAME = "teleop_done"

START_CMDS = (
    "执行全自主运输任务",
    "开始全自主",
    "开始运输",
    "start autonomous transport",
    "start auto transport",
)


class RaicomVoiceNode(object):
    def __init__(self):
        rospy.init_node("raicom_voice", anonymous=False)
        self.language = os.environ.get("ASR_LANGUAGE", "Chinese")
        self.started = False
        self.lock = threading.Lock()

        self.pub_start = rospy.Publisher("/raicom/start_auto", Bool, queue_size=1, latch=True)
        self.pub_start.publish(Bool(data=False))
        self.pub_words = rospy.Publisher("/raicom/voice_cmd", String, queue_size=1)
        self.buzzer_pub = rospy.Publisher("/ros_robot_controller/set_buzzer", BuzzerState, queue_size=1)

        rospy.Subscriber("/asr_node/voice_words", String, self.words_callback, queue_size=5)
        rospy.Subscriber("/raicom/voice_announce", String, self.announce_callback, queue_size=5)
        rospy.Service("/raicom/announce_teleop_done", Trigger, self.srv_announce)
        rospy.Service("/raicom/reset_start", Trigger, self.srv_reset)

        rospy.loginfo("RAICOM voice ready")
        rospy.loginfo("唤醒: 小虎小虎 | 口令: 执行全自主运输任务")
        rospy.loginfo("播报: rosservice call /raicom/announce_teleop_done \"{}\"")

    def play_file(self, path):
        if not os.path.isfile(path):
            return False
        try:
            os.system("amixer -q -D pulse set Master 100% 2>/dev/null; play -q '%s'" % path)
            return True
        except Exception as e:
            rospy.logerr(str(e))
            return False

    def play_name(self, name):
        if voice_play is None:
            return False
        try:
            voice_play.play(name, language=self.language)
            return True
        except Exception as e:
            rospy.logwarn("voice_play fail: %s", e)
            return False

    def announce_teleop_done(self):
        """播报：遥操作区任务已完成"""
        if self.play_file(CUSTOM_WAV):
            rospy.loginfo("played %s", CUSTOM_WAV)
            return True
        if self.play_name(FACTORY_NAME):
            rospy.loginfo("played factory wav %s", FACTORY_NAME)
            return True
        # 最后手段：系统 TTS（若车上有 espeak）
        text = "遥操作区任务已完成"
        ret = os.system('espeak -v zh "%s" 2>/dev/null' % text)
        if ret == 0:
            rospy.loginfo("played via espeak")
            return True
        rospy.logerr("没有播报文件。请准备: %s", CUSTOM_WAV)
        return False

    def srv_announce(self, _req):
        ok = self.announce_teleop_done()
        return TriggerResponse(success=ok, message="teleop_done" if ok else "missing wav")

    def srv_reset(self, _req):
        with self.lock:
            self.started = False
        self.pub_start.publish(Bool(data=False))
        return TriggerResponse(success=True, message="reset")

    def announce_callback(self, msg):
        if msg.data in ("teleop_done", "遥操作区任务已完成", "1"):
            self.announce_teleop_done()

    def words_callback(self, msg):
        raw = msg.data or ""
        try:
            words = json.dumps(raw, ensure_ascii=False)[1:-1]
        except Exception:
            words = raw
        if self.language == "Chinese":
            words = words.replace(" ", "")

        rospy.loginfo("words: %s", words)
        self.pub_words.publish(String(data=words))

        if words in ("唤醒成功(wake-up-success)",):
            self.play_name("awake")
            return
        if words in ("休眠(Sleep)",):
            b = BuzzerState()
            b.freq = 1900
            b.on_time = 0.05
            b.off_time = 0.01
            b.repeat = 1
            self.buzzer_pub.publish(b)
            return
        if words in ("失败5次(Fail-5-times)", "失败10次(Fail-10-times)"):
            return

        for cmd in START_CMDS:
            if words == cmd or cmd in words:
                with self.lock:
                    if self.started:
                        rospy.logwarn("already started, ignore")
                        return
                    self.started = True
                self.pub_start.publish(Bool(data=True))
                rospy.loginfo(">>> RAICOM START AUTO <<<")
                self.play_name("ok") or self.play_name("running")
                return


if __name__ == "__main__":
    RaicomVoiceNode()
    rospy.spin()
