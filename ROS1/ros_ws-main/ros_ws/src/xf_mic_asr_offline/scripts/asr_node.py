#!/usr/bin/env python3
# coding=utf-8
# @Author: Aiden
import os
import rospy
import signal
from std_msgs.msg import String, Bool
from xf_mic_asr_offline.srv import GetOfflineResult

try:
    from xf_mic_asr_offline import voice_play
except ImportError:
    voice_play = None

class ASRNode:
    def __init__(self, name):
        rospy.init_node(name)

        self.init_finish = False
        self.awake_flag = False
        self.recognize_fail_count = 0
        self.recognize_fail_count_threshold = 15
        self.running = True
        self.language = os.environ.get('ASR_LANGUAGE', 'Chinese')
        signal.signal(signal.SIGINT, self.shutdown)

        self.confidence_threshold = rospy.get_param('~confidence', 18)
        self.seconds_per_order = rospy.get_param('~seconds_per_order', 3)

        self.control = rospy.Publisher('~voice_words', String, queue_size=1)
        rospy.Subscriber('/awake_node/awake_flag', Bool, self.awake_flag_callback)

        rospy.wait_for_service('/voice_control/get_offline_result')
        rate = rospy.Rate(10)
        while self.running:
            self.proces_regogniztion()
            rate.sleep()
        rospy.signal_shutdown('shutdown')

    def shutdown(self):
        self.running = False
        rospy.loginfo('shutdown')

    def play_wozai(self):
        """唤醒应答：厂方 awake.wav 就是「我在」"""
        try:
            if voice_play is not None:
                voice_play.play('awake', language=self.language)
                return
        except Exception as e:
            rospy.logwarn('voice_play awake fail: %s', e)
        wav = os.path.expanduser(
            '~/ros_ws/src/xf_mic_asr_offline/src/xf_mic_asr_offline/feedback_voice/awake.wav'
        )
        if os.path.isfile(wav):
            os.system("play -q '%s'" % wav)

    def awake_flag_callback(self, msg):
        if not msg.data:
            return
        # 先回答「我在」，再开始听后续口令
        self.play_wozai()
        self.awake_flag = True
        self.recognize_fail_count = 0
        count_msg = String()
        count_msg.data = "唤醒成功(wake-up-success)"
        self.control.publish(count_msg)

    def proces_regogniztion(self)->None:
        if self.awake_flag:
            response = rospy.ServiceProxy('/voice_control/get_offline_result', GetOfflineResult)(1, self.confidence_threshold, self.seconds_per_order)
            if response.text == "休眠(Sleep)":  # 主动休眠(active sleep)
                self.awake_flag = 0
                self.recognize_fail_count = 0
                print('\033[1;32m休眠(Sleep)\033[0m')
            elif response.result == "ok":  # 清零被动休眠相关变量(clear passive sleep relative variable)
                self.awake_flag = 0
                self.recognize_fail_count = 0
                count_msg = String()
                count_msg.data = response.text
                self.control.publish(count_msg)
                print('\033[1;32mok\033[0m')
            elif response.result == "fail":  # 记录识别失败次数(record the number of recognition failures)
                self.recognize_fail_count += 1
                if self.recognize_fail_count == 5:  # 连续识别失败5次，用户界面显示提醒信息(fail to recognize for consecutive 5 times.Warning occurs on user interface)
                    count_msg = String()
                    count_msg.data = "失败5次(Fail-5-times)"
                    self.control.publish(count_msg)
                    print('\033[1;32m失败5次(Fail-5-times)\033[0m')
                elif self.recognize_fail_count == 10:  # 连续识别失败10次，用户界面显示提醒信息(fail to recognize for consecutive 10 times.Warning occurs on user interface)
                    count_msg = String()
                    count_msg.data = "失败10次(Fail-10-times)"
                    self.control.publish(count_msg)
                    print('\033[1;32m失败10次(Fail-10-times)\033[0m')
                elif self.recognize_fail_count >= self.recognize_fail_count_threshold:  # 被动休眠(passive sleep)
                    self.awake_flag = 0
                    count_msg = String()
                    count_msg.data = "休眠(Sleep)"
                    self.control.publish(count_msg)
                    self.recognize_fail_count = 0
                    print('\033[1;32m休眠(Sleep)\033[0m')

if __name__ == "__main__":
    ASRNode('asr_node')
