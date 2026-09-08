#!/usr/bin/env python3
# coding=utf-8
# @Author: Aiden
# @Date: 2023/11/16
# 环形麦克风阵列Python SDK
import re
import time
import json
import rospy
import serial
import signal
from std_srvs.srv import Trigger
from std_msgs.msg import Int32, Bool
from xf_mic_asr_offline.srv import SetString

class CircleMic:
    def __init__(self, port, flag_pub, angle_pub):
        self.serialHandle = serial.Serial(None, 115200, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE, timeout=5)
        self.serialHandle.rts = False
        self.serialHandle.dtr = False
        self.serialHandle.setPort(port)
        self.serialHandle.open()
        # CH341 开串口常会复位模组，先等它起来再握手
        time.sleep(1.5)
        try:
            self.serialHandle.reset_input_buffer()
            self.serialHandle.reset_output_buffer()
        except Exception:
            pass

        self.running = True
        self.key_type = r"{\"code.*?\"}"
        self.pattern = re.compile(r"{\"content.*?aiui_event\"}")
        self.flag_pub = flag_pub
        self.angle_pub = angle_pub
        self.start_time = time.time()

    # 麦克风阵列切换
    def switch_mic(self, mic="mic6_circle"):
        # mic：麦克风阵列类型，mic4：线性4麦，mic6：线性6麦， mic6_circle：环形6麦
        param ={
            "type": "switch_mic",
            "content": {
                "mic": "mic6_circle"
            }
        }
        param['content']['mic'] = mic 
        header = [0xA5, 0x01, 0x05]  
        res = self.send(header, param)
        if res is not None:
            pattern = re.compile(self.key_type)
            m = re.search(pattern, str(res))
            if m is not None:
                m = m.group(0)
                if m is not None:
                    return m
        
        return False

    # 获取版本信息
    def get_setting(self):
        param ={
            "type": "version"
        }
        
        header = [0xA5, 0x01, 0x05]  
        res = self.send(header, param)
        if res is not None:
            pattern = re.compile(self.key_type)
            m = re.search(pattern, str(res))
            if m is not None:
                m = m.group(0)
                if m is not None:
                    return m
        
        return False

    # 唤醒词更换（浅定制）
    def set_wakeup_word(self, str_pinyin="xiao3 hu3 xiao3 hu3"):
        # 参数为中文拼音
        # 更多参数请参考https://aiui.xfyun.cn/doc/aiui/3_access_service/access_hardware/r818/protocol.html
        param ={
            "type": "wakeup_keywords",
            "content": {
                "keyword": "xiao3 hu3 xiao3 hu3",
                "threshold": "500" 
            }
        }
        
        param['content']['keyword'] = str_pinyin
        header = [0xA5, 0x01, 0x05]
        print('setting wakeup keywords need about 30s', flush=True)
        print('setting ... keyword=%s' % str_pinyin, flush=True)
        self.send(header, param)
        t0 = getattr(self, 'start_time', time.time())
        while time.time() - t0 < 30:
            time.sleep(0.1)
        print('>>>>> wakeup keyword written: %s' % str_pinyin, flush=True)

    # 计算校验和
    def calculate_checksum(self, bytes_list):
        checksum = sum(bytes_list) & 0xFF
        checksum = (~checksum + 1) & 0xFF
        return checksum

    # 数据串口发送
    def send_data(self, header, args):
        packet = header 

        data = bytes(json.dumps(args), encoding="utf8")

        length = len(data)
        low_length = int(length & 0xFF)
        high_length = int(length >> 8)
        
        packet.extend([low_length, high_length])
        packet.extend([0x00, 0x00])
        
        packet.extend(data)
        checksum = self.calculate_checksum(packet)
        packet.append(checksum)
        
        self.serialHandle.write(packet)  # 发送主控消息

    # 发送数据（握手加超时，避免永远卡死写不进唤醒词、也进不了监听）
    def send(self, header, args, handshake_timeout=10, reply_timeout=8):
        old_timeout = self.serialHandle.timeout
        try:
            self.serialHandle.timeout = 0.05
            handshake = [0xa5, 0x01, 0x01, 0x04, 0x00, 0x00, 0x00, 0xa5, 0x00, 0x00, 0x00, 0xb0]
            self.serialHandle.write(handshake)
            t0 = time.time()
            next_hs = t0 + 0.1
            sent = False
            while time.time() - t0 < handshake_timeout:
                if time.time() >= next_hs:
                    self.serialHandle.write(handshake)
                    next_hs = time.time() + 0.1
                recv_data = self.serialHandle.read()
                header_ = [b'\xa5', b'\x01', b'\xff']
                if recv_data == header_[0]:
                    recv_data = self.serialHandle.read()
                    if recv_data == header_[1]:
                        recv_data = self.serialHandle.read()
                        if recv_data == header_[2]:
                            recv_data = self.serialHandle.read(4)
                            self.serialHandle.read((recv_data[1] << 8 | recv_data[0]) + 1)
                            self.send_data(header, args)
                            self.start_time = time.time()
                            sent = True
                            print('>>>>> mic handshake ok', flush=True)
                            break
                        else:
                            recv_data = self.serialHandle.read(4)
                            if recv_data:
                                self.serialHandle.read((recv_data[1] << 8 | recv_data[0]) + 1)
                            self.serialHandle.write(handshake)
                            next_hs = time.time() + 0.1
            if not sent:
                print('>>>>> mic handshake timeout, send keyword anyway', flush=True)
                self.send_data(header, args)
                self.start_time = time.time()

            result = None
            t1 = time.time()
            while time.time() - t1 < reply_timeout:
                recv_data = self.serialHandle.read()
                header_ = [b'\xa5', b'\x01', b'\x04']
                if recv_data == header_[0]:
                    recv_data = self.serialHandle.read()
                    if recv_data == header_[1]:
                        recv_data = self.serialHandle.read()
                        if recv_data == header_[2]:
                            recv_data = self.serialHandle.read(4)
                            result = self.serialHandle.read((recv_data[1] << 8 | recv_data[0]) + 1)
                            break
            return result
        finally:
            self.serialHandle.timeout = old_timeout

    def val_map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    # 检测是否唤醒以及唤醒对应角度
    def get_awake_result(self):
        recv_data = self.serialHandle.read()
        if recv_data == b'\xa5':
            recv_data = self.serialHandle.read()
            if recv_data == b'\x01':
                recv_data = self.serialHandle.read()
                if recv_data == b'\x04':
                    time.sleep(0.01)
                    recv_data = self.serialHandle.read(4)
                    result = self.serialHandle.read((recv_data[1] << 8 | recv_data[0]) + 1)
                    if b'content' in result:
                        m = re.search(self.pattern, str(result).replace('\\', ''))
                        if m is not None:
                            m = m.group(0).replace('"{"', '{"').replace('}"', '}')
                            if m is not None:
                                angle = int(json.loads(m)['content']['info']['ivw']['angle'])
                                if self.flag_pub is not None and self.angle_pub is not None:
                                    msg = Bool()
                                    msg.data = True
                                    self.flag_pub.publish(msg)
                                    angle = self.val_map(angle, 0, 360, 360, 0) + 240  # 和圆形兼容
                                    if angle >= 360:
                                        angle -= 360
                                    msg = Int32()
                                    msg.data = int(angle)
                                    self.angle_pub.publish(msg)
                                    print('>>>>> awake angle: %s' % int(angle), flush=True)
    
class AwakeNode:
    def __init__(self, name):
        rospy.init_node(name)
        
        self.running = True
        signal.signal(signal.SIGINT, self.shutdown)

        mic_type = rospy.get_param('~mic_type', 'mic6_circle')
        port = rospy.get_param('~port', '/dev/ring_mic')
        awake_word = rospy.get_param('~awake_word', 'xiao3 hu3 xiao3 hu3')
        enable_setting = rospy.get_param('~enable_setting', False)
        self.awake_angle_pub = rospy.Publisher('~angle', Int32, queue_size=1)
        self.awake_flag_pub = rospy.Publisher('~awake_flag', Bool, queue_size=1) 
        self.mic = CircleMic(port, self.awake_flag_pub, self.awake_angle_pub)
        rospy.Service('~set_mic_type', SetString, self.set_mic_type_srv)
        rospy.Service('~get_setting', Trigger, self.get_setting_srv)
        rospy.Service('~set_wakeup_word', SetString, self.set_wakeup_word_srv)
        if enable_setting:
            self.mic.switch_mic(mic_type)
            self.mic.set_wakeup_word(awake_word)
        print('>>>>>Wake up word: %s' % awake_word, flush=True)
        rate = rospy.Rate(50)
        while self.running:
            self.mic.get_awake_result()
            rate.sleep()
        rospy.signal_shutdown('shutdown')

    # 设置麦克风类型
    def set_mic_type_srv(self, msg):
        res = self.mic.switch_mic(msg.data) 
        return [True, str(res)]

    # 获取麦克风版本信息
    def get_setting_srv(self, msg):
        res = self.mic.get_setting()
        return [True, str(res)]

    # 设置唤醒词服务
    def set_wakeup_word_srv(self, msg):
        self.mic.set_wakeup_word(msg.data)
        return [True, 'success']

    def shutdown(self):
        self.running = False
        self.mic.serialHandle.close()
        rospy.loginfo('shutdown')

if __name__ == "__main__":
    AwakeNode('awake_node')

