#!/usr/bin/env python3
# encoding: utf-8
import os
import math
import time
import rospy
import threading
import sqlite3 as sql
from enum import Enum
import hiwonder_sdk.misc as misc
import geometry_msgs.msg as geo_msg
import sensor_msgs.msg as sensor_msg
from ros_robot_controller.msg import BuzzerState
from hiwonder_servo_msgs.msg import CommandDuration, MultiRawIdPosDur, RawIdPosDur, ServoStateList

AXES_MAP = 'lx', 'ly', 'rx', 'ry', 'r2', 'l2', 'hat_x', 'hat_y'
BUTTON_MAP = 'cross', 'circle', '', 'square', 'triangle', '', 'l1', 'r1', 'l2', 'r2', 'select', 'start', '', 'l3', 'r3', '', 'hat_xl', 'hat_xr', 'hat_yu', 'hat_yd', ''

# ===== 机械臂舵机配置 =====
ARM_JOINTS = {
    'joint1': 1,   # 底座旋转
    'joint2': 2,   # 大臂俯仰
    'joint3': 3,   # 小臂俯仰
    'joint4': 4,   # 腕部俯仰
    'joint5': 5,   # 腕部旋转(夹爪旋转)
}
GRIPPER_ID = 10    # 夹爪夹放(r_joint)

# 平放向前（避免一按键整臂跳到竖直中位）
HOME_POSITION = {
    'joint1': 500, 'joint2': 750, 'joint3': 0,
    'joint4': 375, 'joint5': 500, 'gripper': 500,
}
# 各关节软限位（脉冲）
JOINT_LIMITS = {
    'joint1': (0, 1000),
    'joint2': (10, 650),
    'joint3': (0, 1000),
    'joint4': (0, 1000),
    'joint5': (0, 1000),
    'gripper': (0, 1000),
}
JOINT_SPEED = 12     # 匀速峰值（不变）
GRIPPER_SPEED = 18
ARM_CMD_PERIOD = 0.04   # 25Hz
SERVO_DURATION = 0.05
# 每周期速度变化上限：越小起步/停下越柔，峰值速度仍是 JOINT_SPEED
JOINT_ACCEL = 3
GRIPPER_ACCEL = 4
# 松开：不对滞后反馈回拉；不额外硬锁
SERVO_LOCK_DURATION = 0.20
LOCK_PUBLISH = False
# 扳机回滞，防止松开后在阈值附近反复启停导致抖动
TRIG_ON = 0.30
TRIG_OFF = 0.12
# 方向键回滞
HAT_ON = 0.60
HAT_OFF = 0.30
# 摇杆回滞（停住不抖）
STICK_ON = 0.18
STICK_OFF = 0.08

# 动作组路径（ROS1 宿主机常见路径；也可用 ~action_path 覆盖）
DEFAULT_ACTION_PATH = '/home/hiwonder/share/arm_pc/ActionGroups'

ID_TO_JOINT = {1: 'joint1', 2: 'joint2', 3: 'joint3', 4: 'joint4', 5: 'joint5', 10: 'gripper'}



class ButtonState(Enum):
    Normal = 0
    Pressed = 1
    Holding = 2
    Released = 3


class ActionGroupController:
    """读取 .d6a 动作组并发布到总线舵机，逻辑与 ROS2 ActionGroupController 一致。"""
    def __init__(self, pub, action_path):
        self.servo_pub = pub
        self.action_path = action_path
        self.running_action = False
        self.stop_running = False

    def stop_action_group(self):
        self.stop_running = True

    def run_action(self, action_name):
        if action_name is None:
            return
        action_file = os.path.join(self.action_path, action_name + '.d6a')
        self.stop_running = False
        if not os.path.exists(action_file):
            self.running_action = False
            rospy.logerr('未能找到动作组文件: %s' % action_file)
            return
        if self.running_action:
            return
        self.running_action = True
        ag = sql.connect(action_file)
        cu = ag.cursor()
        cu.execute('select * from ActionGroup')
        while True:
            act = cu.fetchone()
            if self.stop_running:
                self.stop_running = False
                break
            if act is not None:
                positions = []
                duration = float(act[1]) / 1000.0
                for i in range(0, len(act) - 2, 1):
                    sid = 10 if i + 1 == 6 else i + 1
                    positions.append(RawIdPosDur(int(sid), int(act[2 + i]), duration))
                msg = MultiRawIdPosDur(id_pos_dur_list=positions)
                self.servo_pub.publish(msg)
                time.sleep(duration)
            else:
                break
        self.running_action = False
        cu.close()
        ag.close()


class JoystickController:
    def __init__(self):
        rospy.init_node('joystick_control')
        self.min_value = 0.1
        self.max_linear = rospy.get_param('~max_linear', 0.7)
        self.max_angular = rospy.get_param('~max_angular', 3.0)
        self.machine = rospy.get_param('~machine', 'JetRover_Mecanum')
        self.disable_servo_control = rospy.get_param('~disable_servo_control', False)
        self.action_path = rospy.get_param('~action_path', DEFAULT_ACTION_PATH)
        cmd_vel = rospy.get_param('~cmd_vel', 'hiwonder_controller/cmd_vel')

        # 底盘
        self.jointw = rospy.Publisher('w_joint_controller/command_duration', CommandDuration, queue_size=1)
        self.joy_sub = rospy.Subscriber('ros_robot_controller/joy', sensor_msg.Joy, self.joy_callback)
        self.buzzer_pub = rospy.Publisher('ros_robot_controller/set_buzzer', BuzzerState, queue_size=1)
        self.mecanum_pub = rospy.Publisher(cmd_vel, geo_msg.Twist, queue_size=1)

        # 机械臂
        self.servo_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        self.arm_position = dict(HOME_POSITION)
        self._servo_fb = dict(HOME_POSITION)  # 实车反馈脉冲
        self._fb_ready = False
        self.action_group = ActionGroupController(self.servo_pub, self.action_path)
        self.action_running = False
        # 目标速度（按键）与实际速度（加速度斜坡）
        joint_names = list(ARM_JOINTS.keys()) + ['gripper']
        self._joint_cmd = {name: 0 for name in joint_names}
        self._joint_vel = {name: 0 for name in joint_names}
        self._lock_joints = set()
        self._hat_active = {'hat_xl': 0, 'hat_xr': 0, 'hat_yu': 0, 'hat_yd': 0}
        self._stick_active = {k: False for k in ('lx', 'ly', 'rx', 'ry')}
        self._arm_timer = rospy.Timer(rospy.Duration(ARM_CMD_PERIOD), self._arm_velocity_tick)
        self.servo_fb_sub = rospy.Subscriber(
            '/servo_controllers/port_id_1/servo_states', ServoStateList, self._servo_states_cb, queue_size=1)

        self.last_axes = dict(zip(AXES_MAP, [0.0, ] * len(AXES_MAP)))
        self.last_buttons = dict(zip(BUTTON_MAP, [0.0, ] * len(BUTTON_MAP)))
        rospy.loginfo('joystick_control start: STM32-aligned (mecanum sticks + arm buttons)')
        rospy.loginfo('action path: %s' % self.action_path)

    def clamp(self, val, joint=None, lo=0, hi=1000):
        if joint and joint in JOINT_LIMITS:
            lo, hi = JOINT_LIMITS[joint]
        return max(lo, min(hi, val))

    def _servo_states_cb(self, msg):
        for st in msg.servo_states:
            name = ID_TO_JOINT.get(int(st.id))
            if name is None:
                continue
            self._servo_fb[name] = int(st.position)
        self._fb_ready = True

    def sync_joint_from_fb(self, joint):
        if joint in self._servo_fb:
            self.arm_position[joint] = self.clamp(self._servo_fb[joint], joint)

    def publish_arm(self, joints=None, duration=SERVO_DURATION):
        if self.action_running and joints is not None and set(joints) != {'gripper'}:
            return
        if self.action_running and joints is None:
            return
        if joints is None:
            joints = list(ARM_JOINTS.keys()) + ['gripper']
        if not joints:
            return
        positions = []
        for name in joints:
            if name == 'gripper':
                positions.append(RawIdPosDur(int(GRIPPER_ID), int(self.arm_position['gripper']), duration))
            else:
                positions.append(RawIdPosDur(int(ARM_JOINTS[name]), int(self.arm_position[name]), duration))
        self.servo_pub.publish(MultiRawIdPosDur(id_pos_dur_list=positions))

    def set_joint_vel(self, joint, vel):
        """设置目标速度；实际速度在 tick 里按加速度爬升/下降。"""
        vel = int(vel)
        prev_cmd = self._joint_cmd.get(joint, 0)
        # 由停到动：先跟真实位置对齐，避免首帧猛跳
        if prev_cmd == 0 and self._joint_vel.get(joint, 0) == 0 and vel != 0:
            self.sync_joint_from_fb(joint)
        self._joint_cmd[joint] = vel

    def _accel_step(self, joint, current, target):
        accel = GRIPPER_ACCEL if joint == 'gripper' else JOINT_ACCEL
        if current < target:
            return min(target, current + accel)
        if current > target:
            return max(target, current - accel)
        return current

    def _arm_velocity_tick(self, _event):
        """目标速度按加速度斜坡；峰值仍是 JOINT_SPEED。"""
        moving = []
        for joint in self._joint_vel:
            if self.action_running and joint != 'gripper':
                self._joint_cmd[joint] = 0
                self._joint_vel[joint] = 0
                continue
            target = self._joint_cmd[joint]
            prev = self._joint_vel[joint]
            vel = self._accel_step(joint, prev, target)
            self._joint_vel[joint] = vel
            if vel != 0:
                self.arm_position[joint] = self.clamp(self.arm_position[joint] + vel, joint)
                moving.append(joint)
                self._lock_joints.discard(joint)
            elif prev != 0 and target == 0:
                # 减速到 0：标记柔停，不回拉反馈
                self._lock_joints.add(joint)
        if moving:
            self.publish_arm(moving, SERVO_DURATION)
        if self._lock_joints:
            locks = list(self._lock_joints)
            self._lock_joints.clear()
            if LOCK_PUBLISH:
                self.publish_arm(locks, SERVO_LOCK_DURATION)

    def arm_hold(self, joint, speed, new_state):
        if new_state in (ButtonState.Pressed, ButtonState.Holding):
            self.set_joint_vel(joint, speed)
        elif new_state == ButtonState.Released:
            self.set_joint_vel(joint, 0)

    def _hat_with_hysteresis(self, name, raw_on):
        """方向键回滞，避免停在边缘时反复触发。"""
        if self._hat_active[name]:
            active = 1 if raw_on > HAT_OFF else 0
        else:
            active = 1 if raw_on > HAT_ON else 0
        self._hat_active[name] = active
        return active

    def _stick_with_hysteresis(self, name, value):
        """摇杆回滞：松回死区后保持 0，避免停时来回抖。"""
        mag = abs(value)
        if self._stick_active[name]:
            if mag < STICK_OFF:
                self._stick_active[name] = False
                return 0.0
            return value
        if mag > STICK_ON:
            self._stick_active[name] = True
            return value
        return 0.0

    def run_action_thread(self, action_name):
        self.action_running = True
        for k in self._joint_vel:
            self.set_joint_vel(k, 0)
        rospy.loginfo('run action: %s' % action_name)
        try:
            self.action_group.run_action(action_name)
        except Exception as e:
            rospy.logerr('action error: %s' % str(e))
        self.action_running = False
        # 动作组结束后用反馈刷新，避免下一次首动猛跳
        for name in list(ARM_JOINTS.keys()) + ['gripper']:
            self.sync_joint_from_fb(name)
        rospy.loginfo('action finished: %s' % action_name)

    def axes_callback(self, axes):
        for k in ['lx', 'ly', 'rx', 'ry']:
            axes[k] = self._stick_with_hysteresis(k, axes[k])

        # 底盘：对齐 STM32 app_ps2.c 绿灯摇杆
        # LX=原地转, LY=前后, RX=左右平移, RY=前后(可叠加)
        twist = geo_msg.Twist()
        if self.machine == 'JetRover_Mecanum':
            vx = misc.val_map(axes['ly'], -1, 1, -self.max_linear, self.max_linear)
            vx += misc.val_map(axes['ry'], -1, 1, -self.max_linear, self.max_linear)
            twist.linear.x = max(-self.max_linear, min(self.max_linear, vx))
            twist.linear.y = misc.val_map(axes['rx'], -1, 1, -self.max_linear, self.max_linear)
            twist.angular.z = misc.val_map(axes['lx'], -1, 1, -self.max_angular, self.max_angular)
        elif self.machine == 'JetRover_Tank':
            twist.linear.x = misc.val_map(axes['ly'], -1, 1, -self.max_linear, self.max_linear)
            twist.angular.z = misc.val_map(axes['lx'], -1, 1, -self.max_angular, self.max_angular)
        elif self.machine == 'JetRover_Acker':
            twist.linear.x = misc.val_map(axes['ly'], -1, 1, -self.max_linear, self.max_linear)
            steering_angle = misc.val_map(axes['lx'], -1, 1, -math.radians(150/1000*240), math.radians(150/1000*240))
            if twist.linear.x == 0:
                twist.linear.z = 1
                self.jointw.publish(CommandDuration(data=steering_angle, duration=0.02))
            else:
                if steering_angle != 0:
                    R = 0.213/math.tan(steering_angle)
                    twist.angular.z = twist.linear.x/R
        self.mecanum_pub.publish(twist)

    # ===== 方向键: #000 joint1 / #001 joint2（对齐 STM32 绿灯按键）=====
    # LL→#000P2400, LR→#000P0600, LU→#001P0600, LD→#001P2400
    def hat_yu_callback(self, new_state):
        self.arm_hold('joint2', -JOINT_SPEED, new_state)

    def hat_yd_callback(self, new_state):
        self.arm_hold('joint2', JOINT_SPEED, new_state)

    def hat_xl_callback(self, new_state):
        self.arm_hold('joint1', JOINT_SPEED, new_state)

    def hat_xr_callback(self, new_state):
        self.arm_hold('joint1', -JOINT_SPEED, new_state)

    # ===== 功能键: #002 joint3 / #003 joint4 =====
    # RU→#002P2400, RD→#002P0600, RR→#003P2400, RL→#003P0600
    def triangle_callback(self, new_state):
        self.arm_hold('joint3', JOINT_SPEED, new_state)

    def cross_callback(self, new_state):
        self.arm_hold('joint3', -JOINT_SPEED, new_state)

    def square_callback(self, new_state):
        self.arm_hold('joint4', -JOINT_SPEED, new_state)

    def circle_callback(self, new_state):
        self.arm_hold('joint4', JOINT_SPEED, new_state)

    # ===== L1/R1: #004 joint5 夹爪旋转 =====
    # L1→#004P0600, R1→#004P2400
    def l1_callback(self, new_state):
        self.arm_hold('joint5', -JOINT_SPEED, new_state)

    def r1_callback(self, new_state):
        self.arm_hold('joint5', JOINT_SPEED, new_state)

    # ===== L2/R2 按钮通道（备用；主通道在 axes）=====
    def l2_callback(self, new_state):
        pass

    def r2_callback(self, new_state):
        pass

    # ===== SELECT/START: 对齐 STM32 $DJR! 急停底盘+停臂 =====
    def select_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            self.stop_motion()

    def start_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            self.stop_motion()

    def stop_motion(self):
        self.mecanum_pub.publish(geo_msg.Twist())
        for k in self._joint_vel:
            self.set_joint_vel(k, 0)
        if self.action_running:
            self.action_group.stop_action_group()
        msg = BuzzerState()
        msg.freq = 2500
        msg.on_time = 0.05
        msg.off_time = 0.01
        msg.repeat = 1
        self.buzzer_pub.publish(msg)
        rospy.loginfo('SELECT/START: chassis + arm stopped')

    def l3_callback(self, new_state):
        pass

    def r3_callback(self, new_state):
        pass

    def joy_callback(self, joy_msg):
        axes = dict(zip(AXES_MAP, joy_msg.axes))
        axes_changed = False
        hat_x, hat_y = axes['hat_x'], axes['hat_y']
        hat_xl = self._hat_with_hysteresis('hat_xl', 1.0 if hat_x > 0 else 0.0)
        hat_xr = self._hat_with_hysteresis('hat_xr', 1.0 if hat_x < 0 else 0.0)
        hat_yu = self._hat_with_hysteresis('hat_yu', 1.0 if hat_y > 0 else 0.0)
        hat_yd = self._hat_with_hysteresis('hat_yd', 1.0 if hat_y < 0 else 0.0)
        buttons = list(joy_msg.buttons)
        buttons.extend([hat_xl, hat_xr, hat_yu, hat_yd, 0])
        buttons = dict(zip(BUTTON_MAP, buttons))
        for key, value in axes.items():
            if self.last_axes[key] != value:
                axes_changed = True
        if axes_changed:
            try:
                self.axes_callback(axes)
            except Exception as e:
                rospy.logerr(str(e))

        # 夹爪扳机：对齐 STM32 L2→#005P0600(-) / R2→#005P2400(+)
        r2_val = abs(axes['r2'])
        l2_val = abs(axes['l2'])
        g_cmd = self._joint_cmd['gripper']
        if g_cmd < 0:  # 正在 L2 方向（收）
            if l2_val > TRIG_OFF:
                self.set_joint_vel('gripper', -GRIPPER_SPEED)
            else:
                self.set_joint_vel('gripper', 0)
        elif g_cmd > 0:  # 正在 R2 方向（张）
            if r2_val > TRIG_OFF:
                self.set_joint_vel('gripper', GRIPPER_SPEED)
            else:
                self.set_joint_vel('gripper', 0)
        else:
            if l2_val > TRIG_ON:
                self.set_joint_vel('gripper', -GRIPPER_SPEED)
            elif r2_val > TRIG_ON:
                self.set_joint_vel('gripper', GRIPPER_SPEED)
            else:
                self.set_joint_vel('gripper', 0)

        for key, value in buttons.items():
            if key in ('l2', 'r2'):  # 夹爪改由 axes 扳机控制，跳过按钮通道避免双触发
                continue
            if value != self.last_buttons[key]:
                new_state = ButtonState.Pressed if value > 0 else ButtonState.Released
            else:
                new_state = ButtonState.Holding if value > 0 else ButtonState.Normal
            callback = "".join([key, '_callback'])
            if new_state != ButtonState.Normal:
                if hasattr(self, callback):
                    try:
                        getattr(self, callback)(new_state)
                    except Exception as e:
                        rospy.logerr(str(e))
        self.last_buttons = buttons
        self.last_axes = axes


if __name__ == "__main__":
    node = JoystickController()
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
