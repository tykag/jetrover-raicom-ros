#!/usr/bin/env python3
# encoding: utf-8
import os
import math
import time
import rclpy
import threading
from enum import Enum
from rclpy.node import Node
from sdk.common import val_map
from std_srvs.srv import Trigger
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from ros_robot_controller_msgs.msg import BuzzerState
from servo_controller_msgs.msg import ServosPosition, ServoPosition
from servo_controller.action_group_controller import ActionGroupController

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

# 与 controller/config/init_pose.yaml 的 servo 初始姿势一致，避免一按键整臂跳到错误位姿
HOME_POSITION = {
    'joint1': 500, 'joint2': 750, 'joint3': 0,
    'joint4': 375, 'joint5': 500, 'gripper': 500,
}
JOINT_STEP = 12     # 臂部关节每帧增量
GRIPPER_STEP = 36   # 夹爪每帧增量

# 动作组路径（与原有代码一致）
DEFAULT_ACTION_PATH = '/home/ubuntu/share/arm_pc/ActionGroups'

class ButtonState(Enum):
    Normal = 0
    Pressed = 1
    Holding = 2
    Released = 3

class JoystickController(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)

        self.min_value = 0.1
        self.declare_parameter('max_linear', 0.7)
        self.declare_parameter('max_angular', 3.0)
        self.declare_parameter('disable_servo_control', False)
        self.declare_parameter('machine_type', os.environ['MACHINE_TYPE'])
        self.declare_parameter('action_path', DEFAULT_ACTION_PATH)

        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.disable_servo_control = self.get_parameter('disable_servo_control').value
        self.machine = self.get_parameter('machine_type').value
        self.action_path = self.get_parameter('action_path').value

        # 底盘
        self.joy_sub = self.create_subscription(Joy, 'ros_robot_controller/joy', self.joy_callback, 1)
        self.buzzer_pub = self.create_publisher(BuzzerState, 'ros_robot_controller/set_buzzer', 1)
        self.mecanum_pub = self.create_publisher(Twist, 'controller/cmd_vel', 1)

        # 机械臂
        self.servo_pub = self.create_publisher(ServosPosition, 'servo_controller', 1)
        self.arm_position = dict(HOME_POSITION)
        # 动作组控制器（复用原有类）
        self.action_group = ActionGroupController(self.servo_pub, self.action_path)
        self.action_running = False

        self.last_axes = dict(zip(AXES_MAP, [0.0, ] * len(AXES_MAP)))
        self.last_buttons = dict(zip(BUTTON_MAP, [0.0, ] * len(BUTTON_MAP)))
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('joystick_control start: STM32-aligned (mecanum sticks + arm buttons)')
        self.get_logger().info('action path: %s' % self.action_path)

    def get_node_state(self, request, response):
        response.success = True
        return response

    def clamp(self, val, lo=0, hi=1000):
        return max(lo, min(hi, val))

    def publish_arm(self, joints=None, duration=0.02):
        """只发布指定关节，避免一次把整臂拉到内存里的错误中位。"""
        # 动作组播放时仍允许夹爪；其它关节暂不打断动作组
        if self.action_running and joints is not None and set(joints) != {'gripper'}:
            return
        if self.action_running and joints is None:
            return
        if joints is None:
            joints = list(ARM_JOINTS.keys()) + ['gripper']
        msg = ServosPosition()
        msg.duration = duration
        for name in joints:
            sp = ServoPosition()
            if name == 'gripper':
                sp.id = GRIPPER_ID
            else:
                sp.id = ARM_JOINTS[name]
            sp.position = int(self.arm_position[name])
            msg.position.append(sp)
        self.servo_pub.publish(msg)

    def arm_step(self, joint, delta):
        self.arm_position[joint] = self.clamp(self.arm_position[joint] + delta)
        self.publish_arm([joint])
        if joint == 'gripper':
            self.get_logger().info('gripper -> %d' % self.arm_position['gripper'], throttle_duration_sec=0.5)

    def run_action_thread(self, action_name):
        self.action_running = True
        self.get_logger().info('run action: %s' % action_name)
        try:
            self.action_group.run_action(action_name)
        except Exception as e:
            self.get_logger().error('action error: %s' % str(e))
        self.action_running = False
        self.get_logger().info('action finished: %s' % action_name)

    def axes_callback(self, axes):
        for k in ['lx', 'ly', 'rx', 'ry']:
            if abs(axes[k]) < self.min_value:
                axes[k] = 0

        # 底盘：对齐 STM32 app_ps2.c 绿灯摇杆
        # LX=原地转, LY=前后, RX=左右平移, RY=前后(可叠加)
        twist = Twist()
        if self.machine == 'JetRover_Mecanum':
            vx = val_map(axes['ly'], -1, 1, -self.max_linear, self.max_linear)
            vx += val_map(axes['ry'], -1, 1, -self.max_linear, self.max_linear)
            twist.linear.x = max(-self.max_linear, min(self.max_linear, vx))
            twist.linear.y = val_map(axes['rx'], -1, 1, -self.max_linear, self.max_linear)
            twist.angular.z = val_map(axes['lx'], -1, 1, -self.max_angular, self.max_angular)
        elif self.machine == 'JetRover_Tank':
            twist.linear.x = val_map(axes['ly'], -1, 1, -self.max_linear, self.max_linear)
            twist.angular.z = val_map(axes['lx'], -1, 1, -self.max_angular, self.max_angular)
        elif self.machine == 'JetRover_Acker':
            twist.linear.x = val_map(axes['ly'], -1, 1, -self.max_linear, self.max_linear)
            steering_angle = val_map(axes['lx'], -1, 1, -math.radians(150/1000*240), math.radians(150/1000*240))
            if twist.linear.x == 0:
                twist.linear.z = 1
            else:
                if steering_angle != 0:
                    R = 0.213/math.tan(steering_angle)
                    twist.angular.z = twist.linear.x/R
        self.mecanum_pub.publish(twist)

    # ===== 方向键: #000 joint1 / #001 joint2（对齐 STM32 绿灯按键）=====
    def hat_yu_callback(self, new_state):
        if new_state in (ButtonState.Pressed, ButtonState.Holding):
            self.arm_step('joint2', -JOINT_STEP)

    def hat_yd_callback(self, new_state):
        if new_state in (ButtonState.Pressed, ButtonState.Holding):
            self.arm_step('joint2', JOINT_STEP)

    def hat_xl_callback(self, new_state):
        if new_state in (ButtonState.Pressed, ButtonState.Holding):
            self.arm_step('joint1', JOINT_STEP)

    def hat_xr_callback(self, new_state):
        if new_state in (ButtonState.Pressed, ButtonState.Holding):
            self.arm_step('joint1', -JOINT_STEP)

    # ===== 功能键: #002 joint3 / #003 joint4 =====
    def triangle_callback(self, new_state):
        if new_state in (ButtonState.Pressed, ButtonState.Holding):
            self.arm_step('joint3', JOINT_STEP)

    def cross_callback(self, new_state):
        if new_state in (ButtonState.Pressed, ButtonState.Holding):
            self.arm_step('joint3', -JOINT_STEP)

    def square_callback(self, new_state):
        if new_state in (ButtonState.Pressed, ButtonState.Holding):
            self.arm_step('joint4', -JOINT_STEP)

    def circle_callback(self, new_state):
        if new_state in (ButtonState.Pressed, ButtonState.Holding):
            self.arm_step('joint4', JOINT_STEP)

    # ===== L1/R1: #004 joint5 =====
    def l1_callback(self, new_state):
        if new_state in (ButtonState.Pressed, ButtonState.Holding):
            self.arm_step('joint5', -JOINT_STEP)

    def r1_callback(self, new_state):
        if new_state in (ButtonState.Pressed, ButtonState.Holding):
            self.arm_step('joint5', JOINT_STEP)

    # ===== L2/R2: #005 夹爪（按钮通道备用）=====
    def l2_callback(self, new_state):
        if new_state in (ButtonState.Pressed, ButtonState.Holding):
            self.arm_step('gripper', -GRIPPER_STEP)

    def r2_callback(self, new_state):
        if new_state in (ButtonState.Pressed, ButtonState.Holding):
            self.arm_step('gripper', GRIPPER_STEP)

    # ===== SELECT/START: 对齐 STM32 $DJR! 急停 =====
    def select_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            self.stop_motion()

    def start_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            self.stop_motion()

    def stop_motion(self):
        self.mecanum_pub.publish(Twist())
        if self.action_running:
            self.action_group.stop_action_group()
        msg = BuzzerState()
        msg.freq = 2500
        msg.on_time = 0.05
        msg.off_time = 0.01
        msg.repeat = 1
        self.buzzer_pub.publish(msg)
        self.get_logger().info('SELECT/START: chassis + arm stopped')

    def l3_callback(self, new_state):
        pass

    def r3_callback(self, new_state):
        pass

    def joy_callback(self, joy_msg):
        axes = dict(zip(AXES_MAP, joy_msg.axes))
        axes_changed = False
        hat_x, hat_y = axes['hat_x'], axes['hat_y']
        hat_xl, hat_xr = 1 if hat_x > 0.5 else 0, 1 if hat_x < -0.5 else 0
        hat_yu, hat_yd = 1 if hat_y > 0.5 else 0, 1 if hat_y < -0.5 else 0
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
                self.get_logger().error(str(e))

        # 夹爪扳机：对齐 STM32 L2→#005P0600(-) / R2→#005P2400(+)
        r2_val = abs(axes['r2'])
        l2_val = abs(axes['l2'])
        if l2_val > self.min_value:
            self.arm_step('gripper', -GRIPPER_STEP)
        elif r2_val > self.min_value:
            self.arm_step('gripper', GRIPPER_STEP)

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
                        self.get_logger().error(str(e))
        self.last_buttons = buttons
        self.last_axes = axes

def main():
    node = JoystickController('joystick_control')
    rclpy.spin(node)

if __name__ == "__main__":
    main()
