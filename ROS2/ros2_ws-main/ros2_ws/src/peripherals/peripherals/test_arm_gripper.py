#!/usr/bin/env python3
# 不重启：直接测平放姿势 + 夹爪开合
import time
import rclpy
from rclpy.node import Node
from servo_controller_msgs.msg import ServosPosition, ServoPosition

def pub(node, positions, duration=1.0):
    pub = node.create_publisher(ServosPosition, 'servo_controller', 10)
    time.sleep(0.3)
    msg = ServosPosition()
    msg.duration = duration
    for sid, pos in positions:
        sp = ServoPosition()
        sp.id = sid
        sp.position = pos
        msg.position.append(sp)
    pub.publish(msg)
    print('publish:', positions)
    time.sleep(duration + 0.2)

def main():
    rclpy.init()
    node = Node('test_arm_gripper')
    # 平放向前
    pub(node, [(1, 500), (2, 750), (3, 0), (4, 375), (5, 500), (10, 500)], 1.0)
    # 夹爪张开
    pub(node, [(10, 200)], 0.8)
    # 夹爪闭合
    pub(node, [(10, 700)], 0.8)
    # 夹爪回中
    pub(node, [(10, 500)], 0.5)
    print('done')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
