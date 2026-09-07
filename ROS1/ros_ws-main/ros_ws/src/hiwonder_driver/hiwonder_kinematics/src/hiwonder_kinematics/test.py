#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/03/21
# @author:aiden
# 机械臂运动学库使用实例
import hiwonder_kinematics.transform as transform
from hiwonder_kinematics.forward_kinematics import ForwardKinematics
from hiwonder_kinematics.inverse_kinematics import get_ik, get_position_ik, set_link, get_link, set_joint_range, get_joint_range

###########forward_kinematics##################
fk = ForwardKinematics(debug=True)  # 实例化正运动学，开启打印

print('当前各连杆长度(m):', fk.get_link())  # 详细说明请参考transform里的注释
pulse = transform.pulse2angle([500, 721, 96, 331, 500])  # 舵机脉宽值转为弧度
print('input:', pulse)
res = fk.get_fk(pulse)  #获取运动学正解
print('output:', res)
print('rpy:', transform.qua2rpy(res[1]))
res = get_ik([0.33289165, 0.00378179, 0.43154828], 30, [-180, 180])
if res != []:
    for i in range(len(res)):
        print('rpy%s:'%(i + 1), res[i][1])  # 解对应的rpy值
        pulse = transform.angle2pulse(res[i][0])  # 转为舵机脉宽值
        for j in range(len(pulse)):
            print('output%s:'%(j + 1), pulse[j])
else:
    print('no solution')
