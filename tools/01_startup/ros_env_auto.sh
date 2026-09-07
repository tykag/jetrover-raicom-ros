#!/bin/zsh
# 在任意终端先执行:  source ~/ros_env_auto.sh
# 自动使用小车当前 WiFi/热点 IP，实验室与比赛通用

source ~/ros_ws/devel/setup.zsh

export ROS_IP=$(hostname -I | awk '{print $1}')
export ROS_MASTER_URI=http://${ROS_IP}:11311
unset ROS_HOSTNAME

echo "ROS_IP=$ROS_IP"
echo "ROS_MASTER_URI=$ROS_MASTER_URI"
