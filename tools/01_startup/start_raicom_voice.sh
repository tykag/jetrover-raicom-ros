#!/bin/bash
# WonderEchoPro 赛规语音（不要塞进 bringup 自检）
# 用法：先保证 bringup 已在跑，再：
#   export ROS_MASTER_URI=http://127.0.0.1:11311
#   export ROS_IP=127.0.0.1
#   unset ROS_HOSTNAME
#   bash ~/yolo_models/start_raicom_voice.sh
#
# 串口：默认自动找 /dev/ttyUSB0 或 ttyCH341*；也可：
#   export WONDERECHO_PORT=/dev/ttyUSB0

set -e
source ~/ros_ws/devel/setup.zsh 2>/dev/null || true

if [ -z "$ROS_MASTER_URI" ]; then
  export ROS_MASTER_URI=http://127.0.0.1:11311
  export ROS_IP=127.0.0.1
  unset ROS_HOSTNAME
fi

echo "ROS_MASTER_URI=$ROS_MASTER_URI"
echo "WONDERECHO_PORT=${WONDERECHO_PORT:-auto}"
ls -l /dev/ttyUSB* /dev/ttyCH341* 2>/dev/null || echo "(尚未看到 USB 串口，请确认 WonderEcho 已插上)"

# 不再启动讯飞环麦 mic_init / call.bnf
python3 ~/yolo_models/raicom_voice.py
