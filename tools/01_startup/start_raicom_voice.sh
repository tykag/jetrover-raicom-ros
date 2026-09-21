#!/bin/bash
# WonderEchoPro 赛规语音（不要塞进 bringup 自检）
# 用法：bringup 或 raicom 导航已在跑，再：
#   export WONDERECHO_PORT=/dev/ring_mic
#   bash ~/start_raicom_voice.sh
# 或整栈带语音：
#   ENABLE_VOICE=1 ~/start_raicom_auto.sh
#
# 口令：小鹿小鹿 → 执行运输任务 → /raicom/start_auto

set -e
source ~/ros_ws/devel/setup.zsh 2>/dev/null || true

if [ -z "$ROS_MASTER_URI" ]; then
  export ROS_MASTER_URI=http://127.0.0.1:11311
  export ROS_IP=127.0.0.1
  unset ROS_HOSTNAME
fi

export WONDERECHO_PORT="${WONDERECHO_PORT:-/dev/ring_mic}"

echo "ROS_MASTER_URI=$ROS_MASTER_URI"
echo "WONDERECHO_PORT=$WONDERECHO_PORT"
ls -l /dev/ttyUSB* /dev/ttyCH341* /dev/ring_mic 2>/dev/null || echo "(尚未看到 USB 串口，请确认 WonderEcho 已插上)"

if command -v roslaunch >/dev/null 2>&1 && rospack find raicom >/dev/null 2>&1; then
  roslaunch raicom voice.launch
else
  python3 ~/yolo_models/raicom_voice.py
fi
