#!/bin/bash
# 单独启动讯飞麦克风 + RAICOM 赛规语音（不要塞进 bringup 自检）
# 用法：先保证 bringup/roscore 已在跑，再：
#   source ~/ros_env_auto.sh   # 或手设 127.0.0.1
#   bash ~/yolo_models/start_raicom_voice.sh

set -e
source ~/ros_ws/devel/setup.zsh 2>/dev/null || true

# 默认跟自启一致用本机；需要 WiFi 时改成 ros_env_auto
if [ -z "$ROS_MASTER_URI" ]; then
  export ROS_MASTER_URI=http://127.0.0.1:11311
  export ROS_IP=127.0.0.1
  unset ROS_HOSTNAME
fi

# 与课程/厂方默认一致；若仍 11212 需换有效 appid
APPID="${XF_APPID:-fb1ab2cb}"

echo "ROS_MASTER_URI=$ROS_MASTER_URI"
echo "appid=$APPID"

# 改过 call.bnf 后建议清一次旧语法缓存
# rm -rf ~/ros_ws/src/xf_mic_asr_offline/config/msc/grm/*

roslaunch xf_mic_asr_offline mic_init.launch appid:="$APPID" enable_setting:=False &
MIC_PID=$!
sleep 8

python3 ~/yolo_models/raicom_voice.py
# Ctrl+C 后收尾
kill $MIC_PID 2>/dev/null || true
