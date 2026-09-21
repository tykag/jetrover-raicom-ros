#!/bin/zsh
# 停厂方自启后启动 RAICOM 栈（导航 + YOLO + 状态机）。
#   ~/start_raicom_auto.sh
#   SIDE=B ~/start_raicom_auto.sh
#   ~/start_raicom_auto.sh mapping_only
#   ENABLE_VOICE=1 ~/start_raicom_auto.sh   # 带小鹿小鹿 / 执行运输任务
# 导航已经在跑时不要用这个，改：
#   roslaunch raicom nodes.launch
#   roslaunch raicom voice.launch

set -u
TASK="${1:-full}"
SIDE="${SIDE:-A}"
SKIP_NAV="${SKIP_NAV:-0}"
SKIP_ARG="false"
if [ "$SKIP_NAV" = "1" ]; then
  SKIP_ARG="true"
fi
ENABLE_VOICE="${ENABLE_VOICE:-0}"
VOICE_ARG="false"
if [ "$ENABLE_VOICE" = "1" ]; then
  VOICE_ARG="true"
fi

sudo systemctl stop start_app_node.service || true
killall -9 rosmaster 2>/dev/null || true
sleep 1

export HOME="${HOME:-/home/hiwonder}"
export OPENBLAS_CORETYPE=ARMV8
export PYTHONUNBUFFERED=1
if [ -f /usr/lib/aarch64-linux-gnu/libgomp.so.1 ]; then
  export LD_PRELOAD="/usr/lib/aarch64-linux-gnu/libgomp.so.1${LD_PRELOAD:+:$LD_PRELOAD}"
fi

if [ -f "$HOME/ros_ws/.typerc" ]; then
  source "$HOME/ros_ws/.typerc"
fi
source "$HOME/ros_ws/devel/setup.zsh"
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"

echo "roslaunch raicom raicom.launch SIDE=$SIDE TASK=$TASK skip_nav=$SKIP_ARG enable_voice=$VOICE_ARG"
echo "RViz 里先 2D Pose Estimate。关 RViz 选 Discard。"
echo "  rosservice call /raicom/look_cargo"
echo "  rosservice call /raicom/test_pick"
echo "  语音：小鹿小鹿 → 执行运输任务  （或 rosservice call /raicom/start）"
echo "  rosservice call /raicom/abort"

roslaunch raicom raicom.launch \
  side:="$SIDE" \
  task:="$TASK" \
  skip_nav:="$SKIP_ARG" \
  cycles:="${CYCLES:-2}" \
  allow_high:=false \
  desktop:=true \
  enable_voice:="$VOICE_ARG"
