#!/bin/zsh
# 自主运行（不含语音）：停自启 → 导航 + YOLO + 状态机
# 用法:
#   ~/start_raicom_auto.sh              # 全流程
#   ~/start_raicom_auto.sh mapping_only # 只做分拣区看板锁定
#   SIDE=B ~/start_raicom_auto.sh
#   SKIP_NAV=1 ~/start_raicom_auto.sh mapping_only   # 人已经停在分拣区

set -u
TASK="${1:-full}"
SIDE="${SIDE:-A}"
SKIP_NAV="${SKIP_NAV:-0}"

sudo systemctl stop start_app_node.service || true
killall -9 rosmaster 2>/dev/null || true
sleep 1

export HOME="${HOME:-/home/hiwonder}"
export OPENBLAS_CORETYPE=ARMV8
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

echo "ROS_MASTER_URI=$ROS_MASTER_URI SIDE=$SIDE TASK=$TASK SKIP_NAV=$SKIP_NAV"

roslaunch hiwonder_navigation navigation.launch map:=explore robot_name:=/ master_name:=/ >/tmp/raicom_nav.log 2>&1 &
NAV_PID=$!
echo "navigation pid $NAV_PID"
sleep 12

python3 "$HOME/yolo_models/raicom_yolo_trt.py" _show:=false >/tmp/raicom_yolo.log 2>&1 &
YOLO_PID=$!
echo "yolo pid $YOLO_PID"
sleep 3

cleanup() {
  kill $NAV_PID $YOLO_PID 2>/dev/null || true
}
trap cleanup EXIT

SKIP_ARG="false"
if [ "$SKIP_NAV" = "1" ]; then
  SKIP_ARG="true"
fi

echo "RViz 里先 2D Pose Estimate 对好再:"
echo "  rosservice call /raicom/start"
echo "路点: rostopic pub -1 /raicom/save_pose std_msgs/String \"data: sort\""

python3 "$HOME/yolo_models/raicom_auto.py" _side:="$SIDE" _task:="$TASK" _skip_nav:="$SKIP_ARG"
