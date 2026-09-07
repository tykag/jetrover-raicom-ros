#!/bin/zsh
# 实验室 / 比赛热点通用：自动检测本机 IP 后启动 TRT 视觉
# 自动 IP 同时适用于实验室 192.168.x 与手机热点（IP 不固定）
# 用法: ~/start_raicom_vision.sh

source ~/ros_ws/devel/setup.zsh

export ROS_IP=$(hostname -I | awk '{print $1}')
export ROS_MASTER_URI=http://${ROS_IP}:11311
unset ROS_HOSTNAME

echo "========================================"
echo " ROS_IP         = $ROS_IP"
echo " ROS_MASTER_URI = $ROS_MASTER_URI"
echo "========================================"

if [ -z "$ROS_IP" ]; then
  echo "错误: 没有拿到 IP。请先让小车连上 WiFi/手机热点。"
  exit 1
fi

echo "等待 ROS master..."
ok=0
for i in $(seq 1 90); do
  if rostopic list >/dev/null 2>&1; then
    echo "ROS OK (第 ${i} 秒)"
    ok=1
    break
  fi
  sleep 1
done

if [ "$ok" != "1" ]; then
  echo "错误: 连不上 master。"
  echo "1) sudo systemctl status start_app_node.service"
  echo "2) 自启是否仍写死旧实验室 IP？比赛前应改成自动 IP 启动。"
  echo "3) 临时: sudo systemctl stop start_app_node.service 后手动 bringup"
  exit 1
fi

echo "启动 TRT 视觉..."
exec python3 ~/yolo_models/raicom_yolo_trt.py
