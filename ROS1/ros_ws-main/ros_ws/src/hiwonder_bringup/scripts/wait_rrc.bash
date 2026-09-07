#!/bin/bash
# 自启 ExecStartPre：等扩展板口出现，并等厂方 hw_button_scan 先打开串口。
# 不要等“串口空闲”——button_scan 会一直占着 /dev/rrc，这是出厂设计。
set -u

echo "[wait_rrc] waiting for /dev/rrc"
for i in $(seq 1 40); do
  if [ -e /dev/rrc ]; then
    echo "[wait_rrc] /dev/rrc present after ${i}s"
    break
  fi
  sleep 1
done

if [ ! -e /dev/rrc ]; then
  echo "[wait_rrc] WARN: /dev/rrc still missing, continue anyway"
fi

echo "[wait_rrc] waiting for hw_button_scan.service"
for i in $(seq 1 20); do
  if systemctl is-active --quiet hw_button_scan.service 2>/dev/null; then
    echo "[wait_rrc] hw_button_scan active after ${i}s"
    break
  fi
  sleep 1
done

# 崩溃循环残留的控制器才需要等；不要动 button_scan
if pgrep -f 'ros_robot_controller_node' >/dev/null 2>&1; then
  echo "[wait_rrc] leftover ros_robot_controller, waiting to exit"
  for i in $(seq 1 15); do
    pgrep -f 'ros_robot_controller_node' >/dev/null 2>&1 || break
    sleep 1
  done
fi

# 手开能活，是因为口已经被 button_scan 打开一段时间。再给 USB 一点稳定时间。
sleep 3
ls -l /dev/rrc /dev/ttyACM0 2>/dev/null || true
echo "[wait_rrc] ready"
exit 0
