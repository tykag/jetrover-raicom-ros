#!/bin/bash
# 方案 B：对齐自启与手开。只动启动方式，不改功能、不关 WiFi、不拆语音、不改 required。
# 车上执行：
#   sed -i 's/\r$//' ~/apply_autostart_b.sh
#   bash ~/apply_autostart_b.sh
set -euo pipefail

STAMP="$(date +%Y%m%d%H%M%S)"
SRC_ENV="${HOME}/ros_ws/src/hiwonder_bringup/scripts/source_env.bash"
WAIT_RRC="${HOME}/ros_ws/src/hiwonder_bringup/scripts/wait_rrc.bash"
SVC="/etc/systemd/system/start_app_node.service"

echo "==== 备份 ===="
cp -a "$SRC_ENV" "${SRC_ENV}.bak.${STAMP}"
sudo cp -a "$SVC" "${SVC}.bak.${STAMP}"

echo "==== 1) source_env.bash：127.0.0.1 + OpenBLAS ARMV8 + unset ROS_HOSTNAME ===="
python3 - <<'PY'
import os
path = os.path.expanduser("~/ros_ws/src/hiwonder_bringup/scripts/source_env.bash")
with open(path, "rb") as f:
    raw = f.read().replace(b"\r\n", b"\n").replace(b"\r", b"\n")
new = r"""#!/bin/bash

# systemd 下偶发 HOME/USER 为空，先钉死
export HOME="${HOME:-/home/hiwonder}"
export USER="${USER:-hiwonder}"

source "$HOME/ros_ws/.typerc"

export CUDA_HOME=/usr/local/cuda
export PATH=$CUDA_HOME/bin:$PATH
export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH

# 对齐终端 .hiwonderrc：缺这两项时 numpy/OpenBLAS 会 SIGILL（exit -4）
export OPENBLAS_CORETYPE=ARMV8
if [ -f /usr/lib/aarch64-linux-gnu/libgomp.so.1 ]; then
  export LD_PRELOAD="/usr/lib/aarch64-linux-gnu/libgomp.so.1${LD_PRELOAD:+:$LD_PRELOAD}"
fi
export AUDIODRIVER=alsa
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
export PULSE_SERVER="${PULSE_SERVER:-unix:$XDG_RUNTIME_DIR/pulse/native}"

# 对齐手开成功的那套：不要用 localhost，也不要设 ROS_HOSTNAME
export ROS_IP=127.0.0.1
export ROS_MASTER_URI=http://127.0.0.1:11311
unset ROS_HOSTNAME

if [ $ZSH_VERSION ]; then
  . /opt/ros/melodic/setup.zsh
  . $HOME/ros_ws/devel/setup.zsh
elif [ $BASH_VERSION ]; then
  . /opt/ros/melodic/setup.bash
  . $HOME/ros_ws/devel/setup.bash
else
  . /opt/ros/melodic/setup.sh
  . $HOME/ros_ws/devel/setup.sh
fi
export DISPLAY=:0.0
exec "$@"
"""
with open(path, "w", newline="\n") as f:
    f.write(new)
os.chmod(path, 0o755)
print("[wrote]", path)
PY

echo "==== 2) wait_rrc.bash：等 /dev/rrc，并等厂方 button_scan 先占口 ===="
python3 - <<'PY'
import os
path = os.path.expanduser("~/ros_ws/src/hiwonder_bringup/scripts/wait_rrc.bash")
text = r"""#!/bin/bash
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

if pgrep -f 'ros_robot_controller_node' >/dev/null 2>&1; then
  echo "[wait_rrc] leftover ros_robot_controller, waiting to exit"
  for i in $(seq 1 15); do
    pgrep -f 'ros_robot_controller_node' >/dev/null 2>&1 || break
    sleep 1
  done
fi

sleep 3
ls -l /dev/rrc /dev/ttyACM0 2>/dev/null || true
echo "[wait_rrc] ready"
exit 0
"""
with open(path, "w", newline="\n") as f:
    f.write(text)
os.chmod(path, 0o755)
print("[wrote]", path)
PY

echo "==== 3) systemd：等口、日志进 journal ===="
sudo tee "$SVC" >/dev/null <<'EOF'
[Unit]
Description=start node
After=NetworkManager.service time-sync.target hw_button_scan.service
Wants=NetworkManager.service

[Service]
Type=simple
User=hiwonder
Environment=HOME=/home/hiwonder
Environment=USER=hiwonder
Environment=OPENBLAS_CORETYPE=ARMV8
WorkingDirectory=/home/hiwonder
Restart=always
RestartSec=20
KillMode=control-group
TimeoutStopSec=20
TimeoutStartSec=90
ExecStartPre=/home/hiwonder/ros_ws/src/hiwonder_bringup/scripts/wait_rrc.bash
ExecStart=/home/hiwonder/ros_ws/src/hiwonder_bringup/scripts/source_env.bash roslaunch hiwonder_bringup bringup.launch
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

sudo sed -i 's/\r$//' "$SRC_ENV" "$WAIT_RRC" "$SVC"
chmod +x "$SRC_ENV" "$WAIT_RRC"

echo "==== 4) 停崩溃循环，清残留 ROS ===="
sudo systemctl daemon-reload
sudo systemctl stop start_app_node.service || true
killall -9 roscore rosmaster roslaunch 2>/dev/null || true
sleep 3

echo "==== 5) 当前串口 ===="
ls -l /dev/rrc /dev/ttyACM0 2>/dev/null || echo "WARN: /dev/rrc 还不在"
fuser /dev/rrc /dev/ttyACM0 2>/dev/null || echo "串口当前空闲"

echo "==== 6) 启动自启 ===="
sudo systemctl enable start_app_node.service
sudo systemctl start start_app_node.service
echo "已 start，等 55 秒看是否稳住..."
sleep 55

echo
echo "==== 结果 ===="
sudo systemctl status start_app_node.service --no-pager | head -30
echo
echo "==== journal（wait_rrc / controller）===="
sudo journalctl -u start_app_node.service -n 80 --no-pager | tail -80

echo
echo "连 ROS（新终端）："
echo "  source ~/ros_ws/devel/setup.zsh"
echo "  export ROS_MASTER_URI=http://127.0.0.1:11311"
echo "  export ROS_IP=127.0.0.1"
echo "  unset ROS_HOSTNAME"
echo "  rostopic list"
echo
echo "正常：Active=active (running) 且已经几十秒，不是又只有 5～8 秒。"
echo "若仍 auto-restart：把上面 status + journal 贴回来，再走方案 C（查谁占 ttyACM0）。"
