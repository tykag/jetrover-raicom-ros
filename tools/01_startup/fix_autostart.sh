#!/bin/bash
# 在小车上执行：
#   sed -i 's/\r$//' ~/fix_autostart.sh
#   bash ~/fix_autostart.sh
# 只修自启抢串口 / 嵌套语音，不改相机、雷达、手柄、APP、语音包本身。
set -euo pipefail

STAMP="$(date +%Y%m%d%H%M%S)"
SRC_CHECK="${HOME}/ros_ws/src/hiwonder_bringup/scripts/startup_check.py"
CTRL_LAUNCH="${HOME}/ros_ws/src/hiwonder_driver/ros_robot_controller/launch/ros_robot_controller_node.launch"
UDEV_DST="/etc/udev/rules.d/99-ttyACM0.rules"
SVC="/etc/systemd/system/start_app_node.service"

echo "==== 备份 ===="
cp -a "$SRC_CHECK" "${SRC_CHECK}.bak.${STAMP}"
cp -a "$CTRL_LAUNCH" "${CTRL_LAUNCH}.bak.${STAMP}"
sudo cp -a "$UDEV_DST" "${UDEV_DST}.bak.${STAMP}" 2>/dev/null || true
sudo cp -a "$SVC" "${SVC}.bak.${STAMP}"

echo "==== 1) 修好 udev（只忽略扩展板 1a86:55d4） ===="
sudo tee "$UDEV_DST" >/dev/null <<'EOF'
# 扩展板沁恒 CH343 (1a86:55d4) → /dev/rrc
# 禁止 ModemManager 探测该口；不要动相机/雷达/环麦。
SUBSYSTEM=="usb", ATTR{idVendor}=="1a86", ATTR{idProduct}=="55d4", ENV{ID_MM_DEVICE_IGNORE}="1"
KERNEL=="ttyACM*", SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", GROUP="users", MODE="0777", SYMLINK+="rrc", ENV{ID_MM_DEVICE_IGNORE}="1", ENV{ID_MM_PORT_IGNORE}="1"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb --subsystem-match=tty

echo "==== 2) 关掉 ModemManager（小车没有 4G，不影响开车/臂/相机） ===="
if systemctl list-unit-files | grep -q '^ModemManager'; then
  sudo systemctl disable --now ModemManager || true
fi
systemctl is-active ModemManager && echo "WARN: ModemManager 仍 active" || echo "ModemManager 已停止"

echo "==== 3) 自检不再嵌套拉语音 ===="
python3 - <<'PY'
import os, time, shutil
path = os.path.expanduser("~/ros_ws/src/hiwonder_bringup/scripts/startup_check.py")
with open(path, "rb") as f:
    raw = f.read().replace(b"\r\n", b"\n").replace(b"\r", b"\n")
text = raw.decode("utf-8", "replace")
lines, changed = [], False
for line in text.splitlines(True):
    s = line.lstrip()
    if "os.system" in line and "startup_test" in line and not s.startswith("#"):
        lines.append(line[:len(line)-len(s)] + "# " + s)
        changed = True
    else:
        lines.append(line)
with open(path, "w") as f:
    f.writelines(lines)
print("[patched]" if changed else "[already ok]", path)
print("startup_test 行:")
for i, line in enumerate(open(path), 1):
    if "startup_test" in line or "os.system" in line:
        print(f"  {i}:{line.rstrip()}")
PY

echo "==== 4) 控制器开机抢口时允许自己起来，不要拆掉整场 ===="
python3 - <<'PY'
from pathlib import Path
p = Path.home() / "ros_ws/src/hiwonder_driver/ros_robot_controller/launch/ros_robot_controller_node.launch"
text = p.read_text()
old = 'required="true"'
new = 'respawn="true" respawn_delay="3"'
if old in text and "ros_robot_controller_node.py" in text:
    p.write_text(text.replace(old, new, 1))
    print("[patched] required -> respawn", p)
else:
    print("[already ok or unexpected]", p)
    print(text)
PY

echo "==== 5) systemd：等 /dev/rrc 再启动，日志进 journal ===="
sudo tee "$SVC" >/dev/null <<'EOF'
[Unit]
Description=start node
After=NetworkManager.service time-sync.target
Wants=NetworkManager.service

[Service]
Type=simple
User=hiwonder
Environment=HOME=/home/hiwonder
Environment=USER=hiwonder
Restart=always
RestartSec=15
KillMode=control-group
TimeoutStopSec=20
ExecStartPre=/bin/bash -c 'for i in $(seq 1 40); do [ -e /dev/rrc ] && break; sleep 1; done; sleep 2; exit 0'
ExecStart=/home/hiwonder/ros_ws/src/hiwonder_bringup/scripts/source_env.bash roslaunch hiwonder_bringup bringup.launch
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl stop start_app_node.service || true
killall -9 roscore rosmaster roslaunch 2>/dev/null || true
sleep 2

echo "==== 6) 重启自启 ===="
ls -l /dev/rrc /dev/ttyACM0
sudo systemctl start start_app_node.service
echo "已启动，等 50 秒..."
sleep 50

echo
echo "==== 结果 ===="
sudo systemctl status start_app_node.service --no-pager | head -25
echo
grep -n 'has died\|started with pid.*ros_robot_controller' \
  "$(ls -t ~/.ros/log/*/roslaunch-hiwonder-*.log | head -1)" | tail -20 || true

echo
echo "看 Active 是否已稳定几十秒。连 ROS："
echo "  source ~/ros_ws/devel/setup.zsh"
echo "  export ROS_MASTER_URI=http://127.0.0.1:11311"
echo "  export ROS_IP=127.0.0.1"
echo "  unset ROS_HOSTNAME"
echo "  rostopic list"
echo "语音比赛加分不要靠自启，另开: ~/yolo_models/start_raicom_voice.sh"
