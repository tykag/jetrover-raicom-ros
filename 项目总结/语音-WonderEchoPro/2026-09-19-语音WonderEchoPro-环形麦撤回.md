# 2026-09-19 语音 WonderEchoPro：环形麦误改已撤回

> 时间：2026-09-19 00:03（UTC+8）  
> 功能：语音 / WonderEchoPro  
> 工作区：`d:\Axinbancar\JetRover三合一ROS智能车\3 源码资料`  
> 文档位置：`项目总结/语音-WonderEchoPro/`  
> 实车：宿主机 **ROS1 Melodic**（`hiwonder` / `~/ros_ws`）。日常**不要**切 Docker ROS2。  
> 车 IP（现场 WiFi 已改）：**`10.217.187.242`**（SSH `hiwonder` / `hiwonder`）。旧地址 `192.168.10.110` 作废。

本文覆盖今晚：纠正麦克风硬件、撤回讯飞环形麦改动、并已打到车上。更早的自启 / 视觉 / 臂 / 自主骨架仍以 `项目总结/总体/2026-09-07-总体-交接.md`、`项目总结/总体/2026-09-18-总体-自主运行.md` 为准；**语音以本文 + `2026-09-15-语音WonderEchoPro-赛规串口.md` 为准，不要再跟 09-07 里「环麦已写入小虎小虎」那条。**

---

## 1. 今晚结论（先读这段）

车上语音硬件是 **WonderEcho Pro**（CI1302 语音盒：USB 声卡 + CH340 串口），**不是**讯飞六路环形麦。

2026-09-07 按环形麦改了 `awake_node` / `mic_init` / `asr_node`，并向 `/dev/ring_mic` 发讯飞握手。udev 把 WonderEcho 的 CH340 链成了 `/dev/ring_mic -> ttyUSB1`，所以那套改动会占串口、把盒子打哑，喊「小虎小虎」不会有反应。

**2026-09-19 00:01 已上车（`10.217.187.242`）：**

- 环形麦源码改回厂方（握手超时、拼音 `xiao3 hu3`、asr 播「我在」全部撤回）
- `startup_check.py`：`MIC_TYPE != xf` 时**不再** `roslaunch xf_mic_asr_offline startup_test.launch`
- `.typerc`：`MIC_TYPE=WonderEchoPro`（车上原本已是这个值，保持）
- 当时跑着的 `xf_mic` / `awake_node` 已杀掉，`/dev/ring_mic` 空闲
- 底盘自启 `start_app_node.service` **仍是 active**，没动 OpenBLAS 补丁

重启后也不会再自动拉环形麦。不要再 `roslaunch xf_mic_asr_offline …`。

---

## 2. 串口怎么认

| 设备 | 节点 | 说明 |
|------|------|------|
| `/dev/ttyUSB0` | 雷达 rplidar | **不要**给 WonderEcho 用 |
| `/dev/ttyUSB1` → `/dev/ring_mic` | WonderEcho CH340 | 115200，帧 `AA 55 xx xx FB` |
| USB PnP Audio（`0c76:161f`） | WonderEcho 声卡 | 模组自己播「我在」；上位机播报走串口协议 |
| `/dev/ttyACM0` → `/dev/rrc` | 扩展板 | `hw_button_scan` 一直占着，正常，不要杀 |

`tools/04_vision_car/raicom_voice.py` 里 `find_port()` **先试 `/dev/ttyUSB0`**，会误开雷达。跑语音时必须指定：

```bash
export WONDERECHO_PORT=/dev/ring_mic
# 或
export WONDERECHO_PORT=/dev/ttyUSB1
```

---

## 3. WonderEcho 赛规（未变，固件还没烧进比赛包）

细则：`项目总结/语音-WonderEchoPro/2026-09-15-语音WonderEchoPro-赛规串口.md`  
比赛表：`项目总结/语音-WonderEchoPro/命令词播报词协议列表V3_中文_RAICOM比赛.xlsx`

| 动作 | 方向 | 帧 |
|------|------|-----|
| 喊「小虎小虎」→ 模组说「我在」 | 模组 → 车 | `AA 55 03 00 FB`（唤醒词模式必须是 **主**） |
| 「执行全自主运输任务」 | 模组 → 车 | `AA 55 00 01 FB` → `/raicom/start_auto` |
| 播「遥操作区任务已完成」 | 车 → 模组 | `AA 55 00 02 FB` |

改唤醒词 **不能** 用讯飞 `enable_setting:=true` / 拼音。必须用 `PACK_UPDATE_TOOL.exe` 选 **CI1302**，Type-C 接电脑，按盒子 **RST** 烧比赛表生成的 `.bin`。

没烧比赛固件时，出厂中文包认的是「小幻小幻」，不是「小虎小虎」。

节点：`tools/04_vision_car/raicom_voice.py` → 车上 `~/yolo_models/raicom_voice.py`。

```bash
source ~/ros_ws/devel/setup.zsh
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
export WONDERECHO_PORT=/dev/ring_mic
sed -i 's/\r$//' ~/yolo_models/raicom_voice.py
python3 ~/yolo_models/raicom_voice.py
```

测播报：`rosservice call /raicom/announce_teleop_done "{}"`  
看启动：`rostopic echo /raicom/start_auto`

---

## 4. 仍有效（今晚没动）

**自启：** `start_app_node` → `wait_rrc.bash` → `source_env.bash`（`OPENBLAS_CORETYPE=ARMV8` + libgomp）→ `bringup.launch`。死因是缺 OpenBLAS 环境导致 SIGILL，不是语音、不是 WiFi。不要关 WiFi / ModemManager / 不要跑 `fix_autostart.sh`。不要杀 `hw_button_scan`。

连 ROS（新终端）：

```bash
source ~/ros_ws/devel/setup.zsh
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
```

不要和 `.hiwonderrc` 里的 WiFi IP 混用。有 master 时禁止再 `roscore`。

**视觉 / 臂 / 自主：** 见 `项目总结/总体/2026-09-18-总体-自主运行.md`。语音启动也可用服务代替口令（以当时脚本为准）。

---

## 5. 尚未落地

- [ ] 把比赛 xlsx 烧进 WonderEcho（小虎小虎 / 启动口令 / 中转播报）
- [ ] 车上用 `WONDERECHO_PORT=/dev/ring_mic` 跑通 `raicom_voice`（唤醒帧 + `00 01` + `00 02`）
- [ ] 修正 `raicom_voice.py` 的端口探测顺序（不要先开 `ttyUSB0`）
- [ ] 抓放 yaml 实车标定；黄台双落点示教
- [ ] GitHub 备份：本地有 commit，走代理 `127.0.0.1:7890` 的 `git push` 曾卡住，远程可能仍空

---

## 6. 不要做

- 不要再改 / 再启动 `xf_mic_asr_offline`（`mic_init`、`startup_test`、`awake_node`）
- 不要对 `/dev/ring_mic` 发讯飞 `A5 01` 握手或 `wakeup_keywords` JSON
- 不要把 `MIC_TYPE` 改回 `xf`
- 不要关 WiFi、不要杀 `hw_button_scan`、不要跑 `fix_autostart.sh`
- 不要切 Docker ROS2
- Windows 拷到车后：`sed -i 's/\r$//' 文件`

---

## 7. 今晚改过的文件

本地 + 已 SCP 到车（00:01）：

- `ROS1/.../xf_mic_asr_offline/scripts/awake_node.py`（厂方握手，拼音回 `xiao3 huan4`）
- `ROS1/.../xf_mic_asr_offline/scripts/asr_node.py`（去掉 play_wozai）
- `ROS1/.../xf_mic_asr_offline/launch/mic_init.launch`
- `ROS1/.../hiwonder_bringup/scripts/startup_check.py`（非 xf 不启环形麦）
- `ROS1/.../ros_ws/.typerc`：`MIC_TYPE=WonderEchoPro`

未动：`source_env.bash` 自启补丁、APPID、`call.bnf`。

---

开场可粘贴：续做 RAICOM。先读 `项目总结/语音-WonderEchoPro/2026-09-19-语音WonderEchoPro-环形麦撤回.md`。车 IP `10.217.187.242`。语音是 WonderEchoPro，环形麦改动已撤回且已停进程。不要启 xf_mic。自启 OpenBLAS 补丁仍在。实车 ROS1，不要切 ROS2，不要关 WiFi。
