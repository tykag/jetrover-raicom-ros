# 2026-09-21 语音：小鹿小鹿 + 执行运输任务

> 功能：新固件词表接到全自动启动  
> 平台：ROS1 Melodic、WonderEchoPro（CI1302 USB 串口）  
> 上一份以谁为准：环形麦撤回看 `2026-09-19-语音WonderEchoPro-环形麦撤回.md`；**词表和启动口令以本文为准**  
> 当前结论：协议字节没变。唤醒改成「小鹿小鹿」，口令改成「执行运输任务」，收到 `AA 55 00 01 FB` 后发 `/raicom/start_auto`，由 `raicom_auto` 开全自动。新词要烧进模组才生效。抓放没标完不要喊口令。

## 1. 新词表（截图）

播报模式是 **被**：模组识别后把帧发给小车，由车上 wav 出声，模组自己不播。

| 标签 | 命令词 | 类型 | 播报 | 协议 |
|------|--------|------|------|------|
| 1 | 欢迎语 | 欢迎语 | 欢迎使用小幻 | `AA 55 01 00 FB` |
| 2 | 休息语 | 休息语 | 我去休息啦 | `AA 55 02 00 FB` |
| 3 | **小鹿小鹿** | 唤醒词 | 我在 | `AA 55 03 00 FB` |
| 4～10 | 音量 / 开关播报 | 原厂控制 | 同词 | `AA 55 04～0A 00 FB` |
| 11 | **执行运输任务** | 命令词 | 收到 | `AA 55 00 01 FB` |

中转区播报「遥操作区任务已完成」仍是车上发 `AA 55 00 02 FB`（或放 `teleop_done.wav`），不在这两张截图里。

## 2. 车上链路

```text
小鹿小鹿     → 模组发 03 00 → raicom_voice 播 wozai.wav
执行运输任务 → 模组发 00 01 → /raicom/start_auto=true → raicom_auto 开循环
```

节点在 `raicom` 包：`ROS1/ros_ws-main/ros_ws/src/raicom/scripts/raicom_voice.py`。电脑备份同内容：`tools/04_vision_car/raicom_voice.py`。wav 仍读 `~/yolo_models/voice/`。

```bash
# 导航已在跑
export WONDERECHO_PORT=/dev/ring_mic
roslaunch raicom voice.launch

# 或整栈带语音（标定没完不要用）
ENABLE_VOICE=1 ~/start_raicom_auto.sh
```

看是否启动：`rostopic echo /raicom/start_auto`  
急停：`rosservice call /raicom/abort`

默认 `enable_voice:=false`。下次开机先对墙、存 `pick` / `pick_aim`，再开语音。口令和 `/raicom/start` 走同一条自动流程。

## 3. 必须烧固件

代码只认 5 字节帧，不认汉字。表改了但没烧进 CI1302，盒子仍听「小幻小幻 / 小虎小虎」。用 `PACK_UPDATE_TOOL.exe` 选 CI1302，Type-C，按 RST，烧当前比赛表。

备份：`tools/04_vision_car/backup/2026-09-21-语音小鹿/`
