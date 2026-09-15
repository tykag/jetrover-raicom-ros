# 2026-09-15 语音 WonderEchoPro 赛规（串口协议）

> 硬件：**WonderEchoPro**（CI1302，USB 串口 115200）  
> 固件表（上传烧录用这份）：  
> - `项目总结/语音-WonderEchoPro/命令词播报词协议列表V3_中文_RAICOM比赛.xlsx`  
> - `1 WonderEchoPro/附录/命令词播报词协议列表V3_中文_RAICOM比赛_精简.xlsx`（同内容）  
> 车上节点：`tools/04_vision_car/raicom_voice.py` → `~/yolo_models/raicom_voice.py`

**不要**再用讯飞环麦 `mic_init` / `call.bnf` / APPID。出厂听「小幻小幻」；比赛要烧本表固件才认「小虎小虎」。

---

## 1. 赛规两项与协议

| 加分 | 现场动作 | 串口方向 | 5 字节帧 |
|------|----------|----------|----------|
| 语音播报 5 分 | 进中转区播「遥操作区任务已完成」 | **小车 → 模组** | `AA 55 FF 01 FB` |
| 语音启动 5 分 | 「小虎小虎」→「执行全自主运输任务」 | **模组 → 小车** | 唤醒 `AA 55 03 00 FB`；口令 `AA 55 00 01 FB` |

表里「发送协议 / 接收协议」写成相同，与厂方模板一致：

- **发送协议**：模组识别到词后发给小车（代码里 `read`）
- **接收协议**：小车发给模组触发播报（代码里 `write`）
- 控制段 `01/02/03` 与厂方 `WonderEchoPro.WAKEUP/SLEEP` 对齐，**协议字节不能改**

比赛固件已去掉音量、颜色、导航等演示词，只留：欢迎 / 休息 / 唤醒 / 启动口令 / 中转播报。

---

## 2. 固件表要改什么（已做好比赛表）

上传并烧录：`命令词播报词协议列表V3_中文_RAICOM比赛.xlsx`

| 语义 | 命令词 | 类型 | 模式 | 协议 |
|------|--------|------|------|------|
| 1 | 欢迎语 | 欢迎语 | 被 | `AA 55 01 00 FB` |
| 2 | 休息语 | 休息语 | 主 | `AA 55 02 00 FB` |
| 3 | 小虎小虎 / 你好小虎 | 唤醒词 | 主 | `AA 55 03 00 FB` |
| 11 | 执行全自主运输任务 / 开始全自主 / 开始运输 | 命令词 | 主 | `AA 55 00 01 FB` |
| 13 | 遥操作区任务已完成 | **播报语** | **被** | `AA 55 FF 01 FB` |

制作：启英泰伦平台上传表 → 下固件 → `PACK_UPDATE_TOOL` 烧录（见 `1 WonderEchoPro/10.1.4+固件制作.pdf`）。

---

## 3. 车上怎么跑

```bash
# 终端：地址与自启一致
source ~/ros_ws/devel/setup.zsh
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME

# 可选指定口
# export WONDERECHO_PORT=/dev/ttyUSB0

# 拷贝后
sed -i 's/\r$//' ~/yolo_models/raicom_voice.py ~/yolo_models/start_raicom_voice.sh
bash ~/yolo_models/start_raicom_voice.sh
```

测播报 / 看启动：

```bash
rosservice call /raicom/announce_teleop_done "{}"
rostopic echo /raicom/start_auto
```

先喊「小虎小虎」（模组播「我在」），再喊「执行全自主运输任务」或「开始运输」→ `/raicom/start_auto` 应变 `true`。
