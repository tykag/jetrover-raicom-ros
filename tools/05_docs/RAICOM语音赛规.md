# RAICOM 语音加分（ROS1 环形麦课程 + 赛规）

> 官方手册（以这份为准）：  
> `1 教程资料/3 ROS1课程/11 语音基础课程/1 语音基础课程/配套手册/语音基础课程（环形麦克风阵列）.pdf`  
> 重点看 **第 5 章：替换语音资源包和 APPID**、**第 7 章：通过指令唤醒麦克风**。

实车日常是 **ROS1 + 讯飞环麦**，不要为了语音切到 ROS2。

---

## 1. 赛规要做什么（两项各 5 分）

| 加分 | 要求 | 我们怎么做 |
|------|------|------------|
| 语音播报 | 进中转区播 **「遥操作区任务已完成」** | `/raicom/announce_teleop_done` |
| 语音启动 | 自定义口令启动全自主 | 「小虎小虎」→「**执行全自主运输任务**」→ `/raicom/start_auto=true` |

---

## 2. 先按手册修「构建语法失败！11212」

手册说明：讯飞离线应用**免费约 90 天**，到期要**新建应用**（每人约可申请 5 次）。  
错误码 11212 = 试用授权过期。你日志里的 `appid=5aed8165` 多半已过期。

### 按手册 5.1～5.2 做（车上路径）

1. 打开 https://www.xfyun.cn → 控制台 → 新建应用  
2. **语音识别 → 离线命令词识别** → 记下新 **APPID**  
3. 下载 **离线命令词识别 SDK → Linux MSC**，解压取出 **`common.jet`**  
4. 备份并替换：

```bash
# 车上
cd ~/ros_ws/src/xf_mic_asr_offline/config/msc/res/asr/
ls
# 删掉旧的 common.jet（先备份）
mv common.jet common.jet.bak.$(date +%Y%m%d)
# 把新下载的 common.jet 拷到本目录
```

5. 改 `mic_init.launch` 的 appid（与新 jet **配套**）：

```bash
vim ~/ros_ws/src/xf_mic_asr_offline/launch/mic_init.launch
# <arg name="appid" default="你的新APPID"/>
```

6. 清旧语法缓存后再启：

```bash
rm -rf ~/ros_ws/src/xf_mic_asr_offline/config/msc/grm
```

**注意（手册原文要点）：离线资源 `common.jet` 必须与 APPID 对应，只改 ID 不换 jet 仍会失败。**

### 按手册 7.1 单独测唤醒（不要塞进 bringup）

```bash
source ~/ros_ws/devel/setup.zsh
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME

roslaunch xf_mic_asr_offline mic_init.launch
```

说「小虎小虎」，终端应打印唤醒角度。出现 `10108` 再喊一次即可（手册 7.3.1）。

---

## 3. 自启策略

**不要**在 `startup_check` 里嵌套 `startup_test.launch`（语音失败会拖死整车）。

```text
bringup 自启（无语音自检）
  → 再单独 start_raicom_voice.sh
```

仓库已改：`hiwonder_bringup/scripts/startup_check.py` 跳过嵌套语音。请同步到车上同文件。

---

## 4. 比赛口令与节点（在手册流程通了之后）

| 本仓库 | 车上 |
|--------|------|
| `ROS1/.../xf_mic_asr_offline/config/call.bnf` | 覆盖车上 `call.bnf`（已含「执行全自主运输任务」） |
| `tools/04_vision_car/raicom_voice.py` | `~/yolo_models/raicom_voice.py` |
| `tools/01_startup/start_raicom_voice.sh` | `~/yolo_models/start_raicom_voice.sh` |

播报 wav：

```text
~/yolo_models/voice/teleop_done.wav   # 内容：遥操作区任务已完成
```

```bash
bash ~/yolo_models/start_raicom_voice.sh
rosservice call /raicom/announce_teleop_done "{}"
# 唤醒后说：执行全自主运输任务
rostopic echo /raicom/start_auto
```

---

## 5. 手册章节对照

| 手册 | 用途 |
|------|------|
| 第 3 章 | 接线 / 串口调试 |
| **第 5 章** | **申请 APPID + 替换 common.jet（修 11212）** |
| 第 6 章 | 端口/udev（车上一般已是 `/dev/ring_mic`） |
| **第 7 章** | **`roslaunch ... mic_init.launch` 唤醒测试** |
| 后续「语音交互」课 | 应答 / 改 `call.bnf` 命令词 |

车上设备节点若已是 `/dev/ring_mic`（bringup 日志里有），第 6 章可跳过，优先做第 5 章换证。
