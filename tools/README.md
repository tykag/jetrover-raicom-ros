# tools 目录说明（RAICOM / JetRover）

按模块存放，避免根目录堆文件。

| 文件夹 | 用途 | 谁用 |
|--------|------|------|
| `01_startup/` | ROS 环境、开机后启动视觉 | 小车 |
| `02_dataset/` | 采图 | 小车 |
| `03_train_pc/` | 导出 ONNX、修类别名 | Windows 电脑 |
| `04_vision_car/` | 车上检测脚本 | 小车 `~/yolo_models/` |
| `05_docs/` | 已迁到 `项目总结/`（按功能分类） | 阅读 |
| `06_deploy/` | 本机上传到小车（ini 配 IP/账号，失败回滚） | Windows 电脑 |

## 当前主用（记住这几个）

1. **比赛栈**：`01_startup/start_raicom_auto.sh` → 车上 `~/start_raicom_auto.sh`（`roslaunch raicom raicom.launch`）
2. **运行时代码**：`ROS1/ros_ws-main/ros_ws/src/raicom/`（不要把 `04_vision_car/*.py` 当主源）
3. **姿势/路点备份**：`04_vision_car/*.yaml`；车上以 `~/yolo_models/*.yaml` 为准
4. 赛规语音：`ENABLE_VOICE=1 ~/start_raicom_auto.sh` 或 `roslaunch raicom voice.launch`（口令：小鹿小鹿 → 执行运输任务）
5. 手柄上传：`06_deploy/upload_to_car.py`（ini 已 gitignore）
6. 下次开机：`项目总结/总体/2026-09-21-总体-ROS包与下次开机交接.md`
7. 开发流程：`项目总结/总体/2026-09-21-总体-开发流程.md`
8. 比赛总目标：`项目总结/总体/2026-09-07-总体-交接.md`
9. 手柄说明：`项目总结/机械臂/2026-09-17-机械臂-手柄STM32对齐.md`

`06_deploy/_*.py` 是一次性探车脚本，不要当启动入口。

## 废弃/备用

- `raicom_yolo_detect.py`：旧 torch.hub，不要用
- `raicom_yolo_onnx.py`：CPU 太慢，仅备用
- `raicom_yolo_oneshot.py`：调试用，比赛全自动不用
