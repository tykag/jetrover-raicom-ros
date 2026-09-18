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

1. 开机环境：`01_startup/ros_env_auto.sh`
2. 一键视觉：`01_startup/start_raicom_vision.sh`
3. 检测程序：`04_vision_car/raicom_yolo_trt.py` → 拷到车上 `~/yolo_models/`
4. 看板姿势：`04_vision_car/raicom_arm_pose.py` → 车上 `install` 后 `look_board` / `look_front`
5. 赛规语音（WonderEchoPro 串口）：`04_vision_car/raicom_voice.py` + `01_startup/start_raicom_voice.sh`（协议表见 `项目总结/语音-WonderEchoPro/`，**不用**讯飞 mic_init）
6. **手柄上传**：`06_deploy/upload_to_car.py`（先改 `upload_to_car.ini` 的 IP；密码 ini 已 gitignore）
7. 总启动规则：`项目总结/开机自启/2026-09-06-开机自启-启动规则.md`
8. 项目文档总入口：`项目总结/总体/2026-09-07-总体-交接.md`
9. 手柄说明：`项目总结/机械臂/2026-09-17-机械臂-手柄STM32对齐.md`

## 废弃/备用

- `raicom_yolo_detect.py`：旧 torch.hub，不要用
- `raicom_yolo_onnx.py`：CPU 太慢，仅备用
- `raicom_yolo_oneshot.py`：调试用，比赛全自动不用
