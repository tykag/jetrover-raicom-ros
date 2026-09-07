# JetRover 开发备忘（现状说明）

> 更新日期：2026-09-03  
> 用途：方便后续做 **视觉、自动夹取、运动到固定位置** 等开发。  
> 结论先看：**实车日常跑的是宿主机 ROS1，不是 Docker 里的 ROS2。**

---

## 1. 系统架构（最重要）

| 层级 | 内容 | 作用 |
|------|------|------|
| **宿主机** | 用户 `hiwonder`，工作空间 `~/ros_ws` | **日常开车实际在用这个** |
| **ROS 版本** | **ROS1 Melodic** | `roscore` / `roslaunch` / `catkin build` |
| **开机自启** | `start_app_node.service` | 跑 `roslaunch hiwonder_bringup bringup.launch` |
| **Docker 容器** | 名 `jetrover`，镜像 `jetrover:new` | 里面是 **ROS2 Humble**，路径 `/home/ubuntu/ros2_ws` |
| **ROS2** | 有源码，但自启不走它 | 终端提示「当前环境是ROS2」时才在容器里 |

**开发原则：** 视觉 / 夹取 / 定点，优先按 **ROS1（`~/ros_ws`）** 做；除非明确把自启切到 ROS2。

### 电脑上的源码对应关系

| 环境 | 本仓库路径 |
|------|------------|
| ROS1 | `ROS1/ros_ws-main/ros_ws/` |
| ROS2 | `ROS2/ros2_ws-main/ros2_ws/`（对应车上容器内工作空间） |

---

## 2. 车型与传感器

- 车型：`JetRover_Mecanum`（麦克纳姆轮）
- 雷达：A1（`rplidar`）
- 深度相机：Dabai（`depth_cam`）
- 语音：讯飞 xf
- 网络示例：`ROS_MASTER_URI=http://192.168.10.110:11311`  
  （本机调试可用 `http://127.0.0.1:11311`）
- 版本条示例：`V1.0.15 | App1.0 | 2025-03-18`

---

## 3. 目录与终端怎么区分

| 你在哪 | 怎么认 | 该干什么 |
|--------|--------|----------|
| 宿主机 ROS1 | 提示「当前环境是ROS1」，一般为 `hiwonder%` | 改 `~/ros_ws`、`catkin build`、`systemctl`、桌面拷文件 |
| 容器 ROS2 | 提示「当前环境是ROS2」 | 改 `/home/ubuntu/ros2_ws`、`colcon build`；`docker cp` 必须在宿主机做 |
| 文件管理器 Home | 能见 `ros_ws`，不见 `ros2_ws` | 正常：`ros2_ws` 在容器里 |

### 常用命令对照

```bash
# 宿主机：看自启
sudo systemctl status start_app_node.service

# 宿主机：重启自启（不必整机 reboot）
sudo systemctl restart start_app_node.service

# 宿主机：进 ROS2 容器
docker exec -it -u ubuntu jetrover zsh -l

# 容器里不能 reboot / 没有 docker 命令；要先 exit 回宿主机
```

本机调试 ROS1 连不上 master 时：

```bash
source ~/ros_ws/devel/setup.zsh
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
```

---

## 4. 机械臂与手柄

### 4.1 舵机 ID

| ID | 关节 |
|----|------|
| 1 | joint1 底座旋转 |
| 2 | joint2 大臂俯仰 |
| 3 | joint3 小臂俯仰 |
| 4 | joint4 腕部俯仰 |
| 5 | joint5 腕部旋转 |
| 10 | 夹爪（r_joint） |

### 4.2 推荐「平放向前」初始位

```
joint1=500, joint2=750, joint3=0, joint4=375, joint5=500, gripper=500
```

### 4.3 关键话题 / 节点（ROS1）

| 用途 | 名称 |
|------|------|
| 手柄输入 | `/ros_robot_controller/joy` |
| 舵机指令 | `/servo_controllers/port_id_1/multi_id_pos_dur` |
| 底盘速度 | `hiwonder_controller/cmd_vel` |
| 手柄节点 | `/joystick_control`（包 `hiwonder_peripherals`） |

### 4.4 源码路径

- 车上：`~/ros_ws/src/hiwonder_peripherals/scripts/joystick_control.py`
- 电脑：`ROS1/ros_ws-main/ros_ws/src/hiwonder_peripherals/scripts/joystick_control.py`

### 4.5 手柄操作（当前逻辑）

- 摇杆：底盘
- 方向键 / △○×□ / L1 R1：机械臂关节
- **L2 / R2**：夹爪（数据在 `axes` 的 `l2`/`r2`，不在 buttons）
- 控制方式：按住匀速、松开锁位（减轻卡顿与停后抖动）

### 4.6 已踩过的坑（务必避开）

1. **L2/R2 在 axes，不在 buttons** — 只读按钮会控不了夹爪  
2. **不要把 ROS2（`import rclpy`）文件拷进 ROS1 目录**  
3. **Windows 换行 `\r\n`** 会导致 `python3\r` 无法启动，拷贝后执行：  
   `sed -i 's/\r$//' 文件路径`  
4. **改 ROS2 容器不影响日常手柄** — 自启是 ROS1  
5. 工作空间用 **`catkin build`**，不是 `catkin_make`（会与已有 build 空间冲突）

### 4.7 修改手柄代码后生效流程

```bash
# 1. 确认是 ROS1 文件（应有 rospy，没有 rclpy）
grep -n "import rospy\|import rclpy" ~/Desktop/joystick_control.py

# 2. 覆盖并转 Unix 换行
cp ~/Desktop/joystick_control.py ~/ros_ws/src/hiwonder_peripherals/scripts/joystick_control.py
sed -i 's/\r$//' ~/ros_ws/src/hiwonder_peripherals/scripts/joystick_control.py

# 3. 编译 + 重启自启
cd ~/ros_ws && catkin build hiwonder_peripherals
sudo systemctl restart start_app_node.service
```

调速参数（在 `joystick_control.py` 顶部）：

- `JOINT_SPEED`：臂速度  
- `GRIPPER_SPEED`：夹爪速度  

---

## 5. 后续开发方向建议

### 5.1 视觉

- bringup 已带深度相机、`web_video_server`
- 相关包：`hiwonder_app`、`hiwonder_example`
- 开发前先确认话题：`rostopic list | grep -iE 'image|camera|depth'`

### 5.2 自动夹取

- 执行层仍是总线舵机：`multi_id_pos_dur`，或动作组 `.d6a`
- 动作组路径注意：launch 里可能写 `/home/ubuntu/share/...`，宿主机用户是 `hiwonder`，路径可能需改成 `/home/hiwonder/...`
- 建议流程：视觉得目标 → 坐标变换 → 逆解/示教点 → 发舵机序列 → 夹爪 ID 10
- 与手柄互斥：自动夹取时停 `/joystick_control` 或加状态机锁

### 5.3 运动到固定位置

- **底盘定点**：`hiwonder_navigation` / `hiwonder_slam`，速度话题 `hiwonder_controller/cmd_vel`
- **臂定点**：一组关节脉冲，或动作组 / init 姿态
- 两套坐标系分开：`map/odom/base_footprint`（导航） vs 舵机脉冲 0–1000（臂）

---

## 6. 自启与 bringup 要点

- 服务名：`start_app_node.service`
- 服务内容：`roslaunch hiwonder_bringup bringup.launch`
- `bringup.launch` 中已包含：底盘、深度相机、rosbridge、app、**手柄**、开机自检、初始姿态等
- 查看是否在跑：`sudo systemctl status start_app_node.service`
- 看节点：`rosnode list | grep -iE 'joy|servo|controller'`

---

## 7. 一句话现状

**实车 = ROS1 Melodic + `bringup` 自启；麦克纳姆 + Dabai + 总线臂（关节 1–5 + 夹爪 10）；手柄夹爪与平放初始已打通。视觉 / 自动夹取 / 定点应基于 ROS1 话题与导航包扩展；ROS2 仅在 Docker 容器中作为并行环境。**

---

## 8. 相关本地文件

| 文件 | 说明 |
|------|------|
| `ROS1/.../hiwonder_peripherals/scripts/joystick_control.py` | 已改：夹爪 axes、匀速、锁位消抖 |
| `ROS2/.../peripherals/peripherals/joystick_control.py` | ROS2 版（容器用，非当前自启） |
| `ROS2/.../controller/config/init_pose.yaml` | ROS2 初始姿态配置 |
| 本文档 | `开发备忘-JetRover现状.md` |
