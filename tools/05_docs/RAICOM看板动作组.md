# 看板动作组 look_board / look_front

不转底盘，只播机械臂姿势。深度相机跟着 joint1 转到右手分拣板。

## 1. 拷到小车

把 `tools/04_vision_car/raicom_arm_pose.py` 拷到：

```text
~/yolo_models/raicom_arm_pose.py
```

```bash
sed -i 's/\r$//' ~/yolo_models/raicom_arm_pose.py
```

## 2. 安装（只做一次）

bringup / 舵机已在跑时：

```bash
source ~/ros_ws/devel/setup.zsh
export ROS_MASTER_URI=http://192.168.10.110:11311
export ROS_IP=192.168.10.110
unset ROS_HOSTNAME
python3 ~/yolo_models/raicom_arm_pose.py install
```

会在这些目录各写一份（哪个存在写哪个）：

- `~/software/arm_pc/ActionGroups/`
- `~/share/arm_pc/ActionGroups/`
- `~/yolo_models/ActionGroups/`

生成：`look_board.d6a`、`look_front.d6a`、`look_board_b.d6a`。

## 3. 调用

```bash
python3 ~/yolo_models/raicom_arm_pose.py look_board
python3 ~/yolo_models/raicom_arm_pose.py look_front
```

后续全自动里同样按名字播：`look_board` → 等 `mapping_ready` → `look_front` → 抓取。

## 4. 手柄微调后再存

方向键把板完全收入画面后，若右转不是 875，重装：

```bash
python3 ~/yolo_models/raicom_arm_pose.py install --j1-right 860
```

播放时不要同时掰手柄，避免抢舵机话题。
