# RAICOM 主动视觉抓取与配送设计

> 功能：为待派送区和园区增加主动观察、视觉精靠近、四摞八槽位库存记录与可验证抓放  
> 平台：ROS1 Melodic、JetRover 麦克纳姆、Dabai RGB-D、总线机械臂、TensorRT YOLO  
> 现状依据：`项目总结/总体/2026-09-21-总体-开机同步交接.md`  
> 当前结论：采用“二维图像伺服 + 深度安全判断 + 离散观察姿势 + 示教抓放”。第一阶段只验收单个下层方块完整闭环，但数据模型和接口必须支持四摞八块、三个抓取面。
>
> 离线实现状态：第一阶段算法库、ROS 接口、canary launch 已写入 `raicom` 包，电脑单元测试已通过。尚未连车，未跑 `catkin build`，未部署，不作为实车可用结论。详见 `项目总结/总体/2026-09-22-总体-离线主动视觉第一阶段.md`。

## 1. 目标与范围

最终目标是自动处理待派送区四摞、每摞两块的八个随机方块，根据分拣板给出的 `gear/bolt → P1/P2` 映射，配送到两个园区各自的两摞两层位置。

第一阶段只开放：

- 单个下层方块；
- 自动搜索、视觉靠近和已验证的 `pick_down_low` 高位侧夹；
- 抓后验证；
- 导航到一个园区 approach；
- 黄色台面视觉精靠近；
- 低层放置和放后验证。

第一阶段不开放：

- 高层抓放；
- 八块连续自动运行；
- 未示教的园区第二摞；
- 三维手眼逆解作为默认抓取路径；
- 在无法确认目标、深度或抓取结果时继续运行。

## 2. 已确认的场地与机械约束

待派送区共有四摞八个槽位：

```text
front_left.top       front_left.bottom
front_right.top      front_right.bottom
inner_left.top       inner_left.bottom
inner_right.top      inner_right.bottom
```

抓取面固定为：

1. 正面处理 `front_left` 和 `front_right`，共四块；
2. 左侧处理 `inner_left`，共两块；
3. 右侧处理 `inner_right`，共两块。

每摞的类别随机。上层未移走前，下层类别可能不可见，因此系统不得预先猜测下层类别。

抓取顺序按面清空：

```text
front_left.top → front_left.bottom
→ front_right.top → front_right.bottom
→ inner_left.top → inner_left.bottom
→ inner_right.top → inner_right.bottom
```

不采用“先抓四个上层、再抓四个下层”，以避免重复切换三个抓取面及其导航、定位误差。

机械臂采用侧夹。下层方块不能夹侧面几何中心：夹爪下方的舵机或结构会撞台面。下层统一复用已经实车成功的 `pick_down_low`，在方块侧面上半部夹持。三个抓取面通过改变底盘位置和朝向复用同一机械臂抓取几何。上层必须另行示教，未验证前保持禁用。

## 3. 技术路线

采用二维视觉伺服为主、深度辅助的混合方案：

- YOLO 识别方块类别和图像位置；
- 预设机械臂观察姿势主动改变视角；
- 麦克纳姆底盘利用图像误差做横移和前后慢移；
- 深度只用于距离、稳定性和越界保护；
- 机械臂执行实车示教的固定抓放姿势；
- 不把尚未达到可靠精度的三维手眼逆解作为比赛主路径。

全局导航只负责到达安全观察点：

- 待派送区使用 `pick`、`pick_inner_l`、`pick_inner_r`；
- 园区使用 `park1_approach`、`park2_approach`；
- 最后约 10–40 cm 由局部视觉闭环接管。

## 4. 组件和职责

### 4.1 方块检测与跟踪

在 `raicom_yolo_trt.py` 基础上输出当前帧全部候选，而不是只输出一个目标。每个候选至少包含：

- 时间戳；
- 类别和置信度；
- 边界框及归一化中心；
- 临时跟踪 ID；
- 所属抓取面和槽位 ROI 的匹配结果。

跟踪 ID 只用于同一次观察和对位，不作为跨导航阶段的永久身份。永久身份由四摞八槽位模型承担。

### 4.2 主动观察与局部对位

独立模块负责：

- 执行高、中、低离散观察姿势；
- 在当前抓取面的合法 ROI 中锁定目标；
- 目标不可见时请求限定横移搜索；
- 根据图像误差输出限速底盘命令；
- 根据连续稳定帧、深度和超时判定成功或失败。

该模块不得直接执行抓取，也不得修改任务账本。

### 4.3 八槽位任务账本

每个槽位保存：

```text
state: unknown | visible | locked | picked | failed | unverified
class: gear | bolt | unknown
target_park: P1 | P2 | unknown
face: front | left | right
layer: top | bottom
confidence
attempts
```

约束：

- 上层未验证为已移走时，下层保持 `unknown`；
- 一个槽位最多消费一次；
- 抓取动作完成不等于 `picked`；
- 只有抓后视觉验证通过才更新为 `picked`；
- 类别只在当前槽位稳定检测后写入，不允许在伺服途中切换目标。

### 4.4 园区视觉定位

园区不依赖外贴 AprilTag/ArUco。独立检测模块从 RGB-D 中提取：

- 黄色台面区域；
- 靠车一侧的台面前沿；
- 台面左右边界和中心；
- 已放方块造成的占用区域。

左右图像误差控制麦克纳姆横移，台面前沿深度控制前后慢移。AMCL 只作为区域和总位移保护，不承担最后几厘米的精定位。

### 4.5 状态机

新的 `raicom_mission_node.py` 只编排：

- 抓取面切换；
- 槽位选择；
- 观察、对位、抓取和验证；
- 类别到园区映射；
- 园区槽位选择；
- 配送、放置和验证；
- 重试、跳过、停止和 `/raicom/abort`。

图像算法、跟踪算法和园区分割不得继续堆进状态机文件。

现有 `raicom_auto.py` 在迁移期间只保留兼容入口；新流程验收后再切换 launch，不同时维护两套任务逻辑。

### 4.6 机械臂动作

`raicom_arm.py` 保留示教抓放和舵机反馈，并增加“保持夹爪状态的观察动作”。现有 `Arm.go()` 会发送完整姿势，不能在夹货后直接调用包含张开夹爪值的 `look_cargo`。

夹货后的观察接口必须：

- 只改变 1–5 号关节；
- 显式保持夹爪闭合；
- 继续使用现有舵机到位反馈；
- 在动作前后检查取消状态。

### 4.7 ROS 包内结构

继续使用一个 `raicom` ROS 包，不拆成多个包。包内分成不依赖 ROS 的纯算法库和薄 ROS 适配节点：

```text
raicom/
├── src/raicom_core/
│   ├── inventory.py
│   ├── target_tracker.py
│   ├── slot_assignment.py
│   ├── search_policy.py
│   ├── visual_servo.py
│   ├── depth_filter.py
│   └── park_detector.py
├── scripts/
│   ├── raicom_detector_node.py
│   ├── raicom_alignment_node.py
│   ├── raicom_arm_node.py
│   ├── raicom_park_node.py
│   └── raicom_mission_node.py
├── msg/
├── action/
├── srv/
├── config/
└── test/
```

职责边界：

- `raicom_core` 不导入 `rospy`，输入输出使用普通 Python 数据对象，可在未连接小车的电脑上测试；
- `raicom_detector_node` 负责相机消息、TensorRT 和检测结果发布；
- `raicom_alignment_node` 负责主动观察、目标锁定和局部底盘对位；
- `raicom_arm_node` 负责命名姿势、抓取、放置、舵机反馈和保持夹爪状态；
- `raicom_park_node` 负责黄色台面和园区槽位检测；
- `raicom_mission_node` 负责账本、抓取面顺序、导航及各 action 的编排。

### 4.8 ROS 通信类型

按通信语义选用 ROS1 接口：

- topic 用于连续数据：`/raicom/detections`、`/raicom/park_target`、`/raicom/task_state`；
- actionlib 用于耗时且必须支持取消和反馈的任务：`/raicom/align_target`、`/raicom/arm_task`、`/raicom/run_mission`；
- service 只用于立即完成的查询或设置：`/raicom/status`、`/raicom/abort`、`/raicom/save_pose`；
- YAML 和私有参数保存 ROI、aim、速度、超时、搜索位移和观察姿势。

首期接口文件固定为：

```text
msg/Detection2D.msg
msg/DetectionArray.msg
msg/ParkTarget.msg
msg/TaskState.msg
action/AlignTarget.action
action/ArmTask.action
action/RunMission.action
srv/SavePose.srv
```

检测结果采用结构化消息，不再扩展逗号分隔的 `std_msgs/String`。action 必须在循环内部检查抢占，并在取消、异常和超时时发送零速度。现有 `/raicom/start`、`/raicom/start_auto`、`/raicom/status` 和 `/raicom/abort` 保留兼容，由 mission 节点转接到新 action。

现有 `/raicom/save_pose` topic 在迁移期保留订阅，同时新增可返回成功或失败原因的 `SavePose` service；新代码只调用 service。

## 5. 待派送区完整数据流

每个抓取面的处理流程：

1. 导航到该面的安全观察点；
2. 底盘停止；
3. 机械臂依次执行高、中、低观察姿势；
4. 只在该面合法槽位 ROI 内接收候选；
5. 连续至少三帧检测到同一目标后绑定槽位、类别和跟踪 ID；
6. 若仍不可见，底盘左右各进行一次 10–15 cm 的限定搜索；
7. 若仍不可见，槽位记为 `failed` 并停止盲目尝试；
8. 粗对位，速度上限约 0.06–0.08 m/s；
9. 精对位，速度上限约 0.02–0.03 m/s；
10. 图像误差连续稳定 5–8 帧且深度有效后停止底盘；
11. 执行对应层的示教侧夹；
12. 抓后进入安全复查；
13. 验证通过后更新账本并配送，否则重试或停止。

每个“抓取面 × 层级”使用独立的视觉目标：

```text
front_top_aim / front_bottom_aim
left_top_aim  / left_bottom_aim
right_top_aim / right_bottom_aim
```

这些 aim 表示方块处于可执行对应示教抓取姿势时的画面位置。机械臂高度由示教姿势决定，视觉不得根据像素误差自行下探。

## 6. 夹取动作和抓后验证

### 6.1 夹取

动作顺序：

1. 夹爪张开；
2. 机械臂进入对应层的侧夹姿势；
3. 底盘完成最后的低速图像对位；
4. 底盘完全停止；
5. 夹爪闭合；
6. 先向上抬离台面；
7. 再向后收回到安全携带姿势。

下层使用 `pick_down_low`，其高位夹持可避免夹爪下方舵机碰台。上层需新增实车示教姿势，未完成前 `allow_high` 保持 `false`。

### 6.2 抓后复查

不能直接调用现有 `look_cargo`。复查流程：

1. 保持夹爪闭合；
2. 垂直抬高约 4–6 cm；
3. 收回到专门示教的安全复查姿势；
4. 必要时底盘后退约 8–12 cm；
5. 进入与当前抓取面对应的 `verify_front`、`verify_left` 或 `verify_right`；
6. 在该姿势中同时定义“原槽位 ROI”和“夹持货物 ROI”，重新观察原槽位，并检查夹爪前方是否存在已锁定类别的方块。

判定：

- 原框消失且出现新的下层贴纸：上层抓取成功，下层重新分类；
- 原框消失且该摞为空，同时夹持货物 ROI 中连续检测到已锁定类别：该摞清空；
- 原目标仍在原位：抓取失败；
- 画面被手中方块遮挡：再执行一次限定抬高或后退；
- 仍无法确认：标记 `unverified`，不增加已抓数量，不导航去园区。

## 7. 园区主动观察和放置

抓住并验证方块后：

1. 进入安全携带姿势，夹爪强制保持闭合；
2. 导航到目标园区 approach；
3. 底盘停稳；
4. 相机执行 `look_park_high`、`look_park_mid`、`look_park_low`；
5. 检测黄色台面和已放方块；
6. 横移对齐目标放置槽；
7. 根据台面前沿深度前后慢移；
8. 停车并等待振动衰减；
9. 执行对应园区、摞和层的示教放置姿势；
10. 到位后张开夹爪，保持张开并垂直抬离约 4–6 cm；
11. 收回到该园区专用 `verify_park` 姿势，必要时底盘后退一次限定距离；
12. 在目标槽位 ROI 中确认出现方块，并确认夹持货物 ROI 已清空，然后更新账本。

每个园区的槽位顺序：

```text
stack_a.bottom → stack_a.top → stack_b.bottom → stack_b.top
```

只有下层槽位已验证占用，才能选择其上层槽位。首期仅开放 `stack_a.bottom`。

园区检测不到黄色台面、深度无效、相机被手中方块遮挡或局部移动超限时，回到 approach 或原地停止，不按旧 AMCL 作业点盲放。

## 8. 状态和结果语义

外部状态必须区分：

```text
detected
aligned
grasp_motion_complete
grasp_verified
delivered
place_motion_complete
place_verified
failed
unverified
aborted
```

服务返回“线程已启动”不得表示任务成功。机械动作完成也不得表示货物已抓住或放稳。

## 9. 异常处理和安全边界

- 目标丢失或切换：立即发送零速度并重新锁定；
- 深度无效或离散过大：禁止继续前进和抓取；
- 搜索仅允许预定观察姿势和有限横移，不允许无限游走；
- 每次局部运动设置速度、总位移和超时上限；
- 舵机反馈误差超过现有 25 脉冲阈值：中止动作；
- `/raicom/abort` 在搜索、对位、抓取和放置阶段均可抢占；
- 节点异常退出必须停止底盘；
- 夹着方块时发生异常，保持夹爪当前闭合状态，不自动回默认姿势；
- 未验证抓取不得累计数量或进入配送；
- 未验证放置不得占用园区槽位；
- `allow_high` 在高层抓放实车验收前保持 `false`；
- `grasp_mode=3d` 继续保持关闭。

## 10. 测试和开放顺序

### 10.1 离线测试

为以下纯逻辑建立确定性测试：

- 八槽位状态转换；
- 按抓取面清空顺序；
- 下层在上层移走前保持不可用；
- 类别到 P1/P2 映射；
- 抓取失败和 `unverified` 不累计；
- 园区四槽分配不重复；
- `/abort` 和超时转换。

使用实车录制的 RGB-D/rosbag 回放验证：

- 四摞槽位归属；
- 目标跟踪不跳到邻块；
- 无目标、遮挡、误检和深度孔洞；
- 黄色台面边缘和中心；
- 所有异常输入生成零速度。

未连接小车时还应完成：

- `msg`、`action`、`srv` 定义及包依赖声明；
- 五个节点的参数加载、接口连接和 mock 适配层；
- 假检测、假舵机反馈、假 move_base 与假底盘速度接收器；
- action 成功、超时、取消和节点异常路径；
- launch 文件的节点命名、参数和 remap；
- 纯算法测试不依赖 ROS master、TensorRT、相机或 Jetson。

离线阶段不得写入或宣称已经验证：

- 舵机姿势和台面净空；
- 底盘控制方向正负号；
- RGB 与深度实际对齐；
- 黄色台面实车 HSV/深度阈值；
- 携带方块时的相机遮挡；
- 实车速度、制动距离和最终 aim。

### 10.2 分阶段实车测试

1. 只测试高、中、低观察姿势；
2. 只测试底盘视觉对位，不动夹爪；
3. 空夹爪测试抓取轨迹和台面净空；
4. 正面单个下层块复用 `pick_down_low`；
5. 抓后保持夹爪闭合并复查槽位；
6. 携带方块时测试园区主动观察；
7. 单块完整抓取、配送和低层放置；
8. 连续成功至少五次且无碰撞后，才设计和开放高层；
9. 依次扩展正面四块、左侧两块、右侧两块；
10. 最终账本必须恰好八个 `place_verified`，每个待派送槽位和园区槽位只能消费一次。

关键安全验收：

- `/raicom/abort` 后 0.3 秒内底盘速度归零；
- 目标丢失、深度异常或舵机不到位时不继续动作；
- 下层抓取全过程舵机外壳不接触台面；
- 夹货观察时夹爪命令始终闭合；
- 未验证抓取不得进入配送。

## 11. 配置和版本原则

姿势、视觉 aim、ROI、速度、深度范围和搜索位移均写入车上 `~/yolo_models/*.yaml`，电脑备份位于 `tools/04_vision_car/`。运行时代码只修改：

```text
ROS1/ros_ws-main/ros_ws/src/raicom/
```

不得并行维护两份 `raicom_auto.py`。新姿势和路点必须实车示教、验证后才写为有效值；未验证数字只能作为候选，不能提交为最终实车结论。

## 12. 实施拆分

本设计覆盖两个相关但可独立验收的子系统，实施时拆成三个计划：

1. ROS 接口、纯算法库、待派送区主动观察、八槽位账本和单个下层视觉抓取；
2. 园区黄色台面检测、主动观察和单个低层视觉放置；
3. 三抓取面、上层姿势和八块完整编排。

必须先完成计划 1 的单块抓取验收，再进入计划 2；计划 1 和 2 的完整单块闭环连续成功至少五次后，才进入计划 3。
