import zipfile, os
from xml.sax.saxutils import escape
OUTPUT = r"D:\Axinbancar\JetRover三合一ROS智能车\3 源码资料\ROS2\ros2_ws-main\ros2_ws\JetRover机械臂手柄控制交接报告.docx"
content = [
("h1","JetRover 机械臂手柄控制 — 交接报告",0),
("h2","一、搞清楚了什么事",0),
("h3","1.1 系统整体架构",0),
("p","JetRover三合一ROS智能车采用Jetson Nano + STM32控制板的双层架构：",0),
("bullet","Jetson Nano运行ROS2 Humble，负责高层决策、手柄逻辑、运动学计算",0),
("bullet","STM32控制板负责底层硬件：2.4G手柄接收、总线舵机驱动、电机驱动、IMU采集",0),
("bullet","两者通过串口/dev/rrc通信，波特率1000000（1Mbps）",0),
("h3","1.2 手柄数据链路（输入方向）",0),
("p","手柄(2.4G无线) → STM32控制板(2.4G接收器) → 串口/dev/rrc(1Mbps) → ros_robot_controller节点(Board类解析串口帧) → 发布话题ros_robot_controller/joy(sensor_msgs/Joy, 100Hz) → joystick_control节点订阅",0),
("p","关键事实：",0),
("bullet","串口帧格式：0xAA 0x55 Length Function Data CRC8",0),
("bullet","手柄功能码：0x08（PACKET_FUNC_GAMEPAD）",0),
("bullet","手柄数据：struct <HB4b = buttons_mask(uint16) + hat(uint8) + lx/ly/rx/ry(int8)",0),
("bullet","STM32只做透传，不需要改固件",0),
("h3","1.3 机械臂控制链路（输出方向）",0),
("p","joystick_control节点 → 发布ServosPosition到话题servo_controller → controller_manager节点订阅 → servo_manager.set_position() → 发布到ros_robot_controller/bus_servo/set_position → ros_robot_controller节点订阅 → Board.bus_servo_set_position()发串口指令 → STM32驱动总线舵机",0),
("h3","1.4 舵机配置",0),
("table","关节|舵机ID|中位脉宽|脉宽范围|说明\njoint1|1|500|0~1000|底座旋转\njoint2|2|500|0~1000|大臂俯仰\njoint3|3|500|0~1000|小臂俯仰\njoint4|4|500|0~1000|腕部俯仰\njoint5|5|500|0~1000|腕部旋转(夹爪旋转)\nr_joint|10|700|0~1000|夹爪夹放",0),
("p","配置来源：servo_controller/config/servo_controller.yaml",0),
("h3","1.5 动作组系统",0),
("bullet","文件格式：.d6a = SQLite数据库，表名ActionGroup",0),
("bullet","每行一帧：[序号, 持续时间ms, 舵机1位置, ..., 舵机6位置]",0),
("bullet","舵机映射：第1~5列→ID 1~5，第6列→ID 10（夹爪）",0),
("bullet","播放类：ActionGroupController（servo_controller/action_group_controller.py）",0),
("bullet","默认路径：/home/ubuntu/share/arm_pc/ActionGroups",0),
("bullet","已知动作组：init（回中位）、camera_up、voice_pick、voice_give",0),
("h3","1.6 自启服务",0),
("p","开机自动启动APP自启服务，包含三个核心节点：",0),
("bullet","ros_robot_controller（和STM32通信）",0),
("bullet","controller_manager（舵机控制器，通过controller.launch.py启动）",0),
("bullet","joystick_control（手柄控制，use_joy默认true，通过robot.launch.py启动）",0),
("p","结论：开机后不需要手动启动任何节点，改完代码重新编译+重启自启服务即可。",0),
("h2","二、项目需要注意什么",0),
("h3","2.1 编译与部署",0),
("table","注意项|说明\n必须重新编译|自启服务运行的是install/下编译后的版本，改源码后必须colcon build --packages-select peripherals\n必须重启自启服务|编译完不重启，跑的还是旧代码。可sudo reboot或重启对应systemd服务\n源码路径|/home/ubuntu/ros2_ws/src/peripherals/",0),
("h3","2.2 手柄相关",0),
("table","注意项|说明\nL2/R2是按钮式|不是线性扳机，按下时buttons[8]/[9]=1，同时axes[4]/[5]=1.0，按住持续动，松开即停\n死区0.1|摇杆值小于0.1视为0，防止漂移\n方向键hat值|9=左, 13=右, 11=下, 15=上，在axes[6]/[7]输出±1.0",0),
("h3","2.3 机械臂相关",0),
("table","注意项|说明\n初始位置同步|代码启动时假设舵机在中位500，建议先按SELECT运行init动作组校准\n动作组播放互斥|播放init动作组期间，手动按钮输入被忽略（action_running标志），播完自动恢复\n关节增量速度|每帧增量12脉宽，100Hz≈1200脉宽/秒，从0到1000约0.8秒\n夹爪增量|每帧10脉宽，比臂部关节稍慢",0),
("h3","2.4 底盘相关",0),
("table","注意项|说明\n自启速度受限|通过robot.launch.py启动时max_linear=0.2, max_angular=0.5，比手动启动(0.5/2.0)慢\n车型环境变量|MACHINE_TYPE必须设置为JetRover_Mecanum/JetRover_Tank/JetRover_Acker，影响底盘控制逻辑\n底盘和机械臂独立|推摇杆走路的同时可以按按钮操作机械臂，互不干扰",0),
("h3","2.5 动作组路径",0),
("p","代码默认路径是/home/ubuntu/share/arm_pc/ActionGroups，但用户提到出厂路径是hiwonder/software/arm_pc/ActionGroups，两者可能不一致。如果init动作组找不到，需要用action_path参数修改：",0),
("p","ros2 run peripherals joystick_control --ros-args -p action_path:=/实际/路径/ActionGroups",0),
("h2","三、做了哪些改动",0),
("h3","3.1 改动文件清单",0),
("table","#|文件|改动类型\n1|src/peripherals/peripherals/joystick_control.py|核心重写\n2|src/peripherals/package.xml|添加依赖\n3|src/peripherals/launch/joystick_control.launch.py|参数修改",0),
("h3","3.2 joystick_control.py 详细改动",0),
("p","新增内容：",0),
("bullet","导入ServosPosition、ServoPosition、ActionGroupController、threading、time",0),
("bullet","舵机配置常量：ARM_JOINTS（joint1~5→ID 1~5）、GRIPPER_ID=10、HOME_POSITION",0),
("bullet","参数：action_path（动作组路径，默认/home/ubuntu/share/arm_pc/ActionGroups）",0),
("bullet","发布者：servo_pub → servo_controller话题",0),
("bullet","动作组控制器：ActionGroupController实例 + action_running互斥标志",0),
("bullet","工具方法：clamp()、publish_arm()、arm_step()、run_action_thread()、stop_and_reset()",0),
("p","移除内容：",0),
("bullet","mode模式切换变量（底盘和机械臂同时工作，不需要切换）",0),
("bullet","所有空的pass回调被替换为实际控制逻辑",0),
("h3","3.3 按键映射（最终版）",0),
("table","按键|功能\n左摇杆LY/LX|底盘前后/左右平移\n右摇杆RX|底盘转向\n方向键↑/↓|joint2 大臂俯仰 ±\n方向键←/→|joint1 底座旋转 ±\n△ / ×|joint3 小臂俯仰 ±\n□ / ○|joint4 腕部俯仰 ±\nL1 / R1|joint5 夹爪旋转 ±\nL2 / R2|夹爪夹放 ± (ID 10)\nSELECT|运行init动作组回中位\nSTART|停止底盘 + 停止动作组 + 回中位（点按触发）",0),
("h3","3.4 START键逻辑",0),
("p","START键点按一次执行以下流程（独立线程，不阻塞手柄消息）：",0),
("bullet","1. 停止底盘（发布零速度Twist）",0),
("bullet","2. 如果动作组在播放，停止它（等最多0.5秒）",0),
("bullet","3. 运行init动作组，机械臂回中位",0),
("bullet","4. 蜂鸣器响一声",0),
("p","点按触发：只在ButtonState.Pressed时执行一次，按住不放不会重复触发。",0),
("h3","3.5 package.xml",0),
("p","新增9个exec_depend：rclpy、geometry_msgs、sensor_msgs、std_msgs、std_srvs、servo_controller、servo_controller_msgs、ros_robot_controller_msgs",0),
("h3","3.6 launch文件",0),
("p","disable_servo_control从True改为False",0),
("h2","四、还有什么问题（待验证/待解决）",0),
("h3","4.1 未在实车验证",0),
("table","问题|风险|验证方法\nSTM32固件是否支持手柄透传|高|ros2 topic echo /ros_robot_controller/joy，按手柄看有没有数据\nSTM32固件是否支持总线舵机|高|发布ServosPosition消息看舵动不动\nL2/R2是否有按钮值|中|ros2 topic echo /ros_robot_controller/joy，按L2/R2看buttons[8]/[9]\n动作组init.d6a是否存在|中|检查/home/ubuntu/share/arm_pc/ActionGroups/init.d6a\n动作组路径是否正确|中|按SELECT看日志，报未能找到动作组文件就是路径错了",0),
("h3","4.2 方向可能需要微调",0),
("p","代码中假设的方向可能和实际机械结构相反，需要实车测试后调整：",0),
("table","可能反了的地方|修改位置\n方向键↑变成大臂下压|hat_yu_callback和hat_yd_callback的+/-对调\nL2变成张开、R2变成闭合|l2_callback和r2_callback的+/-对调\nL1/R1旋转方向反了|l1_callback和r1_callback的+/-对调",0),
("h3","4.3 待确认事项",0),
("table","事项|说明\n自启服务名称|重启自启服务需要知道systemd服务名（可能叫jetrover-app或类似），目前未知\njoint5与夹爪旋转的关系|假设joint5（ID 5）就是夹爪旋转。如果车型是JetRover_Acker有独立的w_joint（ID 9），需要把ARM_JOINTS[joint5]从5改成9\n动作组路径差异|用户提到出厂路径hiwonder/software/arm_pc/ActionGroups，代码默认/home/ubuntu/share/arm_pc/ActionGroups，需确认实际路径",0),
("h3","4.4 后续可扩展",0),
("bullet","长按SELECT运行init，短按SELECT切换底盘/机械臂模式（目前是同时工作）",0),
("bullet","START键触发其他预设动作组（如camera_up、voice_pick）",0),
("bullet","订阅/controller_manager/servo_states同步真实舵机位置，而不是启动时假设中位",0),
("bullet","笛卡尔空间控制模式（摇杆控制末端XYZ，调用逆运动学服务）",0),
("h2","五、快速操作手册",0),
("h3","5.1 部署步骤",0),
("bullet","1. 把三个文件传到小车对应位置",0),
("bullet","2. 编译：cd ~/ros2_ws && colcon build --packages-select peripherals && source install/setup.bash",0),
("bullet","3. 重启自启服务（或重启机器人）：sudo reboot",0),
("bullet","4. 验证节点：ros2 node list | grep -E joystick|controller|ros_robot",0),
("bullet","5. 按START停止并复位，然后测试各按键",0),
("h3","5.2 按键速查",0),
("table","按键|功能\n左摇杆|底盘前后/左右\n右摇杆|底盘转向\n方向键↑↓|大臂俯仰\n方向键←→|底座旋转\n△ ×|小臂俯仰\n□ ○|腕部俯仰\nL1 R1|夹爪旋转\nL2 R2|夹爪夹放\nSELECT|回中位（init动作组）\nSTART|停止并复位机身",0),
("p","",0),
("p","报告结束。核心结论：手柄→STM32→Jetson的透传链路不需要改STM32，机械臂控制逻辑全在Jetson的joystick_control.py里，已完成代码修改，待实车验证方向和动作组路径。",0),
]
def make_paragraph(text, style="Normal"):
    text = escape(text)
    ppr = ""
    if style == "Heading1": ppr = '<w:pPr><w:pStyle w:val="Heading1"/></w:pPr>'
    elif style == "Heading2": ppr = '<w:pPr><w:pStyle w:val="Heading2"/></w:pPr>'
    elif style == "Heading3": ppr = '<w:pPr><w:pStyle w:val="Heading3"/></w:pPr>'
    elif style == "Bullet": ppr = '<w:pPr><w:pStyle w:val="ListParagraph"/><w:numPr><w:ilvl w:val="0"/><w:numId w:val="1"/></w:numPr></w:pPr>'
    return f'<w:p>{ppr}<w:r><w:t xml:space="preserve">{text}</w:t></w:r></w:p>'
def make_table(rows):
    tbl_pr = '<w:tblPr><w:tblW w:w="5000" w:type="pct"/><w:tblBorders><w:top w:val="single" w:sz="4" w:space="0" w:color="000000"/><w:left w:val="single" w:sz="4" w:space="0" w:color="000000"/><w:bottom w:val="single" w:sz="4" w:space="0" w:color="000000"/><w:right w:val="single" w:sz="4" w:space="0" w:color="000000"/><w:insideH w:val="single" w:sz="4" w:space="0" w:color="000000"/><w:insideV w:val="single" w:sz="4" w:space="0" w:color="000000"/></w:tblBorders></w:tblPr>'
    grid_cols = len(rows[0].split("|"))
    grid = "<w:tblGrid>" + "".join(['<w:gridCol w:w="{}"/>'.format(9000//grid_cols) for _ in range(grid_cols)]) + "</w:tblGrid>"
    trs = ""
    for i, row in enumerate(rows):
        cells = row.split("|")
        tr = "<w:tr>"
        for cell in cells:
            cell_text = escape(cell.strip())
            if i == 0:
                tc_pr = '<w:tcPr><w:shd w:val="clear" w:color="auto" w:fill="D9E2F3"/></w:tcPr>'
                p = f'<w:p><w:r><w:rPr><w:b/></w:rPr><w:t xml:space="preserve">{cell_text}</w:t></w:r></w:p>'
            else:
                tc_pr = ""
                p = f'<w:p><w:r><w:t xml:space="preserve">{cell_text}</w:t></w:r></w:p>'
            tr += f"<w:tc>{tc_pr}{p}</w:tc>"
        tr += "</w:tr>"
        trs += tr
    return f"<w:tbl>{tbl_pr}{grid}{trs}</w:tbl>"
body = ""
for item in content:
    typ, text, _ = item
    if typ == "h1": body += make_paragraph(text, "Heading1")
    elif typ == "h2": body += make_paragraph(text, "Heading2")
    elif typ == "h3": body += make_paragraph(text, "Heading3")
    elif typ == "p":
        if text: body += make_paragraph(text, "Normal")
        else: body += '<w:p/>'
    elif typ == "bullet": body += make_paragraph(text, "Bullet")
    elif typ == "table":
        rows = text.split("\n")
        body += make_table(rows)
        body += '<w:p/>'
styles_xml = '<?xml version="1.0" encoding="UTF-8" standalone="yes"?><w:styles xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main"><w:docDefaults><w:rPrDefault><w:rPr><w:rFonts w:ascii="Times New Roman" w:eastAsia="宋体" w:hAnsi="Times New Roman"/><w:sz w:val="24"/><w:szCs w:val="24"/></w:rPr></w:rPrDefault><w:pPrDefault><w:pPr><w:spacing w:line="360" w:lineRule="auto"/></w:pPr></w:pPrDefault></w:docDefaults><w:style w:type="paragraph" w:default="1" w:styleId="Normal"><w:name w:val="Normal"/><w:qFormat/></w:style><w:style w:type="paragraph" w:styleId="Heading1"><w:name w:val="heading 1"/><w:basedOn w:val="Normal"/><w:next w:val="Normal"/><w:qFormat/><w:pPr><w:spacing w:before="240" w:after="120"/><w:jc w:val="center"/></w:pPr><w:rPr><w:rFonts w:eastAsia="黑体"/><w:b/><w:sz w:val="32"/><w:szCs w:val="32"/></w:rPr></w:style><w:style w:type="paragraph" w:styleId="Heading2"><w:name w:val="heading 2"/><w:basedOn w:val="Normal"/><w:next w:val="Normal"/><w:qFormat/><w:pPr><w:spacing w:before="200" w:after="100"/></w:pPr><w:rPr><w:rFonts w:eastAsia="黑体"/><w:b/><w:sz w:val="28"/><w:szCs w:val="28"/></w:rPr></w:style><w:style w:type="paragraph" w:styleId="Heading3"><w:name w:val="heading 3"/><w:basedOn w:val="Normal"/><w:next w:val="Normal"/><w:qFormat/><w:pPr><w:spacing w:before="160" w:after="80"/></w:pPr><w:rPr><w:rFonts w:eastAsia="黑体"/><w:b/><w:sz w:val="26"/><w:szCs w:val="26"/></w:rPr></w:style><w:style w:type="paragraph" w:styleId="ListParagraph"><w:name w:val="List Paragraph"/><w:basedOn w:val="Normal"/><w:qFormat/><w:pPr><w:ind w:left="720" w:hanging="360"/></w:pPr></w:style></w:styles>'
numbering_xml = '<?xml version="1.0" encoding="UTF-8" standalone="yes"?><w:numbering xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main"><w:abstractNum w:abstractNumId="0"><w:lvl w:ilvl="0"><w:start w:val="1"/><w:numFmt w:val="bullet"/><w:lvlText w:val="•"/><w:lvlJc w:val="left"/><w:pPr><w:ind w:left="720" w:hanging="360"/></w:pPr></w:lvl></w:abstractNum><w:num w:numId="1"><w:abstractNumId w:val="0"/></w:num></w:numbering>'
document_xml = f'<?xml version="1.0" encoding="UTF-8" standalone="yes"?><w:document xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main"><w:body>{body}<w:sectPr><w:pgSz w:w="11906" w:h="16838"/><w:pgMar w:top="1440" w:right="1440" w:bottom="1440" w:left="1440" w:header="708" w:footer="708" w:gutter="0"/></w:sectPr></w:body></w:document>'
content_types = '<?xml version="1.0" encoding="UTF-8" standalone="yes"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/><Default Extension="xml" ContentType="application/xml"/><Override PartName="/word/document.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.document.main+xml"/><Override PartName="/word/styles.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.styles+xml"/><Override PartName="/word/numbering.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.numbering+xml"/></Types>'
rels = '<?xml version="1.0" encoding="UTF-8" standalone="yes"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="word/document.xml"/></Relationships>'
document_rels = '<?xml version="1.0" encoding="UTF-8" standalone="yes"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles" Target="styles.xml"/><Relationship Id="rId2" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/numbering" Target="numbering.xml"/></Relationships>'
with zipfile.ZipFile(OUTPUT, 'w', zipfile.ZIP_DEFLATED) as zf:
    zf.writestr('[Content_Types].xml', content_types)
    zf.writestr('_rels/.rels', rels)
    zf.writestr('word/document.xml', document_xml)
    zf.writestr('word/_rels/document.xml.rels', document_rels)
    zf.writestr('word/styles.xml', styles_xml)
    zf.writestr('word/numbering.xml', numbering_xml)
print(f"OK: {OUTPUT}")
print(f"Size: {os.path.getsize(OUTPUT)} bytes")
