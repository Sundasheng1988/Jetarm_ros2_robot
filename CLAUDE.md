# CLAUDE.md

> JetArm ROS 2 项目 Agent 工作规则  
> 当前基线：2026-08-02  
> 目标：让 Agent 用最少上下文快速进入当前真实项目状态，同时严格限制硬件风险。

---

## 1. 每次会话先读取

开始任何项目任务前，先读取以下 3 个文件：

```text
docs/runtime_index.md
docs/runtime_debug_guide.md
docs/topic_service_map.md
```

它们分别负责：

```text
runtime_index.md
→ 项目目标、当前阶段、当前 P0、工作顺序和安全边界

runtime_debug_guide.md
→ 当前启动方式、监听命令、Confirm、分级执行和故障排查

topic_service_map.md
→ 节点、Topic、Service、Current / Legacy 和控制路径
```

读取一次并形成摘要即可，不要在同一任务中反复全文读取。

涉及具体代码时，必须继续核对对应源码、launch、YAML、tests 和当前 ROS 2 运行图。文档不是代码事实。

---

## 2. 当前系统基线

### PC

```text
Host role: Development / Intelligence / Runtime
Workspace: /home/sundasheng/ros2_ws
```

主要运行：

```text
YOLO / ROI
Perception Fusion
StableObjectTracker
Parser / Grounding
Runtime / Skill / Action
Verification
RobotOps
RViz / rosbag / offline analysis
Git / documentation / tests
```

### Orin

```text
Host role: Robot-side Hardware Runtime
Workspace: /home/ubuntu/ros2_ws
```

主要运行：

```text
Camera
Kinematics
controller_manager
ServoManager
ros_robot_controller
STM32 / Servo bus
Mobile base
RPLidar
Robot-side TF
start_app_node.service
```

PC 不得启动第二套 Orin 硬件驱动。

### ROS 2

```text
ROS 2 Humble
ROS_DOMAIN_ID=23
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

PC：

```text
CYCLONEDDS_URI=file:///home/sundasheng/ros2_ws/config/cyclonedds/pc_camera_eno1.xml
```

Orin：

```text
CYCLONEDDS_URI=file:///home/ubuntu/ros2_ws/config/cyclonedds/orin_camera_eth0.xml
```

当前固定有线地址：

```text
PC eno1:   192.168.100.2
Orin eth0: 192.168.100.1
```

---

## 3. 当前主线

```text
Camera
→ YOLO / ROI
→ Fusion
→ Stable Tracker
→ Parser
→ Grounding
→ /grounded_task_context
→ real_grounded_runtime_node
→ SkillManager / PickSkill
→ RuntimeAdapter
→ IK / Servo
→ Verification
→ RobotOps
```

Current：

```text
/grounded_task_context
/runtime/preview
/runtime/confirm
/runtime/state
/runtime/log
/runtime/execution_result
/runtime/verification_result
```

Legacy：

```text
/grounded_goal
ground_executor_node
/executor/confirm
/executor/confirm_str
```

不要同时启动 Current Runtime 和 Legacy Executor。

---

## 4. 当前项目状态

已完成：

```text
单一机械臂控制栈
PC / Orin 全 Cyclone DDS
跨机相机 Topic
跨机 Kinematics Service
Dry-run
真实 IK
真实 Servo
首次真实视觉杯子抓取
```

夹爪实机基线：

```text
Servo ID10:
200 = open
700 = close
```

当前重点：

```text
Pick 重复性
Verification 假阳性
ROI / Fusion 稳定性
RobotOps 数据完整性
```

移动底盘当前暂停：

```text
不发布 /cmd_vel
不继续 AMCL / Nav2 参数调试
```

---

## 5. 谁执行什么

### 用户执行

以下命令或操作由用户现场执行：

```text
ros2 launch
长时间运行的 ros2 run
RViz
ros2 bag record
真实机械臂动作
机器人复位
/cmd_vel
systemd start / stop / restart
预计超过 30 秒的现场观察
```

### Agent 可以执行

在当前任务范围内，Agent 可以：

```text
读取指定文件和源码
聚焦源码搜索
编辑用户要求的 PC 文件
语法检查
聚焦单元测试
指定软件包构建
离线分析
git status
聚焦 git diff
只读 ROS 2 graph / process / systemd 调查
```

Jetson 工作区默认只读。未经用户明确批准，不得修改或构建 Jetson 代码。

---

## 6. 修改前要求

编辑任何文件前，先说明：

```text
任务目标
涉及文件
计划修改
验证方法
```

一次只处理一个明确任务。

不得顺带修改无关文件。

---

## 7. 硬件安全规则

未经用户明确批准，Agent 不得：

```text
发布 /cmd_vel
发布 /servo_controller
发布 /grasp
发布 /ros_robot_controller/bus_servo/set_position
触发真实机械臂动作
启动或停止硬件 Bringup
修改 Jetson 运行时代码
修改网络、串口、udev 或 systemd
```

真实执行必须满足：

```text
单一控制栈
require_confirm=true
Stable Object 位姿稳定
无其他主动 /servo_controller 写者
工作区清空
用户现场观察
可立即断电
```

---

## 8. 禁止操作

未经用户批准，不得执行：

```text
sudo
apt / pip install / npm install
curl / wget
rm / rm -rf
chmod / chown
kill / pkill
reboot / shutdown
git reset / clean / restore / rebase / push
全工作区构建
无边界递归搜索
后台 Agent / 并行 Agent / 子 Agent
```

不得清理 Git 状态，不得覆盖用户文件，不得删除历史日志。

---

## 9. 调查与证据规则

所有结论必须区分：

```text
Confirmed Fact
Direct Observation
Inference
Rejected Hypothesis
Unknown
```

Source of Truth 顺序：

```text
当前 ROS 2 graph / process / systemd / journal
→ 当前 install
→ 当前 src / launch / YAML / tests
→ 当前文档
→ 历史日志
```

PC 上看到节点，不代表节点运行在 PC。节点主机必须通过进程、launch 或 systemd 证据确认。

---

## 10. 输出规则

长输出必须写入日志文件。

终端只汇报：

```text
退出码
关键错误
关键结论
最多最后 80 行
```

不要输出完整大文件、完整源码或大段日志。

预计超过 2 分钟的命令，先说明预计耗时。  
预计超过 5 分钟的命令，必须先获得批准。

---

## 11. 构建与测试

只构建用户指定的软件包：

```bash
colcon build --packages-select <package_name>
```

构建输出写入日志。

允许：

```text
python3 -m py_compile <file>
pytest <specific_test>
git diff -- <specific_file>
```

不默认构建整个工作区。

---

## 12. 任务完成格式

完成任务后汇报：

```text
读取了什么
修改了什么
执行了什么验证
退出码和关键结果
未完成或 Unknown
```

然后停止，不自动开始下一项任务。
