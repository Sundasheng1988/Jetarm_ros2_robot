# Robot Runtime — Operations & Debug Guide

> **项目**：JetArm ROS 2 Robot  
> **文档职责**：启动、运行、监听、确认、验收与故障隔离  
> **当前基线日期**：2026-08-02  
> **当前通信基线**：PC 与 Orin 新启动的 ROS 2 进程统一使用 Cyclone DDS，`ROS_DOMAIN_ID=23`  
> **当前主线状态**：已完成一次真实视觉杯子抓取  
> **注意**：Topic / Service 的完整发布订阅关系见 `docs/topic_service_map.md`

---

# 1. 当前系统架构

## 1.1 机器职责

### Orin Nano

负责：

- Gemini 深度相机
- Kinematics
- `controller_manager`
- `ServoManager`
- `ros_robot_controller`
- STM32 与舵机硬件控制
- `/depth_cam/*`
- `/kinematics/*`
- `/servo_controller` 的执行下游
- `/controller_manager/joint_states`

Orin 主栈通过 systemd 自动启动：

```bash
systemctl status start_app_node.service
```

正常状态：

```text
active (running)
```

启动链：

```text
start_app_node.service
→ source /home/ubuntu/.zshrc
→ ros2 launch bringup bringup.launch.py
```

### PC

负责：

- YOLO
- ROI
- Perception Fusion
- Stable Object Tracker
- 自然语言 Parser
- Grounding
- Runtime
- Verification
- RobotOps
- 调试、日志和 SQLite 查询

---

## 1.2 DDS / RMW 基线

### Orin

```bash
export ROS_DOMAIN_ID=23
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/ubuntu/ros2_ws/config/cyclonedds/orin_camera_eth0.xml
```

### PC

```bash
export ROS_DOMAIN_ID=23
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/sundasheng/ros2_ws/config/cyclonedds/pc_camera_eno1.xml
```

PC 新终端检查：

```bash
printenv ROS_DOMAIN_ID
printenv RMW_IMPLEMENTATION
printenv CYCLONEDDS_URI
```

预期：

```text
23
rmw_cyclonedds_cpp
file:///home/sundasheng/ros2_ws/config/cyclonedds/pc_camera_eno1.xml
```

调试时建议：

```bash
export ROS2CLI_DISABLE_DAEMON=1
```

> 修改 RMW 环境后，已经运行的 ROS 2 进程不会动态切换，必须重启进程。

---

# 2. 当前正式执行链

```text
Orin Gemini Camera
    ↓
/depth_cam/rgb/image_raw
    ├─→ simple_yolo_node
    │      ↓
    │   /world_model/objects
    │
    └─→ roi_color_detector_node
           ↓
        /world_model/roi_objects

YOLO + ROI
    ↓
perception_fusion_node
    ↓
/world_model/perception_objects
    ↓
stable_object_tracker_node
    ↓
/world_model/stable_objects
    ↓
grounding_node

自然语言
    ↓
llm_command_parser_node
    ↓
/parsed_command
    ↓
grounding_node
    ↓
/grounded_task_context
    ↓
real_grounded_runtime_node
    ↓
Preview / Confirm
    ↓
SkillManager + PickSkill
    ↓
RuntimeAdapter
    ├─→ /kinematics/set_pose_target
    └─→ /servo_controller
             ↓
       controller_manager
             ↓
         ServoManager
             ↓
       ros_robot_controller
             ↓
            STM32
             ↓
        Physical Robot Arm

/executor/done
    ↓
verification_result_node
    ↓
/runtime/verification_result

/runtime/*
    ↓
robotops_recorder_node
    ↓
SQLite
```

---

# 3. 启动前检查

## 3.1 Orin 主栈

在 Orin 执行：

```bash
systemctl is-active start_app_node.service
```

预期：

```text
active
```

检查关键 Service：

```bash
export ROS2CLI_DISABLE_DAEMON=1

ros2 service list | grep -E \
'/kinematics/get_current_pose|/kinematics/set_pose_target'
```

检查关节状态：

```bash
timeout 10 ros2 topic echo \
  /controller_manager/joint_states \
  --once
```

## 3.2 PC 环境

```bash
cd ~/ros2_ws

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

export ROS2CLI_DISABLE_DAEMON=1
```

检查包：

```bash
ros2 pkg list | grep -E \
'vision_yolo|app|grounding|sketch_runtime|robotops'
```

## 3.3 检查重复节点

在启动 PC 主线前执行：

```bash
ros2 node list | sort | grep -E \
'simple_yolo|roi_color|perception_fusion|stable_object|llm_command_parser|grounding|real_grounded_runtime|verification_result|robotops'
```

检查重复名称：

```bash
ros2 node list | sort | uniq -d
```

进程检查：

```bash
pgrep -af \
'simple_yolo_node|roi_color_detector_node|perception_fusion_node|stable_object_tracker_node|llm_command_parser_node|grounding_node|real_grounded_runtime_node|verification_result_node|robotops'
```

> 如果已经存在相同节点，先停止旧进程，不要重复启动。

---

# 4. 正式启动方式

当前推荐按 3 个常驻终端启动：

```text
Terminal 1：Perception
Terminal 2：Runtime
Terminal 3：RobotOps
```

自然语言输入、Confirm 和 Topic 监听可在其他临时终端中执行。

---

## 4.1 Terminal 1 — Perception Bringup

推荐：

```bash
cd ~/ros2_ws
./scripts/start_cyclone_perception.sh
```

当前该脚本负责启动：

```text
simple_yolo_node
roi_color_detector_node
perception_fusion_node
stable_object_tracker_node
```

等效核心 Launch：

```bash
ros2 launch app perception_bringup.launch.py \
  start_grounding:=false
```

当前感知 Bringup **不启动**：

```text
grounding_node
real_grounded_runtime_node
verification_result_node
robotops_recorder_node
Orin camera driver
```

预期节点：

```bash
ros2 node list | sort | grep -E \
'simple_yolo|roi_color|perception_fusion|stable_object'
```

预期 Topic：

```bash
ros2 topic list | sort | grep -E \
'/vision_target|/roi_vision_target|/world_model/objects|/world_model/roi_objects|/world_model/perception_objects|/world_model/stable_objects'
```

---

## 4.2 Terminal 2 — Runtime Bringup

### 模式 A：Dry-run

用于验证：

```text
真实相机
→ 真实感知
→ 真实 Parser
→ 真实 Grounding
→ Runtime 状态机
→ 模拟 IK
→ 模拟 Servo
```

命令：

```bash
cd ~/ros2_ws

ros2 launch sketch_runtime ground_runtime_bringup.launch.py \
  use_dummy_wm:=false \
  dry_run:=true \
  enable_real_ik:=false \
  enable_real_servo:=false \
  require_confirm:=true \
  run_once:=true
```

预期启动节点：

```text
llm_command_parser_node
grounding_node
real_grounded_runtime_node
verification_result_node
```

预期 Runtime 日志：

```text
DRY_RUN
ik=mock
servo=mock
CONFIRM_REQUIRED
Registered skills: ['pick_skill']
```

---

### 模式 B：真实 IK、Servo 关闭

用于验证：

```text
真实视觉目标
→ 真实调用 /kinematics/set_pose_target
→ 取得真实五轴脉冲
→ 不发布 /servo_controller
→ 机械臂不运动
```

命令：

```bash
cd ~/ros2_ws

ros2 launch sketch_runtime ground_runtime_bringup.launch.py \
  use_dummy_wm:=false \
  dry_run:=false \
  enable_real_ik:=true \
  enable_real_servo:=false \
  require_confirm:=true \
  run_once:=true
```

预期：

```text
adapter mode = LIVE: ik=REAL servo=OFF
```

> 此模式下日志会打印 `servo_move` 与 `gripper_set`，但因为 `servo=OFF`，机械臂不会运动。

---

### 模式 C：完整真实执行

用于：

```text
真实视觉
→ 真实自然语言
→ 真实 Grounding
→ 人工确认
→ 真实 IK
→ 真实 Servo
→ 完整 Pick
```

命令：

```bash
cd ~/ros2_ws

ros2 launch sketch_runtime ground_runtime_bringup.launch.py \
  use_dummy_wm:=false \
  dry_run:=false \
  enable_real_ik:=true \
  enable_real_servo:=true \
  require_confirm:=true \
  run_once:=true
```

预期：

```text
adapter mode = LIVE: ik=REAL servo=REAL
```

当前 `PickSkill` 真实动作序列：

```text
1. Hover
2. 打开夹爪
3. Approach
4. 关闭夹爪
5. Lift
```

已实机确认夹爪：

```text
ID10 pulse=200：打开
ID10 pulse=700：关闭
```

> 此模式会让机械臂真实运动。执行前必须清空工作区域，并保持可立即断电。

---

## 4.3 Terminal 3 — RobotOps

```bash
cd ~/ros2_ws
ros2 launch robotops robotops_recorder.launch.py
```

RobotOps 订阅：

```text
/runtime/state
/runtime/log
/runtime/execution_result
/runtime/verification_result
```

查找数据库：

```bash
find ~/ros2_ws ~/.ros \
  -maxdepth 3 \
  -name 'robotops.db' \
  -printf '%TY-%Tm-%Td %TH:%TM:%TS %p\n' \
  2>/dev/null
```

打开数据库：

```bash
sqlite3 /实际路径/robotops.db
```

SQLite 常用命令：

```sql
.tables
.schema
SELECT COUNT(*) FROM events;
SELECT * FROM events ORDER BY id DESC LIMIT 20;
.quit
```

---

# 5. 自然语言输入

## 5.1 推荐入口

```bash
ros2 topic pub --once \
  /voice_input/input \
  std_msgs/msg/String \
  "{data: '拿起杯子'}"
```

链路：

```text
/voice_input/input
→ llm_command_parser_node
→ /parsed_command
→ grounding_node
→ /grounded_task_context
→ real_grounded_runtime_node
```

首次测试建议使用：

```text
拿起杯子
```

因为 ROI 未提供稳定颜色时，Stable Object 可能输出：

```text
class_name=cup
color=unknown
```

此时：

```text
拿起蓝色杯子
```

可能因为颜色约束而 `no_match`。

---

## 5.2 结构化输入

绕过 Parser，仅测试 Grounding 和 Runtime：

```bash
ros2 topic pub --once \
  /parsed_command \
  std_msgs/msg/String \
  'data: "{\"action\":\"pick\",\"from\":\"cup\",\"to\":\"right_side\",\"raw\":\"拿起杯子\"}"'
```

该方式不验证：

```text
自然语言 → Parser
```

---

## 5.3 键盘节点

旧 `keyboard_input_node` 发布：

```text
/text_input
```

Parser 订阅：

```text
/keyboard_input/input
/voice_input/input
```

需要 remap 才可直接使用：

```bash
ros2 run keyboard_input keyboard_input_node \
  --ros-args \
  -r /text_input:=/keyboard_input/input
```

由于 Grounding 也可能订阅 `/keyboard_input/input` 作为 raw-text fallback，该入口存在双路径风险。主线测试优先使用 `/voice_input/input`。

---

# 6. Preview 与 Confirm

## 6.1 监听 Preview

```bash
ros2 topic echo /runtime/preview
```

Runtime 接收有效任务后应输出：

```text
task_id=...
skill=pick_skill
status=waiting_confirm
```

Runtime 终端日志应出现：

```text
[state] grounded
[state] skill_selected
[state] waiting_confirm
[confirm] waiting for /runtime/confirm
```

## 6.2 简单确认

```bash
ros2 topic pub --once \
  /runtime/confirm \
  std_msgs/msg/String \
  'data: "yes"'
```

支持的确认词通常包括：

```text
yes
y
true
ok
confirm
go
```

## 6.3 取消

```bash
ros2 topic pub --once \
  /runtime/confirm \
  std_msgs/msg/String \
  'data: "cancel"'
```

## 6.4 按 task_id 精确确认

```bash
ros2 topic pub --once \
  /runtime/confirm \
  std_msgs/msg/String \
  'data: "{\"task_id\":\"<TASK_ID>\",\"confirm\":true}"'
```

取消：

```bash
ros2 topic pub --once \
  /runtime/confirm \
  std_msgs/msg/String \
  'data: "{\"task_id\":\"<TASK_ID>\",\"confirm\":false}"'
```

## 6.5 Confirm 超时

默认：

```text
confirm_timeout_sec=300
```

即 5 分钟。

超时后再发送 `yes` 不会执行，因为 pending task 已被取消。

检查订阅：

```bash
ros2 topic info /runtime/confirm --verbose
```

预期：

```text
Subscription count: 1
Node name: real_grounded_runtime_node
```

> `/executor/confirm` 和 `/executor/confirm_str` 属于 Legacy `ground_executor_node`。当前主线不要使用。

---

# 7. 感知四层输出检查

## 7.1 YOLO

世界模型输出：

```bash
timeout 15 ros2 topic echo \
  /world_model/objects \
  --once \
  --full-length
```

原始 DetectionResult：

```bash
timeout 15 ros2 topic echo \
  /vision_target \
  --once \
  --full-length
```

重点：

```text
class_name
confidence
pose.xyz
center_z
```

---

## 7.2 ROI

```bash
timeout 15 ros2 topic echo \
  /world_model/roi_objects \
  --once \
  --full-length
```

原始 DetectionResult：

```bash
timeout 15 ros2 topic echo \
  /roi_vision_target \
  --once \
  --full-length
```

重点：

```text
class_name
color
pose.xyz
pose.rpy
confidence
```

ROI 可能 15 秒内没有输出，此时命令只有 Cyclone DDS 提示，没有消息内容。

---

## 7.3 Fusion

```bash
timeout 15 ros2 topic echo \
  /world_model/perception_objects \
  --once \
  --full-length
```

重点：

```text
class_name
color
pose.xyz
pose.rpy
source
yolo_class
roi_class
match_distance
```

可能的 `source`：

```text
yolo_only
roi_only
yolo_roi_fused
```

---

## 7.4 Stable Tracker

```bash
timeout 15 ros2 topic echo \
  /world_model/stable_objects \
  --once \
  --full-length
```

重点：

```text
track_id
class_name
color
pose.xyz
pose.rpy
confidence_smooth
frames_tracked
class_votes
color_votes
source_votes
```

成功抓取前的稳定输出示例：

```json
{
  "track_id": "track_002",
  "class_name": "cup",
  "color": "unknown",
  "pose": {
    "frame": "base",
    "xyz": [0.237, -0.007, 0.041],
    "rpy": [0.0, 0.0, 1.57]
  },
  "confidence": 0.959,
  "source": "stable",
  "source_votes": {
    "yolo_only": 20
  }
}
```

---

## 7.5 并排持续观察

终端 A：

```bash
ros2 topic echo /world_model/objects
```

终端 B：

```bash
ros2 topic echo /world_model/roi_objects
```

终端 C：

```bash
ros2 topic echo /world_model/perception_objects
```

终端 D：

```bash
ros2 topic echo /world_model/stable_objects
```

按：

```text
Ctrl+C
```

停止。

---

# 8. Grounding 与 Runtime 监听

Parser：

```bash
ros2 topic echo /parsed_command
```

Grounding：

```bash
ros2 topic echo /grounded_task_context
```

Preview：

```bash
ros2 topic echo /runtime/preview
```

状态：

```bash
ros2 topic echo /runtime/state
```

事件流：

```bash
ros2 topic echo /runtime/log
```

执行结果：

```bash
ros2 topic echo /runtime/execution_result
```

Verification：

```bash
ros2 topic echo /runtime/verification_result
```

执行完成：

```bash
ros2 topic echo /executor/done
```

---

# 9. Topic、节点与 Service 快速检查

## 9.1 节点

```bash
ros2 node list | sort
```

主线节点：

```bash
ros2 node list | sort | grep -E \
'simple_yolo|roi_color|perception_fusion|stable_object|llm_command_parser|grounding|real_grounded_runtime|verification_result|robotops'
```

## 9.2 Topic

```bash
ros2 topic list | sort
```

主线 Topic：

```bash
ros2 topic list | sort | grep -E \
'/world_model|/vision_target|/parsed_command|/grounded_task_context|/runtime|/executor/done|/servo_controller'
```

Topic 详细关系：

```bash
ros2 topic info /world_model/stable_objects --verbose
```

```bash
ros2 topic info /grounded_task_context --verbose
```

```bash
ros2 topic info /runtime/confirm --verbose
```

## 9.3 Service

```bash
ros2 service list | sort
```

关键 Service：

```bash
ros2 service list | grep -E \
'kinematics|grounding|init_finish|bus_servo'
```

类型：

```bash
ros2 service type /kinematics/get_current_pose
ros2 service type /kinematics/set_pose_target
```

只读获取当前机械臂位姿：

```bash
ros2 service call \
  /kinematics/get_current_pose \
  kinematics_msgs/srv/GetRobotPose \
  "{}"
```

## 9.4 相机频率

```bash
timeout 15 ros2 topic hz /depth_cam/rgb/image_raw
```

正常约：

```text
30 Hz
```

## 9.5 查看图像

PC 默认已使用 Cyclone DDS，可直接运行：

```bash
ros2 run rqt_image_view rqt_image_view
```

---

# 10. 当前真实抓取验收记录

## 10.1 目标

```text
对象：cup
Stable Object：
  class_name = cup
  color = unknown
  frame = base
  xyz ≈ [0.236, -0.007, 0.041]
```

## 10.2 真实 IK

Hover：

```text
position = [0.236, -0.007, 0.121]
pulses   = [492, 354, 414, 59, 504]
```

Approach：

```text
position = [0.236, -0.007, 0.015]
pulses   = [492, 343, 240, 207, 504]
```

Lift：

```text
position = [0.236, -0.007, 0.121]
pulses   = [492, 354, 414, 59, 504]
```

## 10.3 夹爪

```text
打开：ID10 pulse=200
关闭：ID10 pulse=700
```

## 10.4 结果

```text
真实视觉：PASS
自然语言 Parser：PASS
Grounding：PASS
Confirm：PASS
真实 IK：PASS
真实 Servo：PASS
Hover：PASS
Approach：PASS
Close：PASS
Lift：PASS
杯子离开桌面：PASS
```

当前结论：

```text
首次真实视觉杯子抓取成功
```

仍需继续验证：

```text
重复性
抓取姿态
ROI 稳定性
Verification 可靠性
PlaceSkill
```

---

# 11. 常见问题

## 11.1 Grounding `no_match`

示例：

```text
status=no_match
detail=未找到对象（class=cup, color=blue）
```

检查：

```bash
timeout 15 ros2 topic echo \
  /world_model/stable_objects \
  --once \
  --full-length
```

如果 Stable Object 为：

```text
class_name=cup
color=unknown
```

则：

```text
拿起蓝色杯子
```

无法匹配。

使用：

```text
拿起杯子
```

继续验证主链。

---

## 11.2 ROI 间歇性误检

曾出现：

```text
ROI:
class_name=cube
color=white
xyz=[0.185, -0.011, 0.034]
```

同时 YOLO：

```text
class_name=cup
xyz=[0.237, -0.007, 0.041]
```

风险：

```text
YOLO 类别
+
错误 ROI 位姿
→ yolo_roi_fused
→ Stable Object 位姿跳变
```

真实执行前检查：

```bash
ros2 topic echo /world_model/perception_objects
ros2 topic echo /world_model/stable_objects
```

如果位置在短时间内明显跳变，不要确认执行。

---

## 11.3 Confirm 没反应

检查：

```bash
ros2 topic info /runtime/confirm --verbose
```

可能原因：

- Runtime 未运行；
- 没有 pending task；
- 已超过 300 秒；
- 发到了 Legacy `/executor/confirm`；
- 有重复 Runtime；
- 上一任务 `run_once=true` 已结束。

---

## 11.4 机械臂没有运动

检查启动参数：

```text
enable_real_servo=false
```

时不会运动。

IK-only 正常日志：

```text
mode=LIVE: ik=REAL servo=OFF
```

完整真实执行需要：

```text
dry_run=false
enable_real_ik=true
enable_real_servo=true
```

---

## 11.5 IK 无解

检查 Service：

```bash
ros2 service list | grep /kinematics/set_pose_target
```

检查目标位姿：

```text
x
y
z
```

检查 Runtime 日志：

```text
IK response
success
pulse
```

---

## 11.6 Verification 假阳性

在：

```text
enable_real_servo=false
```

机械臂未执行的情况下，曾出现：

```text
object_no_longer_at_source
state=verified
```

当前 Verification 不能单独作为成功证据。

真实成功必须结合：

- 机械臂确实执行；
- 物体实际离开桌面；
- 执行日志；
- 图片或现场观察。

后续需要：

- 模式门控；
- 连续多帧确认；
- Tracker 丢失与真实位移区分；
- 与 Servo 执行状态关联。

---

## 11.7 Grounding yaw 丢失

曾观察：

```text
Grounding:
rpy=[0.0, 0.0, 1.57]

Runtime IK:
rpy=[0.0, 0.0, 0.0]
```

圆形杯子抓取已成功，但方向敏感物体需要继续修复。

---

## 11.8 临时 IK 节点警告

日志可能出现：

```text
creating temp IK node
Publisher already registered for provided node name
```

本次真实 IK 可正常返回，但后续应：

```text
复用长期 IK Client
```

或：

```text
给临时 IK Node 唯一名称
```

---

## 11.9 Cyclone DDS XML 警告

```text
NetworkInterfaceAddress: deprecated element
```

当前不影响：

```text
Topic
Service
相机
真实 IK
真实 Servo
```

暂不原地修改已验证 XML。

---

## 11.10 多写者风险

以下节点可能发布 `/servo_controller`：

```text
real_grounded_runtime_node / RuntimeAdapter
grasp
llm_voice_agent
face_follow_node
env_scan_node
gesture_player_node
ground_executor_node（Legacy）
```

真实抓取期间不要同时启动可能主动控制机械臂的其他节点。

---

# 12. 停止与清理

## 12.1 停止 PC 节点

在各启动终端按：

```text
Ctrl+C
```

然后检查：

```bash
ros2 node list | sort | grep -E \
'simple_yolo|roi_color|perception_fusion|stable_object|llm_command_parser|grounding|real_grounded_runtime|verification_result|robotops'
```

进程检查：

```bash
pgrep -af \
'simple_yolo_node|roi_color_detector_node|perception_fusion_node|stable_object_tracker_node|llm_command_parser_node|grounding_node|real_grounded_runtime_node|verification_result_node|robotops'
```

## 12.2 Orin 主栈

正常测试结束后不需要停止：

```text
start_app_node.service
```

只有在维护 Orin 主栈时才执行：

```bash
sudo systemctl stop start_app_node.service
```

恢复：

```bash
sudo systemctl start start_app_node.service
```

---

# 13. 单节点故障隔离

> 以下命令只用于排查，不与对应 Bringup 同时运行。

## 13.1 ROI

```bash
ros2 run app roi_color_detector_node \
  --ros-args \
  -p transform_yaml:=/home/sundasheng/ros2_ws/src/app/config/transform.yaml \
  -p lab_config:=/home/sundasheng/ros2_ws/src/app/config/lab_config.yaml
```

## 13.2 YOLO

```bash
ros2 run vision_yolo simple_yolo_node
```

## 13.3 Fusion

```bash
ros2 run app perception_fusion_node
```

## 13.4 Tracker

```bash
ros2 run app stable_object_tracker_node
```

## 13.5 Grounding

```bash
ros2 run grounding grounding_node \
  --ros-args \
  -p publish_runtime:=true \
  -p world_model_topic:=/world_model/stable_objects
```

## 13.6 Runtime

```bash
ros2 run sketch_runtime real_grounded_runtime_node \
  --ros-args \
  -p dry_run:=true \
  -p require_confirm:=true
```

## 13.7 Verification

```bash
ros2 run sketch_runtime verification_result_node
```

## 13.8 RobotOps

```bash
ros2 launch robotops robotops_recorder.launch.py
```

---

# 14. 离线与开发测试

## 14.1 构建 Runtime

```bash
cd ~/ros2_ws

colcon build \
  --packages-select sketch_runtime \
  --symlink-install

source install/setup.bash
```

## 14.2 runtime_test_node

```bash
ros2 run sketch_runtime runtime_test_node
```

自定义：

```bash
ros2 run sketch_runtime runtime_test_node \
  --ros-args \
  -p test_command:="pick cup" \
  -p hover_height:=0.08 \
  -p approach_z:=0.015
```

Launch：

```bash
ros2 launch sketch_runtime runtime_test.launch.py \
  run_once:=true
```

循环：

```bash
ros2 launch sketch_runtime runtime_test.launch.py \
  run_once:=false \
  interval_sec:=2.0
```

> `runtime_test_node` 是内部 Runtime 测试工具，不代表真实视觉主线。

## 14.3 ROI Audit

```bash
cd ~/ros2_ws

colcon build \
  --packages-select app \
  --symlink-install

source install/setup.bash

ros2 run app roi_detection_audit_node
```

自定义样本数：

```bash
ros2 run app roi_detection_audit_node \
  --ros-args \
  -p sample_count:=200
```

结果：

```bash
cat artifacts/perception_audit/roi_perception_audit.md
cat artifacts/perception_audit/audit_summary.json
```

---

# 15. Legacy 接口

当前主线：

```text
/grounded_task_context
→ real_grounded_runtime_node
→ /runtime/preview
← /runtime/confirm
```

Legacy：

```text
/grounded_goal
→ ground_executor_node
→ /executor/preview*
← /executor/confirm
← /executor/confirm_str
```

不要同时启动：

```text
ground_executor_node
real_grounded_runtime_node
```

当前不要使用：

```bash
ros2 topic pub --once \
  /executor/confirm \
  std_msgs/msg/Bool \
  'data: true'
```

当前应使用：

```bash
ros2 topic pub --once \
  /runtime/confirm \
  std_msgs/msg/String \
  'data: "yes"'
```

---

# 16. Mobile Robot Debugging

> 本节属于 Mobile Robot Foundation，与当前机械臂视觉抓取主线分开。

## 16.1 底盘启动

```bash
ros2 launch turn_on_dlrobot_robot tank.launch.py
```

## 16.2 激光雷达

```bash
ros2 launch rplidar_ros rplidar_a1_launch.py \
  serial_port:=/dev/ttyUSB0
```

## 16.3 Odom TF Bridge

```bash
ros2 run mobile_base_bridge odom_tf_bridge_node
```

## 16.4 Laser Static TF

```bash
ros2 run tf2_ros static_transform_publisher \
  0 0 0.15 0 0 0 base_footprint laser
```

## 16.5 检查

```bash
ros2 topic echo /odom_combined --once
```

```bash
ros2 topic echo /scan --once
```

```bash
ros2 topic echo /tf --once
```

```bash
ros2 topic info /mobile_base/sensors/imu_data
```

## 16.6 AMCL

```bash
ros2 launch nav2_bringup localization_launch.py \
  map:=/home/ubuntu/ros2_ws/maps/home_map_clean_02_260705.yaml \
  use_sim_time:=false \
  params_file:=/home/ubuntu/ros2_ws/config/nav2_amcl_params.yaml
```

## 16.7 NAV2
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash

ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=false \
  autostart:=true \
  params_file:=/home/ubuntu/ros2_ws/config/nav2_params.yaml

状态：

```text
AMCL：In validation
Nav2：Not verified
MoveSkill：Missing
Semantic Locations：Missing
```

---

# 17. 当前验收状态

| 模块 | 状态 |
|---|---|
| PC / Orin 全 Cyclone DDS | ✅ Verified |
| 跨机相机 Topic | ✅ Verified |
| 跨机 Kinematics Service | ✅ Verified |
| YOLO | ✅ Verified |
| ROI | 🔶 Working，存在间歇性误检 |
| Perception Fusion | ✅ Working |
| Stable Tracker | ✅ Working |
| Parser | ✅ Verified |
| Grounding | ✅ Verified |
| Runtime Confirm | ✅ Verified |
| Dry-run | ✅ Verified |
| Real IK | ✅ Verified |
| Real Servo | ✅ Verified |
| 首次真实杯子抓取 | ✅ Complete |
| RobotOps | ✅ Implemented，真实任务写入需核查 |
| Verification | 🔶 In validation |
| 抓取重复性 | 🔶 To validate |
| PlaceSkill | ❌ Not verified |
| AMCL | 🔶 In validation |
| Nav2 | ❌ Not verified |
| MoveSkill | ❌ Missing |

---

# 18. 文档维护规则

本文件只维护：

```text
如何启动
如何监听
如何输入
如何确认
如何验收
如何排查
```

Topic / Service 的完整关系维护在：

```text
docs/topic_service_map.md
```

系统组成和架构图维护在：

```text
docs/runtime_architecture.md
```

风险维护在：

```text
docs/runtime_risks.md
```

每次具体开发过程记录在：

```text
docs/dev_log/
```

当以下内容变化时，必须更新本文件：

- Bringup 启动节点变化；
- Runtime 参数变化；
- 输入 Topic 变化；
- Confirm Topic 变化；
- 实机安全流程变化；
- Topic 查看命令变化；
- RobotOps 启动或数据库路径变化；
- Current / Legacy 执行链变化。

