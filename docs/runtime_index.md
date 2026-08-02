# JetArm Robot Runtime — Documentation Index

> 最后更新：2026-08-02
> 长期项目目标：Intelligent Autonomous Mobile Manipulation Robot
> 当前阶段：Fixed-Workspace Task Manipulation MVP
> 当前活动主线：固定工作区内的任务级闭环——AprilTag 标定与自检、目标轮廓、抓取位姿、Pick / Place、Verification 与 RobotOps
> 已解决 P0：Duplicate Mechanical Arm Control-Stack Instances；PC / Orin DDS 统一；首次真实视觉杯子抓取
> 当前 P0：Task MVP 1 — 整理蓝色积木
> 移动底盘状态：PAUSED — AMCL / Nav2 等待后续重新验证

---

## 2026-08-02 Update Snapshot

本次更新只修正当前状态，不改变本文原有的项目目标、历史恢复过程、文档导航和 Source of Truth 结构。

新增已验证事实：

```text
PC + Orin 新启动 ROS 2 进程统一使用 Cyclone DDS
跨机 RGB 图像约 30 Hz
跨机 /kinematics/get_current_pose 验证通过
跨机 /kinematics/set_pose_target 真实 IK 验证通过
Runtime Confirm 验证通过
真实 Servo 完整 Pick 验证通过
机械臂成功拿起杯子
```

当前实机夹爪基线：

```text
Servo ID10:
200 = open
700 = close
```

当前不是“恢复 Runtime 是否能运行”的阶段，而是：

```text
首次真实成功
→ 固定工作区与 AprilTag 坐标基线
→ 目标轮廓与任务级抓取位姿
→ Pick + Place
→ 源区与目标区 Verification
→ RobotOps 数据完整性
→ Retry / Recovery
```

当前任务方向已收敛为有限场景下的真实任务闭环：

```text
Task MVP 1：整理蓝色积木
Task MVP 2：整理圆珠笔
Task MVP 3：搬运空茶杯
```

其中：

```text
object pose ≠ grasp pose
```

YOLO / 分类器负责粗类别，ROI / segmentation 负责精确轮廓，AprilTag 负责工作区坐标与视觉自检，Grasp Pose Estimator 负责真正的抓取点、方向与夹爪开度。

---

## Project Mission

构建一个能够通过大模型指令或传统确定性控制执行真实机器人任务的智能 Runtime。

目标闭环：

```text
Natural Language / LLM / Deterministic Command
→ Command Parsing
→ Grounding
→ TaskContext
→ SkillManager
→ Skill / Action Execution
→ Runtime State Machine
→ Verification
→ Execution Result
→ RobotOps Persistence
→ Retry / Recovery
```

系统最终应具备：

- 大模型驱动与传统确定性控制并存；
- 统一的 Task / Skill / Action 执行框架；
- 可观察、可追踪的任务状态；
- Preview / Confirm 安全机制；
- 动作完成与否的独立验证；
- 失败检测与 Retry / Recovery；
- 任务日志、状态、动作和验证数据持久化；
- Teleop 与自动控制之间的安全仲裁；
- 数据回放、演示采集和 VLA 接入能力；
- 向移动操作机器人扩展。

---

## Strategic Goal

最终目标是 Autonomous Mobile Manipulation Robot。

示例任务：

```text
“去厨房拿一个杯子给我”
```

目标执行链：

```text
NavigateSkill
→ SearchSkill
→ Grounding
→ PickSkill
→ Verification
→ NavigateSkill
→ PlaceSkill
→ Verification
→ RobotOps
```

长期能力包括：

- SLAM 与定位；
- Nav2 自主导航；
- 语义位置；
- 视觉目标识别与世界模型；
- 自然语言 Grounding；
- 机械臂抓取与放置；
- 状态机和任务生命周期；
- 结果验证；
- 失败恢复；
- 数据积累与任务回放；
- Teleop 与安全仲裁；
- VLA 数据与接口准备。

---

## Current Status

| 层级 | 状态 |
|------|------|
| **Project Mission** | 构建具备任务理解、状态控制、动作执行、结果验证、数据积累和失败恢复能力的智能机器人 Runtime |
| **Runtime Foundation** | ✅ 历史完成：TaskContext、TaskState、SkillManager、PickSkill、ActionExecutor、RuntimeAdapter |
| **Real Arm Execution** | ✅ 当前实测：真实 IK、真实 Servo 和完整 Pick 已通过；2026-08-02 成功拿起杯子 |
| **Perception Fusion** | ✅ 历史完成：YOLO + ROI → Perception Fusion |
| **Stable World Model** | ✅ 历史完成：StableObjectTracker |
| **Verification** | 🔶 Runtime 已实现并集成状态机；曾出现 Servo OFF 条件下假阳性，可靠性仍需修复与复验 |
| **RobotOps** | ✅ SQLite 事件持久化与任务历史基础已完成；本次真实 Pick 的数据完整性待核查 |
| **Retry / Recovery** | ⏳ 尚未实现，计划基于真实失败数据开发 |
| **Control Arbitration** | ⏳ 尚未实现 AUTO / MANUAL / PAUSED / ESTOP 仲裁 |
| **Data Collection / VLA** | ⏳ 轨迹、图像和动作数据采集尚未完成 |
| **Current Phase** | 🔶 Fixed-Workspace Task Manipulation MVP |
| **Duplicate Control Stack** | ✅ 已解决：核心机械臂节点均为单实例；底层控制端点恢复一对一 |
| **Mechanical Arm Hardware** | ✅ STM32 通信正常；舵机 ID `1/2/3/4/5/10` 在线；反馈与初始姿态正常 |
| **Network / Time** | ✅ `eth-static` 冷启动自动恢复；Jetson 自动从 PC `192.168.100.2` 校时；机器人服务不以网络或时间为启动门禁 |
| **Cross-host DDS** | ✅ PC 与 Orin 已统一为 Cyclone DDS；跨机图像、Topic 与 Kinematics Service 已验证 |
| **Current P0** | 🔶 Task MVP 1：固定工作区内识别、抓取并收纳蓝色积木 |
| **Current Real-Motion Gate** | ⚠ 已完成分级验收并允许受控真实执行；仍必须保留单一控制栈、稳定目标位姿、`require_confirm=true` 和现场急停条件 |
| **Near-Term Outcome** | 完成“蓝色积木识别 → 抓取 → 放入指定区域 → 双区域验证 → RobotOps 记录”的真实任务闭环 |
| **Mobile Base** | ⏸ 底盘维修中，AMCL/Nav2 暂停 |
| **Long-Term Direction** | Autonomous Mobile Manipulation Robot |

## Current Development Strategy

当前工作不是重新设计整个 Runtime，也不是回到底盘继续调参。

固定机械臂 Runtime 的“恢复与首次真实执行”已经完成：

```text
重复控制栈根因与修复              ✅
单一底层机械臂控制栈              ✅
硬件、网络与时间基线              ✅
PC / Orin 全 Cyclone DDS          ✅
单元测试与 Dry-run                ✅
真实 IK（Servo OFF）              ✅
真实 Servo 完整 Pick              ✅
首次真实视觉杯子抓取              ✅
```

当前主线正式进入：

```text
项目长期目标
└── 智能机器人 Runtime
    └── 当前阶段：固定工作区任务级操作 MVP
        ├── P0：AprilTag 工作区标定与视觉自检
        ├── P0：统一权威坐标链
        ├── P0：积木轮廓与 Grasp Pose
        ├── P0：Pick + Place + 双区域 Verification
        └── P1：RobotOps 任务数据与 Retry / Recovery
```

当前不再把“物体中心坐标更准”作为最终目标；视觉输出必须服务于具体任务，并区分对象位姿与抓取位姿。

移动底盘继续保持暂停：

```text
不发布 /cmd_vel
不继续 AMCL / Nav2 参数调试
恢复底盘开发前重新建立 Odom / IMU / TF / Localization 基线
```

## Current Development Stop Point

### Mobile Base Stop Point — 2026-07-27

本阶段完成了 Localization 与 Nav2 的初步实机验证：

1. CW1 Localization 测试中，初始约 `5°` 的激光点云偏差可在直线运动后得到一定修正。
2. Nav2 短距离直线测试效果较好。
3. 机器人执行约 `30°～45°` 转弯时，在光滑瓷砖地面发生明显打滑。
4. 底盘实际运动不再符合稳定的差速运动模型。
5. RViz2 中激光点云随底盘打滑发生漂移。
6. 当前问题已不能继续单纯通过 AMCL、Nav2、Odom 或 IMU 参数调试解决。

最终开发决策：

```text
停止当前 AMCL / Nav2 参数开发
→ 转入底盘机械检查与维修
→ 当前开发主线切换回机械臂 Runtime
```

---

## Mechanical Arm Recovery Status — 2026-07-28

### Startup Failure

最初现象：

```text
机械臂开机完全不动
舵机未进入初始姿态
ros_robot_controller 启动失败
/dev/ttyUSB0 不存在
```

日志错误：

```text
serial.serialutil.SerialException:
could not open port /dev/ttyUSB0:
No such file or directory
```

### Root Cause

STM32 控制板插入了错误的 USB 接口。

### Recovery Result

重新连接正确 USB 接口后：

```text
/dev/ttyUSB0 恢复
→ ros_robot_controller 正常启动
→ controller_manager / servo_manager 正常工作
→ 舵机反馈恢复
→ 机械臂进入初始姿态
```

结论：

```text
STM32：未发现损坏
机械臂舵机：未发现整体损坏
舵机总线：通信正常
PC ↔ Orin DDS：通信正常
```

---

## Current Mechanical Arm State

当前 `/controller_manager/servo_states` 实测反馈：

| Servo ID | 当前初始位置 |
|----------|--------------|
| 1 | 500 |
| 2 | 560 |
| 3 | 130 |
| 4 | 115 |
| 5 | 500 |
| 10 | 200 |

当前关节名称：

```text
joint1
joint2
joint3
joint4
joint5
r_joint
```

前五个舵机位置与历史 Home 记录一致：

```python
home_pulses = [500, 560, 130, 115, 500]
```

注意：

- ID10 当前开机初始反馈为 `200`；
- 2026-08-02 已通过实机命令与完整 Pick 确认：
  - `200` = 夹爪打开；
  - `700` = 夹爪关闭；
- 历史记录中的 `100`、`500` 不再作为当前 Runtime 的夹爪控制事实；
- 后续修改夹爪参数仍必须通过受控实机验证。

---

## Fixed-Arm Reactivation Progress — 2026-07-30

### Resolved P0 — Duplicate Mechanical-Arm Control Stack

历史运行图曾出现：

```text
2 × /ros_robot_controller
2 × /controller_manager
2 × /servo_manager
2 × /grasp
2 × /kinematics
2 × /buzzer_controller
```

历史风险链：

```text
1 条 /servo_controller 指令
→ 两套 controller_manager 接收
→ 两套 servo_manager 下发
→ 两套 ros_robot_controller 访问控制板
→ 存在重复串口写入与不可复现风险
```

最终修复原则：

- `bringup.launch.py` 保留唯一的直接 SDK / 硬件控制栈入口；
- joystick 子路径增加 `include_sdk` 条件，默认不再启动第二套 SDK；
- `joystick_control` 默认关闭；
- vendor demo `start_app.launch.py` 不再作为并行执行入口；
- PC 不启动 Jetson 硬件驱动。

修复后验收：

```text
/ros_robot_controller ×1
/controller_manager ×1
/servo_manager ×1
/grasp ×1
/kinematics ×1
```

控制端点实测：

```text
/servo_controller
  Publisher:    /grasp ×1
  Subscription: /controller_manager ×1

/ros_robot_controller/bus_servo/set_position
  Publisher:    /servo_manager ×1
  Subscription: /ros_robot_controller ×1
```

结论：重复机械臂控制栈 P0 已关闭。

### Network and Time Hardening

固定网络：

```text
PC eno1:      192.168.100.2/24
Jetson eth0:  192.168.100.1/24
Jetson wlan0: 192.168.149.1/24 — HW-23020672
```

当前有线 Profile：

```text
eth-static
ipv4.method=manual
ipv4.addresses=192.168.100.1/24
ipv4.gateway=""
ipv4.dns=""
ipv4.never-default=yes
connection.autoconnect=yes
```

PC 使用 Chrony 向机器人网段提供 NTP；Jetson 使用 `systemd-timesyncd`，优先时间源包括：

```text
192.168.100.2
192.168.149.2
0.pool.ntp.org
1.pool.ntp.org
```

彻底断电冷启动验收已通过：

```text
eth-static 自动恢复
→ PC 可 SSH 192.168.100.1
→ Jetson 自动选择 Server 192.168.100.2
→ System clock synchronized: yes
→ wifi.service / systemd-timesyncd / start_app_node.service 均 active
```

机器人启动仍不依赖 PC、网络或时间同步；无网络时机器人服务必须继续启动。

### Vendor Wi-Fi Manager Root Cause and Fix

厂商脚本：

```text
/home/ubuntu/wifi_manager/wifi.py
```

历史危险逻辑：

```python
os.system('rm /etc/NetworkManager/system-connections/*')
```

该命令会删除全部 NetworkManager Profile，包括 `eth-static`。这解释了为何普通运行中静态地址存在，但彻底断电后配置消失、系统重新生成 DHCP 类型的 `Wired connection 1`。

修复：

- 删除全目录清空逻辑；
- 保留厂商 AP 重建逻辑；
- 已创建备份：

```text
/home/ubuntu/wifi_manager/wifi.py.backup_20260730
```

### DDS Startup Timing Finding

冷启动单调时间证据：

```text
13.804 s  eth-static 激活成功
14.246 s  机械臂节点进程启动
15.934 s  eth0 物理 carrier 建立
```

现象：机械臂节点在物理载波建立前创建 DDS Participant 时，PC 初始只能发现较晚启动的 rosbridge / camera；在载波已建立后重启 `start_app_node.service`，PC 立即发现完整机械臂节点与 Topic。

已验证：

- PC 与 Jetson 的 `ROS_DOMAIN_ID=23`、`rmw_fastrtps_cpp` 和有线通信正常；
- `ros2 --no-daemon` 结果排除了 daemon 缓存作为主因；
- 载波建立后创建的临时 DDS probe 可被 PC 发现；
- 服务 stop/start 后完整机械臂图可被 PC 发现。

当前工程规则：

- 不使用永久 `network-online.target` 门禁；
- 不等待 PC 响应或 NTP 才启动机器人；
- 如固化启动等待，只允许短时、可超时、永远返回成功的可选 carrier 等待；
- 每次冷启动仍应检查 PC 是否能直接发现完整机械臂图。

### Current Safety Gate

分级验收已经完成到首次真实视觉 Pick：

```text
Runtime 审查                            ✅
单元测试                                ✅
Dry-run 状态流                          ✅
真实 IK（Servo OFF）                    ✅
真实 Servo                              ✅
Hover / Approach / Gripper / Lift       ✅
首次真实视觉杯子抓取                    ✅
```

真实动作现在属于**受控开放**，而不是默认开放。

每次真实执行必须满足：

```text
核心硬件节点保持单实例
PC 不启动第二套 Orin 硬件驱动
require_confirm=true
目标 Stable Object 位姿无明显跳变
工作区清空
操作员现场观察
可立即断电
无其他主动 /servo_controller 写者
```

当前仍禁止：

```text
启动 legacy ground_executor_node 与 Current Runtime 并行执行
直接写 /ros_robot_controller/bus_servo/set_position
未检查目标位姿就发送 /runtime/confirm
同时启用 joystick / tracking / sorting / gesture 等机械臂控制源
首次实验使用 require_confirm=false
```

## Current Control Surface

当前 Orin 常驻硬件栈的基础写入链已经恢复为唯一链路；PC Runtime 启动并进入真实执行时，会通过 `/servo_controller` 增加受控上层写入：

```text
/grasp ×1
→ /servo_controller
→ /controller_manager ×1
→ /servo_manager ×1
→ /ros_robot_controller/bus_servo/set_position
→ /ros_robot_controller ×1
→ STM32
```

Orin 常驻空闲基线曾实测为：

```text
/servo_controller
Publisher count: 1       node: /grasp
Subscription count: 1    node: /controller_manager
```

PC `real_grounded_runtime_node` 启动且 `enable_real_servo=true` 时，`RuntimeAdapter` 也会成为 `/servo_controller` 的发布源。真实执行前必须重新检查发布者列表。

```text
/ros_robot_controller/bus_servo/set_position
Publisher count: 1       node: /servo_manager
Subscription count: 1    node: /ros_robot_controller
```

仓库中仍存在可启动的其他控制源，例如：

```text
tag_stackup
calibration
finger_trace
object_tracking
object_sortting
waste_classification
joystick_control
legacy ground_executor_node
```

这些节点当前未运行，但架构尚未实现统一的：

```text
AUTO / MANUAL 控制模式
全局控制权仲裁
唯一上层命令入口
全局 busy lock
Emergency Stop 状态机
```

因此后续 Runtime 审查必须确认：

- 当前主路径只通过 `RuntimeAdapter` 和 `/servo_controller`；
- 新 Runtime 代码不直接写 `bus_servo/set_position`；
- legacy、demo、teleop 和自动控制路径不会并行获得控制权；
- 在仲裁层实现前，其他控制源保持默认关闭。

## Deployment Overview

ROS 2 是分布式系统。

PC 上看到某个 Topic 或 Node，不代表该节点运行在 PC。节点所在主机必须通过对应主机上的进程、launch、systemd 和日志确认。

### Jetson / Robot Ubuntu

Workspace：

```text
/home/ubuntu/ros2_ws
```

Jetson 默认运行 Robot-side hardware runtime：

```text
ros_robot_controller
controller_manager
servo_manager
grasp
kinematics
depth camera SDK
mobile base driver
RPLidar
robot-side TF
systemd hardware bringup
```

Jetson 对以下物理设备拥有唯一控制权：

```text
STM32
Servo bus
Mechanical arm
Depth camera
Mobile base serial device
LiDAR serial device
```

### PC

Workspace：

```text
/home/sundasheng/ros2_ws
```

PC 默认运行 Intelligence and Project Runtime：

```text
LLM / parser / Agent
YOLO / ROI
Perception Fusion
StableObjectTracker
Grounding
Runtime / Skill / Action
Verification
RobotOps
RViz
rosbag
offline analysis
Git / documentation / tests
```

### Network

当前固定地址：

```text
PC eno1:      192.168.100.2/24
Jetson eth0:  192.168.100.1/24
Jetson wlan0: 192.168.149.1/24
Jetson AP:    HW-23020672
```

当前有线链路采用独立静态 Profile：

```text
Jetson: eth-static
PC:     有线连接 1
```

两侧有线 Profile 均不设置默认网关和 DNS，且 `never-default=yes`，避免机器人直连网线抢占互联网默认路由。

时间链：

```text
Public NTP
→ PC Chrony
→ PC 192.168.100.2:123
→ Jetson systemd-timesyncd
```

冷启动已验证：

```text
System clock synchronized: yes
Server: 192.168.100.2
```

PC 与 Orin 当前使用：

```text
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

2026-08-02 已验证：

- Orin 重启后主 Bringup 继续继承 Cyclone DDS；
- PC 可持续接收约 30 Hz RGB 图像；
- PC 可调用 Orin `/kinematics/get_current_pose`；
- Runtime 可调用 Orin `/kinematics/set_pose_target`；
- 真实 Servo 完整 Pick 成功。

DDS 注意事项：

- 已运行的 ROS 2 进程不会因 shell 环境变化而动态切换 RMW，必须重启；
- 冷启动后仍应检查 PC 是否看到 `/controller_manager`、`/servo_manager`、`/ros_robot_controller`、`/grasp`、`/kinematics`；
- 当前 Cyclone XML 会提示 `NetworkInterfaceAddress` deprecated，但不阻塞已验证功能；
- 不得通过永久等待 `network-online.target` 或 PC 响应来阻塞机器人本地启动。

历史临时地址 `172.20.10.2` 不再作为当前默认地址。

### Deployment Rule

实际运行事实优先于本文档。

发生不一致时必须区分：

```text
Expected Host
Observed Host
Process Evidence
Launch Evidence
Unknown
```

PC 不得启动第二套 Jetson 硬件驱动。

## Runtime Capabilities Already Built

### Task and State

已实现：

- `TaskContext`
- `TaskState`
- `TargetObject`
- `ExecutionResult`
- `WAITING_CONFIRM`
- `VERIFYING`
- `VERIFIED`
- `VERIFICATION_FAILED`

### Skill Runtime

已实现：

- `BaseSkill`
- `SkillRegistry`
- `SkillManager`
- `PickSkill`
- `BaseAction`
- `MoveAction`
- `GripperAction`
- `ActionExecutor`

### Hardware Adapter

已实现：

- `dry_run`
- `enable_real_ik`
- `enable_real_servo`
- 真实 IK 调用
- Servo message adapter
- 独立 gripper command
- Hover-only 安全级别
- 最小完整 Pick

2026-08-02 已验证：

```text
真实目标位姿
→ 真实 IK
→ /servo_controller
→ Hover
→ gripper open
→ Approach
→ gripper close
→ Lift
→ 杯子离开桌面
```

### Verification

已实现：

- precheck target exists
- after-pick target removed
- after-place target present
- `/runtime/verification_result`
- Runtime verification states
- observation-only Verification sidecar

### RobotOps

已实现：

- `robotops` 独立包
- SQLite `events` 单表
- Runtime event persistence
- TaskHistory 查询接口
- `/runtime/state`
- `/runtime/log`
- `/runtime/execution_result`
- `/runtime/verification_result`

---

## Remaining Strategic Runtime Work

### Retry / Recovery

计划能力：

```text
stop-only
ask_user_confirm
offset_retry
auto_retry
```

原则：

- Retry 必须建立在真实失败数据上；
- 先记录失败，再分类失败；
- 不允许盲目自动重试；
- 自动重试需要次数限制和安全边界。

### Teleop and Arbitration

计划能力：

```text
AUTO
MANUAL
PAUSED
EMERGENCY_STOP
```

优先级：

```text
EMERGENCY_STOP > MANUAL > AUTO
```

需要记录：

```text
谁获得控制权
何时获得
为何切换
何时释放
任务是否被中断
```

### Data Collection

计划记录：

- pre-pick image；
- post-pick image；
- post-place image；
- IK 输入输出；
- Servo command；
- Servo state；
- gripper state；
- Runtime state；
- Verification result；
- 人工接管过程；
- 任务成功或失败。

### VLA Readiness

计划能力：

- trajectory + image 数据集；
- task_id 导出；
- SQLite 任务回放；
- VLA command → Runtime input adapter；
- demonstration collection；
- failure/recovery 数据。

---

## Strategic Runtime Roadmap

```text
Runtime Foundation
✅ Task / Skill / Action / Adapter
↓
Real Arm Execution
✅ IK / Servo / Hover / Pick
↓
Stable Perception
✅ Fusion / StableObjectTracker
↓
Verification
✅ Precheck / Postcheck / Runtime State
↓
RobotOps
✅ SQLite Persistence
↓
Current Reactivation
🔶 Single Control Stack / Safe Revalidation
↓
Retry / Recovery
⏳ Failure-aware task recovery
↓
Teleop Arbitration
⏳ AUTO / MANUAL / PAUSED / ESTOP
↓
Data Collection
⏳ Images / Trajectories / Actions / States
↓
VLA Readiness
⏳ Dataset / Replay / VLA Bridge
↓
Mobile Manipulation
⏳ Navigate / Search / Pick / Place
```

---

## Current Reactivation Roadmap

```text
确认重复控制栈根因                         ✅
↓
固化单一机械臂控制栈                       ✅
↓
建立硬件、网络与时间基线                   ✅
↓
PC / Orin 统一 Cyclone DDS                 ✅
↓
审查 Runtime / Skill / Action / Adapter     ✅
↓
恢复单元测试、Dry-run 与状态流              ✅
↓
验证真实 IK，但禁止 Servo                   ✅
↓
最小 Servo 健康测试                         ✅
↓
Hover / Approach / Gripper / Lift           ✅
↓
固定坐标 Pick                               ✅
↓
真实视觉抓取闭环                            ✅
↓
固定工作区与 AprilTag 坐标基线               🔶 CURRENT
↓
RGB 图像 / CameraInfo 与权威坐标链统一       🔶 CURRENT
↓
积木 mask / contour / orientation            ⏳
↓
任务级 Grasp Pose Estimator                  ⏳
↓
PlaceSkill 与固定收纳区域                    ⏳
↓
源区 + 目标区 Verification                   ⏳
↓
RobotOps 数据完整性与任务回放                ⏳
↓
Task MVP 1 重复性测试与失败分类              ⏳
↓
根据真实失败数据开发 Retry / Recovery       ⏳
↓
控制权仲裁与 Teleop                         ⏳
↓
数据采集和 VLA Readiness                    ⏳
```

## Documentation Navigation

文档按任务选择，不要求每个 Agent 全部读取。

### Always Read

| 文件 | 用途 |
|------|------|
| `runtime_index.md` | 项目目标、当前状态、活动任务和停止点 |
| `CLAUDE.md` | Agent 工作规则和禁止事项 |

### Current P0 Task

当前任务范围：完成 Task MVP 1——在固定工作区内整理蓝色积木。

```text
1. 保留 2026-08-02 首次真实 Pick 的日志、图片、task_id、目标位姿和 IK pulses，作为历史基线
2. 固定机械臂观察姿态，使用 AprilTag 建立 workspace frame 与启动自检
3. 修复 RGB image / RGB CameraInfo 配对，并确定唯一权威坐标链
4. 从 ROI / segmentation 获取积木的 mask / contour
5. 计算积木 centroid、minAreaRect、yaw、length、width
6. 生成任务级 grasp candidate：
   - 抓取点
   - 夹爪 yaw
   - 所需开度
   - approach / lift 参数
7. 实现或验证 PlaceSkill，并定义固定收纳区域
8. Verification 必须同时确认：
   - 源区域目标消失
   - 收纳区域目标出现
   - Servo OFF 或短暂漏检不得报告真实成功
9. RobotOps 保存 pre-pick / post-pick / post-place 图像、抓取位姿、IK、执行与验证结果
10. 执行受控重复性测试，记录成功率和失败类型
11. 只在真实失败分类后设计受限 Retry / Recovery
```

当前交付物建议：

```text
task_mvp1_block_sorting_design.md
task_mvp1_block_sorting_dev_log.md
task_mvp1_repeatability_report.md
```

当前禁止：

```text
并行启动 Current Runtime 与 legacy ground_executor_node
直接写 /ros_robot_controller/bus_servo/set_position
未确认 Stable Object 位姿就触发真实执行
同时运行其他主动机械臂控制节点
将单次 verified 直接当作真实成功证据
```

## Subsequent Mechanical Arm Work Order

当前进度与后续顺序：

```text
1. 修复重复启动                              ✅
2. 验证核心节点均只有一个实例                ✅
3. 建立硬件、网络、时间与 DDS 基线           ✅
4. 完成 Runtime / Skill / Adapter 审查        ✅
5. 运行单元测试和 Dry-run                    ✅
6. 验证真实 IK，但禁止 Servo                 ✅
7. 最小幅度 Servo 健康测试                   ✅
8. Hover / Approach                          ✅
9. 固定坐标 Pick                             ✅
10. 真实视觉抓取闭环                         ✅
11. 保存 2026-08-02 首次真实 Pick 证据      ✅
12. AprilTag 固定工作区坐标与自检             🔶 CURRENT
13. RGB CameraInfo 与权威坐标链统一           🔶 CURRENT
14. 积木 mask / contour / orientation         ⏳
15. 任务级 Grasp Pose Estimator               ⏳
16. PlaceSkill 与固定收纳区域                 ⏳
17. 源区 + 目标区 Verification                ⏳
18. RobotOps 任务数据完整性验证               ⏳
19. Task MVP 1 重复性测试                     ⏳
20. 记录真实失败模式                          ⏳
21. 设计 Retry / Recovery                     ⏳
22. Task MVP 2：圆珠笔整理                    ⏳
23. Task MVP 3：空茶杯搬运                    ⏳
```

## Full Mechanical Arm Runtime Audit Scope

> 本章节不是当前重复控制栈排查任务的执行范围。
>
> 只有在单一底层控制栈建立并通过只读健康检查后，才启动该审查。

### Runtime

- `real_grounded_runtime_node`
- `TaskContext`
- `TaskState`
- `SkillManager`
- `SkillRegistry`
- `RuntimeAdapter`

### Skills and Actions

- `BaseSkill`
- `PickSkill`
- `BaseAction`
- `MoveAction`
- `GripperAction`
- `ActionExecutor`

### Motion and Hardware Interfaces

- `/kinematics/set_pose_target`
- `/kinematics/get_current_pose`
- `/servo_controller`
- `/grasp`
- `/controller_manager/joint_states`
- `/controller_manager/servo_states`
- `/ros_robot_controller/bus_servo/set_position`

### Perception and Grounding

- `/world_model/perception_objects`
- `/world_model/stable_objects`
- `/grounded_task_context`
- TargetObject source pose
- camera-to-arm transform
- calibration YAML

### Verification

- precheck；
- post-pick；
- post-place；
- task_id 匹配；
- evidence；
- confidence；
- failure reason；
- Runtime state integration。

### RobotOps

- event schema；
- task_id；
- state timeline；
- execution result；
- verification result；
- image / action / trajectory 扩展接口；
- replay 能力。

### Safety

- 重复控制栈；
- `/servo_controller` 多写者；
- joint / pulse limits；
- workspace limits；
- execution timeout；
- busy lock；
- command concurrency；
- action cancellation；
- emergency stop；
- IK failure handling；
- servo failure handling；
- safe return posture。

---

## Safe Inspection Commands

### PC Bash Environment

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=23
```

### Orin Zsh Environment

```zsh
source /opt/ros/humble/setup.zsh
source ~/ros2_ws/install/setup.zsh
export ROS_DOMAIN_ID=23
```

### Repository State

```bash
pwd
git status
git branch --show-current
git log -1 --oneline
colcon list
```

### ROS 2 Node Uniqueness

```bash
ros2 node list | sort
ros2 node list | sort | uniq -c | sort -nr
```

### Mechanical Arm Feedback

```bash
timeout 5 ros2 topic echo \
/controller_manager/servo_states \
--once

timeout 5 ros2 topic echo \
/controller_manager/joint_states \
--once
```

### Control Endpoint Inspection

```bash
ros2 topic info -v \
/ros_robot_controller/bus_servo/set_position

ros2 topic info -v \
/servo_controller
```

### Duplicate Launch Search

```bash
grep -RInE \
"jetarm_sdk.launch.py|ros_robot_controller.launch.py|servo_controller.launch.py|kinematics_node.launch.py" \
~/ros2_ws/src/bringup \
~/ros2_ws/src/driver \
~/ros2_ws/src/peripherals \
~/ros2_ws/src/app \
~/ros2_ws/src/example \
2>/dev/null
```

```bash
grep -RIn \
"IncludeLaunchDescription" \
~/ros2_ws/src/bringup \
~/ros2_ws/src/peripherals \
~/ros2_ws/src/app \
~/ros2_ws/src/example \
2>/dev/null
```

### Current Service Process Tree

```bash
systemctl cat start_app_node.service

systemctl show \
  -p MainPID \
  -p ExecStart \
  start_app_node.service

MAIN_PID=$(systemctl show \
  -p MainPID \
  --value start_app_node.service)

sudo pstree -ap "$MAIN_PID"
```

---

## Runtime Execution Modes

当前 Runtime 已完成三种模式的分级验收。

### Dry-run

```bash
ros2 launch sketch_runtime ground_runtime_bringup.launch.py   use_dummy_wm:=false   dry_run:=true   enable_real_ik:=false   enable_real_servo:=false   require_confirm:=true   run_once:=true
```

### Real IK / Servo OFF

```bash
ros2 launch sketch_runtime ground_runtime_bringup.launch.py   use_dummy_wm:=false   dry_run:=false   enable_real_ik:=true   enable_real_servo:=false   require_confirm:=true   run_once:=true
```

### Real IK / Real Servo

```bash
ros2 launch sketch_runtime ground_runtime_bringup.launch.py   use_dummy_wm:=false   dry_run:=false   enable_real_ik:=true   enable_real_servo:=true   require_confirm:=true   run_once:=true
```

完整命令、监听方法和 Confirm 流程统一见：

```text
docs/runtime_debug_guide.md
```

---

## Hardware Execution Warning

真实硬件执行已经验证，但必须受控。

始终禁止：

- 启动第二套 `jetarm_sdk.launch.py` 或其他底层控制栈；
- 在 `start_app_node.service` 运行时手动启动重复 SDK；
- 同时启动 legacy `ground_executor_node` 和 Current Runtime；
- 直接发布 `/ros_robot_controller/bus_servo/set_position`；
- 在存在其他主动 `/servo_controller` 写者时执行 Pick；
- 未检查 Stable Object 位姿就发送 Confirm；
- 使用 `require_confirm=false` 进行首次或未知场景实机测试。

真实执行前必须：

- 检查节点唯一性；
- 检查 `/servo_controller --verbose`；
- 确认目标位姿稳定；
- 清空机械臂工作区；
- 操作员现场观察；
- 保持可立即断电。

---

## Serial Device Rules

当前已确认：

```text
ros_robot_controller 默认尝试打开 /dev/ttyUSB0
```

旧文档中的 LiDAR 启动命令也曾使用：

```text
serial_port:=/dev/ttyUSB0
```

因此连接 STM32 与 RPLidar 时，不能只根据 `/dev/ttyUSB0` 名称判断设备。

每次启动前必须检查：

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
ls -l /dev/serial/by-id/ 2>/dev/null
lsusb
```

规则：

- 不得让 STM32 与 LiDAR 配置为同一个串口设备；
- USB 设备编号可能因插拔顺序变化；
- 优先使用 `/dev/serial/by-id/` 或稳定的 udev 规则；
- 修改串口配置前必须确认设备身份；
- STM32 插错 USB 接口可能导致 `/dev/ttyUSB0` 不存在，机械臂无法初始化。

---

## Perception Pipeline

机械臂控制栈修复后，可独立启动感知系统。

### Camera

相机由 Orin 原厂 bringup 启动时，PC 可以看到：

```text
/depth_cam/rgb/image_raw
/depth_cam/depth/image_raw
/depth_cam/depth/points
```

### Perception Bringup

推荐：

```bash
cd ~/ros2_ws
./scripts/start_cyclone_perception.sh
```

等效核心 Launch：

```bash
ros2 launch app perception_bringup.launch.py   start_grounding:=false
```

当前启动：

```text
ROI
YOLO
Perception Fusion
StableObjectTracker
```

Grounding 由：

```text
ground_runtime_bringup.launch.py
```

统一启动，避免重复节点。

注意：

- 原厂 `example/yolov8_node` 当前存在模块缺失错误；
- 当前项目自定义 `vision_yolo/simple_yolo_node` 与原厂示例 YOLO 必须区分；
- 不得将原厂 YOLO 示例节点的启动成功视为当前项目感知链已经启动。

### Manual Startup

```bash
ros2 run app roi_color_detector_node \
--ros-args \
-p transform_yaml:=/home/sundasheng/ros2_ws/src/app/config/transform.yaml \
-p lab_config:=/home/sundasheng/ros2_ws/src/app/config/lab_config.yaml
```

```bash
ros2 run vision_yolo simple_yolo_node
```

```bash
ros2 run app perception_fusion_node
```

```bash
ros2 run app stable_object_tracker_node
```

```bash
ros2 run grounding grounding_node \
--ros-args \
-p publish_runtime:=true \
-p world_model_topic:=/world_model/stable_objects
```

---

## Verification

```bash
ros2 run sketch_runtime verification_result_node
```

该节点应保持 observation-only：

```text
不调用 IK
不发布 Servo
不阻塞 Runtime
不控制硬件
```

---

## RobotOps Recorder

```bash
ros2 launch robotops robotops_recorder.launch.py
```

数据库：

```bash
sqlite3 ~/ros2_ws/robotops.db
```

---

## Runtime Topics

| Topic | 用途 |
|-------|------|
| `/parsed_command` | parser / LLM → grounding |
| `/grounded_goal` | grounding → legacy executor |
| `/grounded_task_context` | grounding → `real_grounded_runtime_node` |
| `/runtime/preview` | Runtime preview / waiting confirm |
| `/runtime/confirm` | 用户确认 |
| `/runtime/state` | Task 状态流 |
| `/runtime/log` | 结构化事件日志 |
| `/runtime/execution_result` | 执行结果 |
| `/runtime/verification_result` | precheck / postcheck / post_place |
| `/executor/done` | 执行完成信号 |
| `/world_model/perception_objects` | YOLO + ROI 融合结果 |
| `/world_model/stable_objects` | 稳定世界模型 |
| `/world_model/roi_objects` | ROI 原始检测流 |

---

## Mechanical Arm Topics

| Topic | 用途 | 当前状态 |
|-------|------|----------|
| `/servo_controller` | 上层舵机控制入口 | ✅ `controller_manager` 订阅；Current Runtime 已通过该 Topic 完成真实 Pick；执行前仍需检查多写者 |
| `/grasp` | GraspNode 输入 | ✅ `grasp` 节点单实例；不属于当前 Runtime 主输入，人工发布仍需单独审批 |
| `/controller_manager/servo_states` | 原始舵机状态 | ✅ 已验证 ID `1/2/3/4/5/10` 有数据 |
| `/controller_manager/joint_states` | 关节角状态 | ✅ 已验证 `joint1~joint5`、`r_joint` 有数据 |
| `/ros_robot_controller/bus_servo/set_position` | 底层总线位置命令 | ✅ 当前 `servo_manager ×1` 发布，`ros_robot_controller ×1` 订阅；禁止上层旁路直写 |
| `/ros_robot_controller/bus_servo/set_state` | 底层舵机状态控制 | 🔶 Runtime 审查阶段核对调用者和安全边界 |
| `/joint_controller` | JointState 控制入口 | 🔶 Runtime 审查阶段核对是否属于当前主路径 |
| `/kinematics/set_pose_target` | IK 求解服务 | ✅ `kinematics` 单实例；PC Runtime 跨机真实 IK 已验证 |
| `/kinematics/get_current_pose` | 当前末端姿态服务 | ✅ `kinematics` 单实例；PC 跨机调用已验证 |

当前舵机基线：

```text
ID1=500  ID2=560  ID3=130  ID4=115  ID5=500
ID10=200（open）
ID10=700（close）
```

## Mobile Base Topics

移动底盘当前处于暂停维修状态。

| Topic | 用途 | 当前状态 |
|-------|------|----------|
| `/scan` | RPLidar 激光雷达数据 | 历史验证通过 |
| `/odom_combined` | 底盘里程计 | 需在维修后重新验证 |
| `/tf` | 动态 TF | 历史运行通过 |
| `/tf_static` | 静态 TF | 历史运行通过 |
| `/cmd_vel` | 底盘速度控制接口 | 当前禁止发布 |
| `/mobile_base/sensors/imu_data` | 原始 IMU 数据 | 是否已在底盘驱动内部用于 yaw，需通过源码确认 |

当前禁止：

```text
启动 tank.launch.py
发布 /cmd_vel
继续 AMCL/Nav2 参数调试
```

---

## Navigation Topics

| Topic | 用途 | 状态 |
|-------|------|------|
| `/map` | Occupancy Grid 地图 | ✅ 已生成家庭地图 |
| `/map_metadata` | 地图元信息 | ✅ |
| `/initialpose` | RViz → AMCL 初始位姿 | 历史验证 |
| `/goal_pose` | RViz → Nav2 目标 | 短距离测试过，尚未完整验收 |
| `/amcl_pose` | AMCL 位姿 | 历史处于验证阶段 |
| `/particle_cloud` | AMCL 粒子云 | 历史可用 |
| `/plan` | Nav2 全局路径 | 未完成正式验收 |

当前地图：

```text
~/ros2_ws/maps/home_map_01_260621.yaml
```

---

## Roadmap Summary

| 阶段 | 状态 | 目标 |
|------|------|------|
| Runtime Platform | ✅ HISTORICALLY COMPLETED | 固定机械臂 Runtime 执行系统 |
| RobotOps Foundation | ✅ HISTORICALLY COMPLETED | Runtime 事件持久化 |
| Verification Runtime | 🔶 HARDENING | 已实现并接入状态机；当前修复假阳性和证据可靠性 |
| Mechanical Arm Reactivation | ✅ COMPLETE | 单一控制栈、Dry-run、真实 IK、真实 Servo 与首次真实视觉 Pick 已通过 |
| Retry / Recovery | ⏳ PLANNED | 基于真实失败数据恢复任务 |
| Teleop Arbitration | ⏳ PLANNED | AUTO / MANUAL / PAUSED / ESTOP |
| Data Collection | ⏳ PLANNED | 图像、动作、状态、轨迹和失败数据 |
| VLA Readiness | ⏳ PLANNED | 数据集、回放、VLA Bridge |
| Mobile Robot Foundation | ⏸ PAUSED | 等待底盘维修和运动一致性恢复 |
| Base Driver | ✅ HISTORICALLY VERIFIED | `/cmd_vel` 与底盘驱动 |
| Odometry | 🔶 REQUIRES REVALIDATION | 底盘滑移破坏运动模型 |
| RPLidar | ✅ HISTORICALLY VERIFIED | `/scan` 可用 |
| SLAM Mapping | ✅ | 家庭地图已生成 |
| AMCL / Nav2 | ⏸ BLOCKED | 转弯受底盘打滑影响 |
| Semantic Locations | ⏳ | 底盘恢复后继续 |
| MoveSkill / NavigateSkill | ⏳ | 底盘恢复后继续 |
| Mobile Manipulation | ⏳ | 底盘与机械臂分别稳定后集成 |

---

## How to Recover Project Context

### Phase 1 — Select Documentation by Task

当前任务是首次真实视觉 Pick 之后的稳定化，不再是 Runtime 执行链审查。

必须读取：

1. `runtime_index.md`
2. `CLAUDE.md`
3. `runtime_debug_guide.md`
4. `topic_service_map.md`
5. `runtime_risks.md`
6. 最新 2026-08-02 真实 Pick 日志
7. Verification 源码与测试
8. Fusion / Tracker 源码与参数
9. RobotOps 源码、launch 与 SQLite schema

按需参考：

- `runtime_architecture.md`
- `runtime_task_schema.md`
- `runtime_target_object.md`
- `runtime_skill_interface.md`
- 历史 Hover / Pick / IK / Servo dev log

当前任务不要求：

- 恢复底盘驱动；
- 继续 AMCL / Nav2 参数调试；
- 重构整个 Runtime；
- 在没有明确验证目标时重复真实动作。

### Phase 2 — Inspect Repository and Deployment

```bash
cd ~/ros2_ws
pwd
git status
git branch --show-current
git log -1 --oneline
colcon list

for pkg in sketch_runtime grounding robotops app vision_yolo; do
    ros2 pkg prefix "$pkg" 2>&1 || true
done
```

确认：

```text
Expected source
Observed source
Install prefix
Executable entry point
Launch entry point
Parameter defaults
```

### Phase 3 — Audit Runtime Main Path

重点检查：

```text
real_grounded_runtime_node
TaskContext / TaskState
SkillManager / SkillRegistry
PickSkill
ActionExecutor
RuntimeAdapter
```

必须回答：

```text
当前主入口是什么？
默认是否 dry_run？
真实 IK 与真实 Servo 如何分别启用？
Preview / Confirm 如何进入状态机？
失败、超时、取消如何传播？
是否存在 legacy executor 或直接底层写入旁路？
src、install 与测试是否一致？
```

### Phase 4 — Produce Current Audit Report

当前报告：

```text
runtime_execution_chain_audit_20260730.md
```

报告至少包含：

```text
workspace / branch / package prefix
current executable and launch entries
main Runtime call graph
parameter defaults and safety gates
legacy path findings
direct hardware-write findings
test inventory and results
src/install inconsistencies
minimum proposed changes
dry-run acceptance checks
unresolved questions
```

### Phase 5 — Dry-run Boundary

只允许：

```text
dry_run=true
require_confirm=true
enable_real_servo=false
```

禁止：

```text
真实 Servo
Hover
Pick
直接发布 /servo_controller
直接发布 /grasp
直接写 bus_servo/set_position
启动 ground_executor_node
```

## Current Target

```text
Intelligent Robot Runtime
↓
Single and Safe Mechanical Arm Control Stack
↓
First Real Visual Pick                         ✅
↓
Fixed-Workspace Task Manipulation MVP           🔶 CURRENT
↓
Task MVP 1：Blue Block Sorting                  🔶 CURRENT
↓
Pick + Place + Dual-Area Verification
↓
RobotOps Data Integrity
↓
Retry / Recovery
↓
Teleop Arbitration
↓
Data Collection
↓
VLA Readiness
↓
Autonomous Mobile Manipulation
```

### Immediate Next Step

```text
建立 AprilTag 固定工作区坐标基线
→ 统一 RGB CameraInfo 与权威坐标链
→ 输出蓝色积木 mask / contour / grasp pose
→ 实现 Pick + Place + 双区域 Verification
```
