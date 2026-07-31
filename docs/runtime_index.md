# JetArm Robot Runtime — Documentation Index

> 最后更新：2026-07-30
> 长期项目目标：Intelligent Autonomous Mobile Manipulation Robot
> 当前阶段：Fixed-Arm Runtime Reactivation and Revalidation
> 当前活动主线：Runtime / Skill / Action / RuntimeAdapter 审查与 Dry-run 复验
> 已解决 P0：Duplicate Mechanical Arm Control-Stack Instances
> 当前 P0：确认当前分支、install、launch、参数与 Runtime 执行链一致；真实动作继续受控
> 移动底盘状态：PAUSED — 底盘正在检查维修

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
| **Real Arm Execution** | ✅ 历史完成：真实 IK、Servo Adapter、Hover-only、最小完整 Pick |
| **Perception Fusion** | ✅ 历史完成：YOLO + ROI → Perception Fusion |
| **Stable World Model** | ✅ 历史完成：StableObjectTracker |
| **Verification** | ✅ Verification Runtime 已实现并集成状态机 |
| **RobotOps** | ✅ SQLite 事件持久化与任务历史基础已完成 |
| **Retry / Recovery** | ⏳ 尚未实现，计划基于真实失败数据开发 |
| **Control Arbitration** | ⏳ 尚未实现 AUTO / MANUAL / PAUSED / ESTOP 仲裁 |
| **Data Collection / VLA** | ⏳ 轨迹、图像和动作数据采集尚未完成 |
| **Current Phase** | 🔶 Fixed-Arm Runtime Reactivation and Revalidation |
| **Duplicate Control Stack** | ✅ 已解决：核心机械臂节点均为单实例；底层控制端点恢复一对一 |
| **Mechanical Arm Hardware** | ✅ STM32 通信正常；舵机 ID `1/2/3/4/5/10` 在线；反馈与初始姿态正常 |
| **Network / Time** | ✅ `eth-static` 冷启动自动恢复；Jetson 自动从 PC `192.168.100.2` 校时；机器人服务不以网络或时间为启动门禁 |
| **Cross-host DDS** | ✅ PC 已验证可发现完整机械臂节点与 Topic；已定位冷启动接口时序风险，仍需保留启动顺序验收 |
| **Current P0** | 🔶 审查当前 Runtime / Skill / Action / RuntimeAdapter 实现、启动入口、参数默认值、`src/install` 一致性和 legacy 旁路 |
| **Current Real-Motion Gate** | ⛔ 在 Runtime 审查、单元测试、Dry-run 与真实 IK 分级验收完成前，禁止真实 Servo、Hover 与 Pick |
| **Near-Term Outcome** | 恢复单一、安全、可验证的机械臂智能任务执行闭环 |
| **Mobile Base** | ⏸ 底盘维修中，AMCL/Nav2 暂停 |
| **Long-Term Direction** | Autonomous Mobile Manipulation Robot |

## Current Development Strategy

当前工作不是重新设计整个 Runtime，也不是回到底盘继续调参。

当前阶段目标仍然是：

```text
恢复并重新验证固定机械臂 Runtime 的真实执行能力
```

2026-07-30 已完成前三个恢复门槛：

```text
重复控制栈根因与修复              ✅
单一底层机械臂控制栈              ✅
最小硬件、网络、时间与 DDS 基线   ✅
```

当前主线正式进入：

```text
项目长期目标
└── 智能机器人 Runtime
    └── 当前阶段：固定机械臂 Runtime 恢复与复验
        └── 当前 P0：Runtime / Skill / Action / RuntimeAdapter 当前状态审查
            └── 下一门槛：单元测试 + Dry-run 状态流
```

移动底盘继续保持暂停：

```text
不启动 tank.launch.py
不发布 /cmd_vel
不继续 AMCL / Nav2 参数调试
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
- 历史记录中的 `100`、`500` 可能分别表示夹爪闭合和完全打开控制值；
- 在源码审查完成前，不得直接将历史夹爪参数用于真实动作；
- 所有历史参数只能作为代码搜索线索，不能自动视为当前控制事实。

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

重复控制栈问题虽已解决，但真实动作尚未开放。当前仍禁止：

```text
真实 Runtime Pick
直接发布 /servo_controller
发布 /grasp
直接发布 bus_servo/set_position
启动 legacy ground_executor_node
启动 joystick / tracking / sorting / calibration 控制机械臂
dry_run=false
enable_real_servo=true
```

解除顺序必须是：

```text
Runtime 审查
→ 单元测试
→ Dry-run 状态流
→ 真实 IK（禁止 Servo）
→ 最小 Servo 健康测试
→ Hover-only
→ 固定坐标 Pick
```

## Current Control Surface

当前实测运行图中的硬件写入链已经恢复为唯一链路：

```text
/grasp ×1
→ /servo_controller
→ /controller_manager ×1
→ /servo_manager ×1
→ /ros_robot_controller/bus_servo/set_position
→ /ros_robot_controller ×1
→ STM32
```

当前端点：

```text
/servo_controller
Publisher count: 1       node: /grasp
Subscription count: 1    node: /controller_manager
```

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

PC 与 Jetson 使用：

```text
ROS_DOMAIN_ID=23
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

DDS 注意事项：

- ROS 2 进程创建 DDS Participant 时，物理接口是否已建立会影响跨主机发现；
- 冷启动后必须检查 PC 是否直接看到 `/controller_manager`、`/servo_manager`、`/ros_robot_controller`、`/grasp`、`/kinematics`；
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
建立硬件、网络、时间与 DDS 安全基线        ✅
↓
审查 Runtime / Skill / Action / Adapter     🔶 CURRENT
↓
恢复单元测试、Dry-run 与状态流              ⏳
↓
验证真实 IK，但禁止 Servo                   ⏳
↓
最小 Servo 健康测试                         ⏳
↓
Hover-only                                  ⏳
↓
固定坐标 Pick                               ⏳
↓
视觉抓取闭环                                ⏳
↓
Verification 实机验证                       ⏳
↓
RobotOps 数据完整性验证                     ⏳
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

### Current P0 Task — Runtime Execution-Chain Audit and Dry-run Revalidation

| 优先级 | 文件或代码 | 用途 |
|--------|------------|------|
| MUST | `runtime_index.md` | 当前阶段、已完成门槛、停止点和安全边界 |
| MUST | `CLAUDE.md` | Agent 工作规则和硬件禁止事项 |
| MUST | `runtime_task_schema.md` | `TaskContext` / `TaskState` 当前契约 |
| MUST | `runtime_target_object.md` | `TargetObject`、坐标与目标语义 |
| MUST | `runtime_skill_interface.md` | Skill / Action / Adapter 接口设计 |
| MUST | `real_grounded_runtime_node` 源码与入口 | 当前 Runtime 主节点、参数默认值和状态流 |
| MUST | `SkillManager` / `PickSkill` / `ActionExecutor` / `RuntimeAdapter` | 当前主执行链和硬件边界 |
| MUST | `setup.py` / launch / YAML / tests | 真实可执行入口、默认参数和测试覆盖 |
| MUST | 当前 `src` 与 `install` 空间 | 排除代码与部署版本偏差 |
| SHOULD | `runtime_architecture.md` | 对照整体设计意图 |
| SHOULD | `topic_service_map.md` | 对照 Runtime、IK、Servo 和反馈接口 |
| SHOULD | `runtime_risks.md` | 检查旁路、并发、超时、取消和安全风险 |
| SHOULD | 机械臂相关 dev log | 追溯历史 Hover、Pick、IK、Servo 验证条件 |

本任务不执行真实动作。感知、Verification 和 RobotOps 只审查接口与依赖，详细实机验收在后续阶段进行。

### Mechanical Arm Runtime Audit

在单一底层控制栈建立后读取：

| 文件 | 用途 |
|------|------|
| `runtime_task_schema.md` | TaskContext / TaskState |
| `runtime_target_object.md` | TargetObject 和坐标语义 |
| `runtime_skill_interface.md` | Skill / Action / Adapter |
| 机械臂相关 dev log | Hover、Pick、IK、Servo 历史过程 |
| Runtime source / config / tests | 当前实现的最终事实来源 |

### Verification / RobotOps / Retry

按需读取：

| 文件或代码 | 用途 |
|------------|------|
| Verification 相关源码与测试 | 结果确认逻辑 |
| RobotOps 源码与数据库 schema | 事件持久化与任务历史 |
| `runtime_risks.md` | Retry、数据和仲裁风险 |
| 最新真实失败日志 | Recovery 策略设计输入 |

### Mobile Base Reference — Currently Paused

仅在恢复底盘开发时读取：

| 文件 | 用途 |
|------|------|
| `dev_log/2026-07-27_nav2 debug.md` | 底盘暂停原因 |
| `dev_log/2026-07-27_terminal_summary.md` | 当日终端状态 |
| AMCL / Nav2 配置和日志 | 维修完成后重新验证 |

---

## Source of Truth Hierarchy

### Runtime Facts

判断当前机器人实际运行状态时：

1. 当前 ROS 2 node / topic / service / endpoint 查询；
2. 当前 systemd 状态、process tree 与 journal；
3. 当前 `install` 空间中的实际 executable、launch 与配置；
4. 当前 `src` 源码、launch 与配置；
5. 当前自动化测试结果；
6. 当前文档；
7. 历史日志与归档文档。

### Design Intent

判断项目目标架构时：

1. 当前 Git 分支源码；
2. 当前 launch 与 YAML 配置；
3. 当前测试；
4. `runtime_index.md`；
5. `topic_service_map.md`；
6. `runtime_architecture.md`；
7. `jetarm_runtime_roadmap.md`；
8. dev log 与 `docs/archive/`。

### Rules

- 文档是调查入口，不是代码事实。
- 文档中的 `COMPLETED` 表示历史上完成过，不代表当前分支仍可直接复现。
- 文档与源码冲突时，必须明确报告冲突。
- `src`、`install`、systemd 和当前 ROS 2 运行图不一致时，必须同时记录。
- 当前运行关系必须使用 launch、源码、配置、进程树与实机 ROS 2 图联合验证。
- 不得只根据文件名或旧日志推断节点当前仍在使用。
- `runtime_session_summary.md`、`runtime_analysis.md`、`refactor_plan.md` 等归档文件只能用于历史追溯。

---

## Current Runtime Path vs Legacy Path

### Current Runtime Main Path Candidate

```text
/parsed_command
→ grounding_node
→ /grounded_task_context
→ real_grounded_runtime_node
→ SkillManager
→ PickSkill
→ ActionExecutor
→ RuntimeAdapter
→ /kinematics/set_pose_target
→ /servo_controller
→ controller_manager
→ ServoManager
→ ros_robot_controller
→ STM32
→ Servos
```

该链路必须通过当前 launch、源码、配置与运行图重新验证。

### Legacy Execution Path

```text
/grounded_goal
→ ground_executor_node
→ hard-coded execution sequence
→ IK
→ /servo_controller
或直接写入
/ros_robot_controller/bus_servo/set_position
```

审查规则：

- `ground_executor_node` 默认视为 legacy；
- 不得默认它仍由当前 launch 启动；
- 必须检查是否仍有脚本或 launch 启动它；
- 必须确认当前 Runtime 是否完全通过 `RuntimeAdapter` 和 `/servo_controller`；
- 不允许新的 Runtime 代码直接写入底层 `bus_servo` Topic。

---

## Current P0 Task

当前任务范围：审查当前部署中的 Runtime 主路径，并建立可安全执行 Dry-run 的事实基线。

```text
1. 读取 runtime_index.md、CLAUDE.md 与 Runtime 接口文档
2. 确认 PC workspace、Git 分支、package prefix 和 install 来源
3. 定位 real_grounded_runtime_node 的 setup.py / launch / YAML / executable
4. 审查 TaskContext、TaskState、SkillManager、PickSkill、ActionExecutor、RuntimeAdapter
5. 确认参数默认值：
   - dry_run
   - require_confirm
   - enable_real_ik
   - enable_real_servo
   - timeout / run_once / dummy world model
6. 绘制当前主路径：
   /parsed_command
   → grounding
   → /grounded_task_context
   → real_grounded_runtime_node
   → SkillManager
   → ActionExecutor
   → RuntimeAdapter
7. 审查 legacy ground_executor_node 是否仍可被默认 launch 或脚本启动
8. 搜索所有直接写入：
   - /servo_controller
   - /grasp
   - /ros_robot_controller/bus_servo/set_position
9. 核对 src / install / launch / tests / 当前 ROS 图是否一致
10. 运行单元测试；只允许 dry_run=true、enable_real_servo=false
11. 形成审查报告和最小修改建议
12. 停止，等待人工批准后再进入真实 IK 阶段
```

当前交付物：

```text
runtime_execution_chain_audit_20260730.md
```

当前禁止：

```text
真实 Servo
Hover
Pick
/runtime/confirm 触发真实动作
legacy executor
直接写底层总线
```

## Subsequent Mechanical Arm Work Order

当前进度与后续顺序：

```text
1. 修复重复启动                              ✅
2. 验证核心节点均只有一个实例                ✅
3. 建立最小硬件、网络、时间与 DDS 基线       ✅
4. 执行完整 Runtime / Skill / Adapter 审查   🔶 CURRENT
5. 运行单元测试和 Dry-run                    ⏳
6. 验证真实 IK，但禁止 Servo                 ⏳
7. 最小幅度 Servo 健康测试                   ⏳
8. Hover-only                                ⏳
9. 固定坐标 Pick                             ⏳
10. 视觉抓取闭环                             ⏳
11. Verification 实机验证                    ⏳
12. RobotOps 数据完整性验证                  ⏳
13. 记录真实失败模式                         ⏳
14. 设计 Retry / Recovery                    ⏳
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

## Safe Runtime Dry-Run

真实硬件控制修复前，只允许 Dry-run：

```bash
ros2 launch sketch_runtime ground_runtime_bringup.launch.py \
  dry_run:=true \
  require_confirm:=true \
  run_once:=true \
  use_dummy_wm:=true
```

单独启动 Runtime 时：

```bash
ros2 run sketch_runtime real_grounded_runtime_node \
--ros-args \
-p dry_run:=true \
-p require_confirm:=true
```

规则：

```text
不得将 dry_run 改为 false
不得启用 enable_real_servo
不得通过 /runtime/confirm 触发真实动作
```

---

## Hardware Execution Warning

当前代码审查阶段禁止：

- 启动第二套 `jetarm_sdk.launch.py` 或其他底层控制栈；
- 在 `start_app_node.service` 运行时手动启动底层 SDK；
- 设置 `dry_run:=false`；
- 设置 `enable_real_servo:=true`；
- 向 `/servo_controller` 发布控制消息；
- 向 `/grasp` 发布控制消息；
- 向 `/ros_robot_controller/bus_servo/set_position` 发布消息；
- 运行真实 IK + Servo 联动；
- 启动 legacy `ground_executor_node`；
- 使用 joystick、object tracking、object sorting、calibration 等节点控制机械臂。

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

```bash
ros2 launch app perception_bringup.launch.py
```

该 launch 预计启动：

```text
ROI
YOLO
Perception Fusion
StableObjectTracker
Grounding
```

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
| `/servo_controller` | 上层舵机控制入口 | ✅ 当前 `grasp ×1` 发布，`controller_manager ×1` 订阅；真实发布仍禁止 |
| `/grasp` | GraspNode 输入 | ✅ `grasp` 节点单实例；当前禁止人工发布 |
| `/controller_manager/servo_states` | 原始舵机状态 | ✅ 已验证 ID `1/2/3/4/5/10` 有数据 |
| `/controller_manager/joint_states` | 关节角状态 | ✅ 已验证 `joint1~joint5`、`r_joint` 有数据 |
| `/ros_robot_controller/bus_servo/set_position` | 底层总线位置命令 | ✅ 当前 `servo_manager ×1` 发布，`ros_robot_controller ×1` 订阅；禁止上层旁路直写 |
| `/ros_robot_controller/bus_servo/set_state` | 底层舵机状态控制 | 🔶 Runtime 审查阶段核对调用者和安全边界 |
| `/joint_controller` | JointState 控制入口 | 🔶 Runtime 审查阶段核对是否属于当前主路径 |
| `/kinematics/set_pose_target` | IK 求解服务 | ✅ `kinematics` 单实例；真实 IK 尚未进入验收 |
| `/kinematics/get_current_pose` | 当前末端姿态服务 | ✅ `kinematics` 单实例；接口待 Runtime 审查 |

当前舵机基线：

```text
ID1=500  ID2=560  ID3=130  ID4=115  ID5=500  ID10=200
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
| Verification Runtime | ✅ HISTORICALLY COMPLETED | 任务完成状态验证 |
| Mechanical Arm Reactivation | 🔶 CURRENT | 单一控制栈与硬件基础已通过；当前审查 Runtime 主执行链并恢复 Dry-run |
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

当前任务是 Runtime 执行链审查，不再是重复控制栈根因分析。

必须读取：

1. `runtime_index.md`
2. `CLAUDE.md`
3. `runtime_task_schema.md`
4. `runtime_target_object.md`
5. `runtime_skill_interface.md`
6. `real_grounded_runtime_node` 源码与入口
7. `SkillManager`、`PickSkill`、`ActionExecutor`、`RuntimeAdapter`
8. Runtime 相关 launch、YAML、setup.py 和 tests
9. 2026-07-30 固定机械臂恢复调试日志

按需参考：

- `runtime_architecture.md`
- `topic_service_map.md`
- `runtime_risks.md`
- 历史 Hover / Pick / IK / Servo dev log

当前任务不要求：

- 恢复底盘驱动；
- 读取 AMCL / Nav2 详细日志；
- 启动感知全链或执行视觉抓取；
- 运行真实 Servo。

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
Runtime / Skill / Action / Adapter Revalidation
↓
Real Arm Task Execution
↓
Verification
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
Runtime / Skill / Action / RuntimeAdapter Current-State Audit
→ Unit Tests
→ Safe Dry-run State-Flow Revalidation
```
