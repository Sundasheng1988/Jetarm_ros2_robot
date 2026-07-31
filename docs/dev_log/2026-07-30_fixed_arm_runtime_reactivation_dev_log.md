# 2026-07-30 Fixed-Arm Runtime Reactivation Dev Log

> 日期：2026-07-30
> 项目：JetArm Robot Runtime
> 当前阶段：Fixed-Arm Runtime Reactivation and Revalidation
> 本日目标：关闭重复控制栈 P0、恢复冷启动网络与时间、建立机械臂只读健康基线、恢复 PC ↔ Jetson 完整 ROS 2 可见性
> 安全边界：全程禁止真实 Pick、Hover、直接 Servo 发布和底层总线旁路写入

---

## 1. Session Outcome

本日完成：

```text
重复机械臂控制栈修复与单实例验收          ✅
PC ↔ Jetson 固定有线网络                   ✅
Jetson 冷启动自动从 PC 校时                ✅
厂商 Wi-Fi 脚本清空 NetworkManager 配置修复 ✅
机械臂 STM32 / 舵机 / 反馈健康基线          ✅
底层控制 Topic 一对一端点验收              ✅
PC ↔ Jetson 跨主机 DDS 可见性恢复           ✅
```

当前进入：

```text
Runtime / Skill / Action / RuntimeAdapter 当前状态审查
→ 单元测试
→ Dry-run 状态流复验
```

当前仍禁止：

```text
dry_run=false
enable_real_servo=true
真实 IK + Servo 联动
Hover
Pick
发布 /servo_controller
发布 /grasp
直接写 /ros_robot_controller/bus_servo/set_position
启动 legacy ground_executor_node
```

---

## 2. Starting Point

### 2.1 Project State

移动底盘保持暂停：

```text
底盘转弯打滑
→ 差速运动模型失效
→ 激光点云与地图漂移
→ AMCL / Nav2 参数调试暂停
```

当前主线切回固定机械臂 Runtime。

### 2.2 Historical P0

此前 ROS 2 图中存在重复机械臂控制栈：

```text
2 × /ros_robot_controller
2 × /controller_manager
2 × /servo_manager
2 × /grasp
2 × /kinematics
2 × /buzzer_controller
```

风险：

```text
一条上层 Servo 指令
→ 两套 controller_manager
→ 两套 servo_manager
→ 两套 ros_robot_controller
→ 可能重复串口写入、报文交错和不可复现动作
```

---

## 3. Duplicate Control-Stack Resolution

### 3.1 Resolution Strategy

最终保留唯一硬件 bringup 路径：

- `bringup.launch.py` 保留直接 SDK / 硬件控制栈；
- joystick 子 launch 使用 `include_sdk` 条件，默认不再 include 第二套 SDK；
- `joystick_control` 默认关闭；
- vendor demo `start_app.launch.py` 不再作为并行执行入口；
- PC 不启动 Jetson 硬件驱动。

### 3.2 Post-fix Node Graph

最终核心节点：

```text
/ros_robot_controller ×1
/controller_manager ×1
/servo_manager ×1
/grasp ×1
/kinematics ×1
```

运行进程由唯一服务启动：

```text
start_app_node.service
└── ros2 launch bringup bringup.launch.py
    ├── ros_robot_controller
    ├── servo_controller
    │   ├── controller_manager
    │   └── servo_manager
    ├── grasp
    ├── search_kinematics_solutions
    ├── web_video_server
    ├── rosbridge
    └── depth camera container
```

结论：重复控制栈 P0 关闭。

---

## 4. Fixed Ethernet and Automatic Time Synchronization

### 4.1 Target Architecture

```text
PC eno1:      192.168.100.2/24
Jetson eth0:  192.168.100.1/24
Jetson wlan0: 192.168.149.1/24
Jetson AP:    HW-23020672
```

要求：

- 机器人热点始终默认广播；
- `start_app_node.service` 无网络、无 PC、时间为 1970 时也必须启动；
- 不增加网络或时间同步门禁；
- PC 和网线存在时，Jetson 自动从 PC 校时；
- 有线连接不得抢占 PC 的互联网默认路由。

### 4.2 PC Chrony

PC 安装并启用 Chrony，允许机器人网段请求：

```conf
allow 192.168.100.0/24
allow 192.168.149.0/24
local stratum 10
```

PC 有线 Profile：

```text
192.168.100.2/24
无 gateway
无 DNS
never-default=yes
```

Chrony 客户端记录确认 Jetson 请求正常：

```text
192.168.100.1  NTP requests increasing
Drop = 0
```

### 4.3 Jetson timesyncd

配置：

```ini
[Time]
NTP=192.168.100.2 192.168.149.2 0.pool.ntp.org 1.pool.ntp.org
PollIntervalMinSec=16
PollIntervalMaxSec=64
```

同步验收：

```text
System clock synchronized: yes
NTP service: active
Server: 192.168.100.2
```

### 4.4 Robot Service Independence

`start_app_node.service` 当前只有顺序声明：

```ini
After=NetworkManager.service time-sync.target
```

没有 `Wants=` / `Requires=` 网络或时间依赖，也没有 `systemd-time-wait-sync` 门禁。

结论：校时失败不会阻止机器人本地启动。

---

## 5. Cold-boot Ethernet Failure Investigation

### 5.1 First Failure

普通重启后曾出现：

```text
PC eno1 = 192.168.100.2/24
carrier = 1
route correct
ping 192.168.100.1 → Destination Host Unreachable
SSH → No route to host
```

通过热点进入 Jetson 后发现：

```text
eth0 disconnected
Wired connection 1: ipv4.method=auto
```

NetworkManager 日志反复显示：

```text
dhcp4 beginning transaction
→ 45 s timeout
→ ip-config-unavailable
→ retry
```

直连网络没有 DHCP 服务器，因此默认 Profile 无法获得地址。

### 5.2 Temporary Static Profile

创建：

```text
eth-static
192.168.100.1/24
manual
autoconnect=yes
never-default=yes
```

普通重启可以恢复，但彻底断电后该 Profile 消失。

---

## 6. Vendor Wi-Fi Manager Root Cause

### 6.1 Evidence

厂商服务：

```text
wifi.service
ExecStart=/home/ubuntu/wifi_manager/wifi.py
```

脚本 `disconnect()` 包含：

```python
os.system('rm /etc/NetworkManager/system-connections/*')
```

该命令在每次 Wi-Fi 管理器启动时清空所有 NetworkManager Profile，不只删除 Wi-Fi，也删除：

```text
eth-static.nmconnection
Wired connection 1.nmconnection
其他保存的 Wi-Fi Profile
```

这解释了：

```text
当前运行中 eth-static 可用
→ wifi.py 已将磁盘 Profile 删除
→ 下一次彻底冷启动找不到 eth-static
→ NetworkManager 自动创建 DHCP Profile
```

### 6.2 Fix

备份：

```text
/home/ubuntu/wifi_manager/wifi.py.backup_20260730
```

删除全目录清空逻辑，同时保留厂商 AP 创建流程。

修复后重新创建 `eth-static`，并关闭默认 DHCP Profile 自动连接。

### 6.3 Cold-boot Acceptance

彻底断电后重新上电，验收结果：

```text
eth-static Profile 仍存在
eth-static 自动绑定 eth0
eth0 = 192.168.100.1/24
PC 可直接 SSH
Jetson 可 ping 192.168.100.2
HW-23020672 正常广播
System clock synchronized: yes
Server: 192.168.100.2
wifi.service active
systemd-timesyncd active
start_app_node.service active
```

结论：有线持久化与自动校时冷启动验收通过。

---

## 7. Mechanical-Arm Hardware Health Baseline

### 7.1 Service and Process State

```text
start_app_node.service = active
```

唯一进程：

```text
ros_robot_controller ×1
controller_manager ×1
servo_manager ×1
grasp ×1
kinematics ×1
```

### 7.2 Serial Device

当前 STM32 串口：

```text
/dev/ttyUSB0 -> ttyCH341USB0
USB VID:PID = 1a86:7523
QinHeng CH340 serial converter
```

当前没有 `/dev/serial/by-id/` 稳定路径。

开放风险：USB 编号仍可能随插拔顺序变化，后续应评估 udev 稳定命名。

### 7.3 Servo Discovery

启动日志检测到：

```text
Servo ID: 1
Servo ID: 2
Servo ID: 3
Servo ID: 4
Servo ID: 5
Servo ID: 10
```

### 7.4 Servo Feedback

`/controller_manager/servo_states`：

| ID | Position | Error |
|---:|---------:|------:|
| 1 | 500 | 0 |
| 2 | 560 | 0 |
| 3 | 130 | 0 |
| 4 | 115 | 0 |
| 5 | 500 | 0 |
| 10 | 200 | 0 |

前五轴与历史 Home 一致：

```python
home_pulses = [500, 560, 130, 115, 500]
```

ID10 当前开机反馈为 `200`。历史夹爪值 `100` / `500` 仍只作为源码搜索线索，不作为当前真实动作参数。

### 7.5 Joint Feedback

```text
joint1
joint2
joint3
joint4
joint5
r_joint
```

实测 position：

```text
0.0
0.2513274123
-1.5498523758
-1.6126842288
0.0
-2.0943951024
```

### 7.6 Control Endpoint Baseline

`/servo_controller`：

```text
Publisher:    /grasp ×1
Subscription: /controller_manager ×1
```

`/ros_robot_controller/bus_servo/set_position`：

```text
Publisher:    /servo_manager ×1
Subscription: /ros_robot_controller ×1
```

结论：当前底层控制链唯一，未发现 joystick、tracking、sorting、calibration 等意外运行发布者。

---

## 8. Cross-host DDS Discovery Timing

### 8.1 Symptom

冷启动后：

- Jetson 本机进程树与节点证明机械臂控制栈已经运行；
- PC 初始只能看到较晚启动的 rosbridge / camera；
- PC 看不到 `/controller_manager`、`/servo_manager`、`/ros_robot_controller`、`/grasp`、`/kinematics`；
- `ros2 --no-daemon` 结果相同，排除 CLI daemon 缓存为主因。

### 8.2 Monotonic Timing Evidence

启用 `ignore-carrier=yes` 后：

```text
13.804 s  eth-static Activation successful
14.246 s  ros_robot_controller / servo_controller / grasp / kinematics process started
15.934 s  eth0 carrier: link connected
```

结论：静态 IP 已提前配置，但机械臂节点仍在物理载波建立前创建 DDS Participant。

### 8.3 Probe Test

载波建立后手动创建无发布、无硬件控制的 ROS 2 probe：

```text
/dds_post_carrier_probe
```

PC 可以立即发现。

### 8.4 Stop/Start Control Test

载波已建立后执行：

```bash
sudo systemctl stop start_app_node.service
sudo systemctl start start_app_node.service
```

随后 PC 立即看到完整节点与 Topic：

```text
/controller_manager
/grasp
/kinematics
/ros_robot_controller
/servo_manager
/controller_manager/servo_states
/controller_manager/joint_states
/servo_controller
/ros_robot_controller/bus_servo/set_position
```

结论：

```text
DDS、ROS_DOMAIN_ID、RMW、路由和防火墙本身正常
根因是冷启动时机械臂节点早于物理 Ethernet carrier 创建 Participant
```

### 8.5 Engineering Constraint

不得采用永久网络门禁：

```text
不等待 PC ping
不等待 NTP
不依赖 network-online.target
无网线时不能阻止机器人启动
```

若固化启动顺序，只能使用：

```text
短时等待 eth0 carrier
→ carrier 就绪立即启动
→ 超时后无条件继续启动
```

当前 PC ↔ Jetson 完整 DDS 可见性已验证；后续冷启动仍需保留验收检查。

---

## 9. Non-blocking Findings

### 9.1 Orbbec Logger

相机日志出现：

```text
Error creating file sink for logger
Failed opening file Log//OrbbecSDK.log.txt
```

但相机成功连接并发布 RGB、Depth、IR 和 PointCloud，当前不构成机械臂 Runtime P0。

### 9.2 ROS Graph Convergence

Jetson 本机 `ros2 node list` 曾短暂只显示少量节点，但 Topic 图和随后查询恢复完整。当前按 DDS 图收敛延迟处理，不作为进程退出证据。

---

## 10. Final Runtime Baseline

### Robot-side Nodes

```text
/controller_manager
/depth_cam/camera_container
/depth_cam/depth_cam
/grasp
/kinematics
/rosapi
/rosapi_params
/rosbridge_websocket
/ros_robot_controller
/servo_manager
/web_video_server
```

### Core Topics

```text
/controller_manager/joint_states
/controller_manager/servo_states
/grasp
/joint_controller
/ros_robot_controller/bus_servo/set_position
/ros_robot_controller/bus_servo/set_state
/servo_controller
/tf
/tf_static
```

### Infrastructure

```text
Jetson eth0 = 192.168.100.1/24
PC eno1 = 192.168.100.2/24
HW hotspot = 192.168.149.1/24
Jetson NTP server = 192.168.100.2
start_app_node.service = active
```

---

## 11. Current Task Tree

```text
修复重复启动                              ✅
验证核心节点均只有一个实例                ✅
建立最小硬件、网络、时间与 DDS 基线       ✅
执行完整 Runtime / Skill / Adapter 审查   🔶 CURRENT
运行单元测试和 Dry-run                    ⏳
验证真实 IK，但禁止 Servo                 ⏳
最小幅度 Servo 健康测试                   ⏳
Hover-only                                ⏳
固定坐标 Pick                             ⏳
视觉抓取闭环                              ⏳
Verification 实机验证                     ⏳
RobotOps 数据完整性验证                   ⏳
记录真实失败模式                          ⏳
设计 Retry / Recovery                     ⏳
```

---

## 12. Immediate Next Step

审查范围：

```text
real_grounded_runtime_node
TaskContext
TaskState
SkillManager
SkillRegistry
PickSkill
BaseAction / MoveAction / GripperAction
ActionExecutor
RuntimeAdapter
```

必须确认：

```text
当前 package prefix 与 install 来源
真实 executable / launch 入口
src 与 install 一致性
参数默认值：dry_run / require_confirm / enable_real_ik / enable_real_servo
主路径是否只通过 RuntimeAdapter
legacy ground_executor_node 是否仍可默认启动
是否存在直接写 /servo_controller、/grasp 或 bus_servo 的旁路
单元测试库存与结果
Dry-run 状态流是否可复现
```

当前交付物：

```text
runtime_execution_chain_audit_20260730.md
```
