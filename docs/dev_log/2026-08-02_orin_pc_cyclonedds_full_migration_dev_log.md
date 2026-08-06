# 2026-08-02 Orin / PC 全 Cyclone DDS 切换与验证记录

> **项目**：JetArm ROS 2 机器人开发  
> **日期**：2026-08-02  
> **状态**：✅ 已验证通过  
> **结论**：Orin 主 Bringup、Orin Gemini 相机及 PC 默认 ROS 2 环境均已切换至 Cyclone DDS；跨机 Topic、跨机 Service、关节状态、图像显示及 Orin 重启持久化均验证正常，可以返回主线开发。

---

## 1. 背景与问题

此前系统采用混合 RMW/DDS 架构：

```text
Orin 主栈：Fast DDS
Orin 相机：Cyclone DDS
PC 感知/相机工具：Cyclone DDS
PC Runtime：Fast DDS
```

已观察到：

- Orin 相机到 PC 的图像 Topic 可以正常传输；
- Cyclone DDS 与 Fast DDS 之间的部分普通 Topic 可以发现并通信；
- PC Cyclone DDS 客户端无法稳定调用 Orin Fast DDS 提供的 ROS 2 Service；
- `/kinematics/get_current_pose` 是视觉与机械臂任务链的重要接口，因此成为主线阻塞点。

关键失败链路：

```text
PC Cyclone DDS
    ↓
/kinematics/get_current_pose
    ↓
Orin Fast DDS
    ✗ Service 调用失败
```

为消除跨 DDS 厂商的 ROS 2 Service 互操作问题，本次将 Orin 主栈和 PC 默认 ROS 2 环境统一切换到 Cyclone DDS。

---

## 2. 双机环境

### Orin

```text
主机名：ubuntu
ROS 2：Humble
工作区：/home/ubuntu/ros2_ws
ROS_DOMAIN_ID：23
有线接口：eth0
有线地址：192.168.100.1
相机：Orbbec Gemini
```

### PC

```text
主机名：sundasheng-OMEN
ROS 2：Humble
工作区：/home/sundasheng/ros2_ws
ROS_DOMAIN_ID：23
有线接口：eno1
有线地址：192.168.100.2
```

---

## 3. Orin 环境入口确认

Orin 当前交互式 Shell：

```text
SHELL=/bin/zsh
argv0=zsh
ZDOTDIR=/home/ubuntu
```

当前终端实际加载：

```text
/home/ubuntu/.zshrc
```

机器人主服务定义：

```ini
# /etc/systemd/system/start_app_node.service

[Service]
Type=simple
User=ubuntu
Environment="PULSE_SERVER=unix:/run/user/1000/pulse/native"
ExecStart=/bin/zsh -c 'source /home/ubuntu/.zshrc && ros2 launch bringup bringup.launch.py'
```

因此正式启动链是：

```text
start_app_node.service
    ↓
source /home/ubuntu/.zshrc
    ↓
ros2 launch bringup bringup.launch.py
```

工作区内仍保留：

```text
/home/ubuntu/ros2_ws/.zshrc
/home/ubuntu/ros2_ws/.hiwonderrc
```

原厂 `~/ros2_ws/.zshrc` 中存在：

```zsh
source $HOME/ros2_ws/.hiwonderrc
```

但当前系统没有通过这两个工作区 rc 文件启动。它们属于原厂模板或历史配置，当前真实入口是：

```text
/home/ubuntu/.zshrc
```

---

## 4. 原 Cyclone 配置风险

Orin 原环境中已有：

```zsh
export CYCLONEDDS_URI=file:///etc/cyclonedds/config.xml
```

但该 XML 绑定：

```xml
<NetworkInterfaceAddress>lo</NetworkInterfaceAddress>
<AllowMulticast>false</AllowMulticast>
<Peer address="localhost"/>
```

这意味着该配置只适合本机回环通信：

```text
lo / localhost
```

如果直接将整个 Orin 主栈切换到 Cyclone DDS，并继续使用 `/etc/cyclonedds/config.xml`，PC 将无法跨机发现 Orin 节点。

因此，本次全 Cyclone 测试改用已经验证过有线通信的配置：

```text
/home/ubuntu/ros2_ws/config/cyclonedds/orin_camera_eth0.xml
```

---

## 5. Orin 配置修改

### 5.1 修改前备份

修改前创建了备份：

```text
/home/ubuntu/.zshrc.pre_all_cyclone_20260802_110207
```

该备份不会被系统自动加载，只用于必要时回滚。

### 5.2 当前生效配置

Orin 当前 `/home/ubuntu/.zshrc` 中的关键环境变量：

```zsh
export VERSION="|V1.0.1|App_ROS2_1.0|2024-12-24|"
export CAMERA_TYPE=GEMINI
export MACHINE_TYPE=JETARM_ADVANCED
export ROS_DOMAIN_ID=23
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export HW_WIFI_AP_SSID=$HW_WIFI_AP_SSID
export CYCLONEDDS_URI=file:///home/ubuntu/ros2_ws/config/cyclonedds/orin_camera_eth0.xml
export need_compile=False
```

核心修改：

```zsh
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

以及：

```zsh
export CYCLONEDDS_URI=file:///home/ubuntu/ros2_ws/config/cyclonedds/orin_camera_eth0.xml
```

### 5.3 生效机制

`start_app_node.service` 每次启动时都会：

```zsh
source /home/ubuntu/.zshrc
```

随后由同一个 `ros2 launch bringup bringup.launch.py` 启动的 ROS 2 子进程继承：

```text
ROS_DOMAIN_ID=23
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CYCLONEDDS_URI=file:///home/ubuntu/ros2_ws/config/cyclonedds/orin_camera_eth0.xml
```

ROS 2 节点初始化时加载：

```text
librmw_cyclonedds_cpp.so
libddsc.so
```

而不再加载：

```text
librmw_fastrtps_cpp.so
libfastrtps.so
```

---

## 6. Orin 运行时确认

### 6.1 Kinematics

进程环境确认：

```text
ROS_LOCALHOST_ONLY=0
ROS_DOMAIN_ID=23
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CYCLONEDDS_URI=file:///home/ubuntu/ros2_ws/config/cyclonedds/orin_camera_eth0.xml
```

`/proc/<PID>/maps` 实际加载：

```text
/opt/ros/humble/lib/aarch64-linux-gnu/libddsc.so.0.10.4
/opt/ros/humble/lib/librmw_cyclonedds_cpp.so
```

结论：

```text
Kinematics 已确认实际运行在 Cyclone DDS。
```

### 6.2 ros_robot_controller

`/proc/<PID>/maps` 实际加载：

```text
/opt/ros/humble/lib/aarch64-linux-gnu/libddsc.so.0.10.4
/opt/ros/humble/lib/librmw_cyclonedds_cpp.so
```

结论：

```text
ros_robot_controller 已确认实际运行在 Cyclone DDS。
```

### 6.3 关于“Orin 所有节点”

工程上可以确认：

```text
由 start_app_node.service 本次启动的主 Bringup 已全局配置为 Cyclone DDS。
```

已直接用运行时动态库确认：

- `kinematics`
- `ros_robot_controller`
- Gemini 相机进程

其他由同一 Bringup 启动、且未单独覆盖 `RMW_IMPLEMENTATION` 的 ROS 2 节点会继承 Cyclone DDS 环境。

严格来说，不能把其他独立 systemd 服务、旧终端手动启动的节点或显式覆盖 RMW 的进程自动归入该结论。

---

## 7. PC 默认环境修改

PC 当前默认终端入口：

```text
/home/sundasheng/.bashrc
```

当前关键配置：

```bash
export ROS_DOMAIN_ID=23
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/sundasheng/ros2_ws/config/cyclonedds/pc_camera_eno1.xml
```

PC Cyclone DDS XML：

```text
/home/sundasheng/ros2_ws/config/cyclonedds/pc_camera_eno1.xml
```

绑定：

```text
eno1 / 192.168.100.2
```

并与 Orin：

```text
eth0 / 192.168.100.1
```

进行静态 Peer 通信。

切换后，普通终端直接启动的新 ROS 2 进程默认使用 Cyclone DDS，不再必须套：

```text
scripts/with_cyclone_camera.sh
```

但现有脚本暂时保留，作为显式环境启动方式和故障排查工具。

修改默认 RMW 后，应停止旧 ROS 2 CLI daemon：

```bash
ros2 daemon stop 2>/dev/null || true
```

诊断时推荐：

```bash
export ROS2CLI_DISABLE_DAEMON=1
```

---

## 8. 验证结果

### 8.1 Orin 服务状态

Orin 重启后：

```text
start_app_node.service = active
```

终端环境：

```text
ROS_DOMAIN_ID=23
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CYCLONEDDS_URI=file:///home/ubuntu/ros2_ws/config/cyclonedds/orin_camera_eth0.xml
```

### 8.2 关节状态

执行：

```zsh
export ROS2CLI_DISABLE_DAEMON=1

timeout 10 ros2 topic echo   /controller_manager/joint_states   --once
```

成功收到六个关节状态：

```text
joint1
joint2
joint3
joint4
joint5
r_joint
```

结论：

```text
控制状态 Topic 正常。
```

### 8.3 PC 到 Orin 的 Kinematics Service

PC 直接执行：

```bash
export ROS2CLI_DISABLE_DAEMON=1

timeout 15 ros2 service call   /kinematics/get_current_pose   kinematics_msgs/srv/GetRobotPose   "{}"
```

返回：

```text
success=True
solution=True
```

有效位姿：

```text
position:
  x=0.13161745459080376
  y=0.0
  z=0.09637809332638891
```

结论：

```text
PC Cyclone DDS → Orin Cyclone DDS Service 调用成功。
```

### 8.4 相机跨机帧率

PC 执行：

```bash
timeout 15 ros2 topic hz /depth_cam/rgb/image_raw
```

观测：

```text
约 30.1～30.3 Hz
```

示例：

```text
average rate: 30.235
average rate: 30.170
average rate: 30.282
```

结论：

```text
Orin Gemini 相机到 PC 的跨机图像 Topic 正常。
```

### 8.5 rqt_image_view

PC 直接执行：

```bash
ros2 run rqt_image_view rqt_image_view
```

可以正常显示：

```text
/depth_cam/rgb/image_raw
```

说明 PC 普通终端默认 Cyclone DDS 配置已生效。

---

## 9. 重启持久化验收

Orin 整机重启后再次验证：

- `start_app_node.service` 自动进入 `active`；
- Kinematics 环境仍为 `rmw_cyclonedds_cpp`；
- Kinematics 实际加载 `librmw_cyclonedds_cpp.so` 和 `libddsc.so`；
- `/controller_manager/joint_states` 正常；
- PC 调用 `/kinematics/get_current_pose` 成功；
- PC 接收 RGB 图像约 30 Hz；
- PC 直接运行 `rqt_image_view` 正常。

结论：

```text
Orin 全 Cyclone DDS 配置通过重启持久化验收。
```

---

## 10. 最终架构

```text
Orin
└─ start_app_node.service
   ├─ source /home/ubuntu/.zshrc
   ├─ 主 Bringup：Cyclone DDS
   ├─ kinematics：Cyclone DDS，已确认
   ├─ ros_robot_controller：Cyclone DDS，已确认
   ├─ controller / grasp 等：继承主 Bringup 环境
   └─ Gemini 相机：Cyclone DDS

PC
├─ /home/sundasheng/.bashrc
├─ 默认 ROS 2 RMW：Cyclone DDS
├─ rqt_image_view：Cyclone DDS
├─ ROS 2 CLI：Cyclone DDS
├─ 感知节点：默认 Cyclone DDS
└─ Runtime：新启动进程默认 Cyclone DDS
```

网络链路：

```text
PC 192.168.100.2 / eno1
        ↕
Orin 192.168.100.1 / eth0
```

DDS/RMW：

```text
PC Cyclone DDS ↔ Orin Cyclone DDS
```

---

## 11. 根因结论

本次问题不是：

- Kinematics 节点损坏；
- Service 类型错误；
- ROS Domain 不一致；
- PC 与 Orin 网络不通；
- 相机节点异常。

当前证据支持的根因是：

```text
在当前 ROS 2 Humble、当前 Fast DDS/Cyclone DDS 版本及现有配置组合下，
PC Cyclone DDS → Orin Fast DDS 的 ROS 2 Service Request/Reply 未正常工作。
```

验证矩阵：

| PC 端 | Orin 端 | 通信类型 | 结果 |
|---|---|---|---|
| Fast DDS | Fast DDS | Service | 通过 |
| Cyclone DDS | Fast DDS | Service | 失败 |
| Cyclone DDS | Fast DDS | Topic | 已验证部分链路可通信 |
| Cyclone DDS | Cyclone DDS | Service | 通过 |
| Cyclone DDS | Cyclone DDS | Topic | 通过 |
| Cyclone DDS | Cyclone DDS | RGB 约 30 Hz | 通过 |

注意：

> 该结论只针对当前设备、软件版本和配置，不应扩展为所有 Fast DDS 与 Cyclone DDS 之间的 ROS 2 Service 都不兼容。

---

## 12. 已知警告

两端运行时出现：

```text
NetworkInterfaceAddress: deprecated element
```

该警告来自 Cyclone DDS XML 中的旧字段。

当前已经验证：

- Service 正常；
- Topic 正常；
- 相机约 30 Hz；
- Orin 重启恢复正常。

因此本阶段不修改已验证稳定的 XML。后续可单独安排配置现代化和回归测试。

---

## 13. 回滚

### 13.1 Orin

备份：

```text
/home/ubuntu/.zshrc.pre_all_cyclone_20260802_110207
```

回滚：

```zsh
sudo systemctl stop start_app_node.service

cp -a   /home/ubuntu/.zshrc.pre_all_cyclone_20260802_110207   /home/ubuntu/.zshrc

source /home/ubuntu/.zshrc

sudo systemctl start start_app_node.service
```

### 13.2 PC

PC `.bashrc` 备份路径以实际创建时输出为准，格式类似：

```text
/home/sundasheng/.bashrc.pre_all_cyclone_YYYYMMDD_HHMMSS
```

回滚：

```bash
cp -a   /home/sundasheng/.bashrc.pre_all_cyclone_YYYYMMDD_HHMMSS   /home/sundasheng/.bashrc

ros2 daemon stop 2>/dev/null || true
source /home/sundasheng/.bashrc
```

RMW 不支持运行中动态切换。回滚后必须重新启动相关 ROS 2 进程。

---

## 14. 稳定基线保护规则

当前阶段不要：

- 删除 Orin `.zshrc` 回滚备份；
- 删除 PC `.bashrc` 回滚备份；
- 删除 `with_cyclone_camera.sh`；
- 重命名 `orin_camera_eth0.xml`；
- 为消除弃用警告直接修改当前 XML；
- 再以 Fast DDS 启动新的主线节点；
- 在未验证的情况下同时修改 DDS、网络和业务代码。

判断进程实际 RMW 时，以运行时加载库为准：

```text
Cyclone DDS：
librmw_cyclonedds_cpp.so
libddsc.so

Fast DDS：
librmw_fastrtps_cpp.so
libfastrtps.so
```

运行时证据优先级：

```text
1. /proc/<PID>/maps
2. /proc/<PID>/environ
3. systemd / Launch / 启动脚本
4. 当前终端 printenv
```

---

## 15. 返回主线开发

DDS/RMW 调试已具备退出条件。

主线恢复链路：

```text
相机图像
→ YOLO / ROI
→ Perception Fusion
→ Stable Object Tracker
→ Grounding
→ Kinematics
→ Executor
→ 抓取执行
→ Verification
→ 完成确认 / Retry
```

此前阻塞点：

```text
PC 感知节点
→ /kinematics/get_current_pose
→ 跨 RMW Service 失败
```

当前：

```text
PC Cyclone DDS
→ Orin Cyclone DDS
→ /kinematics/get_current_pose
→ success=True
```

建议主线恢复顺序：

1. 启动 PC 感知栈；
2. 验证 YOLO、ROI、Fusion、Tracker；
3. 确认 `/world_model/stable_objects`；
4. 启动 Grounding 与 Verification；
5. 以安全参数启动 Runtime；
6. 验证真实视觉对象进入 Grounding；
7. 验证 Kinematics 查询；
8. 最后逐级开放真实 IK 与 Servo。

第一轮安全参数：

```text
use_dummy_wm=false
dry_run=true
require_confirm=true
enable_real_ik=false
enable_real_servo=false
```

第一轮目标：

```text
真实视觉对象
→ Stable Objects
→ Grounding
→ Runtime
```

不直接进行实体抓取。

---

## 16. 最终验收结论

### 已确认

- Orin 当前真实环境入口为 `/home/ubuntu/.zshrc`；
- `start_app_node.service` 会加载该文件；
- Orin 主 Bringup 已默认使用 Cyclone DDS；
- Kinematics 实际加载 Cyclone DDS；
- ros_robot_controller 实际加载 Cyclone DDS；
- Orin Gemini 相机使用 Cyclone DDS；
- PC 默认 ROS 2 环境使用 Cyclone DDS；
- PC 可直接运行 `rqt_image_view`；
- PC 可直接调用 `/kinematics/get_current_pose`；
- `/controller_manager/joint_states` 正常；
- RGB 图像跨机传输约 30 Hz；
- Orin 重启后自动恢复全 Cyclone DDS 配置。

### 最终状态

```text
DDS/RMW 调试：CLOSED
通信基线：STABLE
重启持久化：PASS
主线开发：READY
```
