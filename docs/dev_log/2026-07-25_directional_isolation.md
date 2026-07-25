# Directional Isolation & LaserScan Independent Verification (2026-07-25)

> **阶段**: ACTIVE_DIRECTIONAL_ISOLATION — 方向性运动隔离与 LaserScan 独立几何验证
> **版本**: v1
> **上一份日志**: [2026-07-17~18 Localization Debug](./2026-07-17_18_localization_debug.md)

---

## 1. 今日目标

延续 07-17~18 的 yaw 链路排查，本轮不再依赖 Odom/IMU 自报值，转而用 LaserScan 对真实 SE(2) 运动做独立几何验证，并隔离顺时针 (CW) 与逆时针 (CCW) 两个方向的响应差异。

核心问题：
- 相同绝对值角速度命令下，CW 与 CCW 的实际运动是否对称？
- Odom/IMU/TF 链报告的 yaw 与真实几何 yaw 是否一致？
- Odom 记录的平移是否反映真实平移？

ROS 方向约定：`angular.z < 0` 为 CW，`angular.z > 0` 为 CCW。

> **运行真值边界**：移动底盘三包（`turn_on_dlrobot_robot`、`rplidar_ros`、`mobile_base_bridge`）只在 Jetson 上运行，PC 端副本无运行功能。本日志中"运行时观察"以 Jetson 上录制的 rosbag 为准；"源码审计"以 Jetson `/home/ubuntu/ros2_ws/src` 为准。PC 端 launch/源码不作为运行事实引用。

---

## 2. 静止状态 TF 与传感器基线测试

### 2.1 测试目的

在机器人完全静止、不发 `/cmd_vel` 时录包，用于：
- 检查 TF 在无运动时是否漂移或断流；
- 检查 Odom pose 在静止时是否漂移；
- 检查 IMU orientation / gyro 在静止时是否稳定；
- 检查 LaserScan 静态环境是否稳定；
- 区分"静态链路自身异常"与"旋转运动触发异常"，为后续 E01 动态实验建立基线。

### 2.2 测试环境与数据集

本轮基线测试包含两个静态 bag，证据来源于各自 `metadata.yaml`、`ros2 bag info` 输出与 `analysis/experiment_summary.md`：

| 数据集 | 录制时间 | 时长 | 录制端 | 路径 |
|--------|----------|------|--------|------|
| E00_static | 2026-07-24 ~22:57（分析完成于 2026-07-25） | ~4.995 min | PC 单端 | `test_logs/localization_isolation_v1_20260724/E00_static` |
| E00B_static_dual_01 | 2026-07-25 08:48:31→08:53:31 (+08:00) | jetson 299.119s / pc 299.628s | Jetson 与 PC 双端同步 | `test_logs/.../E00B_static_dual_01/{jetson,pc}` |

- 机器人完全静止，不发 `/cmd_vel`。
- ROS_DOMAIN_ID=23，RMW=rmw_fastrtps_cpp（项目约定）。
- E00B 录包起止时间来自 `jetson_record_start.txt` / `jetson_record_end.txt` / `pc_record_start.txt` / `pc_record_end.txt`。
- 当时是否启动 SLAM/AMCL/Nav2、具体启动了哪些驱动节点：**未从当前可用记录恢复，待补充**（bag 中未见 SLAM/AMCL/Nav2 相关话题，但启动清单未存盘）。

### 2.3 录制话题（Runtime Observation，来自 bag）

E00B Jetson bag 实际录制话题与消息数（`jetson_bag_info.txt`）：

| 话题 | 类型 | Jetson count | PC count |
|------|------|--------------|----------|
| /tf_static | tf2_msgs/msg/TFMessage | 1 | 1 |
| /mobile_base/sensors/imu_data | sensor_msgs/msg/Imu | 5964 | 5971 |
| /odom_combined | nav_msgs/msg/Odometry | 5964 | 5970 |
| /robotvel | dlrobot_robot_msg/msg/Data | 5964 | 5968 |
| /tf | tf2_msgs/msg/TFMessage | 5949 | 5957 |
| /scan | sensor_msgs/msg/LaserScan | 2210 | 2221 |

- **`/cmd_vel` 与 `/robotpose` 未出现在 E00B 静态 bag 中**（静态不发命令；`/robotpose` 虽由驱动发布但本 bag 未录制）。
- E00_static bag 话题集合相同（`/odom_combined` 5974、`/imu_data` 5973、`/robotvel` 5966、`/scan` 2232、`/tf` 4631、`/tf_static` 1）。

### 2.4 TF 检查（Runtime Observation，直接读 bag 的 TFMessage）

从 E00B Jetson bag 直接反序列化 `/tf` 与 `/tf_static`（只读 `mode=ro`）：

- `/tf_static`：恰好 **1 条**静态 TF，`base_footprint -> laser`，translation=(0.0000, 0.0000, 0.1500)，旋转为单位四元数，`conflicting_static_transforms=0`。
- `/tf` 动态流：**唯一一条 edge** `odom_combined -> base_footprint`，共 5949 条；每条 TFMessage 仅含 1 个 transform，`other_edges=none`。
- `/odom_combined`：`header.frame_id=odom_combined`，`child_frame_id=base_footprint`。
- `/scan`：`header.frame_id=laser`，`angle_min≈-3.1241`，`angle_max≈3.1416`，`ranges` 长度 1080（≈360°，~0.33°/beam）。

运行时 TF 树：`odom_combined → base_footprint`（动态 `/tf`）`→ laser`（静态 `/tf_static`）。无其他动态 edge、无静态冲突。

### 2.5 测试结果

**静态 yaw 漂移（E00_static，~5 min，来自 `experiment_summary.md` §6 "Yaw source alignment"）**：

| 源 | yaw Δ (°) | drift (°/min) | 相对 IMU 残差 max (°) |
|----|-----------|---------------|------------------------|
| imu_orientation | 2.4834 | 0.4972 | 0.0000（参考） |
| odom_pose | 2.4834 | 0.4972 | 0.0023 |
| tf_odom_to_base | 2.4198 | 0.4926 | 0.0015 |
| robotvel_integrated | 0.0000 | 0.0000 | -2.5492（median -3.7162） |

- **Confirmed Fact**：静止约 5 分钟内 IMU yaw 漂移约 **+2.48°（~0.50°/min）**，并非为零。
- **Confirmed Fact**：`odom_pose` 与 `tf` 的 yaw 与 `imu_orientation` 残差均 <0.0023°，即静止时 Odom yaw 与 TF yaw 是 IMU yaw 的复制值（同一链路），不构成独立证据。
- **Confirmed Fact**：`robotvel_integrated` 静止时 yaw Δ=0（轮速为零→积分不变），但与 IMU 残差约 -3.7°，说明 robotvel 积分 yaw 与 IMU yaw 不是同一来源。

**静态 Scan 稳定性（E00_static，`experiment_summary.md` §7 "Scan wall fit"）**：

- 2232 帧 scan，`valid_fits=2232`，`failed=0`。
- `wall_angle_laser_deg`：median=3.2503，mad=0.9885，p95=4.7976，max=5.1414。
- `wall_angle_odom_deg`：median=6.8610，mad=1.0366，p95=8.8300，max=9.9749。
- `scans_with_tf_lookup_out_of_tol=393`（TF 查询超差，与下方 TF 交付异常一致）。
- **Inference**：静态环境下 laser 墙角中位数稳定在 ~3.25°（mad~1°），scan 本身可用；但 393 帧 TF 查询超差会污染依赖 tf2 的 scan 处理。

**TF 交付稳定性**：

- E00_static（PC 单端）：`/tf` header max gap ≈ **3.0 s**，bag-ts max gap ≈ **5.6 s**，`odom_missing_TF=1353`（`experiment_summary.md` §2/§3、`E00_REVIEW.md`）。
- E00B 双端（`E00B_REVIEW.md`）：共同 header-stamp 区间 289.296 s。
  - Jetson 端：`odom_missing_TF=0`，odom/TF max gap ≈ **0.054 s**（≈20 Hz，无断流）。
  - PC 端：`odom_missing_TF=33`，TF max gap ≈ **1.850 s**。
  - 跨端：`target TF only_on_jetson=36`（PC 缺 36 条），集中在 `1784940655.528 → 1784940657.277` 约 1.75 s 窗口。
  - `bag_ts` diff (pc−jetson)：median ≈ 0.044 s，p95 ≈ 0.16 s，max ≈ 0.57 s（含跨主机钟偏）。
- **Confirmed Fact**：`odom_combined → base_footprint` TF 在 Jetson 本地发布稳定（无缺失、~20 Hz）；PC bag 中的 TF 缺失/大间隙属于 **PC 端接收/记录侧缺失，而非 Jetson 发布端缺失**。
- **Unknown**：缺失具体发生在 DDS writer / Wi-Fi / PC DDS reader / PC `ros2 bag recorder` 哪一环，尚不能定位。
- **Unknown**：该 TF 交付异常与历史 ~90° 地图/scan 旋转的因果关系未建立。

### 2.6 对后续 E01 的意义

- 静止时 IMU/Odom/TF 链虽存在 ~0.5°/min 的 yaw 漂移与 PC 侧 TF 间歇缺失，但未出现持续旋转或跳变；因此 E01 中观测到的大幅 yaw/平移异常主要由运动过程触发，而非静态链路自激。
- 静态稳定**不能**证明动态旋转时正确；故后续设计 E01 动态 yaw 实验与 LaserScan 独立几何验证。
- PC 侧 TF 缺失被识别为独立运行时风险，但本日志不据此宣布历史 90° 根因。

---

## 3. Jetson 运行节点、话题和 TF 拓扑审计

> 本节严格区分 **A. 运行时观察（Runtime Observation）** 与 **B. 源码审计（Source Audit）**。运行时观察来自 Jetson 录制的 rosbag 与本日此前已保存的 Jetson SSH 运行时审计记录；源码审计来自 Jetson `/home/ubuntu/ros2_ws/src`（详见第 7 节）。PC 端 launch/源码不作为运行事实。
>
> **证据来源声明**：以下 `/tf`、`/tf_static` 发布者拓扑、TF 频率与 `tf2_echo` 结果均来源于本日此前已保存的 SSH 运行时审计记录（Jetson），本日志复用，**未在本次日志撰写时重新执行这些 CLI 命令**。

### 3.1 SSH 与运行环境

- SSH 目标：`ubuntu@172.20.10.2`，Jetson workspace `/home/ubuntu/ros2_ws`。
- ROS_DOMAIN_ID=23，RMW=rmw_fastrtps_cpp（项目约定）。
- 完整 `ros2 node list` 未保留，但关键 `/tf` 与 `/tf_static` 发布者拓扑、TF 频率及 `tf2_echo` 结果已从此前 SSH 运行时审计记录恢复（见 3.4–3.5）。实际 source 的 ROS 环境与具体 launch 清单：未从当前可用记录恢复，待补充。

### 3.2 实际运行节点（Runtime Observation + Source Audit）

节点—话题归属结合"运行时发布者拓扑（SSH `ros2 topic info -v`）"与"Jetson 源码审计（第 7 节）"两证：

| 节点（运行时观察确认） | 订阅 | 发布 | 发布 TF | 职责 |
|--------------------------|------|------|---------|------|
| `dlrobot_robot`（turn_on_dlrobot_robot） | `/cmd_vel` | `/odom_combined`、`/robotvel`、`/mobile_base/sensors/imu_data`、`/robotpose` | `/tf`（休眠，见 3.5） | 串口底盘驱动：cmd_vel 打包下发 STM32，解析 STM32 回传，构造 Odom/IMU/robotvel |
| `odom_tf_bridge_node`（mobile_base_bridge） | `/odom_combined` | `/tf`（`odom_combined→base_footprint`，活跃） | 是（活跃） | 将 `/odom_combined` 桥接为 TF |
| `rplidar_node`（rplidar_ros） | — | `/scan`（frame_id=`laser`） | 否 | 雷达扫描发布 |
| `static_transform_publisher`（`tf2_ros`） | — | `/tf_static`（`base_footprint→laser`） | 是（静态） | 发布雷达安装静态 TF |

- 运行时 `ros2 topic info -v` 确认：`/odom_combined` Publisher count=1（`dlrobot_robot`），Subscription count=1（`odom_tf_bridge_node`）；`/scan` Publisher 为 `rplidar_node`。

### 3.3 运行时数据链

综合 bag 证据与第 7 节源码审计，链路如下：

```
/cmd_vel
  → dlrobot_robot_node (Cmd_Vel_Callback: linear.x/y/angular.z 各 *1000 → signed int16)
  → 串口 (/dev/ttyACM0, 115200) → STM32
  → STM32 回传 24 字节帧
  → dlrobot_robot_node 解析 (Odom_Trans: rx[2..7] /1000)
  → /robotvel (Robot_Vel.X/Y/Z)
  → /odom_combined (Robot_Pos 由 imu_yaw_unwrapped_ 作 yaw + 轮速积分 x/y)
  → odom_tf_bridge_node
  → /tf: odom_combined → base_footprint

rplidar 驱动 → /scan (frame_id=laser)
/tf_static: base_footprint → laser (t=[0,0,0.150])
```

- 运行时 `/odom_combined` 的 `header.frame_id=odom_combined`、`child_frame_id=base_footprint`（bag 直读）。
- 运行时 `/scan` 的 `frame_id=laser`（bag 直读）。

### 3.4 TF 发布责任边界（Runtime Observation，来自此前 SSH 审计记录）

- `/tf` 发布者拓扑：`ros2 topic info /tf --verbose` 显示 **Publisher count = 2**，分别为 `dlrobot_robot` 与 `odom_tf_bridge_node`。
- `/tf_static` 发布者拓扑：`ros2 topic info /tf_static --verbose` 显示 **Publisher count = 1**，为 `static_transform_publisher`。
- `odom_combined → base_footprint`（动态 `/tf`）：实际活跃发布者为 `odom_tf_bridge_node`（mobile_base_bridge）；`dlrobot_robot` 虽注册了 `/tf` publisher，但属休眠发布者（见 3.5）。
- `base_footprint → laser`（静态 `/tf_static`）：发布者为 `static_transform_publisher`（Runtime Observation，不再列为 Unknown）；运行时 bag 直读 1 条、translation=[0,0,0.150]、旋转 identity、无冲突。
- `/odom_combined` 消息：由 `dlrobot_robot` 发布（`ros2 topic info /odom_combined -v`：Publisher count=1）。
- `/mobile_base/sensors/imu_data` 消息：由 `dlrobot_robot` 发布（第 7 节源码审计）。

### 3.5 TF 频率、tf2_echo 与休眠发布者

- `/tf` 实测频率约 **19.998 Hz**，接近 `/odom_combined` 的 20 Hz，而**不是**双路活跃发布时预计的约 40 Hz → 印证只有一路（`odom_tf_bridge_node`）在活跃发布 `odom_combined→base_footprint`。
- `tf2_echo` 结果（此前 SSH 审计记录）：
  - `odom_combined → base_footprint`：连续稳定，无多权威抖动。
  - `base_footprint → laser`：直接静态 TF，translation=[0,0,0.15]，rotation=identity。
  - `base_footprint → base_link`：**不存在**（运行时 TF 树中无 `base_link`）。
  - `map → odom_combined`：**不存在**，因当时 AMCL 未运行（E00/E01 阶段 AMCL 关闭）。
- 休眠发布者（结合 Jetson 源码）：`dlrobot_robot` 中 `TransformBroadcaster` 虽被构造（故会注册 `/tf` publisher），但其 `sendTransform` 调用处于禁用/注释状态，因此它是**休眠 publisher**；`odom_tf_bridge_node` 才是实际活跃的 `odom_combined → base_footprint` TF 发布者。故运行时不存在两个节点同时活跃发布同一条 TF，与 bag 中 `real_conflict=0`、`duplicate_stamps=0` 一致。

### 3.6 运行时观察与源码审计的关系

- **Runtime Observation**：通过 Jetson 录制的 rosbag 与此前 SSH 审计记录确认的话题集合、消息 frame_id、TF edge、`/tf`(`/tf_static`) 发布者拓扑、TF 频率与 `tf2_echo` 结果（见第 2 节与本节 3.2–3.5）。
- **Source Audit**：从 Jetson 实际运行源码确认的 cmd_vel 打包、串口发送、反馈解析、Odom 构造逻辑，以及 `dlrobot_robot` 中 `TransformBroadcaster` 已构造但 `sendTransform` 禁用/注释的休眠状态（详见第 7 节与 3.5）。
- 不得将"源码中存在某段代码"自动写成"运行时一定执行"。本节中"`dlrobot_robot` 为休眠 `/tf` publisher"的结论由"源码 `sendTransform` 被注释"+"运行时 `/tf` 频率 ~20 Hz 而非 ~40 Hz"+"bag 无冲突"三证共同支撑。

### 3.7 本轮审计的结论边界

- ROS 端正负 `angular.z` 打包逻辑对称（第 7 节 Confirmed Fact）。
- ROS 端不进行左右轮速度分解（四轮分解在 STM32 固件，不在本仓库）。
- `/odom_combined` 消息与 `odom_combined→base_footprint` TF 属复制关系（TF 由桥接节点从 `/odom_combined` 派生），不构成两个独立位姿证据。
- IMU orientation → Odom pose yaw → TF yaw 为同一复制链；静止残差 <0.0023° 已佐证（第 2.5 节）。
- STM32 之后的轮速分配、PID、PWM、编码器闭环超出当前 ROS 仓库可见范围，尚不能把具体异常归因于某个 STM32 参数或硬件部件。

---

## 4. E01 方向性实验回顾

- **Rosbag**（Jetson 本地，主证据）：`test_logs/localization_isolation_v1_20260724/E01_dynamic_yaw_90_01/jetson`
- **分析目录**：`test_logs/localization_isolation_v1_20260724/E01_dynamic_yaw_90_01/analysis`
- **确认窗口**：`analysis/e01_event_windows_confirmed.csv`
- **Phase 1 脚本**：`tools/analyze_e01_dynamic_yaw.py`
- **Phase 2A 实际脚本**：`tools/analyze_e01_scan.py`
- **Phase 2A 输出**：`analysis/phase2a`

实验序列（原地低速旋转，地面 0°/+90°/−90° 标记）：

| 段 | 动作 | 方向 | angular.z |
|----|------|------|-----------|
| S0 | 初始静止 | — | 0 |
| R1 | 转 ~+90° | CCW | > 0 |
| S1 | 静止 | — | 0 |
| R2 | 回 ~0° | CW | < 0 |
| S2 | 静止 | — | 0 |
| R3 | 到 ~−90° | CW | < 0 |
| S3 | 静止 | — | 0 |
| R4 | 回 ~0° | CCW | > 0 |
| S4 | 最终静止 | — | 0 |

Bag 基础数据：时长 ~385.02s；`/odom_combined`、`/mobile_base/sensors/imu_data`、`/robotvel` 各 7573；`/scan` 2939；`/tf` 7594；`/tf_static` 1；`/cmd_vel` 1384。

---

## 5. Phase 1 结果（动态 Yaw 与 Odom）

Step A/B 均通过（py_compile=0, import=0, audit=0, metrics=0），`event_window_valid=true`，无 correction nudge、无 standalone nudge。

识别到的四段主旋转（相对 bag 起始秒）：

| 段 | 起止 (s) | 方向 | 持续 (s) | Odom/IMU yaw Δ |
|----|----------|------|----------|----------------|
| R1 | 62.497→112.559 | CCW | 50.062 | +91.09° |
| R2 | 160.236→179.837 | CW | 19.601 | −84.89° |
| R3 | 217.586→237.787 | CW | 20.201 | −84.77° |
| R4 | 269.506→317.469 | CCW | 47.963 | +93.57° |

四段 cmd / IMU gyro / Odom yaw 方向均一致；S0–S4 静止窗口有效。

**关键方向性现象**：两次 CCW（R1/R4）约 90° 各需 ~48–50s，两次 CW（R2/R3）约 90° 各需 ~20s。相同绝对值 cmd 角速度下，CW 与 CCW 响应严重不对称，且同方向两次结果彼此接近（可重复）。

静态平台 yaw 中位数：S0=1.155°, S1=94.915°, S2=1.723°, S3=−89.035°, S4=7.969°。
回零残差：S0→S2 ≈ +0.57°；S0→S4 ≈ +6.81°。
最终 Odom 平移：dx≈+0.0061m, dy≈+0.0011m, 模长≈0.0062m；RobotVel 参考轨迹模长≈0.0111m。

**证据边界**：IMU orientation → Odom pose yaw → TF yaw 属同一复制链，不构成三个独立位姿证据。Odom/TF 一致仅证明复制链通畅，不证明真实运动正确。该复制关系在静止基线（第 2.5 节）已进一步佐证。

---

## 6. Phase 2A：S0→S4 LaserScan 独立配准

仅分析 S0 初始平台 → S4 最终平台。变换约定 `p_S0 = T_S0_S4 * p_S4`，主结果**未使用 Odom/IMU 作为配准初值**（独立粗到细 SE(2) 搜索 + ICP 精化）。

执行：all/odd/even 三组 Scan 子集、正反双向配准、inverse consistency、子集稳定性。

aggregate 结果（中位数）：
- dx = −0.6552 m
- dy = −0.0611 m
- 平移模长 = 0.6581 m
- dyaw = −0.436°
- `aggregate_valid = true`，`aggregate_confidence = high`

稳定性：三组子集平移模长 spread ≈ 0.00156m；yaw spread ≈ 0.0668°；双向最大平移一致性误差 ≈ 0.0131m；双向最大 yaw 一致性误差 ≈ 0.442°；overlap ≈ 0.73–0.75；RMSE ≈ 0.046–0.048m。

与现场及 Odom 比较：
- 现场人工最终位移模长 ≈ 0.666m；LaserScan ≈ 0.6581m；相差 ≈ 0.0079m。
- Odom 位移 ≈ 0.0062m；LaserScan 与 Odom 相差 ≈ 0.6519m。
- **Confirmed Fact**：LaserScan 独立确认机器人真实发生约 0.66m 累计平移，而 Odom/TF 几乎完全遗漏该平移。

Yaw 比较：
- LaserScan 最终 yaw ≈ −0.436°；现场人工 ≈ −2°~−1°；IMU/Odom/TF 最终 yaw ≈ +6.81°。
- Scan 与 IMU/Odom/TF 相差 ≈ 7.25°。
- **Confirmed Fact**：IMU/Mahony → Odom → TF 链的最终 yaw 与独立 LaserScan 几何结果不一致。

**限制**：Phase 2A 只能确认 S0→S4 累计结果，尚不能判断 0.658m 平移主要由哪一段、哪个旋转方向产生。

---

## 7. Jetson 运行源码只读审计

- Jetson：`ubuntu@172.20.10.2`，workspace `/home/ubuntu/ros2_ws`
- 运行真值文件：`/home/ubuntu/ros2_ws/src/turn_on_dlrobot_robot/src/dlrobot_robot.cpp`
- Jetson SHA256：`0ccda7e7189e026c9507d94b311d74e6ea8cb4b169cb234d08b5a3b1307903d0`
- PC 副本 SHA256 与 Jetson **不一致**；移动底盘三包只在 Jetson 运行，PC 副本无运行功能，后续以 Jetson 运行文件为真值。

数据链：`/cmd_vel` → `Cmd_Vel_Callback` → `linear.x/y` 与 `angular.z` 各 `*1000` → signed int16 → 串口下发 STM32。STM32 回传 24 字节帧，`rx[2..3]→Robot_Vel.X`、`rx[4..5]→Robot_Vel.Y`、`rx[6..7]→Robot_Vel.Z`，统一经 `Odom_Trans` 除以 1000。

ROS 端对正负 `angular.z` 处理完全对称：无方向分支、无单方向比例、无正负死区差异、无方向相关限幅。ROS 端**不计算左右轮目标速度**。

当前 ROS 仓库**不包含**：左右轮速度分解、电机 PWM、轮速 PID、编码器闭环、正反转死区补偿。

- Confirmed Fact：当前 CW/CCW 运动不对称不能由可见的 ROS 端 cmd_vel 处理逻辑解释。异常来源位于 ROS 串口下发之后的 STM32 控制、电机驱动、电机/编码器或机械执行链路中，具体位置尚未确定。

---

## 8. Confirmed Facts

1. 相同绝对值 `angular.z` 命令下，CW 与 CCW 旋转时间严重不对称（CW~20s vs CCW~48–50s）。
2. 两次 CW 结果彼此接近、两次 CCW 结果彼此接近，方向差异具有重复性。
3. LaserScan 独立确认真实累计平移约 0.658m。
4. Odom 仅记录约 0.0062m，几乎完全遗漏真实平移。
5. IMU/Odom/TF 最终 yaw（+6.81°）与 LaserScan（−0.436°）相差约 7.25°。
6. ROS 端正负 `angular.z` 的发送逻辑对称，无方向相关代码分支。
7. 当前 Phase 2A 只得到 S0→S4 累计结果，尚未完成逐段方向隔离。
8. 静止约 5 min 内 IMU yaw 漂移约 +2.48°（~0.50°/min）；Odom/TF yaw 为 IMU 的复制值（残差 <0.0023°），不构成独立证据。
9. `odom_combined → base_footprint` TF 在 Jetson 本地发布稳定（~20 Hz、无缺失）；PC bag 中的 TF 缺失属 PC 端接收/记录侧缺失，而非发布端缺失。
10. 运行时 TF 树为 `odom_combined → base_footprint`（动态）`→ laser`（静态，t=[0,0,0.150]），无其他动态 edge、无静态冲突。

---

## 9. Inferences

- CW 旋转可能相对稳定；CCW 可能是主要异常方向。但该假设**尚未**经逐段 LaserScan 结果确认（Current Inference，非结论）。
- Odom 平移近零而真实平移约 0.66m，说明当前轮速/Odom 链无法观测或表达旋转过程中的真实侧滑平移。这既可能来自滑移转向运动模型本身对横向滑移不可观测，也可能叠加 STM32 回传或里程计计算问题；目前尚不能定位到具体环节。
- IMU/Mahony → Odom → TF 链在本次 S0→S4 长序列结束时，与独立 LaserScan 几何结果存在约 7.25° 累计不一致。误差在哪些旋转段形成，仍需逐段分析确认；当前不能据此宣布整条 IMU yaw 链普遍不可信。
- 静态基线显示 IMU 存在 ~0.5°/min 漂移与 PC 侧 TF 间歇缺失；两者构成独立运行时风险，但 E01 的大幅异常主要由运动触发，非静态自激。

---

## 10. Unknowns

- R1/R2/R3/R4 各自产生多少真实平移。
- 0.658m 累计位移主要由哪个方向贡献。
- CW 方向的 Scan 与 Odom 是否确实一致；CCW 方向是否出现更大的 Scan/Odom 分离。
- 各段 Scan yaw 与 Odom yaw 的实际误差。
- STM32 固件内左右轮分解/PID/PWM/死区/编码器比例与正反转差异（固件源码不在本仓库）。
- STM32 回传 `Robot_Vel.X/Y` 为何近零（未回传平移 vs 固件原地旋转强制置零）。
- PC 侧 TF 缺失具体发生在 DDS writer / Wi-Fi / PC reader / PC recorder 哪一环。
- 历史约 90° 地图旋转的完整根因。

---

## 11. 当前假设

CW 可能相对正常，CCW 可能是主要异常方向；但尚未通过逐段 Scan 验证。Odom yaw 看似正常不代表真实 SE(2) 运动正常，必须用 LaserScan 逐段独立验证 CW 与 CCW 各自的真实平移与 yaw。

---

## 12. 下一步调试计划

使用现有 E01 rosbag，对四个相邻静态平台做独立 LaserScan SE(2) 配准：S0→S1 (R1 CCW)、S1→S2 (R2 CW)、S2→S3 (R3 CW)、S3→S4 (R4 CCW)。

每段输出：Scan dx/dy、Scan 平移模长、Scan dyaw、Odom dx/dy、Odom 平移模长、Odom dyaw、Scan/Odom 平移差、Scan/Odom yaw 差、coarse best、distinct second-best、ambiguity ratio、ICP RMSE、overlap、inverse consistency、all/odd/even 子集 spread、valid/confidence、invalid_reason。

算法约束：
- 不使用 Odom/IMU 作为 Scan 主配准初值。
- 90° 平台必须进行足够宽的全局 yaw 搜索。
- 不允许普通单初值 ICP 直接决定结果。
- 环境存在重复结构时必须报告 ambiguous 或 invalid。
- 必须检查逐段矩阵组合是否闭合到 Phase 2A 的 S0→S4 结果。

方向分组：CW = {R2, R3}；CCW = {R1, R4}。

分析目标：
1. 比较 CW 与 CCW 每段真实平移。
2. 比较 CW 与 CCW 的 Scan/Odom yaw 一致性。
3. 判断两次 CW 是否重复；判断两次 CCW 是否重复。
4. 确定 0.658m 累计位移主要来自哪个方向。
5. 验证"CW 可能可用、CCW 异常"假设。

若现有 bag 支持该假设，再设计新的独立实验：CW 90° 单独录包重复 3 次、CCW 90° 单独录包重复 3 次，每次恢复到相同起点，不把多方向混在连续实验中。

---

## 13. 禁止事项

- 不修改原始 rosbag。
- 不覆盖 Phase 1 或 Phase 2A 结果。
- 不修改 ROS 驱动、TF、IMU/Mahony、Odom。
- 不调整 SLAM、AMCL 或 Nav2 参数。
- 不使用 Odom 或 IMU 作为 Scan 主配准初值。
- 不因 Odom yaw 看似正常就认为真实运动正常。
- 不自动启动 Nav2 任务。
- 不宣布历史约 90° 地图旋转根因。
- 不把 PC 端 launch/源码当作运行时真值引用。

---

## 14. Status Snapshot

- **Current stage**: `ACTIVE_DIRECTIONAL_ISOLATION`
- **Current hypothesis**: CW 可能相对正常，CCW 可能是主要异常方向，但尚未通过逐段 Scan 验证。
- **Next action**: 创建或检查逐段 LaserScan 分析工具，之后对 S0→S1、S1→S2、S2→S3、S3→S4 进行离线配准。
- **Phase 1**: 完成（Step A/B 通过，四段旋转识别有效）。
- **Phase 2A**: 完成（S0→S4 独立配准 valid=true, confidence=high；真实平移 0.658m，Odom 0.0062m；yaw 偏差 7.25°）。
- **Jetson 源码审计**: 完成（ROS 端 cmd_vel 对称；左右轮/PID/PWM/编码器在 STM32 固件，不在本仓库）。
- **Static baseline test**: 完成。静止状态 TF/IMU/Odom/Scan 基线检查已完成（E00_static + E00B 双端），具体结论以第 2 节为准：IMU 静态 yaw 漂移 ~0.50°/min；Odom/TF yaw 为 IMU 复制值；`odom_combined→base_footprint` TF 在 Jetson 发布稳定，PC 侧存在 36 条 TF 接收缺失。
- **Runtime graph audit**: 完成。已审计 Jetson 运行话题集合、frame_id、TF edge 与 TF 发布责任边界（第 3 节）；完整 node list 未保留，但关键 `/tf`（2 publishers: `dlrobot_robot` + `odom_tf_bridge_node`）、`/tf_static`（1 publisher: `static_transform_publisher`）发布者拓扑、TF 频率（~19.998 Hz）及 `tf2_echo` 结果已从此前 SSH 运行时审计记录恢复；`dlrobot_robot` 为休眠 `/tf` publisher，`odom_tf_bridge_node` 为活跃发布者。
- **Evidence boundary**: 静态 TF 稳定只能排除静止状态下持续跳变，不能证明动态旋转过程正确。运行节点关系与源码复制链一致性不能替代 LaserScan 独立几何验证。
- **Phase 2B（逐段配准）**: 未启动。
