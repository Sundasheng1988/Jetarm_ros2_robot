# Dev Log — 2026-07-05 第一次真实机器人 Nav2 导航成功

> **当前阶段**: Mobile Robot Foundation — Sprint 8.5
> **开发目标**: AMCL Validation → Nav2 Goal Pose → 完整 Navigation 闭环
> **编写日期**: 2026-07-05

---

## 1. 今日目标

1. ✅ 清理原始 SLAM 地图噪点（GIMP）
2. ✅ 完成 AMCL 定位验证（2D Pose Estimate）
3. ✅ 第一次启动 Navigation2 完整栈
4. ✅ 第一次真实机器人 Goal Navigation（2D Goal Pose）
5. ⚠️ Recovery Tree 已观察到生效，待标准化验证
6. ⏳ Chrony 时间同步（发现需要，未完成）
7. ⏳ 连续 10 次导航测试（已排入明天）

---

## 2. 今日成果（已完成）

| 模块 | 状态 | 说明 |
|------|------|------|
| 底盘驱动 | ✅ | Nano 端 `tank.launch.py` 正常，`/cmd_vel` 可控制底盘 |
| Odom | ✅ | 当前主链路使用 `/odom_combined` |
| Odom→TF | ✅ | Nano 端 `odom_tf_bridge_node` 发布 `odom_combined→base_footprint` |
| 激光 | ✅ | Nano 端 RPLidar 发布 `/scan`，频率以实际为准（7–10Hz） |
| Laser TF | ✅ | Nano 端发布 `base_footprint→laser` |
| 静态地图 | ✅ | 当前推荐 `home_map_clean_02_260705` |
| AMCL | ✅ | 通过 2D Pose Estimate 后，可稳定发布 /amcl_pose 与 map→odom_combined |
| Planner | ✅ | 可完成短距离规划；个别目标点出现 failed to create plan，待进一步排查 |
| Controller | ✅ | 可输出 `/cmd_vel` 驱动底盘 |
| Recovery | ⚠️ | Recovery Tree 已生效并成功恢复过导航，但尚未完成标准化验证 |
| Goal Navigation | ✅ | 通过 RViz 2D Goal Pose 完成首次实机导航成功 |
| 语义导航 | ❌ | 未开始 |
| Mobile Manipulation | ❌ | 未开始 |

---

## 3. 当前地图版本

### 原始地图（SLAM Toolbox 首次建图）

| 字段 | 值 |
|------|------|
| 文件名 | `home_map_01_260621` |
| PGM 路径 | `~/ros2_ws/maps/home_map_01_260621.pgm` |
| YAML 路径 | `~/ros2_ws/maps/home_map_01_260621.yaml` |
| 尺寸 | 365 × 204 px |
| `resolution` | 0.05 |
| `origin` | `[-11.1, -6.32, 0]` |
| 说明 | 原始 SLAM 建图结果，含大量漂浮噪点和孤立 occupied 像素 |

### 第一次清理

| 字段 | 值 |
|------|------|
| 文件名 | `home_map_clean_01_260705` |
| PGM 路径 | `~/ros2_ws/maps/home_map_clean_01_260705.pgm` |
| YAML 路径 | `~/ros2_ws/maps/home_map_clean_01_260705.yaml` |
| 尺寸 | 374 × 213 px（GIMP 裁剪+补齐后） |
| `resolution` | 0.05 |
| `origin` | `[-10.8, -4.88, 0]` |
| 说明 | 第一次 GIMP 手动清理，删除漂浮噪点，保留结构墙体和已知障碍物 |

### 当前推荐（第二次清理）

| 字段 | 值 |
|------|------|
| 文件名 | `home_map_clean_02_260705` |
| PGM 路径 | `~/ros2_ws/maps/home_map_clean_02_260705.pgm` |
| YAML 路径 | `~/ros2_ws/maps/home_map_clean_02_260705.yaml` |
| 尺寸 | 374 × 213 px |
| `resolution` | 0.05 |
| `origin` | `[-10.8, -4.88, 0]` |
| 说明 | 第二次清理 — 进一步清除地图边缘噪点，进一步清理客厅及边缘区域的噪点，作为当前导航测试地图。**当前推荐使用此版本** |

```yaml
# ~/ros2_ws/maps/home_map_clean_02_260705.yaml
image: home_map_clean_02_260705.pgm
mode: trinary
resolution: 0.05
origin: [-10.8, -4.88, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

> ⚠ **origin 变化说明**: 原始地图 origin 为 `[-11.1, -6.32, 0]`。GIMP 编辑时裁剪了边缘空白区域，origin 偏移至 `[-10.8, -4.88, 0]`。此偏移在 YAML 中已正确配置，不影响 AMCL 定位。

---

## 4. 地图编辑流程（GIMP）

### 4.1 打开 PGM

```bash
gimp ~/ros2_ws/maps/home_map_01_260621.pgm
```

### 4.2 手动删除噪点

- 使用 **Pencil Tool** (N) — 大小 3–5 px
- 将漂浮在 free space 中的黑色像素（occupied）涂成 **白色**（255 = free）
- 将墙体外侧的孤立黑色像素删除
- 保留结构性墙体（承重墙、隔断墙）
- 保留结构墙体和确认需要长期保留的固定障碍物。

### 4.3 导出为 PGM

GIMP 导出必须选择 **Raw (P5)** 格式，不能选择 ASCII (P2)：

```
文件 → 导出为 → 文件名: home_map_clean_02_260705.pgm
在导出对话框中:
  ☑ 兼容选项: 选择 "Raw" (P5)
  ☐ 不要选 "ASCII" (P2)
```

验证格式：

```bash
file ~/ros2_ws/maps/home_map_clean_02_260705.pgm
# 期望输出: Netpbm image data, size = 374 x 213, rawbits, greymap
# 如果不是 rawbits，说明选了 ASCII 模式
```

### 4.4 新建 YAML

复制上一版 YAML，修改 `image` 字段指向新 PGM：

```bash
cp ~/ros2_ws/maps/home_map_clean_01_260705.yaml \
   ~/ros2_ws/maps/home_map_clean_02_260705.yaml
# 编辑 image 字段
```

YAML 内容：

```yaml
image: home_map_clean_02_260705.pgm
mode: trinary
resolution: 0.05
origin: [-10.8, -4.88, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

### 4.5 RViz 查看地图

在 RViz 中添加 Map display：

| 配置项 | 值 |
|--------|------|
| Topic | `/map` |
| Color Scheme | `map` |
| **Durability Policy** | **Transient Local** ← 必须设置 |

> ⚠ **Durability Policy 必须为 Transient Local**。SLAM Toolbox 和 map_server 以 latched（延迟）模式发布 `/map`，默认的 `Volatile` 策略收不到地图数据。

---

## 5. Nano 启动流程

所有命令在 **Jetson Orin Nano** 上执行。

### Terminal 1 — 底盘驱动

```bash
# 记得先 source ROS2 环境
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch turn_on_dlrobot_robot tank.launch.py
```

### Terminal 2 — 激光雷达

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch rplidar_ros rplidar_a1_launch.py \
  serial_port:=/dev/ttyUSB0
```

### Terminal 3 — Odom→TF 桥接

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run mobile_base_bridge odom_tf_bridge_node
```

作用：

```
/odom_combined
  ↓
/tf odom_combined → base_footprint
```

### Terminal 4 — 雷达安装 TF

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run tf2_ros static_transform_publisher \
  0 0 0.15 0 0 0 base_footprint laser
```

作用：

```
base_footprint
  ↓
laser
```

### 启动后验证（Nano 端）

```bash
# 验证激光频率（以实际结果为准）
ros2 topic hz /scan

# 验证里程计
ros2 topic echo /odom_combined --once

# 验证 TF 发布
ros2 run tf2_ros tf2_echo odom_combined base_footprint

ros2 run tf2_ros tf2_echo base_footprint laser
```

---

## 6. PC 启动流程

所有命令在 **PC Ubuntu** 上执行。

### Terminal 1 — 环境准备

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

### Terminal 2 — AMCL Localization

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch nav2_bringup localization_launch.py \
  map:=/home/sundasheng/ros2_ws/maps/home_map_clean_02_260705.yaml \
  use_sim_time:=false \
  params_file:=/home/sundasheng/ros2_ws/nav2_params.yaml
```

说明：地图由 `localization_launch.py` 中的 `map_server` 加载。

### Terminal 3 — Navigation Stack

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=false \
  params_file:=/home/sundasheng/ros2_ws/nav2_params.yaml
```

说明：`navigation_launch.py` 不负责加载地图。地图已由 `localization_launch.py` 提供。

### Terminal 4 — RViz

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

rviz2
```

#### RViz Display 配置

| Display | Topic | Durability Policy |
|---------|-------|-------------------|
| Map | `/map` | **Transient Local** ← 必须 |
| LaserScan | `/scan` | — |
| RobotModel | — | — |
| TF | `/tf` | — |
| PoseEstimate | `/initialpose` | — |
| Goal | `/goal_pose` | — |
| Path | `/plan` | — |
| ParticleCloud | `/particle_cloud` | — |

---

## 7. 验证命令

### 7.0 时间检查

PC 端：

```bash
date
timedatectl
```

Nano 端：

```bash
date
timedatectl
```

确认两机时间偏差 < 1 秒。如偏差过大，TF 时间戳会引发 `Message Filter dropping message`。

### 7.1 TF 层级

逐段验证 TF 链路：

```bash
# 验证里程计→基座
ros2 run tf2_ros tf2_echo odom_combined base_footprint

# 验证基座→激光雷达
ros2 run tf2_ros tf2_echo base_footprint laser

# 验证地图→里程计（AMCL 定位成功后出现）
ros2 run tf2_ros tf2_echo map odom_combined
```

完整 TF 树：

```bash
ros2 run tf2_tools view_frames
```

期望结构：

```
map
└── odom_combined
    └── base_footprint
        └── laser
```

> ⚠ `map → odom_combined` 仅当 AMCL 收到 `/initialpose` 并完成定位后才会发布。如果 `map` 不在树中 → 检查 2D Pose Estimate 是否已设置。

### 7.2 生命周期检查

Nav2 节点启动后处于 `inactive [1]` 或 `unconfigured [0]`，自动转换到 `active [3]`：

```bash
ros2 lifecycle get /amcl
# 期望: active [3]

ros2 lifecycle get /planner_server
# 期望: active [3]

ros2 lifecycle get /controller_server
# 期望: active [3]

ros2 lifecycle get /bt_navigator
# 期望: active [3]
```

### 7.3 AMCL 位姿

```bash
ros2 topic echo /amcl_pose --once
```

期望输出：

```
pose:
  pose:
    position:
      x: <当前位姿 x>
      y: <当前位姿 y>
    orientation:
      z: <朝向>
      w: <朝向>
```

### 7.4 激光频率

```bash
ros2 topic hz /scan
```

频率约 **7–10 Hz**，以实际输出为准。

```bash
ros2 topic echo /scan --once
```

### 7.5 里程计

主链路使用 `/odom_combined`：

```bash
ros2 topic echo /odom_combined --once
```

### 7.6 底盘控制指令

```bash
ros2 topic echo /cmd_vel
```

导航执行期间应有持续数据输出（Controller 驱动底盘）。

---

## 8. 今天完成的关键里程碑

### M1 — 第一次成功 AMCL 定位

SLAM Toolbox 建立的地图首次被 AMCL 成功加载并定位。2D Pose Estimate 后，粒子在 5–10 秒内收敛到正确位置，`/amcl_pose` 输出稳定。

定位方法：
1. 在 RViz 点击 **2D Pose Estimate**
2. 拖拽箭头指向机器人实际朝向
3. 等待粒子收敛（观察 particle cloud）
4. 确认 `/amcl_pose` 位姿与真实位置一致

### M2 — 第一次成功 Navigation2 完整栈启动

`localization_launch.py` + `navigation_launch.py` 同时运行后：
- `planner_server`、`controller_server`、`bt_navigator` 全部进入 `active [3]`
- TF 树完整：`map → odom_combined → base_footprint → laser`
- Nav2 成功接收 `/goal_pose`

### M3 — 第一次成功 Goal Navigation（RViz 2D Goal Pose）

在 RViz 手动设置 2D Goal Pose 后：
1. `planner_server` 成功生成全局路径（`/plan` 可见）
2. `controller_server` 输出 `/cmd_vel` 控制底盘移动
3. 机器人沿规划路径顺利行驶到目标点
4. 到达后导航状态标记为 Succeeded

> ⚠ 尚未建立 `kitchen`、`living_room`、`charging_station` 等语义点位。目标点通过 RViz 手动指定。

### M4 — Recovery Behavior 已触发

日志中观察到以下恢复行为序列：

```
Failed to make progress
→ clear costmap
→ backup
→ spin
→ wait
→ Goal succeeded
```

Recovery 已触发并成功恢复导航。

> ⚠ 尚未做标准化障碍物实验（例如固定放置纸箱验证绕障能力）。当前观察来源于实际导航中 Controller 触发的自动恢复。

### M5 — GIMP 地图编辑流程建立

完整流程：SLAM Toolbox 建图 → PGM 导出 → GIMP 噪点清理 → Raw(P5) 导出 → YAML 配置 → AMCL 加载验证。当前已迭代 2 个清理版本。

### M6 — Nav2 完整闭环建立

```
SLAM Toolbox
  ↓ PGM + YAML
GIMP 地图清理
  ↓
AMCL 定位
  ↓
Planner（全局路径）
  ↓
Controller（局部路径）
  ↓
Recovery（绕障 + 重规划）
  ↓
Goal Navigation（成功到达目标点）
```

---

## 9. 已知问题

### 问题 1 — TF 时间同步问题（待验证）

**现象**：
```
Message Filter dropping message: frame 'laser' at time X.XXX
  has timestamp Y.YYY, but code expects Z.ZZZ
```

或：
```
Lookup would require extrapolation into the past
```

**原因分析（待验证）**：
疑似以下因素之一或组合：
- PC/Nano 系统时间不同步 → 时间戳基准不一致
- TF 缓存配置不足（`cache_time` 默认 10s）
- DDS 网络延迟导致消息到达顺序错乱
- Laser 时间戳本身存在问题

**后续方案**：
1. 明天首先检查两机 `date` 偏差
2. 如偏差 > 1s → 安装 chrony
3. 如时间已同步但问题仍存在 → 排查 TF 缓存和 DDS 配置
4. 记录日志供进一步分析

### 问题 2 — Planner 偶尔失败

**现象**：
```
[planner_server]: Failed to create plan.
[planner_server]: Planning algorithm didn't find a valid path.
```

**原因分析**：
1. 地图清理不彻底 — 部分 occupied 像素残留阻挡窄通道
2. 某些角落区域 free space 不够 planner 生成路径
3. 机器人初始位姿偏差较大时 planner 可能失败

**后续方案**：
1. 继续清理地图，特别关注通道/门口区域
2. 增加 `nav2_params.yaml` 中的 inflation radius
3. 如频繁失败，录制 rosbag 进行离线分析

### 问题 3 — 客厅区域灰色阴影（待验证）

**现象**：
在客厅区域，costmap 显示灰色/浅黑色区域（被标记为 obstacle），导致 planner 生成绕行路径或失败。

**确认**：
不是 PGM 问题。关闭 Costmap 和 LaserScan 后，静态地图完全干净。

**初步分析**：
问题来源于 Obstacle Layer + Inflation Layer + 实时 LaserScan 的组合，原因待进一步确认，可能包括：
- 实际低矮障碍物（桌椅腿等）
- 雷达扫到车体自身结构
- 金属表面反射导致异常测量值
- 地面不平整产生的杂波
- `clearing` 参数配置问题

**后续方案**：
1. 录制约 30 秒 rosbag，离线分析 LaserScan 原始数据
2. 检查 obstacle layer 的 `observation_sources` 和 `clearing` 参数
3. 微调 inflation radius 以降低敏感度
4. 必要时排除特定角度范围内的激光数据

---

## 10. 未完成事项（TODO）

### 高优先级

| 序号 | 事项 | 原因 |
|------|------|------|
| 1 | **确认两机时间偏差 / 安装 chrony** | TF 时间戳疑似不同步导致 Message Filter dropping message，影响定位稳定性 |
| 2 | **逐级导航测试（短距离→长距离）** | 当前仅验证了单次导航，需先做 0.5m 短距离测试，逐步扩展到跨区域 |
| 3 | **剩余地图清理** | Planner 偶尔在部分区域失败，需进一步清理噪点 |
| 4 | **Laser TF 迁移到 Nano** | 减少 PC ↔ Nano TF 依赖，降低时间同步敏感度 |

### 中优先级

| 序号 | 事项 | 说明 |
|------|------|------|
| 1 | 建立 `navigation_landmarks.yaml` | 定义 Home / Kitchen / Living Room / Charging Station 语义坐标 |
| 2 | Home 坐标校准 | 确认当前位置 → 语义命名 |
| 3 | Kitchen 坐标校准 | 导航目标点坐标 |
| 4 | Living Room 坐标校准 | 导航目标点坐标 |
| 5 | Charging Station 坐标校准 | 回充座坐标 |

### 低优先级

| 序号 | 事项 | 说明 |
|------|------|------|
| 1 | 语义导航 | Grounding + SemanticLocationResolver → MoveSkill |
| 2 | 移动机械臂协同 | Navigate → Pick | Navigate → Place 完整链路 |
| 3 | 导航参数调优 | PID / max_vel / acceleration 动态调优 |

---

## 11. 明天启动顺序

### Nano 端（4 个 Terminal）

**Terminal 1** — 底盘驱动

```bash
ros2 launch turn_on_dlrobot_robot tank.launch.py
```

**Terminal 2** — 激光雷达

```bash
ros2 launch rplidar_ros rplidar_a1_launch.py \
  serial_port:=/dev/ttyUSB0
```

**Terminal 3** — Odom→TF 桥

```bash
ros2 run mobile_base_bridge odom_tf_bridge_node
```

**Terminal 4** — 雷达安装 TF

```bash
ros2 run tf2_ros static_transform_publisher \
  0 0 0.15 0 0 0 base_footprint laser
```

---

### PC 端

**Terminal 1** — 环境准备

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

**Terminal 2** — Localization

```bash
ros2 launch nav2_bringup localization_launch.py \
  map:=/home/sundasheng/ros2_ws/maps/home_map_clean_02_260705.yaml \
  use_sim_time:=false \
  params_file:=/home/sundasheng/ros2_ws/nav2_params.yaml
```

**RViz** — 定位确认

```bash
rviz2
```

1. 添加 Map Display → Durability Policy 设为 **Transient Local**
2. 点击 **2D Pose Estimate** → 在机器人实际位置点击并拖拽朝向
3. 等待 particle cloud 收敛
4. 验证定位成功：

```bash
ros2 run tf2_ros tf2_echo map odom_combined
```

确认输出 `map → odom_combined` 存在。

**Terminal 3** — Navigation

确认定位成功后启动导航：

```bash
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=false \
  params_file:=/home/sundasheng/ros2_ws/nav2_params.yaml
```

确认 Planner / Controller / BT Navigator 全部 `active [3]`。

**RViz** — 2D Goal Pose 测试

先做短距离测试：

1. **前方 0.3–0.5m** — 确认直行
2. **左前方 0.5m** — 确认转弯
3. **右前方 0.5m** — 确认转弯

稳定后再做长距离测试。

---

## 12. 明天调试计划

### Step 1 — 检查时间同步

```bash
# Nano 端
date

# PC 端
date
```

确认两机时间偏差 < 1 秒。

> 如偏差过大 → 安装 chrony（详见下方安装命令）。

### Step 2 — 查看 TF 树

```bash
ros2 run tf2_tools view_frames
```

确认：

```
map
└── odom_combined
    └── base_footprint
        └── laser
```

### Step 3 — 逐级导航测试

先做短距离，稳定后逐步扩展。

**短距离测试（同一区域）**：

| 测试 | 方向 | 距离 | Planner | Controller | Recovery | 用时 | 结果 |
|------|------|------|---------|------------|----------|------|------|
| 1 | 前方 | 0.5m | | | | | |
| 2 | 前方 | 0.5m | | | | | |
| 3 | 前方 | 1.0m | | | | | |
| 4 | 左前方 | 0.5m | | | | | |
| 5 | 左前方 | 1.0m | | | | | |
| 6 | 右前方 | 0.5m | | | | | |
| 7 | 右前方 | 1.0m | | | | | |
| 8 | 原路返回 | — | | | | | |
| 9 | 原地旋转 | 180° | | | | | |

**长距离测试（跨区域，待短距离稳定后）**：

| 测试 | 路线 | 距离 | Planner | Controller | Recovery | 用时 | 结果 |
|------|------|------|---------|------------|----------|------|------|
| 1 | 客厅→过道 | ~3m | | | | | |
| 2 | 过道→客厅 | ~3m | | | | | |

### Step 4 — 建立 navigation_landmarks.yaml（导航测试完成后）

在导航测试稳定后，通过 RViz 2D Pose Estimate 读取关键位置坐标并记录：

```yaml
# ~/ros2_ws/maps/navigation_landmarks.yaml
landmarks:
  home:
    x:      # 待填：当前位置 → 从 /amcl_pose 读取
    y:
    yaw:
    map: home_map_clean_02_260705
  kitchen:
    x:      # 待填：导航至厨房后读取 /amcl_pose
    y:
    yaw:
    map: home_map_clean_02_260705
  living_room:
    x:      # 待填
    y:
    yaw:
    map: home_map_clean_02_260705
  charging_station:
    x:      # 待填
    y:
    yaw:
    map: home_map_clean_02_260705
```

坐标获取方法：导航至目标点后，执行 `ros2 topic echo /amcl_pose --once` 读取 x/y/z/w。

### Chrony 安装（如果需要）

```bash
# 在两台机器上都执行
sudo apt install chrony

# 配置 Nano 端指向 PC
# Nano: /etc/chrony/chrony.conf 添加
# server <pc-ip> iburst

# 配置 PC 端作为 NTP 服务器
# PC: /etc/chrony/chrony.conf 取消注释
# allow <nano-subnet>

# 重启 chrony
sudo systemctl restart chrony

# 验证同步
chronyc tracking
```

---

## 13. 今日总结

**M1 — 第一次真实机器人 Nav2 导航成功。**

今天完成了从 SLAM 建图到 Nav2 自主导航的完整闭环。核心路径为：

```
SLAM Toolbox（建图）
  → PGM（原始地图）
  → GIMP（手动噪点清理）
  → YAML（地图配置）
  → AMCL（粒子滤波定位）
  → Planner（全局路径规划）
  → Controller（局部路径跟踪 + 绕障）
  → Recovery（异常行为恢复）
  → Goal Navigation（自主到达目标点）
```

这是机器人项目从"固定机械臂操作平台"向"自主移动操作机器人"转型的关键里程碑。机器人首次在真实家居环境中，基于激光雷达 SLAM 地图 + RViz 2D Goal Pose 完成了自主导航。

**遗留的主要问题**是 PC/Nano 之间的系统时间偏差疑似导致 TF Message Filter dropping。明天首要任务是确认两机时间偏差，必要时安装 chrony，然后进行短距离→长距离的逐级导航测试。

**同时记录**：地图清理是值得的。第一次 GIMP 清理后的地图在 AMCL 定位和 Planner 生成上都显著优于原始地图。第二次清理进一步改善了窄通道区域的导航成功率。

---

---

## 14. 明日启动 Checklist

### Nano

- [ ] `tank.launch.py` — 底盘驱动
- [ ] `rplidar` — 激光雷达
- [ ] `odom_tf_bridge_node` — Odom→TF 桥
- [ ] `static_transform_publisher` — Laser TF
- [ ] `ros2 topic hz /scan` — 确认激光数据
- [ ] `ros2 topic echo /odom_combined --once` — 确认里程计

---

### PC — Localization

- [ ] `localization_launch.py` — AMCL + map_server
- [ ] `rviz2` — 可视化
- [ ] Map Durability Policy = **Transient Local**
- [ ] **2D Pose Estimate** — 设置初始位姿
- [ ] `tf2_echo map odom_combined` — 确认定位

---

### PC — Navigation

- [ ] `navigation_launch.py` — Planner + Controller + BT Navigator
- [ ] `planner_server` → `active [3]`
- [ ] `controller_server` → `active [3]`
- [ ] `bt_navigator` → `active [3]`

---

### 短距离导航测试

- [ ] 前方 0.5m
- [ ] 左前方 0.5m
- [ ] 右前方 0.5m

---

### 长距离导航测试

- [ ] 客厅短路径（同一区域）
- [ ] 跨区域路径（如客厅→过道）

---

### 故障排查

- [ ] PC/Nano `date` 偏差 < 1s
- [ ] `view_frames` — TF 树完整
- [ ] 如出现 `Message Filter dropping message` → 记录时间戳偏差

---

## 15. 修改记录

| 版本 | 日期 | 修改内容 |
|------|------|----------|
| v1 | 2026-07-05 | 初稿 |
| v2 | 2026-07-05 | 修正部署拓扑：Odom TF 桥和 Laser TF 确认在 Nano 端运行，PC 端删除对应启动项；修正 Goal Navigation 描述去掉未建立的语义点位；修正 Recovery 描述改为日志观测（无标准障碍物实验）；修正激光频率为 7–10Hz 以实际为准；修正 Odom 验证统一使用 `/odom_combined`；补充 TF 逐段验证命令；补充时间检查；补充启动 Checklist；所有不确定结论标记为"待验证" |

---

> **相关文档**:
> - [文档首页](../runtime_index.md)
> - [路线图](../jetarm_runtime_roadmap.md)
> - [Topic/Service 真相](../topic_service_map.md)
> - [风险分析](../runtime_risks.md)
> - [Nav2 参数配置](../../nav2_params.yaml)
