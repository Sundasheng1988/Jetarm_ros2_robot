# ROS2 Rosbag 离线分析工具清单

本文件用于帮助 Agent 根据任务类型选择正确工具。

## 使用规则

1. 开始 rosbag 离线分析前，必须先读取本文件。
2. 优先选择一个主工具，不得一次运行所有工具。
3. 用户明确指定工具时，只能使用用户指定的工具。
4. 工具用途不匹配时，应停止并报告，不得强行运行。
5. 不得因为分析 rosbag 而扩展到 STM32、固件、电机、PID、SLAM、AMCL 或 Nav2 源码审计。
6. 不得修改工具、开发新算法或搜索替代工具，除非用户明确批准。
7. 低置信度或 invalid 结果必须原样保留，不得为了得到有效结果反复调参。
8. 所有工具均为离线工具，不得发布 `/cmd_vel`，不得操作物理机器人。

---

## 1. 单次旋转 Cmd / IMU / Odom / TF 分析

### 工具

`/home/sundasheng/ros2_ws/tools/analyze_single_rotation_bag.py`

### 适用实验

单次旋转：

`S0 → R1 → S1`

例如：

- 单次 CW
- 单次 CCW
- 相同输入下的方向重复性测试
- 当前 `lidar_tf_validation_v2_20260726` 四个单次旋转包

### 分析内容

- `/cmd_vel` 非零消息数、持续时间、频率和方向
- IMU orientation yaw
- IMU `angular_velocity.z`
- IMU角速度积分
- Odom `Δx、Δy、平移量、Δyaw`
- `odom_combined → base_footprint` TF
- Odom与TF一致性
- TF连续性
- `base_footprint → laser` 静态TF
- 自动生成 `S0、R1、S1`
- 生成供Scan工具使用的静态窗口CSV

### 不适用

- 四段连续旋转E01实验
- Scan点云配准
- AMCL或map→odom分析
- 静止基线实验

---

## 2. 两个静态窗口的LaserScan独立SE(2)配准

### 工具

`/home/sundasheng/ros2_ws/tools/register_laserscan_static_window_pair_se2.py`

### 适用实验

比较机器人运动前后的两个静态Scan窗口：

- 起始静止窗口
- 结束静止窗口
- 单次CW或CCW后的真实空间变换
- 独立验证Odom是否漏记平移或yaw

### 输入要求

- rosbag中包含 `/scan`
- rosbag中包含 `/tf_static`
- 提供窗口CSV
- 窗口CSV必须包含：
  - `S0`
  - `S4`

对于单次旋转实验，可以将结束静止窗口 `S1` 映射为 `S4`。

### 分析内容

- Scan配准 `dx`
- Scan配准 `dy`
- 平移模长
- `dyaw`
- RMSE
- overlap
- all / odd / even子集稳定性
- 正向与反向配准一致性
- valid
- confidence

### 证据特点

- 不使用Odom作为初值
- 不使用IMU作为初值
- 不使用动态TF作为初值
- 静态TF只用于将LaserScan转换到 `base_footprint`

### 注意

大范围yaw搜索计算量较大。

接近90°旋转时可能需要：

`--yaw-search-deg 100`

未经用户批准，不得反复改变搜索参数。

### 不适用

- 动态过程逐帧分析
- IMU或Odom积分
- AMCL分析
- 没有静态Scan窗口的bag

---

## 3. E01四段旋转序列专用分析

### 工具

`/home/sundasheng/ros2_ws/tools/analyze_e01_four_rotation_sequence.py`

### 仅适用

历史E01四段旋转实验：

`S0 → R1 → S1 → R2 → S2 → R3 → S3 → R4 → S4`

### 分析内容

- 四个旋转阶段自动检测
- correction nudge识别
- IMU、Odom、TF yaw链
- 四段角速度积分
- S0→S4最终平移和yaw残差
- E01现场真值比较

### 强制限制

- 只用于历史E01实验
- 内部包含E01专用窗口结构和现场物理真值
- 不得用于单次CW或CCW bag
- 不得通过伪造R2、R3、R4窗口绕过门控

---

## 4. E00静止基线数据完整性审计

### 工具

`/home/sundasheng/ros2_ws/tools/audit_e00_stationary_bag_integrity.py`

### 适用实验

机器人全程静止的E00基线实验。

### 分析内容

- Topic消息数量
- header时间戳连续性
- bag时间戳连续性
- 重复时间戳
- 冲突数据
- transport delay
- `/odom_combined`与TF时间戳对应
- IMU、Odom、TF yaw源对齐
- 静止状态下墙线角度稳定性
- `/tf_static`检查

### 不适用

- 动态旋转实验
- Scan前后位姿配准
- AMCL因果分析
- 四段E01实验

---

## 5. Localization中Scan与TF时间关系分析

### 工具

`/home/sundasheng/ros2_ws/tools/analyze_localization_scan_tf_timing.py`

### 适用问题

- LaserScan时间戳与TF时间戳是否对齐
- Scan开始、中间、结束时刻的TF覆盖
- TF插值区间是否过大
- 扫描期间机器人是否发生明显旋转
- bag时间戳与header时间戳延迟
- `/cmd_vel`、IMU、Odom、TF与Scan的时间对应关系

### 典型使用场景

- 地图或Scan在旋转时异常
- 怀疑Scan畸变
- 怀疑TF发布频率不足
- 怀疑跨主机时间同步问题

### 不适用

- 独立Scan-to-Scan SE(2)配准
- 单次旋转汇总指标
- 纯静止基线
- STM32或电机分析

---

## 6. Localization中map/odom坐标系隔离分析

### 工具

`/home/sundasheng/ros2_ws/tools/analyze_localization_map_odom_frame_isolation.py`

### 适用问题

区分异常发生在：

- `map → odom_combined`
- `odom_combined → base_footprint`
- Scan相对Odom
- AMCL修正链

### 分析内容

- map frame墙线方向
- odom frame墙线方向
- raw map→odom变化
- 持续异常事件时间
- Odom来源与TF来源覆盖率
- Scan异常与map→odom修正的先后关系

### 典型使用场景

- AMCL定位期间地图突然旋转
- 需要判断异常来自AMCL还是Odom/Scan上游
- 需要隔离map frame与odom frame影响

### 不适用

- 不包含map→odom或AMCL数据的单次旋转bag
- 独立Scan-to-Scan位姿估计
- 普通Cmd/IMU/Odom汇总

---

# 快速选择表

| 当前任务 | 首选工具 |
|---|---|
| 单次CW或CCW的Cmd、IMU、Odom、TF | `analyze_single_rotation_bag.py` |
| 运动前后LaserScan独立配准 | `register_laserscan_static_window_pair_se2.py` |
| 历史E01四段旋转序列 | `analyze_e01_four_rotation_sequence.py` |
| E00全程静止数据质量审计 | `audit_e00_stationary_bag_integrity.py` |
| Scan、TF、IMU、Odom时间关系 | `analyze_localization_scan_tf_timing.py` |
| map与odom坐标系异常隔离 | `analyze_localization_map_odom_frame_isolation.py` |

# 组合使用规则

当前单次旋转方向对称性实验应使用：

1. `analyze_single_rotation_bag.py`
2. `register_laserscan_static_window_pair_se2.py`

第一个负责：

- Cmd
- IMU
- Odom
- TF

第二个负责：

- 独立LaserScan空间变换

不得调用其余四个工具，除非任务本身符合对应场景或用户明确批准。
