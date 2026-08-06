# Nav2 Localization Debug Report

## LaserScan 与 Map 偏转问题调查记录

**日期**：2026-07-13

---

### 问题描述

在 RViz 中使用 AMCL Localization 时：

- `/scan` 发布的 LaserScan 点云（紫色）
- 静态地图 `/map`（黑色）

在机器人旋转过程中出现角度偏移。

**主要现象：**

1. 机器人原地旋转后，LaserScan 与地图存在旋转角误差。
2. 偏差角度随旋转速度变化。
3. 停止后仍可能存在角度偏差。

---

### 1. 当前系统链路

当前 Localization TF 链：

```
map
 |
 |  AMCL publish TF
 |
odom_combined
 |
 |
base_footprint
 |
 |
laser
```

**相关节点：**

- `/amcl`
- `/map_server`
- `/rplidar_node`
- `/odom_tf_bridge_node`
- `/dlrobot_robot`
- `rviz`

---

### 2. 初始怀疑点列表

当前排查方向：

| 编号 | 怀疑点 | 状态 |
|------|--------|------|
| 1 | Laser TF 外参错误 | 已检查，暂未发现异常 |
| 2 | Odom yaw 角度错误 | 已检查，基本排除 |
| 3 | AMCL 更新频率不足 | 已验证可能性较低 |
| 4 | **LaserScan 角度零点/方向问题** | **当前重点怀疑** |
| 5 | IMU 数据影响 | 未完成验证 |
| 6 | AMCL scan matching 参数 | 已检查基础参数 |
| 7 | LaserScan 时间同步/扫描频率 | 部分确认 |

---

### 3. TF 外参检查

**怀疑原因：** 如果 `base_footprint → laser` 存在 yaw 偏差，会导致 LaserScan 整体旋转偏移。

**验证方法：**

```bash
ros2 run tf2_ros tf2_echo laser base_footprint
```

**结果：**

```
Translation:  [0.000, 0.000, -0.150]
Rotation RPY: [0, 0, 0]
```

**当前结论：** 已确认 `laser` yaw ≈ 0，没有发现明显旋转外参错误。

**状态：** ✅ TF 外参问题可能性降低。

---

### 4. Odom yaw 链路检查

**怀疑原因：** 如果 `odom_combined → base_footprint` 角度错误，AMCL 和 LaserScan 都会受到影响。

**验证方法：**

```bash
ros2 run tf2_ros tf2_echo odom_combined base_footprint
```

**验证记录：**

| 阶段 | yaw |
|------|------|
| 旋转前 | 0° |
| 旋转后 | -73.275° |

同时 `Translation: [-0.005, 0.001]`，说明基本原地旋转，odom yaw 有明显变化。

**当前结论：** Odom 可以正确反映机器人旋转，odom yaw 错误可能性降低。

**状态：** ✅ Odom yaw 问题可能性降低。

---

### 5. AMCL 状态检查

**怀疑原因：** AMCL 更新频率低可能导致 `map → odom_combined` TF 更新滞后。

**验证方法：**

```bash
ros2 topic hz /amcl_pose
```

**结果：** average rate ≈ **0.2 Hz**

**AMCL 参数检查：**

| 参数 | 值 |
|------|------|
| `update_min_a` | 0.1 |
| `update_min_d` | 0.1 |
| `transform_tolerance` | 3.0 |
| `laser_model_type` | likelihood_field |
| `max_beams` | 60 |
| `laser_max_range` | 12.0 m |

**AMCL node 状态：**

```bash
ros2 lifecycle get /amcl
# → active [3]
```

**当前结论：** AMCL 正常运行，有发布 `/amcl_pose`，有发布 TF。虽然 `/amcl_pose` 频率较低，但当前现象不能直接证明是 AMCL 发布频率导致。

**状态：** ✅ AMCL 频率问题暂不作为第一嫌疑。

---

### 6. Fixed Frame = `odom_combined` 实验

**目的：** 绕过 AMCL，验证 LaserScan 旋转异常是否依赖 AMCL。

**实验方法：** RViz 设置 Fixed Frame = `odom_combined`，观察机器人旋转。

**观察结果：** 机器人旋转约 130°，TF 坐标轴（红绿坐标）发生旋转。但是 LaserScan 紫色点云没有同步出现对应旋转——点云形状基本保持，位置有少量变化，没有明显跟随机器人旋转。

**当前结论：** 该实验说明问题可能存在于 `odom_combined → base_footprint → laser → scan data` 链路，因为 Fixed Frame 已经不依赖 AMCL。

---

### 7. LaserScan 数据检查

**`/scan` topic：**

```bash
ros2 topic info /scan
# Publisher count: 1
# Subscription count: 2
```

**scan 频率：**

```bash
ros2 topic hz /scan
# ≈ 7.6 ~ 7.8 Hz
```

**LaserScan 参数：**

| 参数 | 值 |
|------|------|
| `frame_id` | laser |
| `angle_min` | -3.124 |
| `angle_max` | 3.141 |
| `angle_increment` | 0.005806 |
| `scan_time` | 0.120 s |
| `range_max` | 12 m |

**当前结论：** LaserScan 正常发布，频率约 8 Hz，角度范围正常。但是 laser 角度零点方向和 laser scan 方向定义仍未完成验证。

---

### 8. 不同旋转速度实验

**实验目的：** 验证 LaserScan / map 偏差是否与旋转速度相关。

**实验条件：** 机器人原地旋转约 90°。记录 odom yaw、AMCL yaw、RViz 中 scan 与 map 偏差。

**实验结果：**

| 状态 | odom yaw | AMCL yaw | 变化量 |
|------|----------|----------|--------|
| 旋转前 | 0° | ≈ -91° | — |
| 旋转后 | -73.275° | ≈ -167° | — |
| 变化 | -73.275° | ≈ -76° | ≈一致 |

**RViz 观察：**

- 旋转前：LaserScan 基本贴合地图。
- 旋转后：LaserScan 出现角度偏差。

**当前结论：** 偏差角与旋转过程相关。需要进一步验证 scan 角度定义、scan 时间戳、lidar 驱动处理。

---

### 9. 当前排查结论（2026-07-13）

**已降低可能性：**

1. **TF 静态外参** — `laser → base_footprint` yaw ≈ 0，未发现异常。
2. **Odom yaw 错误** — odom yaw 能正确反映机器人旋转。
3. **AMCL 完全失效** — AMCL active，发布 `amcl_pose`，yaw 变化与 odom 一致。

**当前最高优先级怀疑：**

1. ⭐⭐⭐⭐⭐ **LaserScan 角度零点/方向**
   - 需要验证 lidar 0° 方向是否与机器人正前方一致
   - scan 是否存在固定角度 offset

2. ⭐⭐⭐⭐ **LaserScan 时间因素**
   - timestamp 是否正确
   - lidar 扫描周期是否造成旋转误差

3. ⭐⭐⭐ **IMU 影响**
   - odom_combined 是否融合 IMU
   - IMU yaw 是否存在延迟

---

### 10. 下一步验证计划

**Test 1：确认 Laser 正前方向**

```bash
ros2 run tf2_ros tf2_echo base_footprint laser
```

确认 laser frame 方向。

**Test 2：固定速度旋转实验**

记录不同 `angular.z`（如 -0.05、-0.1、-0.2）下偏差角是否变化。

**Test 3：检查 IMU 输入**

```bash
ros2 topic echo /imu
```

确认 yaw 变化是否正常。

---

### 当前核心假设

LaserScan 数据本身的角度定义或时间处理存在问题，导致机器人旋转时 scan 在空间中的旋转不能正确反映机器人真实旋转。

**当前状态：** Localization 框架正常，问题集中进入 **LaserScan 数据链路**。

---

### 参考文档

- [2026-07-11 Nav2 Debug Log](./2026-07-11_nav2_debug.md) — Odom yaw 链路排查记录（v7）
