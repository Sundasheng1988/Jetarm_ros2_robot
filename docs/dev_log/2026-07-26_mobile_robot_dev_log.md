# 2026-07-26 移动机器人开发记录  
## 小车物理改造、CW/CCW 复测、离线分析工具标准化与阶段结论

> **工作日期**：2026-07-26  
> **整理日期**：2026-07-27  
> **项目阶段**：移动底盘 Localization 上游根因隔离  
> **测试根目录**：`/home/sundasheng/ros2_ws/test_logs/lidar_tf_validation_v2_20260726`  
> **说明**：物理改造与四组测试数据采集发生于 7 月 26 日；四包联合离线分析和最终汇总在 7 月 27 日完成。本文只记录对话、rosbag、现场记录、分析输出和工具清单能够支持的内容。

---

# 0. 本次工作的目标

本次工作的核心不是直接宣告“历史地图旋转 90° 的根因已经找到”，而是完成以下四个问题的隔离：

1. 新的雷达静态 TF 是否真实发布并记录进 rosbag；
2. 修正雷达安装外参后，LaserScan 是否能够稳定、可信地测出约 90° 旋转；
3. LaserScan 所反映的真实运动是否与人工观察、IMU、Odom 和 TF 一致；
4. 当前异常更像雷达 TF 配置问题，还是 Odom / 实际运动描述问题。

同时重新检查之前观察到的 CW / CCW 方向不对称现象，并建立可重复使用的标准化离线分析工具链。

---

# 1. 小车物理改造与硬件调查

## 1.1 已确认完成的物理改造

本次四个 CW / CCW 测试包全部基于 7 月 26 日完成的新硬件结构采集，不是旧的满载机械臂结构。

### 改造前结构

改造前照片显示：

- 底盘左侧安装一块大型锂电池；
- 右侧安装完整机械臂总成；
- 雷达、计算单元、电池和机械臂共同安装在上层平台；
- 大质量部件分布在底盘两侧和较高位置，整体载荷较大，质量分布明显不均匀。

### 改造后结构

改造后照片显示：

- 约 4 kg 的大型锂电池已拆除；
- 一套完整机械臂结构已拆除；
- 计算单元和雷达重新布置在较低、较集中的上层安装板上；
- 上层结构明显简化，悬臂载荷和高位载荷减少；
- 用户已重新调整整车重心。

**Confirmed Fact：**

1. 本次测试使用的是完成轻量化改造后的新硬件结构；
2. 已拆除约 4 kg 的锂电池；
3. 已拆除一套完整机械臂结构；
4. 剩余计算单元和雷达已重新布置；
5. 整车总质量、质心位置、转动惯量和各轮法向载荷分布均已相对旧结构发生变化。

**Direct Observation：** 从改造前后照片可以明确看到，旧结构中的大型电池和机械臂总成在新结构中均已移除；新结构上层载荷更低、更集中。

**Inference：**

- 总质量下降会降低达到相同角加速度所需的驱动力矩；
- 高位和偏置载荷减少，会降低偏航转动惯量以及左右轮/脚轮受力不均；
- 重心重新布置可能改变轮胎附着、脚轮拖曳、原地旋转中心和旋转时的横向滑移；
- 因此，新结构下 CW / CCW 约 23 秒完成 90°，不能直接与旧 E01 满载结构的 CW 约 20 秒、CCW 约 48–50 秒做单变量对比。

**Unknown：**

- 机械臂总成的准确质量；
- 改造前后整车总质量；
- 改造前后质心的精确三维坐标；
- 四个车轮/脚轮的静态轮荷；
- 轻量化、重心调整和测试协议变化分别贡献了多少改善。

### 雷达安装位置调整

四个新测试包中均记录了新的静态 TF：

```text
base_footprint -> laser
translation = [0.105, 0.000, 0.210] m
rotation = identity
```

旧 E00 / E01 基线使用的是：

```text
translation = [0.000, 0.000, 0.150] m
rotation = identity
```

因此，相对旧配置，本次新配置在 `base_footprint` 坐标系中表现为：

- 雷达前向偏置增加：`+0.105 m`
- 雷达高度增加：`+0.060 m`
- 雷达安装朝向保持 identity rotation

四个 rosbag 的分析均得到：

```text
match_expected = true
static TF conflict = false
```

**Confirmed Fact：** 新雷达安装外参已经正确写入四个测试包，不存在“参数修改但实际没有生效”的证据。

## 1.2 当天完成的其他硬件调查

7 月 26 日的对话还记录了以下硬件调查：

- 检查 Orin Nano 在原机械结构中的安装方式；
- 检查锂电池接线方式，讨论充电和放电是否共用同一接口；
- 讨论充电时是否需要关闭电源开关。

这些调查与底盘轻量化改造处于同一天，但当前证据仍不足以确认：

- 电池开关、BMS 和充放电接口的最终接线方案；
- 充电操作规范是否已通过电池或 BMS 厂商资料确认。

因此，以上电气操作细节仍保留为 `Unknown`，不与已经确认完成的机械轻量化改造混为一谈。

---

# 2. CW / CCW 单次旋转复测

## 2.1 测试输入

四个测试包：

```text
TF01_CW_zminus0p15_n230_run01_lightweight
TF01_CW_zminus0p15_n230_run02_lightweight
TF02_CCW_zplus0p15_n230_run01_lightweight
TF02_CCW_zplus0p15_n230_run02_lightweight
```

命令条件：

- CW：`angular.z = -0.15 rad/s`
- CCW：`angular.z = +0.15 rad/s`
- 发布频率约 `10 Hz`
- 每个方向重复两次
- 每次目标约 90°

## 2.2 实际非零命令数量

| Run | 方向 | Cmd 总数 | 非零 Cmd | 零速 Cmd | 非零持续时间 |
|---|---:|---:|---:|---:|---:|
| CW Run01 | CW | 233 | 230 | 3 | 23.001 s |
| CW Run02 | CW | 234 | 230 | 4 | 23.001 s |
| CCW Run01 | CCW | 243 | 238 | 5 | 23.802 s |
| CCW Run02 | CCW | 238 | 238 | 0 | 23.761 s |

**Confirmed Fact：**

- CCW Run01 的 243 是 `/cmd_vel` 总消息数，不是非零运动命令数；
- 实际非零命令为 238，另外 5 条是零速停止命令；
- 本次 CW 与 CCW 的命令持续时间都约 23 秒，未复现旧 E01 中 CW 约 20 秒、CCW 约 48–50 秒的严重时间不对称。

---

# 3. Yaw 对比：人工、雷达、IMU、Odom 与 TF

为便于比较，CW 记为负，CCW 记为正。

| Run | 人工记录 | LaserScan | IMU orientation | IMU gyro 积分 | Odom yaw | TF yaw |
|---|---:|---:|---:|---:|---:|---:|
| CW Run01 | -88.000° | -87.294° | -88.664° | -81.340° | -88.664° | -88.664° |
| CW Run02 | -91.000° | -91.279° | -92.432° | -84.971° | -92.432° | -92.432° |
| CCW Run01 | +90.000° | +90.412° | +92.329° | +85.143° | +92.328° | +92.328° |
| CCW Run02 | +93.000° | +94.318° | +97.514° | +90.315° | +97.514° | +97.514° |

## 3.1 雷达相对人工记录的准确度

| Run | LaserScan 与人工的绝对差 |
|---|---:|
| CW Run01 | 0.706° |
| CW Run02 | 0.279° |
| CCW Run01 | 0.412° |
| CCW Run02 | 1.318° |

平均绝对差约：

```text
0.679°
```

四次 LaserScan 配准均为：

```text
valid = true
confidence = high
```

**Confirmed Fact：** 新雷达 TF 条件下，LaserScan 对 yaw 的测量与人工记录高度一致。本次测试中，雷达没有把真实约 90° 的旋转误判成明显不同的角度。

## 3.2 CW / CCW 的 yaw 对称性

LaserScan 平均绝对转角：

```text
CW  ≈ 89.29°
CCW ≈ 92.37°
```

CCW 比 CW 多约 3°，但 CCW 同时比 CW 多 8 条非零命令。

按非零 Cmd 数量归一化：

| Run | Scan 绝对 yaw / 非零 Cmd |
|---|---:|
| CW Run01 | 0.3795°/Cmd |
| CW Run02 | 0.3969°/Cmd |
| CCW Run01 | 0.3799°/Cmd |
| CCW Run02 | 0.3963°/Cmd |

方向均值：

```text
CW  ≈ 0.38820°/Cmd
CCW ≈ 0.38809°/Cmd
```

**Derived Result：** 在本次轻量化、±0.15 rad/s、约 23 秒测试条件下，CW 与 CCW 的单位命令实际转角几乎完全一致。

**Rejected Hypothesis（仅针对本次新硬件结构和测试条件）：** “CCW 本身明显比 CW 慢，导致达到约 90° 需要两倍以上时间”没有被复现。

**Evidence Boundary：** 由于本次测试同时移除了约 4 kg 电池和完整机械臂，并调整了重心，新测试与旧 E01 不是同一机械载荷条件。本结果只能说明严重方向时间不对称在新结构下没有复现，不能单独证明旧 E01 的异常完全来自软件或已经永久消失。

## 3.3 IMU orientation、Odom 与 TF 的证据边界

四个 Run 中：

```text
IMU orientation yaw ≈ Odom yaw ≈ TF yaw
```

残差小于约 `0.0004°`。

这并不代表存在三份独立物理证据。当前链路为：

```text
IMU orientation
    -> Odom pose orientation
    -> odom_combined -> base_footprint TF
```

**Confirmed Fact：** 三者的一致性主要证明复制和发布链内部一致，不能当成三种独立传感器共同验证真实 yaw。

相对独立的物理证据是：

- 人工现场记录；
- LaserScan SE(2) 配准；
- IMU `angular_velocity.z` 的离线积分。

## 3.4 IMU gyro 积分偏小

四个 Run 中，gyro 积分比 IMU orientation 少约 7.2°：

| Run | Orientation 与 gyro 积分差 |
|---|---:|
| CW Run01 | 7.324° |
| CW Run02 | 7.461° |
| CCW Run01 | 7.186° |
| CCW Run02 | 7.199° |

当前工具中：

- gyro 主要在 `R1` 非零命令窗口积分；
- orientation 使用 `S0` 静止窗口到 `S1` 静止窗口的姿态差。

**Inference：** 约 7° 差异至少可能部分来自积分窗口不完全一致。

**Unknown：** 在统一时间窗口后，是否仍存在 gyro 比例系数、时间基准或坐标投影误差。

---

# 4. 位移对比：人工、雷达、Odom 与 TF

| Run | 人工中心位移 | LaserScan 位移 | Odom 位移 | TF 位移 |
|---|---:|---:|---:|---:|
| CW Run01 | 14.0 cm | 10.875 cm | 0.442 cm | 0.442 cm |
| CW Run02 | 22.0 cm | 11.066 cm | 0.619 cm | 0.619 cm |
| CCW Run01 | 10.0 cm | 2.573 cm | 0.042 cm | 0.042 cm |
| CCW Run02 | 10.0 cm | 3.302 cm | 0.813 cm | 0.813 cm |

LaserScan 位移分量：

| Run | Scan dx | Scan dy | Scan 总位移 |
|---|---:|---:|---:|
| CW Run01 | -0.233 cm | -10.873 cm | 10.875 cm |
| CW Run02 | +0.375 cm | -11.059 cm | 11.066 cm |
| CCW Run01 | -1.542 cm | +2.091 cm | 2.573 cm |
| CCW Run02 | -2.132 cm | +2.521 cm | 3.302 cm |

## 4.1 雷达位移的重复性

- CW 两次：10.875 cm、11.066 cm，差约 0.19 cm；
- CCW 两次：2.573 cm、3.302 cm，差约 0.73 cm。

**Direct Observation：** LaserScan 位移结果在同方向重复测试中较稳定，不像随机配准输出。

## 4.2 人工与雷达的关系

人工记录与 LaserScan 都显示：

```text
CW 位移 > CCW 位移
```

但绝对数值没有完全一致。

**Confirmed Fact：** 人工与 LaserScan 对方向性排序一致。

**Unknown：** 人工记录中的“车体中心”是否与 `base_footprint` 原点相同。由于参考点没有统一，当前不能把人工厘米值和 Scan 的 `base_footprint` SE(2) 平移作为同一物理量直接校准。

## 4.3 Odom 平移漏记

四个 Run 的 Odom / TF 平移都接近零，而人工和 LaserScan 均观察到实际平面位移。

**Confirmed Fact：**

- 机器人旋转过程中并非严格绕 `base_footprint` 原地转动；
- Odom / TF 没有完整记录旋转过程中发生的平面运动；
- CW 的 Scan 平面位移约 11 cm，明显大于 CCW 的约 2.6–3.3 cm。

**Unknown：** 位移不对称来自轮胎打滑、左右轮运动差、旋转中心偏移、脚轮/机械阻力、编码器/运动学模型，还是其他原因。

---

# 5. 本次四个目标的回答状态

| 问题 | 状态 | 当前结论 |
|---|---|---|
| 1. 新雷达 TF 是否正确发布和记录 | 已回答 | 四包均为 `[0.105, 0, 0.210]`、identity、无冲突 |
| 2. 雷达能否准确测出约 90° 旋转 | 已回答 | 与人工平均差约 0.68°；四次均 valid/high |
| 3. 雷达真实运动是否与 IMU/Odom/TF 一致 | 已回答 | yaw 大致一致；平移明显不一致；Odom/TF 漏记平移 |
| 4. 历史问题来自雷达 TF 还是 Odom/运动链 | 部分回答 | 当前雷达 TF 已排除；Odom 运动描述存在明确缺口；历史地图旋转根因未闭环 |

---

# 6. 离线分析工具开发与标准化

## 6.1 新增/整理的活动工具

工具索引中定义了以下六个用途明确的活动工具：

| 工具 | 专用场景 |
|---|---|
| `analyze_single_rotation_cmd_imu_odom_tf.py` | 单次旋转 `S0 -> R1 -> S1` 的 Cmd、IMU、Odom、TF 分析 |
| `register_laserscan_static_window_pair_se2.py` | 两个静态窗口之间的独立 LaserScan SE(2) 配准 |
| `analyze_e01_four_rotation_sequence.py` | 历史 E01 四段旋转序列 |
| `audit_e00_stationary_bag_integrity.py` | E00 静止基线和数据完整性审计 |
| `analyze_localization_scan_tf_timing.py` | Localization 中 Scan/TF 时间关系诊断 |
| `analyze_localization_map_odom_frame_isolation.py` | map 与 odom 坐标系异常隔离 |

## 6.2 工具选择规则

已建立：

```text
/home/sundasheng/ros2_ws/tools/TOOLS_INDEX.md
```

核心规则：

1. 按实验形态选工具，不按创建日期或版本号猜测；
2. 单次 CW / CCW 先分析 Cmd/IMU/Odom/TF，再做独立 Scan 配准；
3. E01、E00、Localization timing、map/odom isolation 各用专用工具；
4. 不一次运行全部工具；
5. 工具不适配时停止，不擅自修改工具或设计新算法。

## 6.3 工具边界已明确

- LaserScan 配准不使用 Odom、IMU 或动态 TF 作为初值；
- 静态 TF 只用于把 Scan 表达到 `base_footprint`；
- Localization 两个工具的墙线角使用 4-theta / 90° 周期折叠，不能表示绝对 90° 墙线旋转；
- Localization 工具使用单一中点 TF 处理整帧 Scan，没有逐光束 deskew；
- 大范围 yaw 搜索计算量较高，应串行运行；
- `v6_postprocess_frame_isolation.py` 未获得完整源码，不作为活动工具推荐。

## 6.4 Agent 执行规则标准化

`CLAUDE.md` 增加了：

- 白名单输入路径；
- 低上下文模式；
- 用户负责硬件和长时间 ROS 运行；
- Agent 可执行聚焦源码检查、离线分析、指定包构建和验证；
- 不发布 `/cmd_vel`；
- 不修改 Jetson 运行时代码；
- 不启用子 Agent、后台 Agent 或并行重计算；
- 长输出写入日志；
- 结论必须区分 Confirmed Fact、Direct Observation、Inference、Rejected Hypothesis 和 Unknown；
- 任务结束后停止，不自动扩展到旧任务。

本次四包分析验证了这些规则整体可用。

## 6.5 仍需确认的工具命名一致性

`TOOLS_INDEX.md` 的标准活动名称为：

```text
analyze_single_rotation_cmd_imu_odom_tf.py
```

但本次四包汇总执行记录中实际调用的是：

```text
analyze_single_rotation_bag.py
```

**Unknown：** 后者是旧文件、兼容别名，还是活动目录中仍未完成替换的副本。

这是工具安装/命名一致性问题，不影响本次已经产出的数值，但下一次使用前应确认并统一。

---

# 7. 当前最可靠的阶段结论

## Confirmed Fact

1. 新雷达静态 TF `[0.105, 0, 0.210]` 已正确记录，方向为 identity，无冲突；
2. LaserScan 对 yaw 的结果与人工记录高度一致，平均绝对差约 0.68°；
3. 四次 Scan 配准全部 valid、high confidence；
4. 本次拆除约 4 kg 电池和机械臂、重新调整重心后的轻量化结构中，CW 和 CCW 均在约 23 秒内完成约 90° 旋转；
5. 按非零 Cmd 归一化后，CW 与 CCW 的实际 yaw 响应几乎完全一致；
6. 旧 E01 的严重 CW/CCW 时间不对称没有在本次测试中复现；
7. IMU orientation、Odom yaw 和 TF yaw 是一条复制/发布链，不是三份独立物理证据；
8. Odom / TF 严重低估旋转过程中发生的平面位移；
9. CW 的 Scan 平面位移约 11 cm，明显大于 CCW 的约 2.6–3.3 cm；
10. 当前雷达 TF 配置本身不能解释电机或轮速变化。

## Inference

1. CCW 总转角略大，主要可由额外 8 条非零 Cmd 和约 0.8 秒更长命令时间解释；
2. gyro 积分与 orientation 相差约 7°，至少部分可能来自积分窗口不一致；
3. 当前主要异常已经从“雷达 TF 是否生效”转向“真实运动与 Odom 运动描述不一致”。

## Rejected Hypothesis

仅针对本次测试条件，可以否定：

- 新雷达 TF 没有生效；
- LaserScan 无法正确测出约 90° 旋转；
- CCW 必然需要 CW 两倍以上的时间才能完成相同角度；
- IMU orientation、Odom 和 TF 是三份独立证据。

## Unknown

1. CW 平移明显大于 CCW 的物理原因；
2. 人工中心位移与 `base_footprint` 位移之间的参考点关系；
3. 同窗口积分后 gyro 是否仍有稳定比例误差；
4. CCW 包中 `odom_missing_tf=24/51` 是否影响下游 SLAM/AMCL；
5. 旧 E01 的严重时间不对称在原载荷/协议下是否会再次出现；
6. STM32、PID、PWM、编码器、轮距、机械阻力或电机驱动是否参与位移异常；
7. 历史地图整体约 90° 旋转的完整根因。

---

# 8. 下一步开发方向（修订）

后续工作按照**两条主线**推进。第一条主线包含两个相互关联但必须分别验证的子任务；第二条主线恢复 Localization / Nav2 下游验证。

---

## 主线 A — 上游实体运动与雷达外参

### A1. 继续研究新物理结构下的 CW / CCW 不一致

#### 目标

解释新轻量化结构下仍然存在的方向差异，重点包括：

- CW / CCW 平面位移不一致；
- CW Scan 位移约 11 cm，而 CCW 约 2.6–3.3 cm；
- 实际旋转中心是否随方向变化；
- 重心、轮荷、轮胎滑移、从动轮/脚轮阻力和机械安装是否参与该差异。

当前已经确认的是：

```text
yaw 响应：CW / CCW 基本对称
平面位移：CW 明显大于 CCW
```

因此下一阶段不再只问“CCW 为什么慢”，而是研究：

```text
为什么相同量级的 yaw 会伴随不同的二维平移
```

#### 测试原则

1. 固定当前新硬件结构，不在同一轮测试中同时改变多个变量；
2. CW / CCW 使用相同角速度、命令频率、非零命令数量和起止逻辑；
3. 每个方向至少重复 3 次；
4. 记录相同的人工参考点、LaserScan、Odom、TF 和 IMU；
5. 更换起始朝向和测试位置，用于区分底盘方向性与地面方向性；
6. 记录电池电压、地面条件、轮胎/从动轮初始朝向和可见机械状态；
7. 一次只改变一个物理变量，例如重心位置、附加载荷位置或从动轮初始状态。

#### 关键测量

- 在底盘上明确标记 `base_footprint` 的物理地面投影点；
- 记录旋转前后该点的二维坐标；
- 记录车体 yaw；
- 记录四轮/支撑点的静态受力条件；若具备条件，可分别称量各轮轮荷；
- 使用俯视视频或固定相机记录完整轨迹；
- 人工记录必须测量同一个刚体参考点，不再使用未定义的“车体中心”。

#### 预期输出

```text
physical_cw_ccw_test_matrix.csv
physical_reference_point_measurements.csv
field_observation.md
对应 rosbag
俯视视频或图像
```

---

### A2. 区分真实平移与雷达 TF 平移误差

#### 当前证据边界

本次四包已经证明：

```text
TF 中记录的值 = [0.105, 0.000, 0.210]
match_expected = true
static TF conflict = false
```

这只证明：

> TF 消息按照设定值成功发布并写入 rosbag。

它还没有完全证明：

> `[0.105, 0.000, 0.210]` 就是雷达光学中心相对 `base_footprint` 的真实物理外参。

尤其需要区分：

1. **TF 配置状态是否正确写入；**
2. **TF 数值是否与实物安装几何一致；**
3. **LaserScan 推算出的平移是否为真实 `base_footprint` 平移。**

当前 LaserScan 工具先使用静态 TF 将点云表达在 `base_footprint`，再进行 SE(2) 配准。因此：

- 静态 TF 的平移 `x/y` 不准确，会直接影响推算出的 base 平移；
- 静态 TF 的 yaw 不准确，会影响平移向量的方向表达；
- 相对 yaw 本身对固定杆臂偏差不敏感，因此“Scan yaw 与人工一致”不能单独证明整个静态外参准确。

#### 为什么现有四包不能单独解决该问题

当前自由旋转测试中，以下两种效应同时存在：

```text
机器人自身真实发生平移
雷达静态外参可能存在误差
```

只依靠同一个 LaserScan 配准结果，无法完全把两者分开。

必须增加一份独立的 `base_footprint` 真实运动基准。

#### 推荐验证方法

##### 方法 1：外部视觉真值，优先推荐

- 在底盘上安装 ArUco / AprilTag 或清晰刚体标记；
- 使用固定俯视相机；
- 独立计算底盘参考点的 `x、y、yaw`；
- 同时计算 LaserScan 在候选 TF 下得到的 `x、y、yaw`；
- 比较外部视觉、LaserScan、Odom 和人工测量。

##### 方法 2：明确物理参考点的人工测量

- 在底盘上标记 `base_footprint` 地面投影点；
- 使用垂线、激光点或刚性定位杆把该点投影到地面；
- 旋转前后测量同一个点的二维位置；
- 测量雷达光学中心相对该点的物理 `x/y/z`；
- 检查雷达安装朝向相对车体轴线的 yaw。

##### 方法 3：受约束旋转标定

在能够机械约束已知旋转中心的条件下：

- 让指定的 `base_footprint` 参考点保持不动；
- 分别做 CW / CCW 和多个角度；
- 正确 TF 应使 Scan 推算出的 base 平移接近外部真值；
- 若仍出现稳定残余平移，再调整外参或检查点云配准模型。

#### 本子任务需要回答

```text
Q1. base_footprint 的物理原点究竟在哪里？
Q2. 雷达光学中心相对该原点的真实 x/y/z/yaw 是多少？
Q3. 当前 [0.105, 0, 0.210] 与实测值是否一致？
Q4. 使用实测 TF 后，Scan 平移与外部真值是否一致？
Q5. CW / CCW 平移差异中，有多少是真实底盘运动，有多少来自外参误差？
```

---

## 主线 B — 恢复 Localization / Nav2，并录制完整证据

### B1. 为什么现在可以恢复下游验证

当前新结构测试中：

- 没有再次出现 yaw 与 Odom 相差巨大的现象；
- CW / CCW 实际 yaw 基本对称；
- 新雷达 TF 已按照设定值稳定记录；
- LaserScan yaw 与人工记录一致；
- 但 Odom 平移漏记和 CW / CCW 平移不一致仍存在。

因此现在有必要恢复 Localization / Nav2，观察这些上游现象在下游系统中如何表现。

这一步的目的不是假定问题已经解决，而是检查：

```text
Scan 是否继续相对地图偏移
map -> odom 是否出现异常修正
AMCL 是否能吸收或放大 Odom 平移误差
历史约 90° 地图旋转是否还会复现
Nav2 转弯和回程时是否出现定位失稳
```

### B2. 分阶段执行

#### 阶段 1：Localization only

先启动：

- map server；
- AMCL；
- LaserScan；
- Odom / TF；
- RViz。

先不启动 Nav2 控制器。

测试动作：

1. 初始化位姿；
2. 静止观察；
3. 原地 CW 约 90°；
4. 原地 CCW 约 90°；
5. 短距离直行；
6. 转弯后返回；
7. 观察 Scan 与 map 是否持续重合；
8. 观察 `map -> odom_combined` 是否跳变；
9. 观察 `/amcl_pose` 和粒子云是否异常旋转或漂移。

#### 阶段 2：Nav2 短路径验证

只有 Localization only 没有出现明显失稳后，再启动 Nav2：

- 固定初始位姿；
- 固定短距离目标点；
- 低速运行；
- 先单程，再回程；
- 最后再测试包含 90° 转弯的路线。

不得一开始就运行长路径或复杂路线，否则难以隔离异常发生阶段。

### B3. rosbag 录制原则

为了区分发布侧与 PC 网络记录侧问题，优先采用：

```text
Jetson 端主 rosbag：记录源端事实
PC 端可选同步 rosbag：观察 DDS/网络传输差异
```

关键话题至少包括：

```text
/scan
/tf
/tf_static
/odom_combined
/mobile_base/sensors/imu_data
/cmd_vel
/amcl_pose
/particle_cloud
/map
/map_updates
/initialpose
/goal_pose
/plan
```

实际话题名应在启动后用 `ros2 topic list` 确认，不能仅根据历史名称猜测。

终端日志应单独保存：

```text
localization_launch.log
nav2_launch.log
rviz_observation.md
field_observation.md
```

### B4. 本主线的判断标准

需要分别回答：

1. 旋转后 Scan 是否仍与 map 重合；
2. `map -> odom_combined` 是否发生持续或突发的大角度修正；
3. Odom 平移漏记是否被 AMCL 修正；
4. AMCL 修正后是否产生地图整体旋转或跳变；
5. CCW 的 TF 交付缺口是否再次出现；
6. Nav2 回程时是否比去程更容易失稳；
7. 新轻量化结构和新 TF 下，历史问题是否仍可复现。

---

## 两条主线的关系

```text
主线 A：
解释真实底盘运动，并校准雷达外参

主线 B：
观察这些上游状态进入 Localization / Nav2 后的系统表现
```

两条主线可以交替推进，但证据不能混用：

- Localization / Nav2 表现不能替代雷达物理外参标定；
- 雷达外参正确也不能自动证明 Odom 或 AMCL 正常；
- 下游没有复现地图旋转，也不能单独证明物理平移问题已经消失；
- 物理测试发现位移差异，也不能直接宣称它就是历史地图旋转的唯一根因。

当前推荐顺序：

```text
A1 建立统一物理参考点
A2 完成一次 TF / 实际平移对照
B1 Localization only 录包
B2 Nav2 短路径录包
A1/A2 根据下游现象继续迭代
```
# 9. 关键输出文件

四包联合输出目录：

```text
/home/sundasheng/ros2_ws/test_logs/lidar_tf_validation_v2_20260726/analysis_cw_ccw_4runs
```

核心文件：

```text
four_run_metrics.csv
four_run_scan_results.csv
cw_ccw_comparison.md
evidence_index.csv
claude_md_compliance_report.md
```

工具索引：

```text
/home/sundasheng/ros2_ws/tools/TOOLS_INDEX.md
```

---

# 10. 最终状态快照

```text
雷达 TF 消息：按设定值写入，无冲突
雷达物理外参精度：尚待独立标定
雷达 yaw 测量：准确、稳定、高置信度
CW/CCW yaw：本次测试基本对称
旧 E01 严重时间不对称：本次未复现
真实平面位移：明确存在
CW/CCW 平移：明显不对称，CW > CCW
Odom/TF 平移：严重漏记
gyro 与 orientation 差异：待统一窗口验证
Localization/Nav2：准备恢复并录制新 bag
历史地图旋转根因：尚未闭环
下一阶段：物理/TF 标定主线 + Localization/Nav2 回归主线
```
