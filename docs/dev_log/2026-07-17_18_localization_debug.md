        # Localization Debug Log (2026-07-17 ~ 2026-07-18)

        > **阶段**: Yaw 链路根因定位 → IMU yaw 替换 → Baseline 基准测试 → 故障链时序分析
        > **版本**: v2
        > **上一份日志**: [2026-07-13 Laserscan Map Rotation Debug](./2026-07-13_laserscan_map_rotation_debug.md)

        ---

        ## Background

        自 2026-07-11 起，机器人旋转后 LaserScan 与静态地图出现角度偏移。前期排查已排除 TF 静态外参、AMCL 完全失效等问题。本次调试分两阶段：

        - **7/17-7/18 白天**: 量化 yaw 误差层级，实施 IMU yaw 替换，初步验证
        - **7/18 晚间**: 建立 Baseline v0 基准测试体系，执行 A→B→A 测试，捕获故障链完整时序

        ---

        ## Objectives

        1. 量化 Odom yaw 与真实旋转角度的误差
        2. 确认误差来源位于 ROS2 积分层还是 STM32 上游
        3. 评估 IMU yaw 作为替代源的可行性并实施替换
        4. 建立可复现的基准测试体系 (Baseline v0)
        5. 通过 A→B→A 测试捕获完整故障链，定位根因层级

        ---

        ## Environment

        | 项目 | 值 |
        |------|-----|
        | 机器人平台 | 差速轮式底盘 (DLRobot) |
        | 运行节点 | Jetson (全部节点) |
        | ROS 版本 | Humble |
        | IMU | MPU6050 (Mahony AHRS, 无磁力计) |
        | 雷达 | RPLIDAR |
        | 地面 | 木地板 |
        | 标定基准 | 地面胶带 90°/180° 十字标记 |
        | 测试路线 | A 点 (房间起点) ↔ B 点 (目标位置)，含直行 + 转弯 |

        ---

        ## Experiments

        ### H1: Odom yaw 存在方向不对称且不可通过单比例系数修正

        **Hypothesis**: 轮式里程计 yaw 在顺时针和逆时针方向存在不同量级的误差，且不能通过统一系数修正。

        **Goal**: 量化 CW/CCW 方向下 Odom yaw、RobotVel integral、IMU orientation 的误差。

        **Procedure**:

        1. 底盘静止 5 秒后，以 `angular.z = -0.05` 顺时针旋转至胶带标定的 90°，记录 odom yaw。重复 3 次。
        2. 以 `angular.z = +0.05` 逆时针旋转至 90°，重复 3 次。
        3. 每次之间重启底盘驱动使 odom yaw 归零。
        4. 录制 topics: `/odom_combined`, `/robotvel`, `/mobile_base/sensors/imu_data`, `/cmd_vel`, `/tf`。

        **Commands**:

        ```bash
        ros2 bag record /odom_combined /robotvel /mobile_base/sensors/imu_data /cmd_vel /tf \
        -o ~/ros2_ws/debug_bags/odom_yaw_calibration

        ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.05}}"

        ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
        ```

        **Observed Result**:

        | 测试 | 方向 | cmd_vel 持续 | Odom yaw | IMU orientation yaw |
        |------|------|-------------|----------|---------------------|
        | 1 | CW -0.05 | 60.36 s | -73.32° | -94.36° |
        | 2 | CW -0.05 | 62.00 s | -73.05° | -93.58° |
        | 3 | CW -0.05 | 61.10 s | -73.32° | -93.01° |
        | 4 | CCW +0.05 | 67.80 s | +85.30° | +94.15° |
        | 5 | CCW +0.05 | 95.90 s | +118.42° | +100.49° |
        | 6 | CCW +0.05 | 96.80 s | +122.18° | +99.14° |

        RobotVel.z 积分与 Odom yaw 高度一致（偏差 < 2°），说明 ROS2 C++ 积分层 (`Robot_Pos.Z += Robot_Vel.Z * Sampling_Time`) 工作正常。

        **Conclusion**:

        - **Confirmed**: CW 方向 Odom 少计约 17°（平均 73.23° vs 真实 90°），重复性高（std < 0.2°）
        - **Confirmed**: CCW 方向 Odom 不稳定（85°~122°），存在打滑导致的运行间差异
        - **Confirmed**: IMU 整体比 Odom 更接近真实角度，但仍有 3°~10° 误差
        - **Confirmed**: RobotVel.z integral ≈ Odom yaw，ROS2 积分层正确，误差来源于 STM32 输出的 `Robot_Vel.Z`

        ---

        ### H2: IMU 静态零偏是 IMU yaw 误差的主要来源

        **Hypothesis**: 长时间低速旋转放大了陀螺仪零偏，导致 IMU yaw 累计误差。

        **Goal**: 测量静止状态下 gyro z 零偏，判断是否能解释 IMU 的 3°~10° 误差。

        **Procedure**:

        1. 录制包含多段静止区间的 rosbag
        2. 分析四次静止段 (初始、CW后、复位后、CCW后) 的 `angular_velocity.z` 均值
        3. 比较静态零偏积分与 IMU orientation 实际漂移

        **Observed Result**:

        | 静止阶段 | Gyro Z 均值 |
        |----------|------------|
        | 初始静止 | 0.000149 rad/s |
        | 顺时针后 | 0.000135 rad/s |
        | 手动复位后 | 0.000130 rad/s |
        | 逆时针后 | 0.000168 rad/s |

        平均零偏: 0.000146 rad/s ≈ 0.00834°/s。100 秒理论累计 < 1°。

        **Conclusion**:

        - **Rejected**: 静态 gyro bias 不是 IMU 误差的主要来源
        - IMU 的 9°~13° 误差更可能是动态比例误差 (scale error)，而非固定偏置
        - CW 比例约 1.10，CCW 比例约 1.16，两方向不完全一致

        ---

        ### H3: 提高角速度可减少低速粘滑导致的误差

        **Hypothesis**: 将角速度从 ±0.05 提高到 ±0.10 rad/s 可减少打滑和积分时间，改善测量精度。

        **Goal**: 以 ±0.10 rad/s 执行 180° 旋转，比较 IMU 与 Odom 的误差变化。

        **Procedure**:

        1. 以 `angular.z = -0.10` 顺时针旋转至 180° 标记
        2. 以 `angular.z = +0.10` 逆时针旋转至 180° 标记
        3. 使用 analyze_yaw_bag_v2.py 自动分段分析

        **Observed Result**:

        | 数据源 | CW ΔYaw | 误差 (vs -180°) | CCW ΔYaw | 误差 (vs +180°) |
        |--------|---------|-----------------|----------|-----------------|
        | IMU Orientation | -186.97° | -6.97° | +190.98° | +10.98° |
        | IMU Gyro Integral | -188.91° | -8.91° | +191.92° | +11.92° |
        | Odom ΔYaw | -156.22° | +23.78° | +264.60° | +84.60° |
        | RobotVel Integral | -157.74° | +22.26° | +262.17° | +82.17° |

        CW/CCW 不对称性:

        | 数据源 | CW 误差 | CCW 误差 | 不对称比 |
        |--------|---------|----------|---------|
        | IMU Orientation | -6.97° | +10.98° | 1.6× |
        | IMU Gyro Integral | -8.91° | +11.92° | 1.3× |
        | Odom ΔYaw | +23.78° | +84.60° | 3.6× |
        | RobotVel Integral | +22.26° | +82.17° | 3.7× |

        逆时针耗时 (139s) 比顺时针 (112s) 多 27 秒 (约 +24%)。

        **Conclusion**:

        - **Observed**: IMU 链路 CW/CCW 对称性优于 Odom (不对称比 1.3~1.6× vs 3.6~3.7×)
        - **Confirmed**: Odom/RobotVel 方向不对称严重，CCW 误差约为 CW 的 3.6 倍
        - **Confirmed**: RobotVel ≈ Odom，确认两者同源 (STM32 串口输出)
        - **Confirmed**: IMU Orientation ≈ Gyro Integral (差 1°~2°)，内部一致
        - 提高速度未消除 Odom 不对称；current evidence points to STM32 或编码器层面
        - 轮距参数 (wheel_separation) 无法解释 CW/CCW 的 3.6 倍不对称比

        ---

        ### H4: ROS2 驱动源码包含编码器运动学计算

        **Hypothesis**: `dlrobot_robot.cpp` 中包含左右轮速度到 `Robot_Vel.Z` 的计算逻辑，可在 ROS2 层修复。

        **Goal**: 审查全部 ROS2 驱动源码，定位 `Robot_Vel.Z` 的计算链路。

        **Procedure**:

        1. 审查 `dlrobot_robot.cpp` 完整源码
        2. 审查 `Quaternion_Solution.cpp`
        3. 审查 `dlrobot_robot.h`
        4. 搜索编码器、轮距、轮速等关键词

        **Observed Result**:

        - `Robot_Vel.Z = Odom_Trans(Receive_Data.rx[6], Receive_Data.rx[7])` — 从 STM32 串口帧直接读取
        - 整个 ROS2 包无任何 `encoder`, `wheel_base`, `wheel_separation`, `left_speed`, `right_speed` 等关键词
        - `Robot_Pos.Z += Robot_Vel.Z * Sampling_Time` — 仅做时间积分
        - `Quaternion_Solution.cpp` 为 Mahony AHRS，使用固定 `SAMPLING_FREQ = 20.0f`
        - Odom yaw 和 IMU yaw 在代码中完全独立，无融合

        **Conclusion**:

        - **Confirmed**: ROS2 层无编码器运动学代码，`Robot_Vel.Z` 由 STM32 预计算后通过串口发送
        - **Confirmed**: ROS2 仅负责积分和发布，无法在 ROS2 层修复轮式 yaw 误差
        - **Confirmed**: 要修复轮式 yaw 需修改 STM32 固件（未获取源码）
        - IMU 姿态完全在 ROS2 层计算 (Mahony AHRS)，具备修改灵活性

        ---

        ### H5: 将 Odom yaw 来源从轮式里程计切换为 IMU 可修复定位

        **Hypothesis**: 使用 IMU orientation yaw 替代轮式 `Robot_Vel.Z` 积分作为 `/odom_combined` 的 yaw 来源，可消除 LaserScan 旋转偏移。

        **Goal**: 修改源码将 odom pose yaw 和 angular.z 从轮式切换为 IMU，并验证效果。

        **Procedure**:

        1. 修改 `dlrobot_robot.h`：增加 `imu_yaw_raw_`, `imu_yaw_offset_`, `imu_yaw_odom_`, `imu_yaw_initialized_` 状态变量
        2. 修改 `dlrobot_robot.cpp`：
        - 增加 `normalize_angle()` 函数
        - 在 `Quaternion_Solution()` 后提取 IMU yaw
        - 初始化时记录 `imu_yaw_offset_`
        - `Robot_Pos.Z = imu_yaw_odom_` (替代 `Robot_Pos.Z += Robot_Vel.Z * Sampling_Time`)
        - x/y 积分改用 `imu_yaw_odom_`
        - `odom.twist.twist.angular.z = Mpu6050.angular_velocity.z`
        - 保留 `/robotvel.z = Robot_Vel.Z` 作为 STM32 诊断通道
        3. 调整 `Control()` 执行顺序: 串口读取 → Quaternion_Solution → 提取 IMU yaw → x/y 积分 → 发布
        4. 编译并测试

        **Observed Result**:

        编译成功。启动驱动后验证：

        | 检查项 | 结果 |
        |--------|------|
        | `/odom_combined` 发布频率 | 20.008 Hz (稳定) |
        | `/mobile_base/sensors/imu_data` 发布频率 | 19.988 Hz (稳定) |
        | 静止初始 yaw | ≈ 0.19° (正常) |
        | 顺时针旋转 ~90° 后 yaw | ≈ -93.7° (幅度正确) |
        | 转回初始方向后 yaw | ≈ 5.09° (回零误差 4.9°) |
        | 静止 65 秒漂移 | 0.567° → ~0.52°/min |
        | IMU Orientation ≈ Odom yaw | 小数点后两位一致 (替换生效) |

        **Conclusion**:

        - **Verified**: IMU yaw 成功接入 `/odom_combined`
        - **Verified**: 动态旋转幅度正确 (90° → odom 约 93.7°)
        - **Verified**: 方向符号正确 (CW 为负, CCW 为正)
        - **Observed**: 存在约 5° 回零误差和 0.52°/min 静态漂移 (纯 IMU 正常现象)

        ---

        ### H6: IMU yaw 替换后 LaserScan 旋转偏移已消除

        **Hypothesis**: 切换 yaw 来源后，机器人旋转时 LaserScan 应能保持与地图重合。

        **Goal**: 在 RViz 中观察旋转前后 LaserScan 与地图的对齐情况。

        **Procedure**:

        1. 启动底盘驱动、雷达、odom_tf_bridge、AMCL、map_server
        2. 机器人初始朝向 0°，记录 odom yaw 并截图
        3. 顺时针旋转至约 -90°，记录 odom yaw 并截图
        4. 逆时针转回 0°，记录 odom yaw 并截图

        **Observed Result**:

        | 时刻 | 机械角度 | Odom yaw | LaserScan vs Map |
        |------|---------|----------|------------------|
        | 初始 | ~0° | ~0.19° | 基本重合 |
        | CW 旋转后 | ~-85° | -90.2° | 偏转约 5° (与转动方向同向) |
        | 转回 | ~0° | +8.37° | 反向偏转约 7-8° |

        - 点云偏转方向与 odom yaw 误差方向一致
        - 旋转过程中点云能够重新拟合地图

        **Conclusion**:

        - **Verified**: LaserScan 不再出现几十度级别的旋转偏移
        - **Verified**: 点云偏转 ≈ odom/IMU yaw 误差 (点云忠实跟随 TF，无独立漂移)
        - **Observed**: 残留约 5°~8° 误差，来自 IMU yaw 的回零残差
        - 原先问题 (旋转后点云严重失配) 已从"严重错误"降级为"小角度残差"

        ---

        ### H7: Nav2 在 IMU yaw 替换后可完成短距离导航

        **Hypothesis**: 当前 IMU yaw 精度足够支撑 Nav2 短距离直线导航。

        **Goal**: 执行 Nav2 Goal Pose，验证完整导航链路。

        **Procedure**:

        1. 启动 Nav2 (AMCL + Planner + Controller + BT Navigator)
        2. 在房间内设置约 3m 直线 Goal Pose: (-4.44, 2.00) → (-7.44, 1.64)
        3. 观察日志和实际运动

        **Observed Result**:

        - 导航距离: ~3.02m, 耗时约 19s, 平均速度约 0.16 m/s
        - `[bt_navigator]: Goal succeeded`
        - 全链路: bt_navigator → controller_server → cmd_vel → 底盘 正常

        **Conclusion**:

        - **Verified**: AMCL + map → odom_combined → base_footprint → laser TF 链正常
        - **Verified**: Controller 闭环控制有效，Goal Checker 正确判定到达
        - 首个完整 Nav2 成功导航里程碑

        ---

        ### H8: 在 Jetson 上重新建图可改善定位质量

        **Hypothesis**: 节点全部迁移至 Jetson 后，重新建图可生成当前架构下的基准地图。

        **Goal**: 在 Jetson 本地运行 SLAM Toolbox 重新建图，评估地图质量。

        **Procedure**:

        1. 停止 AMCL 和 Nav2
        2. 启动底盘驱动、雷达、odom_tf_bridge、slam_toolbox (online_async)
        3. 缓慢驾驶机器人完成全屋闭环
        4. 保存地图

        **Observed Result**:

        - 建图前半段：走廊和房间轮廓清晰正常
        - 回到起点附近后：地图左半部分发生约 90° 整体旋转
        - 地图呈现"拧转"效果：右侧保持原方向，左侧被旋转

        **Conclusion**:

        - **Observed**: 建图闭环时地图被大幅旋转 (~90°)
        - Current leading hypothesis: yaw 链路在闭环时的连续性/跳变
        - SLAM 参数调整无法修复此级别的结构性问题

        ---

        ### H9: 当前 IMU yaw 实现存在多项工程缺陷

        **Hypothesis**: 源码中存在导致 yaw 链路不稳定的实现问题，可通过逐个修正改善定位质量。

        **Goal**: 深度审查 IMU yaw 替换后的全部代码，列出所有已知缺陷并评估优先级。

        **Procedure**:

        1. 逐行审查 `dlrobot_robot.cpp` 中 IMU yaw 相关代码
        2. 检查陀螺仪零偏校准、协方差、frame 一致性、TF 发布、时间戳、积分逻辑
        3. 对每个问题评估严重度和修复优先级

        **Observed Result**:

        审查确认以下 11 个问题，按严重度排序：

        | # | 问题 | 严重度 | 影响 |
        |---|------|--------|------|
        | 1 | 无陀螺仪 Z 轴零偏标定 | P0 | 静止时 yaw 持续漂移，每分钟 ~0.5° |
        | 2 | IMU orientation_covariance[8] = 1e-6 | P0 | 过度高估 IMU 精度，后续滤波器无法修正 |
        | 3 | odom 消息 frame_id 默认 "odom"，AMCL 配置使用 "odom_combined" | P0 | frame 名不一致可能导致 TF 查询失败 |
        | 4 | C++ 节点不发布 TF，完全依赖外部 odom_tf_bridge_node.py | P0 | 存在 frame 不一致和多发布源风险 |
        | 5 | Sampling_Time 在串口读取前生成，且读取失败也更新 last_time | P1 | 异常 dt 导致积分跳跃 |
        | 6 | IMU 和 odom 消息各自调用 now()，同一串口包产生不同时间戳 | P1 | 时间戳不一致影响 TF 插值 |
        | 7 | Robot_Vel.Z 仍来自轮速，用于运动/静止状态判断 | P1 | 判断逻辑使用不可靠的轮速角速度 |
        | 8 | 运动状态判断使用 `Robot_Vel.X == 0` (浮点精确比较) | P2 | 传感器数据几乎不可能严格为 0 |
        | 9 | yaw unwrap 缺少异常跳变保护 | P2 | AHRS 偶发异常帧直接破坏 odom yaw |
        | 10 | 纯 AHRS orientation yaw 无磁力计约束，作为唯一 odom yaw 源 | P1 | 长期漂移不可避免 |
        | 11 | `Quaternion_Solution` 硬编码 SAMPLING_FREQ = 20.0f | P2 | 实际频率不严格匹配时引入比例误差 |

        **Conclusion**:

        - **Assessment**: 当前实现将 odom yaw 从轮速积分切换为 IMU AHRS 输出，但缺少 gyro bias 标定等基础校准
        - 对小角度测试有明显改善，但对长期运行和反复旋转仍有较高概率偏差
        - 最小修复顺序: frame 统一 → gyro bias 标定 → 协方差修正 → 时间戳统一 → 异常 dt/jump 保护

        ---

        ### H10: Baseline v0 系统拓扑检查

        **Hypothesis**: 建立系统拓扑基线，确认不存在 TF 多发布源、frame 不一致等基础配置问题。

        **Goal**: 在修改任何代码之前，记录当前系统的完整拓扑状态。

        **Procedure**:

        ```bash
        mkdir -p ~/ros2_ws/test_logs/baseline_v0

        ros2 node list | tee ~/ros2_ws/test_logs/baseline_v0/node_list.txt
        ros2 topic list -t | tee ~/ros2_ws/test_logs/baseline_v0/topic_list.txt
        ros2 topic info /tf --verbose | tee ~/ros2_ws/test_logs/baseline_v0/tf_publishers.txt
        ros2 topic info /tf_static --verbose | tee ~/ros2_ws/test_logs/baseline_v0/tf_static_publishers.txt
        ros2 topic echo /odom_combined --once | tee ~/ros2_ws/test_logs/baseline_v0/odom_once.txt

        timeout 15 ros2 topic hz /odom_combined | tee ~/ros2_ws/test_logs/baseline_v0/odom_hz.txt
        timeout 15 ros2 topic hz /mobile_base/sensors/imu_data | tee ~/ros2_ws/test_logs/baseline_v0/imu_hz.txt
        timeout 15 ros2 topic hz /scan | tee ~/ros2_ws/test_logs/baseline_v0/scan_hz.txt

        cd ~/ros2_ws/test_logs/baseline_v0
        ros2 run tf2_tools view_frames
        ```

        **Observed Result**:

        - odom 消息内部 frame: `frame_id: odom_combined`, `child_frame_id: base_footprint` (与 AMCL 配置一致)
        - TF 树结构: `map → odom_combined → base_footprint → base_link → laser`
        - 频率: odom/IMU ~20Hz, scan ~5.7Hz
        - A 点和 B 点坐标已保存为固定参考

        **Conclusion**:

        - **Verified**: 基础 TF 拓扑正常，frame 名一致
        - 建立 A、B 固定测试点，后续所有测试可复现

        ---

        ### H11: 手动 A→B→A 基准测试 (Baseline v0 run_01)

        **Hypothesis**: 在纯 AMCL 定位 (无 Nav2 控制) 下执行 A→B→A，可暴露定位链的漂移特征，排除 Nav2 控制变量。

        **Goal**: 建立 Baseline v0 的首个定位基准数据。

        **Procedure**:

        1. 启动底盘、雷达、TF、AMCL (不启动 Nav2)
        2. 机器人置于 A 点，设置初始位姿，等待 AMCL 收敛
        3. 录制 rosbag，人工遥控执行 A→B→A
        4. 记录到点物理误差和 RViz 截图

        **Observed Result**:

        - 回程终点 LaserScan 相对地图偏转约 50°
        - 分析结果:

        | 指标 | 全段变化 |
        |------|---------|
        | IMU yaw Δ | +182.06° |
        | Odom yaw Δ | +182.06° |
        | odom→base yaw Δ | +182.02° |
        | AMCL yaw Δ | -129.46° |
        | map→odom yaw Δ | +44.34° |

        - Odom 位置积分严重异常: `y ≈ -10.7 m` (实际路线仅数米)
        - IMU/Odom yaw 一致 (无内部矛盾)，但 AMCL 大幅修正 map→odom

        **Conclusion**:

        - **Confirmed**: IMU → odom → TF 链路内部一致 (IMU yaw ≈ Odom yaw ≈ odom→base yaw)
        - **Observed**: AMCL 修改了 map→odom (全段变化 +44°)
        - **Observed**: odom 位置积分严重漂移 (y ≈ -10.7 m)，x/y 积分逻辑或轮速单位需进一步检查
        - Current evidence suggests: AMCL 在被动响应 odom 误差

        ---

        ### H12: Nav2 A→B 自动导航基准测试 (nav2_run_01)

        **Hypothesis**: Nav2 可自动完成 A→B 导航，验收当前定位链在真实导航场景下的表现。

        **Goal**: 使用 Nav2 自动执行 A→B，评估到点精度和定位稳定性。

        **Procedure**:

        1. 启动完整 Nav2 (Planner + Controller + BT Navigator)
        2. 使用保存的 B 点坐标通过 `ros2 action send_goal /navigate_to_pose` 发送目标
        3. 记录到点误差和 Nav2 日志

        **Observed Result**:

        第一次尝试: ABORTED (4 次 recovery, current_pose 间歇变为 base_footprint/0,0)

        第二次尝试: SUCCEEDED

        | 指标 | 值 |
        |------|-----|
        | 到点位置误差 | ~10.7 cm |
        | 到点航向误差 | ~8°~11° |
        | goal_checker.xy_goal_tolerance | 0.15 m |
        | goal_checker.yaw_goal_tolerance | **0.20 rad** (~11.46°) |
        | goal_checker.stateful | true |

        **Conclusion**:

        - **Verified**: Nav2 可完成 A→B 导航
        - **Verified**: goal_checker.yaw_goal_tolerance = 0.2 rad 解释了 8°~11° 到点航向偏差仍判定成功
        - 动态修改为 `0.05 rad` (~2.86°) 后重新测试，预期终点航向误差将显著改善
        - 第一次 ABORTED 的间歇性 current_pose 失效仍需分析

        ---

        ### H13: B→A 返程大漂移故障捕获 (b_to_a_yaw_drift_02)

        **Hypothesis**: 执行 B→A 返程时，可捕获从正常定位到完全失锁的完整故障链。

        **Goal**: 获取包含故障全过程的 rosbag，通过时序分析确定最先发生异常的信号。

        **Procedure**:

        1. 从 B 点发送 A 点 goal，同时录制 rosbag
        2. 包含 topics: `/tf`, `/tf_static`, `/scan`, `/odom_combined`, `/imu_data`, `/amcl_pose`, `/cmd_vel_nav`, `/cmd_vel`, `/goal_pose`, `/plan`, `/local_plan`, `/robotvel`, `/robotpose`
        3. 漂移发生后保持静止录制，不中断不修正

        **Observed Result**:

        Bag 数据: 时长 ~214s, `/cmd_vel_nav`: 488, `/cmd_vel`: 1118, Odom/IMU: 4258, AMCL: 62, map→odom TF: 565

        **完整故障时序**:

        | 时间 | 事件 | 详情 |
        |------|------|------|
        | t=0~15s | 静止基线 | 激光正常贴合地图 |
        | t=15.76s | Nav2 开始旋转 | `/cmd_vel_nav angular.z = -0.6`, velocity_smoother 逐渐加速底盘 |
        | **t=21.12s** | **LaserScan 首次偏移** | scan shift = -3.32° (第一个超过 3°阈值的事件) |
        | **t=21.66s** | **AMCL 开始修正** | map→odom yaw 在 0.010s 内改变 -1.265° |
        | t=23.89s | AMCL 连续修正 | 多次快速 map→odom 修正 |
        | t=79.61s | Laser 偏移扩大 | scan shift = -7.44° |
        | t=86~91s | 速度命令停止 | cmd_vel_nav 最后约 86s, cmd_vel 最后约 91s |
        | **t=94.46s** | **Laser 最大偏移** | scan shift = -23.64° (完全脱离地图) |
        | t=94~105s | map→odom 大幅摆动 | 从 -35.73° → -46.01° → -31.87° → -43.75° |
        | t=99.15s | 最大单次修正 | 0.607s 内修正 +14.14° |

        **IMU/Odom vs map→odom 对比 (91~105s)**:

        | 信号 | 变化量 |
        |------|--------|
        | IMU/Odom yaw | ~2.0° 漂移 |
        | map→odom yaw | 10°~14° 大幅修正 |
        | Laser scan shift | ~23.6° |

        **关键发现**:

        - **LaserScan 偏移先于 AMCL 修正**: t=21.12s 首次偏移, t=21.66s AMCL 才开始修正 (延迟 ~0.54s)
        - **AMCL 是结果而非原因**: Laser 先漂移，AMCL 随后被动修正 map→odom
        - **IMU/Odom 不是主因**: IMU/Odom 仅漂移 ~2°，而 map→odom 修正 10°~14°，Laser 偏移 ~23.6°
        - **Nav2 速度命令正常**: cmd_vel_nav → velocity_smoother → cmd_vel 链路工作正常

        **Conclusion**:

        - **Confirmed**: 故障链为: 旋转过程 → Laser 与预测位姿逐渐失配 → AMCL 开始修正 map→odom → 定位解不稳定 → 完全失锁
        - **Rejected**: AMCL 主动误修正作为触发源的假设 (Laser 先偏，AMCL 后修)
        - **Rejected**: IMU/odom 静态漂移作为主要根因 (变化量 ~2° vs 故障量 23.6°)
        - 触发源缩小至两个方向: (a) LaserScan 与动态 TF 时间戳不同步; (b) 旋转期间 odom/真实角度比例不准确

        ---

        ## Results

        ### Root Cause Chain (Current Understanding)

        ```
        STM32 固件 (未获取源码)
        ├── 编码器 → 左右轮速度 → Robot_Vel.Z  (存在 CW/CCW 不对称)
        └── 串口发送 rx[6:7] → ROS2 Odom_Trans() → Robot_Pos.Z 积分
                                                        ↓
                                                轮式 Odom yaw (严重错误)
                                                        ↓
                                                已切换为 IMU yaw
                                                        ↓
                                IMU yaw 实现存在 11 项工程缺陷 (无 gyro bias 标定等)
                                                        ↓
                                旋转过程中 LaserScan 与 TF 时间戳不同步 (当前最高怀疑)
        ```

        ### Fault Chain (B→A Failure Timeline)

        ```
        Nav2 发送旋转命令 (t=15.76s)
                ↓
        底盘执行旋转
                ↓
        LaserScan 与预测位姿逐渐失配 (t=21.12s, -3.32°)  ← 首个可检测事件
                ↓
        AMCL 开始连续修正 map→odom (t=21.66s)             ← 滞后 ~0.5s
                ↓
        Laser 偏移持续扩大 (-7.44° → -23.64°)              ← 累积失配
                ↓
        map→odom 大幅摆动 (10°~14°, t=94~105s)            ← 定位崩溃
                ↓
        导航失败, 无法到达 A 点
        ```

        ### IMU Yaw Replacement (Completed)

        | 修改项 | 文件 | 状态 |
        |--------|------|------|
        | 增加 IMU yaw 状态变量 | dlrobot_robot.h | Done |
        | normalize_angle() | dlrobot_robot.cpp | Done |
        | 四元数 → yaw 提取 | dlrobot_robot.cpp | Done |
        | Robot_Pos.Z = IMU yaw | dlrobot_robot.cpp | Done |
        | x/y 积分用 IMU yaw | dlrobot_robot.cpp | Done |
        | odom angular.z = IMU gyro.z | dlrobot_robot.cpp | Done |
        | 保留 /robotvel.z 为诊断通道 | dlrobot_robot.cpp | Done |

        ### Code Issues Summary (from H9 Review)

        | # | 问题 | 优先级 |
        |---|------|--------|
        | 1 | 无 gyro Z 轴零偏标定 | P0 |
        | 2 | IMU covariance 1e-6 (过度自信) | P0 |
        | 3 | odom frame_id 默认值与 AMCL 配置一致性 | P0 |
        | 4 | 依赖外部 Python TF bridge | P1 |
        | 5 | Sampling_Time 计算逻辑 | P1 |
        | 6 | 消息时间戳不统一 | P1 |
        | 7 | 运动判断使用轮速角速度 | P1 |
        | 8 | 浮点精确比较 (== 0) | P2 |
        | 9 | yaw unwrap 无跳变保护 | P2 |
        | 10 | 纯 AHRS yaw 无绝对约束 | P1 |
        | 11 | SAMPLING_FREQ 硬编码 | P2 |

        ### Key Metrics

        | 指标 | 修改前 (轮式 yaw) | 修改后 (IMU yaw) |
        |------|------------------|-----------------|
        | CW 90° 误差 | ~17° 少计 | ~4°~7° 多计 |
        | CCW 90° 误差 | 不稳定 (85°~122°) | ~6°~11° 多计 |
        | CW/CCW 不对称比 | 3.6× (Odom) | 1.3~1.6× (IMU) |
        | 回零误差 | 严重 | ~5°~8° |
        | 静态漂移 | N/A (轮式不漂) | ~0.52°/min |
        | LaserScan 旋转偏移 | 几十度 | ~5°~8° (小角度) / ~23° (连续旋转) |
        | A→B Nav2 导航 | 未测试 | 成功 (位置 ~10cm, 航向 ~11°) |
        | B→A Nav2 导航 | 未测试 | 失败 (定位失锁, Laser 偏移 23.6°) |

        ---

        ## Conclusions

        1. **轮式里程计 yaw 不可用于定位**: STM32 输出的 `Robot_Vel.Z` 存在严重 CW/CCW 方向不对称 (CW 欠转 ~24°, CCW 过冲 ~85°)，ROS2 层无法修复。根因在 STM32 固件中的编码器运动学计算。

        2. **IMU yaw 替换方向正确但实现不完整**: 将 yaw 来源从轮式切换为 IMU 后，小角度旋转测试明显改善。但当前实现本质上是"将轮速 yaw 漂移替换为未经零偏校准的陀螺仪 yaw 漂移"，存在 11 项已知工程缺陷。

        3. **故障链时序已通过 B→A bag 分析建立**: LaserScan 偏移先被检测到 (t=21.12s)，AMCL 随后开始修正 map→odom (t=21.66s, 延迟 ~0.5s)。IMU/Odom 同期仅漂移 ~2°，显著小于 ~23.6° 的 Laser 偏移和 10°~14° 的 map→odom 摆动。该时序排除了 AMCL 主动触发和 IMU 静态漂移两种假设，但尚未唯一确定触发源。

        4. **Current leading hypothesis — LaserScan/TF 时间戳同步**: 旋转过程中 LaserScan 与动态 TF 的时间差，在角速度放大的情况下可产生显著的角度误差。例如 0.6 rad/s × 0.2s 延迟 ≈ 6.9° 偏移，与观察到的 3°~7° 初始偏移量级吻合。需要 Scan-TF 时间差定量分析来验证或排除此假设。

        ---

        ## Next Steps

        1. **P0**: 升级分析脚本，增加 Scan-TF 时间差 (scan_tf_age) 计算，验证旋转时是否存在时间戳错位
        2. **P0**: 静止 10 分钟漂移测试 — 排除运动干扰，判断 IMU、AMCL 或 Scan/TF 哪个在静止时自行漂移
        3. **P0**: 精确 90° 地面真值旋转标定 (CW + CCW 各 3 次)，使用独立物理基准判断 IMU yaw 比例误差
        4. **P1**: Fixed Frame = odom_combined 纯 odom 测试 — 隔离 AMCL，判断 LaserScan 在 odom 坐标系下是否稳定
        5. **P1**: 按最小修复顺序实施代码修正: frame 统一 → gyro bias 标定 → 协方差修正 → 时间戳统一 → 异常保护
        6. **P2**: 评估 `robot_localization` EKF (融合 wheel odom + IMU) 作为长期方案
        7. **P2**: 获取 STM32 固件源码，从根源修复轮式里程计

        ---

        ## Debug Status Snapshot

        ### Current Phase

        Localization — 故障链时序已确认，进入根因隔离阶段 (区分 Laser/TF Timing vs IMU yaw Scale Error)。

        ### Highest Priority Issue

        **LaserScan 与动态 TF 时间戳同步 — 旋转过程中 Laser 先于 AMCL 发生偏移 (t=21.12s)，IMU/Odom 变化量 (~2°) 无法解释 23.6° 的 Laser 偏移和 10°~14° 的 map→odom 摆动。**

        ### Verified

        - ROS2 C++ 积分层工作正常
        - 轮式 Odom yaw 误差来源于 STM32 上游
        - IMU yaw 替换显著改善小角度 LaserScan 对齐
        - Nav2 A→B 短距离直线导航成功
        - IMU ≈ Odom yaw 内部一致 (无 IMU→odom 内部矛盾)
        - map→odom 在 AMCL 不修正时保持稳定
        - goal_checker.yaw_goal_tolerance = 0.2 rad 解释了终点航向偏差
        - 故障链时序: Laser 偏移先被检测到 (t=21.12s) → AMCL 随后修正 (t=21.66s)

        ### Rejected

        - 网络延迟不是主要根因
        - PC 运行 AMCL 不是主要根因
        - 静态 gyro bias 不是 IMU 误差主要来源
        - 轮式 Odom yaw 不可通过单比例系数修正
        - ROS2 驱动层无编码器运动学代码 (无法在 ROS2 层修复轮式 yaw)
        - SLAM 参数调优无法解决 90° 级地图旋转
        - AMCL 作为 Laser 偏移的触发源 (Laser 先偏移，AMCL 后修正)
        - IMU/odom 静态漂移作为大漂移的主因 (变化量 ~2° vs 故障量 ~23.6°)

        ### Still Suspected

        1. ⭐⭐⭐⭐⭐ **LaserScan/TF 时间戳同步** (Current Leading Hypothesis) — 旋转时 Scan 时间戳与 TF 查询时间不匹配，角速度放大时间误差
        2. ⭐⭐⭐⭐ **IMU yaw 旋转期间比例/动态误差** — 真实旋转角与 IMU/odom 报告角不一致，累积导致 Laser 逐渐失配
        3. ⭐⭐⭐ **AMCL 旋转运动模型参数** — alpha1~5 与实际底盘运动学不匹配，旋转预测误差逐渐扩大
        4. ⭐⭐ **odom 位置积分异常** — 手动测试中 odom 位置漂至 y=-10.7m，怀疑 x/y 积分或轮速单位问题
        5. ⭐⭐ **odom_combined → base_footprint TF 多发布源** — 需确认 C++ 驱动和 Python bridge 未同时发布同一条 TF

        ### Next Verification

        1. 升级分析脚本增加 scan_tf_age 计算，分析 B→A bag 的 15~30s 首次偏移窗口
        2. 静止 10 分钟漂移测试 (排除运动干扰)
        3. 精确 90° 地面真值旋转标定 (CW+CCW 各 3 次)
        4. Fixed Frame = odom_combined 纯 odom 测试 (隔离 AMCL)

        ### Open Questions

        - STM32 固件中角速度的具体计算公式 (未获取源码)
        - 旋转时 Scan header.stamp 与 TF 查询时间的具体差值
        - IMU gyro scale 误差是否需要双向标定系数
        - 木地板打滑对 IMU 振动噪声的量化影响
        - odom 位置积分为何漂至 y=-10.7m (x/y 积分逻辑或轮速单位)

        ---

        ## Current Status

        ### Completed

        - Nav2 全部节点迁移至 Jetson
        - Odom yaw 来源从轮式切换为 IMU
        - IMU yaw 旋转响应验证 (幅度、方向正确)
        - Nav2 A→B (~3m) 直线导航成功
        - Baseline v0 系统拓扑检查
        - A、B 固定测试点建立
        - B→A 故障链完整时序捕获

        ### Verified

        - AMCL 在 Jetson 上运行正常
        - map → odom_combined → base_footprint → laser TF 链可用
        - IMU 动态旋转量正确，IMU ≈ Odom yaw 内部一致
        - /odom_combined 和 IMU 发布频率稳定在 20Hz
        - 故障链时序: Laser 偏移先被检测到 (t=21.12s) → AMCL 随后修正 (t=21.66s) → 定位崩溃
        - goal_checker.yaw_goal_tolerance 过宽导致终点航向偏差

        ### Rejected

        - 网络延迟不是主要根因
        - PC 运行 AMCL 不是主要根因
        - 静态 gyro bias 不是 IMU 误差主要来源
        - 轮距参数无法解释 CW/CCW 不对称
        - SLAM 参数调优无法修复 90° 级地图旋转
        - AMCL 不是 LaserScan 偏移的触发源
        - IMU/odom 静态漂移不是大漂移主因

        ### Still Suspected

        - LaserScan/TF 时间戳同步 (P0, Current Leading Hypothesis — 尚未验证)
        - IMU yaw 旋转期间比例误差 (P0)
        - AMCL 旋转运动模型参数不匹配 (P1)
        - odom x/y 位置积分异常 (P1)
        - odom_combined → base_footprint TF 多发布源 (P1)

        ### High Priority

        - 升级分析脚本增加 Scan-TF 时间差计算
        - 静止 10 分钟漂移测试 (隔离各层独立漂移)
        - 精确 90° 地面真值旋转标定
        - 按顺序修复 11 项代码缺陷 (先 frame/gyro bias/covariance)
