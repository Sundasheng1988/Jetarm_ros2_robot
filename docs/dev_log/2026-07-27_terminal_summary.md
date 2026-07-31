## 启动顺序

### Nano（Orin）

**Terminal 1 — 底盘驱动**

```bash
ros2 launch turn_on_dlrobot_robot tank.launch.py
```

**Terminal 2 — 激光雷达**

```bash
ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/ttyUSB0
```

如 USB0 无设备，尝试：

```bash
ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/ttyUSB1
```

**Terminal 3 — Odom → TF 桥**

```bash
ros2 run mobile_base_bridge odom_tf_bridge_node
```


**Terminal 4 — 雷达安装 TF**

```bash
ros2 run tf2_ros static_transform_publisher \
  0 0 0.15 0 0 0 base_footprint laser
```
```bash
ros2 run tf2_ros static_transform_publisher \
  0.105 0 0.21 0 0 0 base_footprint laser
```
**Terminal 5 — ACML**

*** Jetarm ***
ros2 launch nav2_bringup localization_launch.py \
  map:=/home/ubuntu/ros2_ws/maps/home_map_clean_02_260705.yaml \
  use_sim_time:=false \
  params_file:=/home/ubuntu/ros2_ws/config/nav2_amcl_params.yaml


**Terminal 6 — NAV2**
ros2 launch nav2_bringup localization_launch.py \
  map:=/home/ubuntu/ros2_ws/maps/home_map_clean_02_260705.yaml \
  use_sim_time:=false \
  params_file:=/home/ubuntu/ros2_ws/config/nav2_params.yaml



**Terminal 7 — 发布cmd_vel驱动指令**

ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.15}}"

ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.15}}"

ros2 topic pub \
  --rate 10 \
  --times 238 \
  /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.15}}"

ros2 topic pub \
  --rate 10 \
  --times 238 \
  /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.15}}"

  ros2 topic pub \
  --rate 10 \
  --times 238 \
  /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.30}}"

### Terminal 8 rosbag录制示例： 终端A：录制CCW Run01
在Jetson执行：

source /opt/ros/humble/setup.zsh
source ~/ros2_ws/install/setup.zsh

export ROS_DOMAIN_ID=23
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

cd ~/ros2_ws

TEST_ROOT="$HOME/ros2_ws/test_logs/lidar_tf_validation_v2_20260726"
RUN_NAME="TF02_CCW_zplus0p15_n230_run01_lightweight"
RUN_DIR="$TEST_ROOT/$RUN_NAME"

mkdir -p "$RUN_DIR"

cat > "$RUN_DIR/test_conditions.txt" <<'EOF'
Configuration: lightweight chassis
Run: 01
Direction: CCW
Command:
  linear.x: 0.0
  angular.z: +0.15
  rate: 10 Hz
  times: 230
  nominal_duration: 23.0 s
Lidar TF:
  parent: base_footprint
  child: laser
  x: 0.105 m
  y: 0.000 m
  z: 0.210 m
  roll: 0
  pitch: 0
  yaw: 0
Comparison reference:
  TF01_CW_zminus0p15_n230_run01_lightweight
  TF01_CW_zminus0p15_n230_run02_lightweight
Purpose:
  Equal-input CW versus CCW directional comparison
EOF

date -Ins | tee "$RUN_DIR/record_start_time.txt"

ros2 bag record \
  -o "$RUN_DIR/bag" \
  /cmd_vel \
  /robotvel \
  /mobile_base/sensors/imu_data \
  /odom_combined \
  /scan \
  /tf \
  /tf_static \
  /PowerVoltage

等待看到：

All requested topics are subscribed. Stopping discovery...

之后让机器人保持静止约10秒。

###  终端B：发送CCW命令

source /opt/ros/humble/setup.zsh
source ~/ros2_ws/install/setup.zsh

export ROS_DOMAIN_ID=23
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

send_zero() {
  for i in 1 2 3 4 5; do
    ros2 topic pub --once \
      /cmd_vel geometry_msgs/msg/Twist \
      "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
      >/dev/null 2>&1
  done
}

trap 'send_zero' EXIT INT TERM

echo "CCW Run01 start:"
date -Ins

ros2 topic pub \
  --rate 10 \
  --times 230 \
  /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.15}}"

send_zero

echo "CCW Run01 stop:"
date -Ins
echo "测试完成，已发送5次零速度。"

trap - EXIT INT TERM

### 补现场观察
RUN_DIR="$HOME/ros2_ws/test_logs/lidar_tf_validation_v2_20260726/TF02_CCW_zplus0p15_n230_run01_lightweight"

cat > "$RUN_DIR/field_observation.txt" <<'EOF'
Run ID: TF02_CCW_zplus0p15_n230_run01_lightweight
Configuration: lightweight chassis
Direction: CCW
Command: angular.z=+0.15, rate=10 Hz, times=230

Actual rotation angle: ____ deg
Center net displacement: ____ cm
Displacement direction: ____
Rotation center behavior: ____
Wheel slip observed: yes / no / uncertain
Abnormal vibration or noise: ____
Power cable influence: none / possible / obvious
Video filename: ____
Observer notes: ____
EOF

## PC端同步两个CCW包
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ROOT="$HOME/ros2_ws/test_logs/lidar_tf_validation_v2_20260726"

mkdir -p "$ROOT"

for RUN in \
  TF02_CCW_zplus0p15_n230_run01_lightweight \
  TF02_CCW_zplus0p15_n230_run02_lightweight
do
  mkdir -p "$ROOT/$RUN"

  rsync -avh --progress \
    "ubuntu@172.20.10.2:/home/ubuntu/ros2_ws/test_logs/lidar_tf_validation_v2_20260726/$RUN/" \
    "$ROOT/$RUN/"
done

### 检查四个测试都在一起：
find "$ROOT" \
  -maxdepth 1 \
  -type d \
  \( -name 'TF01_CW*' -o -name 'TF02_CCW*' \) \
  -printf '%f\n' \
  | sort