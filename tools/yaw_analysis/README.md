# yaw_analysis — ROS2 rosbag yaw calibration toolkit

Analyse yaw (heading) accuracy from multiple sensor/data sources in
ROS2 Humble rosbags.  Designed for mobile robot chassis debugging and
calibration.

## Quick start

```bash
# Source ROS2 first
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Run analysis
python3 analyze.py ~/ros2_ws/debug_bags/test_bag \
    --target-angle 180 \
    --imu-topic /mobile_base/sensors/imu_data \
    --odom-topic /odom_combined \
    --robotvel-topic /robotvel \
    --cmd-topic /cmd_vel
```

## Output

| File | Description |
|------|-------------|
| Terminal | Colourised table of every segment and error summary |
| `report.md` | Same content as terminal, saved to disk |
| `report.csv` | Machine-readable one-row-per-segment |
| `plot.png` | 5-panel time-series figure (cmd_vel, IMU yaw, odom, robotvel, gyro Z) |

### Plot layout

```
subplot 1: cmd_vel angular.z (°/s) — step plot
subplot 2: IMU yaw + gyro integral overlay
subplot 3: Odom yaw
subplot 4: RobotVel integral
subplot 5: Gyro Z raw angular velocity (°/s)
```

Segment backgrounds are colour-coded:
- **Red** tint = CW
- **Green** tint = CCW
- **Grey** tint = Static

## Automatic segment detection

Segments are detected from `/cmd_vel angular.z`:

| Condition | Classification |
|-----------|---------------|
| `abs(z) <= threshold` | **STATIC** |
| `z > threshold` | **CCW** (counter-clockwise, positive yaw) |
| `z < -threshold` | **CW** (clockwise, negative yaw) |

Three tunable parameters control detection:

| Argument | Default | Description |
|----------|---------|-------------|
| `--threshold` | 0.02 rad/s | Dead band around zero |
| `--merge-gap` | 0.5 s | Merge same-type commands within this gap |
| `--settle-time` | 1.0 s | Pad segment windows for sensor settling |

## CSV schema

| Column | Description |
|--------|-------------|
| `segment` | Name (e.g. `CW_1`, `STATIC_2`) |
| `type` | `CW`, `CCW`, or `STATIC` |
| `start_time`, `end_time` | UNIX seconds |
| `expected_angle` | Target rotation (degrees) |
| `imu_orientation_delta` | ΔYaw from IMU quaternion |
| `imu_orientation_error` | measured − expected |
| `imu_gyro_integral` | Cumulative gyro integration |
| `imu_gyro_error` | measured − expected |
| `odom_delta` | ΔYaw from odometry |
| `odom_error` | measured − expected |
| `robotvel_integral` | Cumulative robot_vel integration |
| `robotvel_error` | measured − expected |
| `gyro_bias` | Mean gyro Z (rad/s, static only) |
| `gyro_std` | Std dev of gyro Z (static only) |
| `orientation_drift` | ΔYaw across the static window |
| `odom_drift` | ΔYaw from odom across the static window |

## Architecture

```
tools/yaw_analysis/
    __init__.py    — Package exports
    analyze.py     — CLI entry point and pipeline orchestration
    parser.py      — Rosbag reading → structured DataFrames
    segment.py     — CW/CCW/STATIC detection + metrics computation
    report.py      — Terminal, Markdown, and CSV report generation
    plot.py        — Multi-panel matplotlib figure
    utils.py       — Quaternion → yaw, angle arithmetic, integration
    README.md      — This file
```

### Data flow

```
bag/ ─→ BagParser.parse()
                │
                ▼
           BagData (DataFrames per topic)
                │
                ▼
         SegmentDetector.detect(cmd_vel)
                │
                ▼
         List[Segment] (type + time window)
                │
                ▼
         compute_segment_metrics(segments, bag_data)
                │
                ├──→ ReportGenerator.generate()
                │         ├── terminal (stdout)
                │         ├── report.md
                │         └── report.csv
                │
                └──→ PlotGenerator.generate()
                          └── plot.png
```

### Extending

**Add a new topic:**

1. Add a topic parameter to `BagParser.__init__`.
2. Add a `_parse_<topic>()` method to `BagParser`.
3. Add a `pd.DataFrame` attribute to `BagData`.
4. Add a subplot to `PlotGenerator`.

**Add a new analysis module:**

1. Create a new module (e.g. `fft_analysis.py`).
2. Accept `BagData` and `list[Segment]` as inputs.
3. Hook it into `analyze.py:run()`.

## Dependencies

- ROS2 Humble (rosbag2_py, rclpy, std_msgs, geometry_msgs, sensor_msgs, nav_msgs)
- Python >= 3.10
- numpy, pandas, matplotlib

## Development

```bash
# Lint
ruff check .
ruff format --check .

# Check types (optional)
mypy --strict analyze.py
```
