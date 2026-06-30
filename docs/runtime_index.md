  # JetArm Robot Runtime — Documentation Index

  > 最后更新：2026-06-29 | 当前阶段：Mobile Robot Foundation（移动机器人阶段） | 当前目标：AMCL → Nav2 → MoveSkill

  ---

  ## Current Status

  | 项 | 状态 |
  |----|------|
  | **里程碑** | **✅ Runtime Platform 完成；✅ RobotOps Foundation 完成；✅ 家庭 SLAM 建图完成；🔶 AMCL 验证中** |
  | **当前阶段** | **Mobile Robot Foundation（移动机器人阶段）** |
  | **已完成** | Runtime 执行链、真实 IK+Servo、Perception Fusion、StableObjectTracker、Verification Runtime、RobotOps(SQLite)、RPLidar A1、/scan、/odom_combined、TF Bridge、SLAM Toolbox、home_map_01_260621 |
  | **进行中** | AMCL Localization Validation |
  | **下一步** | Nav2 Goal Pose → Semantic Locations → MoveSkill → Mobile Manipulation |
  | **已知限制** | Nav2 尚未验证；MoveSkill 未实现；Semantic Location Registry 未实现；移动底盘串口存在偶发 IOException；WiFi DDS 对网络环境敏感 |

  ---

  ## Recommended Reading Order

  | 顺序 | 文件 | 用途 |
  |------|------|------|
  | 1 | `runtime_index.md` | **本文件** — 项目入口 |
  | 2 | `jetarm_runtime_roadmap.md` | 当前路线图（Runtime → Mobile Robot） |
  | 3 | `topic_service_map.md` | ROS2 Topic / Service 真相 |
  | 4 | `runtime_architecture.md` | Runtime Mermaid 架构图 |
  | 5 | `runtime_risks.md` | 当前技术债与风险 |
  | 6 | `runtime_task_schema.md` | TaskContext / TaskState |
  | 7 | `runtime_target_object.md` | TargetObject 统一对象模型 |
  | 8 | `runtime_skill_interface.md` | BaseSkill / PickSkill / MoveSkill |
  | 9 | `runtime_debug_guide.md` | 调试命令速查 |

  ---


  ## Key Launch Commands

  ### Runtime（完整执行链）

  ```bash
  ros2 launch sketch_runtime ground_runtime_bringup.launch.py \
    dry_run:=true require_confirm:=true run_once:=true use_dummy_wm:=true
  ```

  ### Perception Pipeline（Manual Startup）

  ```bash
  # ROI 检测
  ros2 run app roi_color_detector_node \
  --ros-args \
  -p transform_yaml:=/home/sundasheng/ros2_ws/src/app/config/transform.yaml \
  -p lab_config:=/home/sundasheng/ros2_ws/src/app/config/lab_config.yaml

  # YOLO
  ros2 run vision_yolo simple_yolo_node

  # Perception Fusion
  ros2 run app perception_fusion_node

  # Stable Object Tracker
  ros2 run app stable_object_tracker_node

  # Grounding（Stable World Model Integration）
  ros2 run grounding grounding_node \
  --ros-args \
  -p publish_runtime:=true \
  -p world_model_topic:=/world_model/stable_objects
  ```

  ### Perception 一键启动

  ```bash
  ros2 launch app perception_bringup.launch.py
  ```

  ### Runtime 节点

  ```bash
  ros2 run sketch_runtime real_grounded_runtime_node \
  --ros-args \
  -p dry_run:=true \
  -p require_confirm:=true
  ```

  ### Verification

  ```bash
  ros2 run sketch_runtime verification_result_node
  ```

  ### RobotOps Recorder

  ```bash
  ros2 launch robotops robotops_recorder.launch.py
  ```

  ### 数据库查询

  ```bash
  sqlite3 ~/ros2_ws/robotops.db
  ```

  ### Mobile Base

  ```bash
  ros2 launch turn_on_dlrobot_robot tank.launch.py
  ```

  ### LiDAR

  ```bash
  ros2 launch rplidar_ros rplidar_a1_launch.py \
  serial_port:=/dev/ttyUSB0
  ```

  ### TF

  ```bash
  ros2 run mobile_base_bridge odom_tf_bridge_node

  ros2 run tf2_ros static_transform_publisher \
  0 0 0.15 0 0 0 base_footprint laser
  ```

  ### SLAM

  ```bash
  ros2 launch slam_toolbox online_async_launch.py
  ```

  ### AMCL

  ```bash
  ros2 launch nav2_bringup localization_launch.py \
  map:=/home/sundasheng/ros2_ws/maps/home_map_01_260621.yaml
  ```

  ### 调试与测试

  ```bash
  # 查看 Grounding 输出
  ros2 topic echo /grounded_goal

  # 查看 Runtime Context
  ros2 topic echo /grounded_task_context

  # 查看 Stable Objects
  ros2 topic echo /world_model/stable_objects

  # 发送测试命令
  ros2 topic pub --once \
  /parsed_command \
  std_msgs/msg/String \
  'data:
  "{\"action\":\"pick\",
  \"from\":\"blue_cup\",
  \"to\":\"right_side\",
  \"raw\":\"拿起蓝色杯子放右边\"}"'

  # 确认执行
  ros2 topic pub --once \
  /runtime/confirm \
  std_msgs/msg/String \
  'data: "yes"'

  # 查看里程计
  ros2 topic echo /odom_combined --once

  # 控制小车前进
  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 5
  ```

  ---

  ## Runtime Topics

  | Topic                             | 用途                                       |
  | --------------------------------- | ---------------------------------------- |
  | `/parsed_command`                 | llm_parser → grounding                   |
  | `/grounded_goal`                  | grounding → old executor                 |
  | `/grounded_task_context`          | grounding → real_grounded_runtime_node   |
  | `/runtime/preview`                | Runtime preview (waiting_confirm)        |
  | `/runtime/confirm`                | 用户确认 (yes/no or JSON)                    |
  | `/runtime/state`                  | 任务状态流                                    |
  | `/runtime/log`                    | 结构化事件日志 (event_id / state / timestamp)   |
  | `/runtime/execution_result`       | 执行结果                                     |
  | `/runtime/verification_result`    | 验证结果 (precheck / postcheck / post_place) |
  | `/executor/done`                  | 执行完成信号                                   |
  | `/world_model/perception_objects` | YOLO + ROI 融合输出                          |
  | `/world_model/stable_objects`     | 稳定世界模型                                   |
  | `/world_model/roi_objects`        | ROI 原始检测流                                |

  ---

  ## Mobile Base Topics

  | Topic            | 用途                           |
  | ---------------- | ---------------------------- |
  | `/scan`          | RPLidar 激光雷达数据               |
  | `/odom_combined` | 底盘里程计（odom → base_footprint） |
  | `/tf`            | 动态 TF                        |
  | `/tf_static`     | 静态 TF                        |
  | `/cmd_vel`       | 底盘速度控制接口                     |

  ---

  ## Navigation Topics

  | Topic             | 用途                           |
  | ----------------- | ---------------------------- |
  | `/map`            | Occupancy Grid 地图            |
  | `/map_metadata`   | 地图元信息                        |
  | `/initialpose`    | RViz Initial Pose → AMCL 初始化 |
  | `/goal_pose`      | RViz Goal Pose → Nav2 导航目标   |
  | `/amcl_pose`      | AMCL 估计机器人位姿                 |
  | `/particle_cloud` | AMCL 粒子滤波可视化                 |
  | `/plan`           | Nav2 全局路径（待验证）               |


  ---

  ## Roadmap Summary

  | 阶段 | 状态 | 目标 |
  |--------|------|------|
  | Runtime Platform | ✅ COMPLETED | Runtime 执行系统（Sprint 1–7A） |
  | Mobile Robot Foundation | 🔄 IN PROGRESS | 移动机器人基础能力 |
  | 8.1 | ✅ | Base Driver |
  | 8.2 | ✅ | Odometry |
  | 8.3 | ✅ | RPLidar Integration |
  | 8.4 | ✅ | SLAM Mapping |
  | 8.5 | 🔄 | AMCL / Nav2 Validation |
  | 8.6 | ⏳ | Semantic Locations |
  | 9 | ⏳ | Persistent World Model |
  | 10 | ⏳ | NavigateSkill / SearchSkill / Mobile Manipulation |
  | 11 | ⏳ | Demonstration Collection / VLA Readiness |
  | 12 | ⏳ | Retry / Recovery |

  ---


  ## How to Recover Project Context

  1. Read `runtime_index.md`
  2. Read `jetarm_runtime_roadmap.md`
  3. Read `topic_service_map.md`

  4. Source ROS2:

  ```bash
  source ~/ros2_ws/install/setup.bash
  ```

  5. Build Runtime:

  ```bash
  colcon build --packages-select sketch_runtime --symlink-install
  ```

  6. 启动感知系统：

  ```bash
  ros2 launch app perception_bringup.launch.py
  ```

  7. 启动 Runtime：

  ```bash
  ros2 run sketch_runtime real_grounded_runtime_node \
  --ros-args -p dry_run:=true -p require_confirm:=true
  ```

  8. 启动移动底盘：

  ```bash
  ros2 launch turn_on_dlrobot_robot tank.launch.py
  ```

  9. 当前地图：

  ```text
  ~/ros2_ws/maps/home_map_01_260621.yaml
  ```

  10. 当前开发目标：

  ```text
  AMCL Validation
  ↓
  Nav2 Goal Pose
  ↓
  Semantic Locations
  ↓
  MoveSkill
  ↓
  Mobile Manipulation
  ```
  11. Read `CLAUDE.md`

  12. Source of Truth Documents:

  - `topic_service_map.md`
  - `runtime_architecture.md`
  - `jetarm_runtime_roadmap.md`

  Never infer architecture without verifying against these documents.
