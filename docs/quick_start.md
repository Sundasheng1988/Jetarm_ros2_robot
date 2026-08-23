# JetArm Quick Start

> 只记录当前正式启动命令。
> 详细调试与故障排查见 `docs/runtime_debug_guide.md`。

快速跳转：

- PC 环境 → §1
- Rebecca Voice → §2
- Vision → §3
- Manipulation → §4
- RobotOps → §5
- Mobile Robot / Nav2 → §6

# 1. PC ROS 环境

```bash
rosenv     # 查看当前模式
rosvoice   # PC-only Voice / Parser / Navigation 回归
rosrobot   # PC ↔ Orin
```

# 2. Rebecca Voice（在 PC）

## Voice-only / PC-only 回归

```bash
cd ~/ros2_ws
rosvoice

ros2 run llm_voice_agent rebecca_voice_start \
  play_audio:=true
```

用于：语音聊天 / Voice mode / Navigation confirmation 状态机回归。

## 真实 Voice → Nav2

前提：Orin 已按第 6 节启动 nav_bringup。

> 以下 Parser / Navigation / Rebecca Voice 分别运行在独立终端时，
> **每个终端都必须先执行 `cd ~/ros2_ws && rosrobot`**。
> 环境变量是 per-shell 的，漏掉会导致部分节点留在 localhost DDS。

```bash
# Terminal 1 — Parser
cd ~/ros2_ws
rosrobot
ros2 run llm_parser llm_command_parser_node

# Terminal 2 — Navigation semantic layer（executor + goto_place）
cd ~/ros2_ws
rosrobot
ros2 launch place_manager navigation.launch.py \
  launch_done_sayer:=false

# Terminal 3 — Rebecca Voice
cd ~/ros2_ws
rosrobot
ros2 run llm_voice_agent rebecca_voice_start \
  play_audio:=true
```

> rosvoice = PC-only；rosrobot = 真实跨机。

# 3. Vision / Perception（在 PC）

```bash
cd ~/ros2_ws
rosrobot

./scripts/start_cyclone_perception.sh
```

启动：YOLO / ROI / Perception Fusion / Stable Object Tracker。

# 4. Manipulation Runtime（在 PC）

三个模式均在 PC 执行；需要真实 Camera / Kinematics 时先：

```bash
cd ~/ros2_ws
rosrobot
```

Dry-run：

```bash
ros2 launch sketch_runtime ground_runtime_bringup.launch.py \
  use_dummy_wm:=false \
  dry_run:=true \
  enable_real_ik:=false \
  enable_real_servo:=false \
  require_confirm:=true \
  run_once:=true
```

Real IK / Servo OFF：

```bash
ros2 launch sketch_runtime ground_runtime_bringup.launch.py \
  use_dummy_wm:=false \
  dry_run:=false \
  enable_real_ik:=true \
  enable_real_servo:=false \
  require_confirm:=true \
  run_once:=true
```

Real Robot Arm：

```bash
ros2 launch sketch_runtime ground_runtime_bringup.launch.py \
  use_dummy_wm:=false \
  dry_run:=false \
  enable_real_ik:=true \
  enable_real_servo:=true \
  require_confirm:=true \
  run_once:=true
```

> 真实机械臂执行必须满足：现场观察、可立即断电、单一控制栈。

# 5. RobotOps（在 PC）

```bash
cd ~/ros2_ws
ros2 launch robotops robotops_recorder.launch.py
```

# 6. Mobile Robot / Nav2（在 Orin）

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.zsh
source ~/ros2_ws/install/setup.zsh

ros2 launch nav_bringup nav_bringup.launch.py \
  serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

当前导航地图：

```text
/home/ubuntu/ros2_ws/maps/home_map_navsafe_01_260809.yaml
```

该 Bringup 一次启动：Mobile Base / RPLidar / Odom TF / Static TF / Map Server / AMCL / Nav2 / RViz。

# 7. 最常用监听

Voice：

```bash
ros2 topic echo /speech_query
ros2 topic echo /voice_input/input
ros2 topic echo /parsed_command
```

Navigation：

```bash
ros2 topic echo /runtime/execution_result
ros2 topic echo /amcl_pose
```

Vision：

```bash
ros2 topic echo /world_model/stable_objects
```

Runtime：

```bash
ros2 topic echo /runtime/state
ros2 topic echo /runtime/log
```

# 8. 文档入口

```text
我要启动            → docs/quick_start.md
我要排故            → docs/runtime_debug_guide.md
我要查 Topic / Service → docs/topic_service_map.md
我要看当前开发状态    → docs/runtime_index.md
我要看架构           → docs/runtime_architecture.md
```
