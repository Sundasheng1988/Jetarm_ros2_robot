# JetArm Quick Start

> 只记录当前正式启动命令。
> 详细调试与故障排查见 `docs/runtime_debug_guide.md`。

快速跳转：

- PC 环境 → §1
- Rebecca Voice / Natural-language Navigation → §2
- Vision → §3
- Manipulation → §4
- RobotOps → §5
- Mobile Robot / Nav2 → §6
- 常用监听 → §7

# 1. PC ROS 环境

统一使用：

```bash
rosenv     # 查看当前模式
rosvoice   # PC-only：Voice / Parser / Navigation 回归
rosrobot   # PC ↔ Orin：真实跨机
```

当前规则：

- `rosvoice`
  - `ROS_DOMAIN_ID=23`
  - `rmw_cyclonedds_cpp`
  - `ROS_LOCALHOST_ONLY=1`
  - 使用 `config/cyclonedds/pc_localhost.xml`
  - CycloneDDS `ParticipantIndex=auto`
  - `MaxAutoParticipantIndex=120`
- `rosrobot`
  - `ROS_DOMAIN_ID=23`
  - `rmw_cyclonedds_cpp`
  - `ROS_LOCALHOST_ONLY=0`
  - 优先 `eno1 / 192.168.100.x`
  - 若机器人经 Wi-Fi / hotspot，则使用 `wlo1 / 172.20.10.x`
  - 对应 CycloneDDS XML 同样使用 `MaxAutoParticipantIndex=120`
- 两种模式均设置 `ROS2CLI_DISABLE_DAEMON=1`。

> `rosvoice` / `rosrobot` 的模式切换采用 fail-closed：
> preflight 未通过时不会把当前 shell 留在“半切换”DDS 状态。
>
> 环境变量是 **per-shell** 的。每个新 Terminal 都必须重新执行
> `rosvoice` 或 `rosrobot`。
>
> 若再次出现：
>
> ```text
> Failed to find a free participant index for domain 23
> ```
>
> 不要临时手写 DDS 参数；先查 `docs/runtime_debug_guide.md`。

# 2. Rebecca Voice / Natural-language Navigation（在 PC）

## 2.1 Voice-only / PC-only 回归

```bash
cd ~/ros2_ws
rosvoice

ros2 run llm_voice_agent rebecca_voice_start   play_audio:=true
```

用于：

- 语音聊天
- Voice mode
- Wake / mute / TTS interrupt
- Navigation confirmation 状态机

> 这一模式 **不等于完整 Navigation integration**。
> 未启动 Parser / Navigation semantic layer 时，不会形成
> `/parsed_command -> /goto_place` 链路。

---

## 2.2 PC-only Natural-language Navigation 回归

用于验证：

```text
Voice
→ Rebecca
→ Parser
→ navigation_executor_node
→ /goto_place
→ goto_place_node
```

此模式不要求 Orin / Nav2 / 小车在线。

如果 Nav2 没启动，`goto_place_node` 应安全返回 Nav2 Action Server 不可用；
这属于 PC-only 边界回归，不会驱动真实底盘。

### Terminal 1 — Parser

```bash
cd ~/ros2_ws
rosvoice

ros2 run llm_parser llm_command_parser_node
```

### Terminal 2 — Navigation semantic layer

```bash
cd ~/ros2_ws
rosvoice

ros2 launch place_manager navigation.launch.py   launch_done_sayer:=false
```

`navigation.launch.py` 已包含：

```text
goto_place_node
navigation_executor_node
```

因此使用该 launch 后，**不要再额外启动**：

```bash
ros2 launch place_manager goto_place.launch.py
ros2 run place_manager navigation_executor_node
```

否则可能产生重复 `/goto_place` Service 或重复 Executor。

### Terminal 3 — Rebecca Voice

```bash
cd ~/ros2_ws
rosvoice

ros2 run llm_voice_agent rebecca_voice_start   play_audio:=true
```

典型测试：

```text
“瑞贝卡，让Eric去餐厅”
→ Rebecca 请求确认
→ “是的”
→ /voice_input/input
→ /parsed_command
→ navigation_executor_node
→ /goto_place
```

Nav2 OFF 时，预期最终安全失败，不应产生真实运动。

---

## 2.3 真实 Voice → Nav2 → Eric

前提：

1. Orin 已按 §6 启动 `nav_bringup`
2. PC 与 Orin 网络已连通
3. PC 每个相关 Terminal 都执行 `rosrobot`
4. `places.yaml` 中已有目标地点

### Terminal 1 — Parser

```bash
cd ~/ros2_ws
rosrobot

ros2 run llm_parser llm_command_parser_node
```

### Terminal 2 — Navigation semantic layer

```bash
cd ~/ros2_ws
rosrobot

ros2 launch place_manager navigation.launch.py   launch_done_sayer:=false
```

### Terminal 3 — Rebecca Voice

```bash
cd ~/ros2_ws
rosrobot

ros2 run llm_voice_agent rebecca_voice_start   play_audio:=true
```

完整链路：

```text
Speech
→ FunASR
→ Rebecca / confirmation
→ /voice_input/input
→ llm_command_parser_node
→ /parsed_command
→ navigation_executor_node
→ /goto_place
→ goto_place_node
→ places.yaml
→ Nav2 /navigate_to_pose
→ Eric
```

> `navigation.launch.py` 本身 **不启动 Nav2**。
> Nav2 / Localization / Mobile Base 仍由 Orin 的 `nav_bringup` 启动。
>
> Rebecca Voice 已有 `executor_done_sayer` 时，
> Navigation launch 必须使用：
>
> ```bash
> launch_done_sayer:=false
> ```
>
> 避免重复结果播报。

---

## 2.4 Place Manager（仅地点管理时需要）

如果只是导航到已有地点，不需要单独启动 `place_manager_node`。

只有需要：

- 保存当前位置
- 删除地点
- 查询地点
- 列出地点

时启动：

```bash
cd ~/ros2_ws
rosrobot

ros2 launch place_manager place_manager.launch.py
```

常用服务：

```text
/save_place
/delete_place
/list_places
/get_place
```

导航执行使用的是：

```text
/goto_place
/cancel_navigation
```

由 `goto_place_node` 提供。

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
ros2 launch sketch_runtime ground_runtime_bringup.launch.py   use_dummy_wm:=false   dry_run:=true   enable_real_ik:=false   enable_real_servo:=false   require_confirm:=true   run_once:=true
```

Real IK / Servo OFF：

```bash
ros2 launch sketch_runtime ground_runtime_bringup.launch.py   use_dummy_wm:=false   dry_run:=false   enable_real_ik:=true   enable_real_servo:=false   require_confirm:=true   run_once:=true
```

Real Robot Arm：

```bash
ros2 launch sketch_runtime ground_runtime_bringup.launch.py   use_dummy_wm:=false   dry_run:=false   enable_real_ik:=true   enable_real_servo:=true   require_confirm:=true   run_once:=true
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

ros2 launch nav_bringup nav_bringup.launch.py   serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

当前导航地图：

```text
/home/ubuntu/ros2_ws/maps/home_map_navsafe_01_260809.yaml
```

该 Bringup 一次启动：

```text
Mobile Base
RPLidar
Odom TF
Static TF
Map Server
AMCL
Nav2
RViz
```

# 7. 最常用监听

> 监听 Terminal 也必须先进入与被测系统相同的模式：
>
> PC-only 用 `rosvoice`；
> 跨机实测用 `rosrobot`。

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

Navigation Graph：

```bash
ros2 node list | grep -E 'goto_place|navigation_executor'
ros2 service list -t | grep -E '^/goto_place|^/cancel_navigation'
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
我要启动              → docs/quick_start.md
我要排故              → docs/runtime_debug_guide.md
我要查 Topic / Service → docs/topic_service_map.md
我要看当前开发状态      → docs/runtime_index.md
我要看架构             → docs/runtime_architecture.md
```
