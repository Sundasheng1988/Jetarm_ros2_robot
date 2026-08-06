# JetArm ROS2 Robot — Runtime Analysis

> [BASELINE / HISTORICAL ANALYSIS]
>
> 分析日期：2026-05-17 | 原始目标：为 Robot Runtime v0.1 重构提供系统级理解
> 原始第一阶段规则已 superseded — 代码已进入 Sprint 5。
>
> **当前状态**:
> - Sprint 5 ✅: Stable World Model — ROI audit → StableObjectTracker → PerceptionFusionNode → Grounding switch
> - Sprint 6 ✅: Verification Runtime — Sidecar → Runtime Integration → Event Enrichment → Post Place Verification
> - Runtime Execution v0.1: real pick verified on hardware
> - Preview/Confirm、SkillManager+PickSkill、ActionExecutor 全部运行
> - /runtime/log enriched (event_id / state / timestamp), /runtime/verification_result published
> - TaskState includes WAITING_CONFIRM, VERIFYING, VERIFIED, VERIFICATION_FAILED
> - **当前阶段**: Sprint 6 Close — Verification Runtime Completed
> - **Next Phase**: Sprint 7A — RobotOps Foundation (SQLite persistence, task history, event store)

---

## 1. 系统部署架构（两机分工）

| 机器 | 角色 | 运行内容 |
|------|------|----------|
| **PC Ubuntu** | 上层逻辑 / 感知 / 交互 | `llm_voice_agent`（语音全栈）、`llm_parser`、`llm_executor`、`grounding`、`vision_yolo`、`app`、`keyboard_input` |
| **JetArm Orin Nano** | 底层控制 / 硬件驱动 | `servo_controller`、`kinematics`（IK .so 服务）、`ros_robot_controller`、相机驱动、STM32 总线 |

> **关键约束**：JetArm 原厂 IK `.so` 必须运行在 JetArm 上。PC 侧通过 ROS2 service 调用。

---

## 2. 包职责总览（共 18 个包）

### 2.1 上层逻辑包（PC 侧 — Runtime 重构重点）

| 包名 | 节点 | 职责 | 架构层 |
|------|------|------|--------|
| `llm_voice_agent` | `llm_voice_agent` / `speech_dialog_funasr_node` / `tts_speaker_node` / `executor_done_sayer` | 语音全栈：ASR(FunASR) → LLM(Ollama) → TTS(Piper)，含唤醒词、L1/L2/L3 状态管理、人脸跟踪控制、槽位提取 | 交互层（需下沉为纯入口） |
| `llm_parser` | `llm_command_parser_node` | 关键词匹配式指令解析：中文 → `{"action":"pick","from":"red_cube","to":"right_side"}` | 推理层（待标准化） |
| `grounding` | `grounding_node` / `wm_from_tf` / `wm_dummy_pub` | 将解析结果匹配到场景中具体对象与目标位姿 | Grounding 层 |
| `llm_executor` | `ground_executor_node` | 接收 grounded goal → 生成轨迹 → IK → 舵机脉冲 → 执行 | 执行层（**核心写死区**） |
| `vision_yolo` | `app_compatible_yolo_node` | YOLOv8 检测 + LAB 色块方块检测 → `/vision_target` + `/world_model/objects` | 感知层 |
| `app` | `calibration_node` / `object_pose_publisher` / `roi_color_detector_node` 等 | 标定(AprilTag)、物体位姿发布、ROI 颜色检测 | 工具 / 感知层 |
| `keyboard_input` | `keyboard_input_node` | 终端键盘文本输入 → `/text_input` | 交互层 |

### 2.2 中间层 / 桥接包

| 包名 | 节点 | 职责 | 架构层 |
|------|------|------|--------|
| `social_robot` | `face_follow_node` / `gesture_player_node` / `env_scan_node` / `static_env_report_node` / `world_model_node` | 社交交互：MediaPipe 人脸跟随、头部手势(nod/shake)、环境扫描(EnvObjectArray)、世界模型融合 | 交互/感知层 |
| `joint_pulse_converter` | `state_to_pulse_node` | 弧度→脉冲实时转换与日志（调试用） | 调试工具 |

### 2.3 底层 / 硬件适配包（**不应修改**）

| 包名 | 节点 | 职责 | 备注 |
|------|------|------|------|
| `servo_controller` | `controller_manager` + `grasp`(GraspNode) | 舵机管理、JointPositionController(弧度↔脉冲)、JointTrajectoryActionController、夹爪序列 | **不可修改** |
| `ros_robot_controller_msgs` | — | 硬件驱动消息定义（ServosPosition/BusServoState/BuzzerState 等） | 原厂接口 |
| `kinematics_msgs` | — | IK 服务/消息定义（SetRobotPose/GetRobotPose/SetJointValue 等） | 原厂接口 |

### 2.4 接口定义包（只含 msg/srv）

| 包名 | 内容 |
|------|------|
| `vision_interfaces` | `DetectionResult.msg`（并行数组：class_name/confidence/center_x/center_y/center_z），`TargetPose.msg` |
| `robot_interfaces` | `EnvObject.msg`（label/confidence/pose），`EnvObjectArray.msg` |
| `servo_controller_msgs` | `ServosPosition.msg`（duration + position_unit + ServoPosition[]），`Grasp.msg`，`ServoStateList.msg` 等 |

### 2.5 仿真包

| 包名 | 用途 |
|------|------|
| `jetarm_6dof_description` | URDF/XACRO 模型描述（6 DOF arm + gripper） |
| `robot_moveit_config` | MoveIt2 配置（kinematics/planners/controllers） |
| `robot_gazebo` | Gazebo / Ignition Gazebo 仿真启动 |

---

## 3. 当前指令到动作的完整链路

### 主链路 A：键盘/语音输入 → 抓取动作

```
用户输入 "红色方块放到右边"
    │
    ▼
[keyboard_input_node] → /text_input  或  [llm_voice_agent] → /keyboard_input/input , /voice_input/input
    │
    ▼
[llm_command_parser_node]  @ /keyboard_input/input + /voice_input/input
    │ keyword match: "红色"→red, "方块"→cube, "右边"→right_side
    │ action 固定为 "pick"
    │
    ▼  /parsed_command  {"action":"pick","from":"red_cube","to":"right_side","raw":"红色方块放到右边"}
    │
    ▼
[grounding_node]  @ /parsed_command
    │ ① split_from_token("red_cube")→class="cube",color="red"
    │ ② _select_object(cube,red) 在 self.objects 中模糊子串匹配 → 按 confidence↓/recency↓/proximity↑ 排序取第一
    │ ③ place_map["right_side"]→target_pose
    │ ⚠: 订阅 /world_model/roi_objects 但发布方用 /world_model/objects — topic 不匹配
    │
    ▼  /grounded_goal  {"intent":"pick","object_id":3,"source_pose":{...},"target_pose":{...},"status":"ok"}
    │
    ▼
[ground_executor_node]  @ /grounded_goal
    │ 硬编码 10 步 pick-and-place 序列:
    │  ① hover_source      ② gripper_open(pulse=200)    ③ approach_pick
    │  ④ gripper_close(700) ⑤ lift(0.08m)                ⑥ hover_target
    │  ⑦ approach_place    ⑧ gripper_open(200)           ⑨ lift
    │  ⑩ return_home([500,560,130,115,500])
    │
    │ 对每步 move: 调 IK service /kinematics/set_pose_target → pulses → _finalize_pulses_with_yaw(ch5=wrist)
    │ 发布预览 → /executor/preview* (5个topic) → 等待 /executor/confirm
    │
    ▼  /ros_robot_controller/bus_servo/set_position  (ServosPosition: id 1-5+j10)
    │
    ▼
[ros_robot_controller] (JetArm 硬件驱动) → STM32 → 舵机
```

### 主链路 B：语音全栈（llm_voice_agent 内部）

```
麦克风 → [speech_dialog_funasr_node]  WebRTC VAD → FunASR → wake word 检测 → post-filter
    │ /speech_query
    ▼
[llm_voice_agent]  _on_query()
    │ L1(pause/resume) → L2(wake/sleep) → L3(mute/unmute) → mode(chat/task)
    │ task mode: slot extraction (COLOR_MAP + CLASS_MAP + SIDE_MAP) → confirm → → /keyboard_input/input
    │ chat mode: Ollama API → /speech_reply
    │
    ▼  /speech_reply → [tts_speaker_node]  Piper TTS → audio playback
```

### 主链路 C：视觉检测独立路

```
/depth_cam/rgb/image_raw → [app_compatible_yolo_node]
    │ YOLOv8(on full image) + ROI 过滤 + homography → 世界坐标
    │ cube fallback: top-down warp + LAB + adaptive threshold + geometry filter
    ├── /vision_target (DetectionResult, 供 executor._vision_confirm())
    └── /world_model/objects (含 world frame 坐标, 供 grounding 使用)
         ⚠ topic 名不匹配 — grounding 订阅 /world_model/roi_objects
```

### 主链路 D：Verification Runtime 链路 (Sprint 6 新增)

```
grounding_node
  → /grounded_task_context
  → real_grounded_runtime_node    (executing → verifying)
  → /executor/done
  → verification_result_node      (precheck / postcheck / post_place)
  → /runtime/verification_result
  → real_grounded_runtime_node    (_on_verification_result)
  → VERIFIED / VERIFICATION_FAILED
```

验证阶段: precheck (目标在源位置附近) → postcheck (目标从源位置消失) → post_place (目标出现在目标位置)。
Runtime 收到 verification_result 后根据 success 字段过渡到 VERIFIED 或 VERIFICATION_FAILED。

---

## 4. executor 中写死的逻辑（`ground_executor_node`）

### 4.1 动作序列完全写死

`_handle_goal()` 方法 (`executor_node.py:345-499`) 无条件执行固定 10 步序列，不支持其他动作类型/条件分支/动态调整。

### 4.2 写死的参数

| 参数 | 写死的值 | 位置 |
|------|----------|------|
| 夹爪张开 | `pulse=200` | `_handle_goal()` |
| 夹爪闭合 | `pulse=700` | `_handle_goal()` |
| 悬停高度 | `hover_height=0.08 m` | `declare_parameter()` |
| 接近高度 | `approach_z=0.015 m` | `declare_parameter()` |
| 提起高度 | `lift_height=0.08 m` | `declare_parameter()` |
| 回位脉冲 | `[500, 560, 130, 115, 500]` | `home_pulses` 参数 |
| IK fallback | `[500, 500, 500, 500, 500]` | `_ik_get_pulses()` 返回 |
| 运动时长 | `move_duration_ms=2000` | 参数 |
| 确认等待 | `confirm_wait_sec=300.0` | 参数 |

### 4.3 无任务状态机

```python
self._busy = True         # 仅一把互斥锁，新 goal 直接丢弃
self._last_goal_hash      # JSON sorted hash 去重 + 800ms throttle
```

- 无 `task_id` 生成
- 无 `cancel` / `retry` / `pause`
- 无执行结果返回给调用方（仅日志 `[done] 执行完成`）
- IK 调用与舵机发布紧耦合在 `_exec_move()` 中

### 4.4 腕部映射写死

`_yaw_to_ch5()` 将 rpy[2](yaw 弧度) → ch5 舵机脉冲，含 `wrist_clip45` 将角度钳制到 ±45°，逻辑内嵌在 executor 中不应在此层处理。

---

## 5. grounding 如何表达目标对象

### 5.1 输入 (`/parsed_command`)

```json
{"action": "pick", "from": "red_cube", "to": "right_side", "raw": "红色方块放到右边"}
```

`from` token 格式: `<color>_<class>` 由 `split_from_token()` 解析。

### 5.2 输出 (`/grounded_goal`)

```json
{
  "intent": "pick",
  "object_hints": {"class": "cube", "color": "red"},
  "object_id": 3,
  "source_pose": {"frame": "table", "xyz": [-0.27, -0.15, 0.02], "rpy": [0, 0, 1.57]},
  "target_pose": {"frame": "table", "xyz": [-0.25, -0.20, 0.02], "rpy": [0, 0, 1.57]},
  "used_last_object_memory": false,
  "status": "ok",
  "detail": ""
}
```

### 5.3 关键问题

1. **Topic 不匹配 (P0)**：grounding 订阅 `/world_model/roi_objects`，所有发布方用 `/world_model/objects`
2. **模糊匹配**：`_select_object()` 用 `cls in obj or obj in cls` 子串匹配；`cls=None` 时匹配全部对象
3. **place_map 硬编码**：7 个槽位写死在 `_default_place_map()` 中，可通过 YAML 参数覆盖但结构僵硬
4. **intent 白名单**：仅处理 `pick/hold/place/pour/grasp/release/move`
5. **无统一 TargetObject**：不同来源(dummy/TF/vision)输出字段略有差异(`angle_deg` 仅 cube 有)

---

## 6. PC vs JetArm 运行分区

| 位置 | 模块 |
|------|------|
| **PC** | `llm_voice_agent`, `llm_parser`, `grounding`, `llm_executor`, `vision_yolo`, `app`, `keyboard_input`, `social_robot`, `joint_pulse_converter` |
| **JetArm** | `servo_controller`, `ros_robot_controller`(驱动), `kinematics`(IK .so), 相机驱动, STM32 |

> `llm_executor` 直接调用 IK service `/kinematics/set_pose_target` 并直接写 `/ros_robot_controller/bus_servo/set_position`，**绕过 `servo_controller/controller_manager` 安全层**。

---

## 7. 适合重构为 Skill Runtime 的模块

| 模块 | 当前状态 | 优先级 |
|------|----------|--------|
| `llm_executor/executor_node.py` | 硬编码 pick-and-place 序列，IK/舵机紧耦合 | **P0** |
| `grounding/grounding_node.py` | place_map 硬编码，topic 不匹配，无统一 TargetObject | **P0** |
| `llm_parser/llm_command_parser_node.py` | 关键词匹配，action 固定 "pick"，无 LLM 实际参与 | P1 |
| `llm_voice_agent/llm_voice_agent_node.py` | 槽位提取+confirm 逻辑内嵌，承担了 executor 职责 | P1 |

**Sprint 5-6 Update**: `sketch_runtime` 已替代 `llm_executor` 成为 Runtime 执行层。
TaskContext 状态机包含 WAITING_CONFIRM、EXECUTING、VERIFYING、VERIFIED、VERIFICATION_FAILED。
verification_result_node 作为 sidecar 运行 precheck / postcheck / post_place 验证。post_place 验证框架已实现。

## 8. 属于底层驱动，不应修改

| 模块 | 原因 |
|------|------|
| `servo_controller` (全部) | JetArm 原厂硬件适配层 |
| `ros_robot_controller_msgs` | 原厂硬件驱动接口 |
| `kinematics_msgs` | 原厂 IK 服务接口 |
| `kinematics` (IK .so wrapper) | 原厂逆运动学求解器 |
| `servo_controller_msgs` | 原厂舵机控制消息 |
| `robot_interfaces` / `vision_interfaces` | ROS2 接口定义（可扩展但不可删除） |
| `jetarm_6dof_description` | 机械臂物理模型 (URDF) |
| `STM32` 固件 | 硬件固件 |
| `simulations/` 全部 | 仿真工具 |

---

## 9. 控制冲突分析

### 9.1 `/servo_controller` 多写者

以下节点均可发布到 `/servo_controller` topic：

| 节点 | 触发条件 | 风险 |
|------|----------|------|
| `ground_executor_node` | 自动任务执行中 | **高** |
| `face_follow_node` | 人脸跟踪持续运行 | **高** |
| `grasp` (GraspNode) | 收到 `/grasp` 消息 | **高** |
| `llm_voice_agent` | 唤醒手势 | 中 |
| `env_scan_node` | 环境扫描 | 中 |
| `gesture_player_node` | nod/shake 手势 | 低 |

**无任何控制权仲裁。**

### 9.2 双层命令路径

```
A: executor → IK → /ros_robot_controller/bus_servo/set_position  (绕过 controller_manager)
B: executor → /servo_controller → controller_manager → 同上
```

路径 A 绕过 servo_controller 的 `JointPositionController` 限位保护。

---

## 10. Teleop / RobotOps / Verification 已有基础

| 领域 | 已有 | 缺失 |
|------|------|------|
| **Teleop** | `keyboard_input_node` 文本输入、`app/actions.py` goto_left/right/home、`auto_ik_pipeline.launch.py` | 遥控优先级、控制权仲裁(AUTO/MANUAL/ESTOP)、手柄/Web 输入 |
| **RobotOps** | `ground_executor_node` 预览发布(`/executor/preview*`×5)、`joint_pulse_converter` 脉冲日志 | 统一 task_id、持久化日志、结构化事件、图像快照、dashboard API |
| **Verification** | `_vision_confirm()` 可选、`/vision_target` 可用、`executor_done_sayer` | 抓取后确认、目标消失检测、超时检测、retry 逻辑 |

---

## 11. 已知问题汇总

| # | 问题 | 类 | 严重度 |
|---|------|-----|--------|
| 1 | `/world_model/objects` ↔ `/world_model/roi_objects` topic 不匹配 | Bug | P0 |
| 2 | executor `_handle_goal()` 动作序列硬编码 | Design | P0 |
| 3 | 多节点无仲裁写 `/servo_controller` | Safety | P0 |
| 4 | 无 task_id / 任务状态机 | Missing | P0 |
| 5 | GraspNode `targe2`/`targe3` 拼写错误 (应为 `target2`/`target3`) | Bug | P1 |
| 6 | `app/get_robot_pose_client` 入口文件不存在 | Bug | P1 |
| 7 | llm_parser `parse()` action 始终 `"pick"` | Design | P1 |
| 8 | PC 侧直接写 `/ros_robot_controller/bus_servo/set_position` | Safety | P1 |
| 9 | grounding `wm_from_vision` 源文件不存在（launch 中引用） | Missing | P2 |
| 10 | `social_robot` SDK 文件(13个)与主系统紧耦合 | Complexity | P2 |
| 11 | grounding `_select_object()` 子串匹配可能误匹配 | Design | P2 |
| 12 | `ObjectPosePublisher` 使用 `dt_apriltags`(外部包) 可能不存在 | Dep | P2 |

---

## 12. Current Runtime Limitations

Sprint 6 完成后系统能力边界。

| 领域 | 限制 |
|------|------|
| **Skill 支持** | pick_skill only — place_skill, move_skill, home_skill 未实现 |
| **Place 执行** | post_place verification 框架存在，place execution 路径未实现 |
| **验证模式** | Verification Runtime 已完成 dry-run 验证；/ place verification 尚未完成真实硬件验证。
| **任务匹配** | FIFO matching only — 不支持并发任务验证 |
| **重试/恢复** | 未实现 — 延后至 Sprint 11 |
| **RobotOps** | 无 SQLite 持久化、无 dashboard、无 task 回放 |
| **VLA** | 未集成 — data collection 管线尚未建立 |

---

## 13. Why RobotOps Is The Next Runtime Layer

从架构角度，Runtime 在 Sprint 6 完成后具备以下数据流:

- `/runtime/log`: 结构化事件流 (event_id, task_id, event, state, timestamp, data)
- `/runtime/state`: 任务状态流 (CREATED → ... → VERIFIED / VERIFICATION_FAILED)
- `/runtime/verification_result`: 验证结果 (precheck, postcheck, post_place)
- `/runtime/execution_result`: 执行结果 (success, reason, evidence)

当前所有数据仅存在于 ROS2 topic buffer — 节点重启或 topic 超时后不可恢复。

Sprint 7A 目标: 将以上流写入 SQLite，以 task_id 索引，支持离线查询与重放。

需要的架构能力:
- **Event Store** — /runtime/log 事件流 → SQLite 持久化
- **Task History** — task_id → 完整状态历史 (state_history + timestamps)
- **Event Replay** — 从 SQLite 重建 Task → Execution → Verification → Result 完整链路

这将使 RobotOps dashboard (Sprint 7B) 和 VLA data collector (Sprint 9) 有持久化数据源，无需重复订阅 topic 获取历史数据。

---

## 14. 核心结论

当前系统是 **ROS2 节点拼接系统**，不是 **Robot Runtime**。

关键缺失链：

```
任务抽象 ❌ → Skill 调度 ❌ → 控制权仲裁 ❌ → 结构化日志 ❌ → 成功判定 ❌
```

重构应按 Roadmap 优先级推进：

```
P0: 统一任务执行框架（最大瓶颈：executor 写死 + grounding topic 不匹配）
P1: 遥控输入 + 安全接管（控制权仲裁，解决多写者冲突）
P2: RobotOps 日志/监控/回传（task_id + 持久化记录）
P3: 任务验证 / 失败检测（抓取前后对比 + 超时 + retry）
P4: IK / 抓取精度优化
P5: 语音系统重构
P6: 高级 IK / VLA / 数据集
```
