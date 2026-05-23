# JetArm Robot Runtime — Documentation Index

> 最后更新：2026-05-23 | 当前 Sprint: 5 (Perception Fusion + Stable World Model) | 下一个 Sprint: 6 (Verification Runtime)

---

## Current Status

| 项 | 状态 |
|----|------|
| **里程碑** | **✅ 首次 Runtime 驱动的真实机械臂抓取执行验证通过** |
| **当前 Sprint** | 5 — Perception Fusion + Stable World Model (`🟡` Fusion 完成，待切换 grounding) |
| **已完成** | Servo Message Adapter、ActionExecutor 时序同步、真实 IK+Servo 全链路、ROI audit ✅、StableObjectTracker ✅、PerceptionFusionNode ✅ |
| **进行中** | 将 StableObjectTracker 输入切换到 `/world_model/perception_objects` |
| **已知限制** | ROI 颜色/类名不稳定 (Sprint 5.3 DEFERRED)；YOLO 语义优先策略降低影响；无 Verification Runtime |

---

## Recommended Reading Order

| 顺序 | 文件 | 用途 |
|------|------|------|
| 1 | `runtime_index.md` | **本文件** — 入口索引 |
| 2 | `runtime_session_summary.md` | 当前会话压缩 — 架构/里程碑/命令/调试教训 |
| 3 | `jetarm_runtime_roadmap.md` | 完整路线图 — Sprint 1~9 进度 + 未来计划 |
| 4 | `runtime_analysis.md` | 系统运行时分析 — 18包职责/3条链路/控制冲突 |
| 5 | `topic_service_map.md` | ROS2 通信矩阵 — 50+ topic/service 速查 |
| 6 | `runtime_architecture.md` | Mermaid 架构图集 — 5 张图 |
| 7 | `runtime_risks.md` | 风险评估 — P0/P1/P2 14 项 |
| 8 | `runtime_task_schema.md` | TaskContext/TaskState 数据结构设计 |
| 9 | `runtime_target_object.md` | TargetObject 统一数据结构设计 |
| 10 | `runtime_skill_interface.md` | BaseSkill/PickSkill/RuntimeAdapter 接口设计 |
| 11 | `runtime_debug_guide.md` | ROS2 终端调试命令 — launch/echo/confirm |

---

## Key Launch Commands

```bash
# 完整 Runtime 集成 (dry_run, 单次, Preview/Confirm)
ros2 launch sketch_runtime ground_runtime_bringup.launch.py \
  dry_run:=true require_confirm:=true run_once:=true use_dummy_wm:=true

# IK 验证模式 (真实 IK, 不发 servo)
ros2 launch sketch_runtime ground_runtime_bringup.launch.py \
  dry_run:=false enable_real_ik:=true enable_real_servo:=false \
  require_confirm:=true

# 跳过确认 (自动执行)
ros2 launch sketch_runtime ground_runtime_bringup.launch.py require_confirm:=false

# 循环执行
ros2 launch sketch_runtime ground_runtime_bringup.launch.py \
  run_once:=false interval_sec:=2.0
```

---

## Key Topics

| Topic | 用途 |
|-------|------|
| `/parsed_command` | llm_parser → grounding |
| `/grounded_goal` | grounding → old executor |
| `/grounded_task_context` | grounding → real_grounded_runtime_node |
| `/runtime/preview` | Runtime preview (waiting_confirm) |
| `/runtime/confirm` | 用户确认 (yes/no or JSON) |
| `/runtime/state` | 任务状态流 |
| `/runtime/log` | 结构化事件日志 |
| `/runtime/execution_result` | 执行结果 |
| `/executor/done` | 执行完成信号 |
| `/world_model/perception_objects` | YOLO+ROI 融合输出 (perception_fusion_node) |
| `/world_model/stable_objects` | 稳定世界模型 (StableObjectTracker) |

---

## Current Safety Rules

- `dry_run` must default `true` — never connect real hardware without user opt-in
- `require_confirm` must default `true` — never skip Preview/Confirm without user opt-in
- `enable_real_ik` defaults `false` — never call real IK without explicit toggle
- `enable_real_servo` defaults `false` — never publish servo commands without explicit toggle
- Never modify `servo_controller`, `kinematics`, `ros_robot_controller`
- Never bypass Preview/Confirm when servo is enabled
- Dry-run tests pass locally (65 tests) even without ROS2 runtime

---

## Roadmap Summary

| Sprint | 状态 | 目标 |
|--------|------|------|
| 1-2 | **COMPLETED** | sketch_runtime 骨架 + TaskBuilder 集成 |
| 3-4.3 | **COMPLETED** | 测试节点 + Preview/Confirm + IK 安全分级 |
| 4.4-4.8 | **COMPLETED** | Servo adapter fix + Action Sequence + 真实硬件 pick |
| 5 | **NEARLY COMPLETE** | Perception Fusion / Stable World Model (5.1 ✅, 5.2 ✅, 5.3 ⏸, 5.4 ✅) |
| 6 | PLANNED | Verification Runtime (depends on Sprint 5 grounding switch) |
| 7 | PLANNED | Retry / Recovery |
| 8 | PLANNED | RobotOps Dashboard |
| 9 | PLANNED | Teleop / Safety Control |
| 10 | PLANNED | Data Logger / VLA readiness |

---

## How to Recover Project Context

1. Read `runtime_index.md` (this file) first
2. Read `runtime_session_summary.md` for the compressed implementation state
3. Read `jetarm_runtime_roadmap.md` for the forward plan
4. Source the ROS2 workspace: `source ~/ros2_ws/install/setup.bash`
5. Build: `colcon build --packages-select sketch_runtime --symlink-install`
6. Run tests: `PYTHONPATH=src/sketch_runtime python3 -m pytest src/sketch_runtime/test/ -q`
7. Use `docs/runtime_debug_guide.md` for launch/echo/confirm commands
