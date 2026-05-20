# JetArm Robot Runtime — Documentation Index

> 最后更新：2026-05-20 | 当前 Sprint: 4.8 — Milestone 达成 | 下一个 Sprint: 5 (Verification Runtime)

---

## Current Status

| 项 | 状态 |
|----|------|
| **里程碑** | **✅ 首次 Runtime 驱动的真实机械臂抓取执行验证通过** |
| **当前 Sprint** | 4.8 完成 — Action Sequence + 时序同步 + 真实硬件验证 |
| **已解决** | Servo Message Adapter 修复 (`float` position)、ActionExecutor 时序同步、真实 IK+Servo 全链路 |
| **未解决** | grasp precision 尚不稳定、无 Verification Runtime、无碰撞检测、无 retry/logic |
| **当前分支** | `feature/sketch_runtime_sprint3` |
| **已实现闭环** | LLM → parser → grounding → Preview/Confirm → SkillManager → PickSkill → RuntimeAdapter(dry_run) → /executor/done |
| **IK 状态** | 真实 IK 服务可用，返回正确脉冲，temp node 模式通过 |
| **Servo 状态** | 消息格式待修复，硬件尚未运动 |
| **安全默认** | dry_run=true, require_confirm=true, enable_real_ik=false, enable_real_servo=false |

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
| 1 | **COMPLETED** | sketch_runtime 包 + TaskContext/TargetObject/Skill 骨架 |
| 2 | **COMPLETED** | TaskBuilder 集成 grounding→TaskContext |
| 3 | **COMPLETED** | runtime_test_node + ground_runtime_bringup |
| 4.1 | **COMPLETED** | real_grounded_runtime_node + 集成 launch |
| 4.2 | **COMPLETED** | Preview/Confirm 安全层 + timeout + simple yes/no |
| 4.3 | **COMPLETED** | PickSkill source_pose fix + IK 安全分级 + 真实 IK |
| 4.4 | **COMPLETED** | Servo Message Adapter 修复 |
| 4.5 | **COMPLETED** | Hover-only safety test |
| 4.6 | **COMPLETED** | Action Sequence 架构 (MoveAction/GripperAction/ActionExecutor) |
| 4.7 | **COMPLETED** | Full pick 真实硬件执行验证 |
| 4.8 | **COMPLETED** | **里程碑: 首次 Runtime 驱动真实硬件抓取** |
| 5 | **IN_PROGRESS** | Verification Runtime (5.1-5.6 已规划) |
| 5.1 | PLANNED | Verification Architecture — VerificationResult + stages + reasons |
| 5.2 | PLANNED | World Model Reader — 订阅/解析 /world_model/objects |
| 5.3 | PLANNED | Precheck Verification — 执行前确认目标存在 |
| 5.4 | PLANNED | After Pick Verification — 抓取后确认目标消失 |
| 5.5 | PLANNED | Verification Logs — /runtime/verification topic |
| 5.6 | PLANNED | Retry Plan — design only, Sprint 6 implement |
| 6 | PLANNED | RobotOps / Monitoring |
| 7 | PLANNED | Teleop / Safety Control |
| 8 | PLANNED | Skill Library Expansion |
| 9 | PLANNED | Data Logger / VLA readiness |

---

## How to Recover Project Context

1. Read `runtime_index.md` (this file) first
2. Read `runtime_session_summary.md` for the compressed implementation state
3. Read `jetarm_runtime_roadmap.md` for the forward plan
4. Source the ROS2 workspace: `source ~/ros2_ws/install/setup.bash`
5. Build: `colcon build --packages-select sketch_runtime --symlink-install`
6. Run tests: `PYTHONPATH=src/sketch_runtime python3 -m pytest src/sketch_runtime/test/ -q`
7. Use `docs/runtime_debug_guide.md` for launch/echo/confirm commands
