# JetArm Robot Runtime — Session Summary

> Sprint 1 → 5.4 实现状态快照 | 2026-05-23 | 分支: `feature/sketch_runtime_sprint3`
>
> **🎯 里程碑达成: 首次 Runtime 驱动的真实硬件 pick 执行验证通过**
> **🔧 Sprint 5 进展**: ROI audit → StableObjectTracker → PerceptionFusionNode →
> **📋 融合规则确立**: YOLO=语义优先, ROI=位姿+颜色, `/world_model/perception_objects` → stable_objects
> **⚠ 已知限制**: ROI 颜色/类名不稳定仍未根本解决；YOLO 语义优先策略降低影响
> ** 📋 Current Next Sprint:
      Sprint 5.5
      StableObjectTracker:
      perception_objects
      → stable_objects

---

## Current Architecture Chain

Perception Pipeline
────────────────────────────────────────

camera
  → YOLO
      (/world_model/objects)
  + ROI
      (/world_model/roi_objects)

  → perception_fusion_node
      (/world_model/perception_objects)

  → StableObjectTracker
      (/world_model/stable_objects)
      [CURRENT: input still = /world_model/roi_objects
       TARGET : switch to /world_model/perception_objects]

Language Pipeline
────────────────────────────────────────

natural language input
  → llm_parser
      (keyword match → /parsed_command)

Fusion / Runtime Pipeline
────────────────────────────────────────

grounding_node
  Inputs:
    - /parsed_command
    - world model (CURRENT: roi_objects
                   TARGET: stable_objects)

  publish_runtime=true
  → /grounded_task_context

real_grounded_runtime_node
  (subscription-driven)

  → TaskBuilder
      (build TaskContext from grounded JSON)

  → SkillManager.select("pick")
      → PickSkill(adapter)

  → Preview / Safety Layer

      if require_confirm:

      publish:
      /runtime/preview

      wait:

      /runtime/confirm
      (yes / no / JSON)

  → PickSkill.execute()

      adapter.ik_solve(
          position,
          rpy
      )

      adapter.servo_move(
          pulses
      )

      adapter.gripper_set(
          id,
          pulse
      )

  → /runtime/execution_result
      (success / fail)

  → /executor/done
      (Bool)

---

## Completed Milestones

| Sprint | Milestone | Key Files |
|--------|-----------|-----------|
| 1 | `sketch_runtime` ROS2 package created | `package.xml`, `setup.py`, `setup.cfg` |
| 1 | `TaskContext` / `TaskState` (11 states) | `task_context.py` |
| 1 | `TargetObject` (16 fields + factories) | `target_object.py` |
| 1 | `ExecutionResult` | `execution_result.py` |
| 1 | `BaseSkill` ABC (precheck/execute/postcheck/cleanup) | `base_skill.py` |
| 1 | `SkillRegistry` + `SkillManager` (intent→skill map) | `skill_registry.py` |
| 1 | `RuntimeAdapter` skeleton (dry_run only) | `runtime_adapter.py` |
| 1 | `PickSkill` (5-step pick sequence) | `skills/pick_skill.py` |
| 2 | `TaskBuilder` (parsed/grounded/wm → TaskContext) | `runtime_task_builder.py` |
| 2 | Grounding: optional `publish_runtime` → `/grounded_task_context` | `grounding_node.py` (+22 lines) |
| 2 | Executor: `/executor/done` signal | `executor_node.py` (+8 lines) |
| 3 | `runtime_test_node` (end-to-end dry_run test) | `runtime_test_node.py` |
| 3 | `runtime_test.launch.py` | launch file |
| 3.1 | `run_once`/`interval_sec`/`dry_run` params | test node + launch |
| 4.1 | `real_grounded_runtime_node` (subscription-driven) | `real_grounded_runtime_node.py` |
| 4.1 | `ground_runtime_bringup.launch.py` (integrated) | launch file |
| 4.1 | `publish_runtime: true` in grounding_params.yaml | config |
| 4.2 | Preview/Confirm safety layer (`/runtime/preview`, `/runtime/confirm`) | runtime node |
| 4.2 | Confirm timeout (default 300s) + cancel path | runtime node |
| 4.2 | `WAITING_CONFIRM` added to TaskState enum | `task_context.py` |
| 4.2 | Simple string confirm: "yes"/"no"/"confirm"/"cancel" | runtime node |
| 4.3 | PickSkill source_pose key fix: `"pose"` → `"source_pose"` | `skills/pick_skill.py` |
| 4.3 | RuntimeAdapter safety split: `dry_run`/`enable_real_ik`/`enable_real_servo` | `runtime_adapter.py` |
| 4.3 | Real IK integration: `_call_ik_blocking()` with temp node | `runtime_adapter.py` |
| — | 65 unit tests, all passing | `test/` |

---

## Confirmed Working Paths

### 1. IK Service
```bash
ros2 service call /kinematics/set_pose_target kinematics_msgs/srv/SetRobotPose \
  "{position: [0.186, -0.018, 0.115], pitch: 80.0, pitch_range: [-90.0, 90.0], resolution: 1.0}"
```
Response: `success=True, pulse=[476, 422, 313, 55, 504], rpy=[1.0, 80.0, -5.22]`

### 2. RuntimeAdapter Temp Node Pattern
- `_call_ik_blocking()` creates a fresh `rclpy.create_node("runtime_adapter_ik_client")` per call
- Builds client, calls `call_async()`, spins temp node, returns result, destroys node in `finally`
- Wrapped in `asyncio.to_thread()` from async `ik_solve()` — no executor conflicts
- Returns real pulse values correctly

### 3. Dry-Run Full Chain (65 tests pass)
```
parsed_command dict → TaskBuilder.from_parsed_command → TaskContext(PARSED)
grounded_goal dict   → TaskBuilder.from_grounded_goal  → TaskContext(GROUNDED)
SkillManager.select(intent="pick") → "pick_skill"
PickSkill(adapter).execute() → 5 steps → ExecutionResult(success=True)
```

---

## Current Integration Issue: Perception Fusion → StableObjectTracker Wiring

Current state:

camera
→ YOLO (/world_model/objects)
+ ROI (/world_model/roi_objects)
→ perception_fusion_node
→ /world_model/perception_objects

StableObjectTracker already exists and publishes:

/world_model/stable_objects

But CURRENT input is still:

/world_model/roi_objects

Next integration:

Sprint 5.5
perception_objects
→ StableObjectTracker

Sprint 5.6
stable_objects
→ grounding

Known limitation:

ROI class/color instability still exists.

Mitigation strategy:

YOLO:
semantic priority

ROI:
pose/color/yaw priority

## Safety Defaults

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `dry_run` | `true` | Mock IK + log-only servo |
| `require_confirm` | `true` | Must confirm before execute |
| `enable_real_ik` | `false` | Real IK service call |
| `enable_real_servo` | `false` | Real servo publish |
| `run_once` | `true` | Execute once then stop |
| `confirm_timeout_sec` | `300.0` | 5 min confirm window |
| publish_roi_only | true | ROI fallback |
| enable_fusion | true | perception fusion |

---

## Key Commands

```bash
# Dry-run full chain (safe default)
ros2 launch sketch_runtime ground_runtime_bringup.launch.py

# IK-only (real IK, no servo)
ros2 launch sketch_runtime ground_runtime_bringup.launch.py \
  dry_run:=false enable_real_ik:=true enable_real_servo:=false require_confirm:=true

# Listen to preview
ros2 topic echo /runtime/preview

# Confirm (simple)
ros2 topic pub --once /runtime/confirm std_msgs/msg/String 'data: "yes"'

# Cancel
ros2 topic pub --once /runtime/confirm std_msgs/msg/String 'data: "no"'

# Listen to execution result
ros2 topic echo /executor/done

# Run unit tests (no ROS2 needed)
cd Jetarm_ros2_robot
PYTHONPATH=src/sketch_runtime python3 -m pytest src/sketch_runtime/test/ -q
```

---

## Debugging Lessons Learned

1. **Never spin the same ROS2 node from a background thread** — causes executor conflicts, `future.result() → None`
2. **Use temporary IK client nodes** (`rclpy.create_node()` + `destroy_node()` in finally) for real service calls from async context
3. **Never bypass Preview/Confirm** — always confirm before allowing real hardware motion
4. **Verify message format before publishing** — servo messages have specific field types that differ between msg packages
5. **ActionExecutor must wait between steps** — `asyncio.sleep(duration_ms/1000.0)` after each MoveAction and GripperAction
6. **Runtime success does NOT imply perception stability** — `/world_model/roi_objects` is a RAW detection stream. Same physical object may be labeled differently across frames (cup red → cup black → cylinder red). A StableObjectTracker with temporal voting must precede Verification Runtime.

---

## Current Stable Test Flow

| 模式 | 命令 | 说明 |
|------|------|------|
| **Dry-run full chain** | `ros2 launch sketch_runtime ground_runtime_bringup.launch.py` | 默认 safety 全开 |
| **IK-only** | `... dry_run:=false enable_real_ik:=true enable_real_servo:=false require_confirm:=true` | 真实 IK 不发 servo |
| **Full pick (实物)** | `... dry_run:=false enable_real_ik:=true enable_real_servo:=true require_confirm:=true` | 完整真实硬件执行 |
| **Hover only** | 通过 `skill_params: {execution_stage: hover_only}` | 仅悬停至目标上方 |
| **Auto execute** | `... require_confirm:=false` | 跳过确认直接执行 |

---

## Current Known Limitations

| 领域 | 限制 |
|------|------|
| **ROI 类名/颜色** | 不稳定 — 形状分类 (cup/cube/cylinder) 和 LAB 颜色 (red/black) 仍会跨帧跳变。战略决策: YOLO 语义优先，
      ROI provides:
      pose.xyz
      pose.rpy
      color

      YOLO provides:
      semantic class_name

      Fusion:
      class ← YOLO
      pose/color ← ROI |
| **抓取精度** | grasp precision 尚不稳定，需进一步标定和补偿 |
| **验证逻辑** | 无 Verification Runtime — 无法自动判断抓取是否成功 |
| **碰撞检测** | 无碰撞/力矩异常检测 |
| **重试/恢复** | 无 retry 或 recovery 逻辑，失败后无法自动重试 |
| **物体检测** | 无 object-loss 检测 — 不知道目标是否掉落 |
| **放置验证** | 无放置成功判定 — 不知道物体是否到达目标区域 |

---

## Completed Milestones (Sprint 4.4 → 4.8)

| Sprint | Milestone |
|--------|-----------|
| 4.4 | Servo Message Adapter 修复 — `float()` position cast |
| 4.5 | Hover-only safety test |
| 4.6 | Action Sequence 架构 — MoveAction/GripperAction/ActionExecutor |
| 4.7 | Full pick 真实硬件执行验证 — 首次 Runtime 驱动真实机械臂 |
| 4.8 | **🎯 里程碑达成** — 文档更新，标记完整验证闭环 |
| 5.1 | ✅ | ROI Detection Audit |
| 5.2 | 🔶 | StableObjectTracker implemented |
| 5.3 | ⏸ | ROI robustness deferred |
| 5.4 | ✅ | Perception Fusion |
| 5.5 | ⬜ NEXT | Tracker input switch |
| 5.6 | ⬜ PLANNED | Grounding switch |

## Sprint 5 Progress (5.1 → 5.4)

| Sprint | Status | Milestone |
|--------|--------|-----------|
| 5.1 | ✅ | Raw Detection Audit — `roi_detection_audit_node` 量化 label stability |
| 5.2 | 🔶 | StableObjectTracker — 已实现，输入切换待 integration (Sprint 5.5) |
| 5.3 | ⏸ | ROI Shape/Color Robustness — 部分调研，未正式完成 |
| 5.4 | ✅ | Perception Fusion Node — YOLO+ROI 融合 → `/world_model/perception_objects` |
| 5.5 | ⬜ NEXT| StableObjectTracker 输入切换到 perception_objects |
| 5.6 | ⬜ PLANNED | Grounding 切换到 `/world_model/stable_objects` |

**融合规则**: class_name=YOLO, color/pose/rpy=ROI。YOLO 未检测到的物体保留 roi_only fallback。

## Package Snapshot

```
src/sketch_runtime/
├── sketch_runtime/
│   ├── __init__.py              # exports TaskState/TaskContext/TargetObject/...
│   ├── task_context.py          # TaskState enum (11 states) + TaskContext
│   ├── target_object.py         # TargetObject + from_world_model/from_detection_result
│   ├── execution_result.py      # ExecutionResult dataclass
│   ├── base_skill.py            # BaseSkill ABC
│   ├── skill_registry.py        # SkillRegistry + SkillManager
│   ├── runtime_adapter.py       # RuntimeAdapter (safety split + temp IK node)
│   ├── runtime_task_builder.py  # TaskBuilder: parsed/grounded/wm → TaskContext
│   ├── runtime_test_node.py     # Self-contained test node (simulated data)
│   ├── real_grounded_runtime_node.py  # Production node (subscription-driven + Preview/Confirm)
│   └── skills/
│       └── pick_skill.py        # PickSkill (5-step: hover→open→approach→close→lift)
├── launch/
│   ├── runtime_test.launch.py
│   └── ground_runtime_bringup.launch.py
└── test/
    ├── test_task_context.py     # 17 tests
    ├── test_target_object.py    # 9 tests
    ├── test_skill_registry.py   # 23 tests
    └── test_task_builder.py     # 16 tests
```

---

## Modified Existing Files

| File | Change |
|------|--------|
| `grounding/grounding/grounding_node.py` | +`publish_runtime` param → `/grounded_task_context` |
| `grounding/config/grounding_params.yaml` | +`publish_runtime: true` |
| `llm_executor/llm_executor/executor_node.py` | +`/executor/done` Bool publisher |
