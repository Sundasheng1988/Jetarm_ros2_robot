# JetArm Robot Runtime — Session Summary

> Sprint 1 → 7A Session Snapshot | 2026-06-14
>
> Sprint 4 里程碑:
首次 Runtime 驱动的真实硬件 Pick 执行验证通过

> Sprint 6 里程碑:
Verification Runtime Dry-Run 闭环验证通过

> Sprint 7A 里程碑:
RobotOps Foundation — SQLite persistence for runtime events
> **⚠ 已知限制**: ROI 颜色/类名不稳定仍未根本解决；pick skill还未实现； 

---

## Current Project State (2026-06)

Project Status:

Phase A — Runtime Platform
✅ COMPLETED

Completed:

* Runtime
* Grounding
* Verification Runtime
* RobotOps Foundation

Current Focus:

Phase B — Mobile Robot Foundation

Current Sprint:

Sprint 8.1 — Base Driver Bringup

Current Hardware:

* Jetson Orin Nano
* Mobile Base
* STM32 Controller
* Gemini Depth Camera
* Slamtec Lidar (planned bringup)
* JetArm Manipulator

North Star:

Autonomous Mobile Manipulation Robot

User:
"Go to the kitchen and bring me a cup."

System:

NavigateSkill
→ SearchSkill
→ Grounding
→ PickSkill
→ NavigateSkill
→ PlaceSkill
→ Verification
→ RobotOps Record

---

## Strategic Direction Change

### Completed — Phase A: Runtime Platform

* Runtime execution framework (Sprint 1 → 4)
* Perception + Stable World Model (Sprint 5)
* Verification Runtime (Sprint 6)
* RobotOps Foundation — SQLite persistence (Sprint 7A)

### Current Focus — Phase B

Epic 8 — Mobile Base Integration

* Sprint 8.1 Base Driver
* Sprint 8.2 Odometry Validation
* Sprint 8.3 Lidar
* Sprint 8.4 SLAM
* Sprint 8.5 Navigation2

Epic 9 — Mobile World Model

Epic 10 — Mobile Agent Runtime

### Next Milestone

Autonomous Mobile Manipulation Robot

---

## Mobile Base Status

Current workspace:

* turn_on_dlrobot_robot
* depend

Purpose:

Bring up the mobile chassis driver.

Future deployment target:

Jetson Orin Nano

Sprint 8.1 validation targets:

* /cmd_vel
* /odom
* /tf

---

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
      (/world_model/stable_objects)   ✅ completed

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
    - /world_model/stable_objects   ✅ Sprint 5.6

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

  → verification_result_node
      (precheck / postcheck / post_place)

  → /runtime/verification_result
      (stage + success + reason + evidence)

  → real_grounded_runtime_node
      (_on_verification_result)

  → verified / verification_failed

---

## Completed Milestones

| Sprint | Milestone | Key Files |
|--------|-----------|-----------|
| 1 | `sketch_runtime` ROS2 package created | `package.xml`, `setup.py`, `setup.cfg` |
| 1 | `TaskContext` / `TaskState` (includes VERIFYING, VERIFIED, VERIFICATION_FAILED) | `task_context.py` |
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
| 5.5 | perception_fusion → StableObjectTracker integrated | stable_tracker |
| 5.5 | stable_objects pipeline validated | tracker + grounding |
| 5.6 | Grounding switched to /world_model/stable_objects | `grounding_node.py`, `perception_bringup.launch.py` |
| 6.1 | Verification Sidecar (verification_result_node) | `verification_result.py`, `verification_result_node.py` |
| 6.2 | Runtime Integration (VERIFYING/VERIFIED/VERIFICATION_FAILED) | `real_grounded_runtime_node.py`, `task_context.py` |
| 6.3 | Runtime Event Visibility (event_id/state/timestamp enrichment) | `real_grounded_runtime_node.py` |
| 6.4 | Post Place Verification | `verification_result_node.py` |
| 6.5 | Runtime Verification Logs | /runtime/log + /runtime/verification_result
        Notes:
        - /runtime/event not implemented
        - RobotOps persistence not implemented
        - Audit database not implemented|
| 6.6 | ⏳ Deferred to Sprint 11 | — |

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

### 4. Verification Runtime Path (dry-run validated)
```
/grounded_task_context
  → real_grounded_runtime_node (executing → verifying)
  → /executor/done (true)
  → verification_result_node (precheck / postcheck / post_place)
  → /runtime/verification_result
  → real_grounded_runtime_node (_on_verification_result)
  → verified / verification_failed
```

Validated scenarios:
- precheck success (object found near source)
- postcheck success (object removed from source)
- postcheck failure (object still at source)
- post_place success (object found at target)
- post_place failure (object not at target)
- full state flow: executing → verifying → verified
- full state flow: executing → verifying → verification_failed

---

## Current Integration Status: Perception Fusion → StableObjectTracker

Completed:

✅ YOLO + ROI → perception_objects (Sprint 5.4)
✅ perception_objects → StableObjectTracker → stable_objects (Sprint 5.5)
✅ stable_objects → grounding (Sprint 5.6)
✅ grounding → /grounded_task_context → runtime (Sprint 4.1)
✅ runtime → execution → verification → result (Sprint 6)
✅ runtime → SQLite persistence (Sprint 7A)

Known limitation:

ROI class/color instability still exists.

Mitigation strategy:

YOLO:
semantic priority

ROI:
pose/color/yaw priority

---

## ROI Tuner Conclusions (2026-05)

Workspace:

tools/vision_roi_tuner/

Result:

Autonomous visual tuning was stopped.

Current rule:

YOLO
→ semantic authority

ROI
→ pose
→ color candidates

Color:

if confidence < threshold:

unknown

Current observation:

blue cup:
stable

block/cube:
not reliable

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
| **ROI Color** | ROI tuner completed. YOLO = semantic authority, ROI = pose/color candidates |
| **ROI 类名/颜色** | 不稳定 — 形状分类和 LAB 颜色跨帧跳变 |
| **抓取精度** | grasp precision 尚不稳定，需进一步标定和补偿 |
| **碰撞检测** | 无碰撞/力矩异常检测 |
| **Skill 支持** | Runtime 当前仅支持 pick_skill。place_skill 未实现。post_place verification 框架存在，但无 place 执行路径。 |
| **重试/恢复** | 无 retry 或 recovery 逻辑 — 延后至 Sprint 11 |
| **持久化** | ✅ RobotOps Foundation completed (Sprint 7A) — SQLite persistence active |
| **并发任务** | FIFO matching only — no concurrent task verification |
| **实物验证** | place 验证未在真实硬件测试 — 当前仅 dry-run / topic 仿真 |

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
| 5.2 | ✅ | StableObjectTracker implemented |
| 5.3 | ⏸ | ROI robustness deferred |
| 5.4 | ✅ | Perception Fusion |
| 5.5 | ✅ | Tracker input switch |
| 5.6 | ✅ | Grounding switch |

## Sprint 5 Progress (5.1 → 5.6)

| Sprint | Status | Milestone |
|--------|--------|-----------|
| 5.1 | ✅ | Raw Detection Audit — `roi_detection_audit_node` 量化 label stability |
| 5.2 | ✅ | StableObjectTracker — 已实现 |
| 5.3 | ⏸ | ROI Shape/Color Robustness — 部分调研，未正式完成 |
| 5.4 | ✅ | Perception Fusion Node — YOLO+ROI 融合 → `/world_model/perception_objects` |
| 5.5 | ✅ | StableObjectTracker 输入切换到 perception_objects |
| 5.6 | ✅ | Grounding 切换到 `/world_model/stable_objects` |

**融合规则**: class_name=YOLO, color/pose/rpy=ROI。YOLO 未检测到的物体保留 roi_only fallback。

## Package Snapshot

```
src/sketch_runtime/
├── sketch_runtime/
│   ├── __init__.py
│   ├── task_context.py          # TaskState enum (VERIFYING / VERIFIED / VERIFICATION_FAILED) + TaskContext
│   ├── target_object.py         # TargetObject + from_world_model/from_detection_result
│   ├── execution_result.py      # ExecutionResult dataclass
│   ├── base_skill.py            # BaseSkill ABC
│   ├── skill_registry.py        # SkillRegistry + SkillManager
│   ├── runtime_adapter.py       # RuntimeAdapter (safety split + temp IK node)
│   ├── runtime_task_builder.py  # TaskBuilder: parsed/grounded/wm → TaskContext
│   ├── base_action.py           # BaseAction ABC + StepResult
│   ├── actions.py               # MoveAction + GripperAction
│   ├── action_executor.py       # ActionExecutor
│   ├── verification_result.py   # VerificationResult dataclass
│   ├── verification_result_node.py  # Verification sidecar node
│   ├── runtime_test_node.py     # Self-contained test node (simulated data)
│   ├── real_grounded_runtime_node.py  # Production node (subscription-driven + Preview/Confirm)
│   └── skills/
│       └── pick_skill.py        # PickSkill (5-step: hover→open→approach→close→lift)
├── launch/
│   ├── runtime_test.launch.py
│   └── ground_runtime_bringup.launch.py
└── test/
    ├── test_task_context.py
    ├── test_target_object.py
    ├── test_skill_registry.py
    ├── test_task_builder.py
    ├── test_actions.py
    ├── test_verification.py
    ├── test_verification_integration.py
    └── test_runtime_events.py
```

---

## Modified Existing Files

| File | Change |
|------|--------|
| `grounding/grounding/grounding_node.py` | +`publish_runtime` param → `/grounded_task_context` |
| `grounding/config/grounding_params.yaml` | +`publish_runtime: true` |
| `llm_executor/llm_executor/executor_node.py` | +`/executor/done` Bool publisher |

---

## Session Resume Entry

When opening a new chat:

Read:

1 runtime_index.md
2 runtime_session_summary.md
3 topic_service_map.md

Current Goal:
Sprint 7A COMPLETED

Next:
Sprint 8.1 — Base Driver Bringup (Phase B)
---

## Sprint 5.6 Completion

Date: 2026-05-30

Completed:

- grounding_node switched to /world_model/stable_objects
- perception_bringup.launch.py created
- StableObjectTracker verified
- Grounding verified
- /grounded_goal verified
- /grounded_task_context verified
- cup → status=ok verified

Known limitation:

- ROI color detection may return unknown
- blue_cup matching can fail
- class-only matching currently recommended

## Sprint 6.1 Completion — Verification Sidecar

Date: 2026-05-30
Commit: 1a6298d feat(runtime): add verification sidecar node

New node: `verification_result_node` (sketch_runtime package)

Subscribes:
- /grounded_task_context
- /world_model/stable_objects
- /executor/done

Publishes:
- /runtime/verification_result

Behavior:
- Observation only — no IK, no servo, no hardware control
- No runtime blocking — does not affect real_grounded_runtime_node
- FIFO matching — one active verification context only
- class_name + position proximity matching
- color is optional — works when color == "unknown"

Validation Results:
- Precheck success verified
- Postcheck failure verified
- Postcheck success verified

Known limitations at Sprint 6.1:
- No task_id correlation — uses FIFO matching (runtime is sequential)
- Postcheck only verifies disappearance from source area (not placement at target)
- No retry or recovery logic

---

## Sprint 6.2 Completion — Runtime Integration

Date: 2026-06-07

Completed:

- `real_grounded_runtime_node` subscribes to `/runtime/verification_result`
- `_on_verification_result()` handler transitions task state
- New TaskContext states: `VERIFYING`, `VERIFIED`, `VERIFICATION_FAILED`
- Verification data stored in `ctx.result["verification"]`

State flows:

```
grounded → skill_selected → waiting_confirm → executing → verifying → verified
```

or

```
grounded → skill_selected → waiting_confirm → executing → verifying → verification_failed
```

Runtime transitions to `VERIFYING` after successful execution, publishes `/executor/done=true`, then waits for the verification sidecar result. On success → `VERIFIED`; on failure → `VERIFICATION_FAILED`.

Modified files:
- `real_grounded_runtime_node.py` — subscription + handler + state transitions
- `task_context.py` — new states + completed_at for VERIFIED/VERIFICATION_FAILED

---

## Sprint 6.3 Completion — Runtime Event Visibility

Date: 2026-06-07

Completed:

- `/runtime/log` enriched events: `event_id`, `event`, `state`, `timestamp`, `data`
- `event_id` format: `evt_{task_id}_{seq:04d}` — monotonic per-task sequencing
- Events emitted at each state transition: grounded_task_received, execution_started, verification_started, verification_complete
- `/runtime/state` publishes task state including verification phases

Tests:
- Added runtime event coverage tests (event_id format, sequencing, state-at-emission)

---

## Sprint 6.4 Completion — Post Place Verification

Date: 2026-06-07

Completed:

- `verification_result_node` detects `intent=place` and stores pending place context
- `_run_post_place_check()` verifies target object exists near target_xyz
- Publishes stage `"post_place"` on `/runtime/verification_result`
- Class match required; color match optional; distance threshold 0.1 m

Place flow:

```
grounded_task_context (intent=place) → store pending
executor/done=true → timer delay → post_place check → publish result
```

Cases tested:
- Case A: post_place success (object found at target)
- Case B: post_place failure (object not found at target)
- Case C: pick context clears place pending, place context clears pick pending

---

## Sprint 6.5 Completion — Runtime Verification Logs

Date: 2026-06-07

Achieved through Sprint 6.1 + 6.3 work. No additional code required.

- `/runtime/verification_result` — verification events (precheck, postcheck, post_place)
- `/runtime/log` — enriched event stream with event_id, state, timestamp
- Task → Execution → Verification → Result playback achievable from current topics

Note: No separate `/runtime/event` topic exists. `/runtime/log` serves the event stream function.

Not implemented (deferred to Sprint 8+):
- `/runtime/event` as a separate topic (not needed — `/runtime/log` serves this role)
- RobotOps persistence
- Audit database

---

## Sprint 6.6 — Deferred Items

Date: 2026-06-07

Deferred to Sprint 11:

- `offset_retry` — offset and re-attempt
- `ask_user_confirm` — prompt user for recovery decision
- `manual_recovery` — operator-driven recovery

`RETRYING` state exists in `TaskContext` as a placeholder. Retry / Recovery remains a Sprint 11 design topic. No implementation.

---

## Implemented Topics (Sprint 6)

| Topic | Publisher | Subscriber |
|-------|-----------|------------|
| `/runtime/verification_result` | `verification_result_node` | `real_grounded_runtime_node` |
| `/runtime/state` | `real_grounded_runtime_node` | — (VERIFYING/VERIFIED/VERIFICATION_FAILED phases) |
| `/runtime/log` | `real_grounded_runtime_node` | — (event_id/state/timestamp enrichment) |
| `/executor/done` | `real_grounded_runtime_node` | `executor_done_sayer`, `verification_result_node` |

---

## Next Sprint Entry Criteria

Sprint 7A COMPLETED.

Current Active Sprint:

Sprint 8.1 — Base Driver Bringup

Phase B — Mobile Robot Foundation

Epic 8: Mobile Base Integration
  - Sprint 8.1 Base Driver (CURRENT)
  - Sprint 8.2 Odometry Validation
  - Sprint 8.3 Lidar
  - Sprint 8.4 SLAM
  - Sprint 8.5 Navigation2

Epic 9: Mobile World Model

Epic 10: Mobile Agent Runtime
  - NavigateSkill, SearchSkill, DockSkill
  - Mobile Manipulation Tasks