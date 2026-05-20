# JetArm Robot Runtime — Session Summary

> Sprint 1 → 4.3 实现状态快照 | 2026-05-18 | 分支: `feature/sketch_runtime_sprint3`

---

## Current Architecture Chain

```
natural language input
  → llm_parser (keyword match → /parsed_command)
  → grounding_node (publish_runtime=true → /grounded_task_context)
  → real_grounded_runtime_node (subscription-driven)
    → TaskBuilder (builds TaskContext from grounded JSON)
    → SkillManager.select("pick") → "pick_skill"
    → SkillManager.instantiate → PickSkill(adapter)
    → if require_confirm: publish /runtime/preview → wait
    → /runtime/confirm (yes/no/JSON) → execute
    → PickSkill.execute()
      → adapter.ik_solve(position, rpy)  [dry_run or _call_ik_blocking]
      → adapter.servo_move(pulses)       [dry_run or real publish]
      → adapter.gripper_set(id, pulse)   [dry_run or real publish]
    → /runtime/execution_result (success/fail/reason)
    → /executor/done (Bool)
```

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

## Current Unresolved Issue: Servo Message Adapter

**Symptom**: `enable_real_servo=true` publishes to `/servo_controller` but fails:
```
"The 'position' field must be of type 'float'"
```
Mechanical arm does not move.

**Root cause**: The `ServoPosition` message definition differs between `servo_controller_msgs` and `ros_robot_controller_msgs`:
- `servo_controller_msgs/ServoPosition`: `uint16 id`, `float32 position`
- `ros_robot_controller_msgs/ServoPosition`: `uint16 id`, `uint16 position`

RuntimeAdapter currently imports from `servo_controller_msgs` but the `controller_manager` may expect a different format or field type.

**Next Sprint (4.4)**: Inspect working servo publish format from old code (`executor_node.py`, `grasp.py`, `actions.py`), map arm pulses to correct servo IDs, fix message field types.

---

## Safety Defaults

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `dry_run` | `true` | Mock IK + log-only servo |
| `require_confirm` | `true` | Must confirm before execute |
| `enable_real_ik` | `false` | Real IK service call |
| `enable_real_servo` | `false` | Real servo publish |
| `run_once` | `true` | Execute once then stop |
| `confirm_timeout_sec` | `300.0` | 5 min confirm window |

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

---

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
