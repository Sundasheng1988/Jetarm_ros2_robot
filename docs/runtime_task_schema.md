# Robot Runtime v0.1 — Task Schema

> 设计目标：定义统一的 TaskContext 数据结构、生命周期状态机、ROS2 通信格式
> 兼容：Teleop / Verification / RobotOps（P1-P3 逐步接入）

---

## 1. TaskContext 数据结构

```python
# sketch_runtime/task_context.py
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Dict, Any, List
import time

class TaskState(Enum):
    CREATED          = "created"           # task_id 生成，尚未解析
    PARSED           = "parsed"            # parser 完成，有 parsed_command
    GROUNDED         = "grounded"          # grounding 完成，有 target_object + target_pose
    SKILL_SELECTED   = "skill_selected"    # skill_mgr 已选定 skill
    EXECUTING        = "executing"         # skill 执行中
    DONE             = "done"              # 执行成功
    FAILED           = "failed"            # 执行失败（含 reason）
    CANCELLED        = "cancelled"         # 用户/系统取消
    PAUSED           = "paused"            # Teleop 接管时暂停（P1 引入）
    RETRYING         = "retrying"          # 失败后重试中

@dataclass
class TaskContext:
    # ── 核心标识 ──
    task_id: str                              # "task_a1b2c3d4_1715900000"

    # ── 输入 ──
    user_command: str = ""                    # 原始输入："抓取红色杯子放到右边"
    source: str = "keyboard"                  # 来源: "keyboard" | "voice" | "teleop" | "web"

    # ── 解析结果 ──
    parsed_command: Optional[Dict] = None     # {action, from, to, raw}

    # ── Grounding 结果 ──
    target_object: Optional[Dict] = None      # TargetObject dict
    target_pose: Optional[Dict] = None        # {frame, xyz, rpy}

    # ── Skill 选择 ──
    selected_skill: str = ""                  # "pick_skill" | "place_skill" | "move_skill"
    skill_params: Dict = field(default_factory=dict)  # skill 特定参数

    # ── 状态 ──
    state: TaskState = TaskState.CREATED
    state_history: List[Dict] = field(default_factory=list)  # [{state, ts, detail}]

    # ── 结果 ──
    result: Optional[Dict] = None             # {success, reason, evidence}

    # ── 时间戳 ──
    created_at: float = field(default_factory=time.time)
    parsed_at: Optional[float] = None
    grounded_at: Optional[float] = None
    started_at: Optional[float] = None
    completed_at: Optional[float] = None

    # ── 控制 ──
    priority: int = 0                         # 0=normal, 1=high, 2=urgent (P1)
    retry_count: int = 0
    max_retries: int = 2

    # ── 元数据 ──
    metadata: Dict = field(default_factory=dict)  # 可扩展字段

    def transition(self, new_state: TaskState, detail: str = ""):
        old = self.state
        self.state = new_state
        self.state_history.append({
            "from": old.value,
            "to": new_state.value,
            "timestamp": time.time(),
            "detail": detail
        })
        if new_state == TaskState.PARSED:
            self.parsed_at = time.time()
        elif new_state == TaskState.GROUNDED:
            self.grounded_at = time.time()
        elif new_state == TaskState.EXECUTING:
            self.started_at = time.time()
        elif new_state in (TaskState.DONE, TaskState.FAILED, TaskState.CANCELLED):
            self.completed_at = time.time()

    @property
    def elapsed_ms(self) -> float:
        if self.completed_at:
            return (self.completed_at - self.created_at) * 1000
        return (time.time() - self.created_at) * 1000

    def to_dict(self) -> dict:
        return {
            "task_id": self.task_id,
            "state": self.state.value,
            "user_command": self.user_command,
            "source": self.source,
            "parsed_command": self.parsed_command,
            "selected_skill": self.selected_skill,
            "priority": self.priority,
            "retry_count": self.retry_count,
            "created_at": self.created_at,
            "elapsed_ms": self.elapsed_ms,
        }
```

---

## 2. 状态转移图

```mermaid
graph TD
    CREATED("CREATED<br/>task_id 生成")
    PARSED("PARSED<br/>parser 完成")
    GROUNDED("GROUNDED<br/>grounding 完成<br/>target_object + target_pose")
    SKILL_SEL("SKILL_SELECTED<br/>skill_mgr 选定 skill")
    EXECUTING("EXECUTING<br/>skill 执行中")
    DONE("DONE<br/>成功")
    FAILED("FAILED<br/>失败 + reason")
    CANCELLED("CANCELLED<br/>取消")
    RETRYING("RETRYING<br/>重试中")

    CREATED --> PARSED
    PARSED --> GROUNDED
    PARSED --> CANCELLED
    GROUNDED --> SKILL_SEL
    GROUNDED --> CANCELLED
    SKILL_SEL --> EXECUTING
    EXECUTING --> DONE
    EXECUTING --> FAILED
    EXECUTING --> CANCELLED
    FAILED --> RETRYING
    RETRYING --> SKILL_SEL
    FAILED --> CANCELLED
    RETRYING --> CANCELLED
    CREATED --> CANCELLED
```

---

## 3. 状态转移规则表

| 从 | 到 | 触发条件 | 负责节点 |
|----|-----|----------|----------|
| `CREATED` | `PARSED` | `/parsed_command` 到达 | `runtime_state_node` |
| `CREATED` | `CANCELLED` | 用户取消 or 超时未收到 parsed | `runtime_state_node` |
| `PARSED` | `GROUNDED` | `/grounded_goal` 到达且 `status="ok"` | `runtime_state_node` |
| `PARSED` | `CANCELLED` | grounding 返回 `no_match` / `no_target` | `runtime_state_node` |
| `GROUNDED` | `SKILL_SELECTED` | `skill_manager` 映射成功 | `ground_executor_node` |
| `GROUNDED` | `CANCELLED` | 无可用 skill (unknown_skill) | `ground_executor_node` |
| `SKILL_SELECTED` | `EXECUTING` | `control_arbiter` 授权 (P2) | `ground_executor_node` |
| `EXECUTING` | `DONE` | `skill.execute()` 返回 `success=true` | `ground_executor_node` |
| `EXECUTING` | `FAILED` | `skill.execute()` 返回 `success=false` | `ground_executor_node` |
| `EXECUTING` | `CANCELLED` | 用户取消 or ESTOP 触发 | `ground_executor_node` |
| `FAILED` | `RETRYING` | `retry_count < max_retries` | `ground_executor_node` |
| `RETRYING` | `SKILL_SELECTED` | 重新进入 skill 选择流程 | `ground_executor_node` |
| `FAILED` | `CANCELLED` | `retry_count >= max_retries` or 用户跳过 | `ground_executor_node` |
| `RETRYING` | `CANCELLED` | 用户取消 | `ground_executor_node` |

---

## 4. ROS2 Topic 通信格式

### 4.1 `/runtime/state` — 任务状态广播

```json
{
  "task_id": "task_a1b2c3d4_1715900000",
  "state": "executing",
  "user_command": "抓取红色杯子放到右边",
  "source": "voice",
  "selected_skill": "pick_skill",
  "priority": 0,
  "retry_count": 0,
  "progress": {
    "step": 4,
    "total": 7,
    "description": "gripper_close_pick"
  },
  "created_at": 1715900000.0,
  "elapsed_ms": 3123.0,
  "timestamp": 1715900003.123
}
```

### 4.2 `/runtime/log` — 结构化事件日志

```json
{
  "task_id": "task_a1b2c3d4_1715900000",
  "event": "state_transition",
  "data": {
    "from": "skill_selected",
    "to": "executing",
    "detail": "skill pick_skill started, control authorized"
  },
  "timestamp": 1715900003.456
}
```

可选事件类型:
- `task_created`
- `state_transition`
- `parsed_command`
- `grounded_goal`
- `skill_selected`
- `ik_call` (p3 position + pitch + range → pulses)
- `servo_command` (pulses + duration + dry_run)
- `execution_step_completed`
- `execution_result`

### 4.3 `/runtime/execution_result` — 执行结果

```json
{
  "task_id": "task_a1b2c3d4_1715900000",
  "success": true,
  "reason": "pick_and_place_completed",
  "confidence": 0.92,
  "evidence": {
    "gripper_closed": true,
    "object_moved": true,
    "object_placed": true,
    "servo_reached": true,
    "duration_ms": 12300,
    "ik_calls": 6,
    "ik_failures": 0
  },
  "timestamp": 1715900015.789
}
```

---

## 5. 与 Teleop / RobotOps / Verification 的兼容设计

| 层 | 兼容点 |
|---|--------|
| **Teleop** | `source` 区分 `keyboard`/`voice`/`teleop`；`priority` 允许手动命令优先；`PAUSED` 状态为 Teleop 接管预留 |
| **RobotOps** | `state_history[]` 记录完整状态转移时间线；`/runtime/log` 按 `task_id` 分组聚合；`evidence` 持久化到 SQLite |
| **Verification** | `result.evidence` 供 Verification 节点填充；`retry_count`/`max_retries` 驱动 retry 决策 |
| **未来扩展** | `metadata: dict` 可承载 VLA 模型版本、IK 初始猜测、用户意图向量等字段，不改接口 |

---

## 6. Task 生命周期时序

```
time →
──────────────────────────────────────────────────────────────────────────────
CREATED         PARSED    GROUNDED   SKILL_SEL  EXECUTING          DONE
  │              │          │           │          │                │
  │.transition() │          │           │          │                │
  ├─ /parsed_cmd│          │           │          │                │
  │              ├─ /grounded_goal     │          │                │
  │              │          ├─ skill_mgr.select() │                │
  │              │          │           ├─ skill.precheck()        │
  │              │          │           │    skill.execute()       │
  │              │          │           │    ┌──────────────────┐  │
  │              │          │           │    │ ik + servo +     │  │
  │              │          │           │    │ gripper steps    │  │
  │              │          │           │    └──────────────────┘  │
  │              │          │           │     skill.postcheck()    │
  │              │          │           │            ├─ success ───┤
  │              │          │           │            │             │
  │              │          │           │            ├─ fail ──> RETRYING
  │              │          │           │            │              │
  /runtime/state /runtime/state   /runtime/state  /runtime/state /runtime/state
  /runtime/log   /runtime/log     /runtime/log    /runtime/log   /runtime/log
```

---

## 7. 与现有 ground_executor_node 的兼容过渡

Sprint 1 最小改动方案（不改原 10 步序列）：

```python
# 在 ground_executor_node.on_goal() 开头注入
def on_goal(self, msg):
    data = json.loads(msg.data)
    ctx = TaskContext(
        user_command=data.get("raw", ""),
        parsed_command=data,
    )
    ctx.transition(TaskState.GROUNDED)  # 当前阶段：直接从 grounded 进入

    # 发布 task_id + 状态
    self.pub_state.publish(String(data=json.dumps({
        "task_id": ctx.task_id,
        "state": ctx.state.value,
        ...
    })))

    # 原有逻辑不变（Sprint 2 再切换为 Skill 调度）
    self._handle_goal(data)

    # 执行完成后发布结果
    ctx.transition(TaskState.DONE)  # or FAILED
    self.pub_state.publish(...)
```
