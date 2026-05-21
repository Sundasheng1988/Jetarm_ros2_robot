# JetArm Robot Runtime v0.1 — Refactor Plan

> 基于 runtime_analysis.md 和 topic_service_map.md 的发现
> 目标：安全、分阶段地将"ROS2 节点拼接系统"升级为"Robot Runtime"
>
> **⚠ 本文档为历史重构计划。当前 Roadmap 请见: [jetarm_runtime_roadmap.md](jetarm_runtime_roadmap.md)**

---

## Refactor Status Snapshot (2026-05-20)

| 阶段 | 状态 |
|------|------|
| `sketch_runtime` 包 + TaskContext/TargetObject/Skill 骨架 | ✅ COMPLETED |
| TaskBuilder grounding→Runtime 集成 | ✅ COMPLETED |
| Preview/Confirm 安全层 | ✅ COMPLETED |
| RuntimeAdapter (safety split + temp IK node) | ✅ COMPLETED |
| 真实 IK 调用 | ✅ COMPLETED |
| 真实 Servo 发布 | ✅ COMPLETED |
| ActionExecutor + Action Sequence 架构 | ✅ COMPLETED |
| 真实硬件 pick 执行验证 | ✅ COMPLETED |
| **当前阻塞** | 🔴 Perception instability — `/world_model/roi_objects` RAW detection unstable |
| **下一个 Sprint** | Stable World Model (→ Verification Runtime) |
| **当前分支** | `feature/sketch_runtime_sprint3` |
| **测试** | 82 tests passing |

---

## 0. 重构原则

1. **不改硬件层**：`servo_controller`、`ros_robot_controller`、`kinematics`(IK .so)、STM32 固件 永不应修改
2. **PC 侧重构，JetArm 侧冻结**：运行时重构仅限于 PC 上层逻辑
3. **保持原链路可用**：新 Runtime 与原系统并行，不破坏已有 Keyboard→Grab 链路
4. **每阶段可回滚**：通过 ROS2 remap 或 `use_new_runtime` 参数开关控制

---

## 1. Sprint 1（最小闭环，2-3 周）

### 目标

建立 Robot Runtime v0.1 的最小闭环：

```
输入: "抓取红色杯子"
  → Parser 输出 ParsedCommand
  → Grounding 输出 TargetObject
  → Executor 选择 pick_skill
  → pick_skill 调用原厂 IK + servo_controller
  → 返回 success / failed / reason
```

### Sprint 1 任务

#### 任务 1.1: 修复 Topic 不匹配 (P0 Bug)

**问题**: `grounding_node` 订阅 `/world_model/roi_objects`，但 `app_compatible_yolo_node`/`wm_dummy_pub`/`wm_from_tf` 发布到 `/world_model/objects`

**修复方案**: 在 `grounding_node` 或 launch 中统一 topic：

```python
# 方案 A: 在 ground_bringup.launch.py 中 remap
Node(
    package='grounding', executable='grounding_node',
    remappings=[('/world_model/roi_objects', '/world_model/objects')]
)
```

```python
# 方案 B: 在 grounding_node.py 中将订阅参数化
self.declare_parameter('world_model_topic', '/world_model/objects')
```

**推荐方案 A** — 最小改动，不改源码。

#### 任务 1.2: 新建 `sketch_runtime` 包

```text
src/sketch_runtime/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/sketch_runtime
├── config/
│   └── sketch_runtime_params.yaml
└── sketch_runtime/
    ├── __init__.py
    └── sketch_runtime_node.py     # SketchRuntimeNode
```

**SketchRuntimeNode 职责**：
- 订阅 `/parsed_command` 和 `/grounded_goal`
- 生成 `task_id`（UUID v4 + timestamp）
- 创建 `TaskContext` 对象
- 发布 `/runtime/state`（`String` JSON，含 task_id/state/result）
- 发布 `/runtime/log`（`String` JSON，结构化事件）

**模型二选一**：
- 如果 SketchRuntime 本身还要完成具体执行逻辑，可以用 `SketchExecutorNode` 或 `runtime_executor` 命名。
- 最终命名建议: `runtime_executor`（先打出 `task_id` + `state` 骨架）

**文件内容**：

```python
# runtime_executor_node.py (最小版本)
import json, uuid, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RuntimeExecutorNode(Node):
    def __init__(self):
        super().__init__('runtime_executor_node')
        self.sub_cmd = self.create_subscription(String, '/parsed_command', self.on_cmd, 10)
        self.sub_goal = self.create_subscription(String, '/grounded_goal', self.on_goal, 10)
        self.pub_state = self.create_publisher(String, '/runtime/state', 10)
        self.pub_log = self.create_publisher(String, '/runtime/log', 10)

    def _task_id(self):
        return f"task_{uuid.uuid4().hex[:12]}_{int(time.time())}"

    def on_cmd(self, msg):
        data = json.loads(msg.data)
        tid = self._task_id()
        self.pub_state.publish(String(data=json.dumps({
            "task_id": tid, "state": "parsed",
            "parsed_command": data, "timestamp": time.time()
        })))
        self.pub_log.publish(String(data=json.dumps({
            "task_id": tid, "event": "parsed_command", "data": data
        })))

    def on_goal(self, msg):
        data = json.loads(msg.data)
        tid = data.get("task_id") or self._task_id()
        self.pub_state.publish(String(data=json.dumps({
            "task_id": tid, "state": "grounded",
            "grounded_goal": data, "timestamp": time.time()
        })))
```

**指标**：至少 `/runtime/state` 能吐出带 `task_id` 的事件。

**注意**：此节点为纯分析/记录节点，不进入控制回路，不影响机械臂安全。

**依赖项**：仅 `rclpy`, `std_msgs` — 零新依赖。

**回滚**：删除节点或移除 launch 即可。

#### 任务 1.3: 新建 `TargetObject` 标准化消息

在 `vision_interfaces` 或新建接口包中增加（推荐扩展 `vision_interfaces`）：

```msg
# TargetObject.msg
string task_id
string name
string class_name
string color
float32 confidence
float32 center_x
float32 center_y
float32 center_z
float32 world_x
float32 world_y
float32 world_z
string source
float64 timestamp
```

**注意**：消息字段应比当前 `DetectionResult.msg` 的并行数组更适合单帧单目标，且新增 `source` 字段表示数据来源（`yolo`, `roi_color`, `dummy`, `tf`）。

**可选变量**：如果暂时不想碰 `vision_interfaces` 的现有 `.msg`，也可在新的 `runtime_interfaces` 包中定义。

#### 任务 1.4: 在 executor 中引入 Skill 选择（最小改动）

**不改动** `_handle_goal()` 的 10 步序列，仅增加：

1. 生成 `task_id` 并发布到 `/runtime/state`
2. 在 `on_goal()` 中记录 `task_id` 到 `/runtime/log`
3. 在 `_handle_goal()` 完成后发布执行结果 `{task_id, success, reason}`

```python
# executor_node.py 修改点（最小 diff）

def on_goal(self, msg):
    # ... existing validation ...
    self._current_task_id = f"task_{uuid.uuid4().hex[:12]}_{int(time.time())}"
    self.pub_log.publish(... {event: "execution_started", task_id: ...})
    try:
        self._handle_goal(data)
        self.pub_log.publish(... {event: "execution_done", task_id: ..., success: true})
    except Exception as e:
        self.pub_log.publish(... {event: "execution_failed", task_id: ..., reason: str(e)})
```

#### 任务 1.5: 新建 `skill_manager` 骨架（不连接控制回路）

```text
src/sketch_runtime/sketch_runtime/skill_manager.py
```

```python
# skill_manager.py
class SkillManager:
    SKILLS = {
        "pick": "pick_skill",
        "place": "place_skill",
        "move": "move_skill",
    }
    @classmethod
    def select(cls, intent: str) -> str:
        return cls.SKILLS.get(intent, "unknown_skill")
```

此为纯语意层骨架，不进入控制回路。只为验证 TaskContext → Skill 映射可行性。

### Sprint 1 验收标准

- [ ] `/world_model/objects` 能被 grounding 正确接收
- [ ] `runtime_executor_node` 为每个任务生成 `task_id`
- [ ] `/runtime/state` 能追踪 parsed → grounded → execution 三阶段
- [ ] `/runtime/log` 输出结构化 JSON 事件
- [ ] `ground_executor_node` 执行完成后发布 `{task_id, success, reason}`
- [ ] `skill_manager` 骨架可根据 intent 返回 skill name
- [ ] 原有 pick-and-place 链路仍然可用（不破坏）

### Sprint 1 不做的事

- 不重写 executor 的动作序列
- 不新建 skill 文件/插件系统
- 不引入 task 队列/并发
- 不修改 servo_controller
- 不重写 IK
- 不修改 yyolo/vision

---

## 2. Sprint 2（Skill Runtime + Teleop 基础）[SUPERSEDED — see jetarm_runtime_roadmap.md]

### 目标

将硬编码动作序列改造为可扩展的 Skill 系统，并建立 Teleop 控制权仲裁。

### 任务

#### 2.1 Skill 抽象层

```python
# skill_manager.py
class BaseSkill:
    async def execute(self, ctx: TaskContext) -> ExecutionResult:
        ...

class PickSkill(BaseSkill): ...
class PlaceSkill(BaseSkill): ...
class MoveSkill(BaseSkill): ...
class HomeSkill(BaseSkill): ...
```

Skill 通过 adapter 调用原厂 IK 和 servo_controller，不直接操作硬件。

#### 2.2 teleop_input 包

```text
src/teleop_input/
├── package.xml
├── setup.py
├── teleop_input/
│   ├── __init__.py
│   ├── keyboard_teleop_node.py   # 键盘 → cartesian/angular velocity
│   └── gamepad_teleop_node.py    # 手柄 → 同上
└── launch/teleop.launch.py
```

#### 2.3 控制权仲裁

```text
src/control_arbitration/
├── control_arbitration/
│   └── arbiter_node.py
```

`ArbiterNode` 维护全局状态：`AUTO` / `MANUAL` / `PAUSED` / `EMERGENCY_STOP`。

所有控制节点 (`ground_executor`, `face_follow`, `teleop_*`) 在发送舵机命令前必须经过 `ArbiterNode`。

**ArbiterNode 的具体机制**：

- 提供一个 `ArbiterNode` (node name `control_arbiter`)
- 发布 `/control/state` topic (String, `AUTO|MANUAL|PAUSED|ESTOP`)
- 通过 `/control/request` (String) 和 `/control/ack` (Bool) 实现锁的申请与释放
- **注意**：具体实现时需考虑 ArbgiterNode 如果走 `sync_client / sync_response` 可能会引入执行延迟；另一种方案是 executor 在发 servo 前 spin_until_future_complete 等待仲裁确权

再往后（P2-P3）的 RobotOps 日志和 Verification 模块在 Sprint 完成后、actor 稳定后再引入。

---

## 3. Sprint 3（RobotOps 日志 + 监控）[SUPERSEDED]

## 4. Sprint 4（验证 / 失败检测）[SUPERSEDED]

## 5. Sprint 5+（IK 精度优化 / 语音 / VLA）[SUPERSEDED]

## 6. 包/模块影响矩阵 [SUPERSEDED]

## 7. 风险清单 [SUPERSEDED — risks maintained in runtime_risks.md]

## 8. 推荐 Git 分支策略 [SUPERSEDED]

## 9. Sprint 1 具体文件变更清单 [ARCHIVED — historical reference]

## 10. 最终架构目标 [SUPERSEDED]

```
user command → parser → grounding → executor → skill runtime
                                │                │
                                ▼                ▼
                        task_manager       robot_ops (log)
                                │
                                ▼
                        control_arbiter
                                │
                        ┌───────┴───────┐
                        ▼               ▼
                  AUTO mode       MANUAL mode
                        │               │
                        ▼               ▼
                 skill execution    teleop input
                        │               │
                        └───────┬───────┘
                                ▼
                    [servo_controller / IK]  ← 永不该修改的底层
                                │
                                ▼
                        [JetArm Hardware]
```
