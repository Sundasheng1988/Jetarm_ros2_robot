# JetArm Robot Runtime v0.1 — Refactor Plan

> 基于 runtime_analysis.md 和 topic_service_map.md 的发现
> 目标：安全、分阶段地将"ROS2 节点拼接系统"升级为"Robot Runtime"

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

## 2. Sprint 2（Skill Runtime + Teleop 基础）

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

## 3. Sprint 3（RobotOps 日志 + 监控）

### 目标

每个任务可追踪、可复盘。

### 任务

#### 3.1 robot_ops 包

```text
src/robot_ops/
├── robot_ops/
│   ├── __init__.py
│   ├── task_logger_node.py       # 订阅 /runtime/* 写 SQLite
│   ├── state_monitor_node.py     # 实时状态汇总
│   ├── event_recorder.py         # 事件时序记录
│   └── dashboard_api.py          # 最小 REST API (aiohttp)
└── logs/                          # SQLite 存储
```

#### 3.2 记录字段

每个 `task_id` 记录：user_command → parsed_command → selected_skill → vision_result → target_pose → ik_result → servo_command → execution_status → error_reason → image_snapshot → timestamps

#### 3.3 注意

- `image_snapshot` 可先存路径而非 base64
- `dashboard_api` 用 aiohttp 最小版本（一个 `/api/tasks` 端点 + 一个 `/api/task/<id>` 端点）
- DB 用 SQLite 单文件（零配置，适合嵌入式场景，且不需要额外启动服务）

---

## 4. Sprint 4（验证 / 失败检测）

### 目标

不只"执行动作"，还要"判断是否成功"。

### 任务

#### 4.1 验证节点

```python
# verification/verification_node.py
class VerificationNode(Node):
    def verify_pick(self, task_ctx: TaskContext) -> VerificationResult:
        # 1. 抓取前检测目标是否存在
        # 2. 抓取后检测目标是否从原位置消失
        # 3. 检测夹爪 ID10 是否闭合到合理位置
        # 4. 检测 servo 是否到位
        # 5. 检测是否超时
```

#### 4.2 验证结果格式

```json
{
  "task_id": "task_abc123",
  "success": false,
  "confidence": 0.65,
  "reason": "目标物体抓取后仍存在",
  "evidence": "gripper_closed=False, object_still_present=True",
  "timestamp": 1715900000.0
}
```

#### 4.3 与 Executor 集成

Executor 在 skill 执行完成后调用 VerificationNode，根据 `success` 决定是否 retry。

---

## 5. Sprint 5+（IK 精度优化 / 语音 / VLA）

### 5.1 IK / 抓取精度 (P4)

- 继续使用原厂 IK `.so`
- 排查抓不准原因：相机标定 → 深度噪声 → 夹爪 tip 补偿 → 抓取高度
- 每次抓取记录 `target_pose` 和 `ik_result` 用于偏差分析

### 5.2 语音重构 (P5)

- `llm_voice_agent` 回归纯语音入口定位
- ASR 结果直接发到 `/parsed_command_text`，由统一 parser 处理
- TTS 支持 interrupt、任务状态自然播报

### 5.3 VLA 接入 (P6)

- VLA 作为高级策略模块接入 Runtime，不绕过 ROS2/IK/Servo/仲裁
- 遥控数据采集 → 轨迹记录 → 图像-动作对齐 → 数据集构建

---

## 6. 包/模块影响矩阵

| 包 | Sprint 1 | Sprint 2 | Sprint 3 | Sprint 4 |
|----|----------|----------|----------|----------|
| `grounding` | Fix topic | — | — | — |
| `llm_executor` | Add task_id | Full refactor | Log integration | Verification integration |
| `llm_parser` | — | Standardize ParsedCommand | — | — |
| `vision_yolo` | — | — | Image snapshot | Object detection query |
| `app` | — | — | — | — |
| `servo_controller` | — | — | — | — |
| **新建** `sketch_runtime` | ✅ | Expand | — | — |
| **新建** `teleop_input` | — | ✅ | — | — |
| **新建** `control_arbitration` | — | ✅ | — | — |
| **新建** `robot_ops` | — | — | ✅ | — |
| **新建** `verification` | — | — | — | ✅ |

---

## 7. 风险清单

| 风险 | 缓解策略 |
|------|----------|
| ground_executor 重构引入 regression | 保留 `dry_run=True` 模式；先并存不切换 |
| 仲裁节点增加延迟 | 默认模式 `AUTO` 无锁，仅在冲突时介入 |
| logging 增加 I/O 压力 | 异步写，batch flush |
| 验证结果误判 | 先跑 "保守模式"：只有高置信度才判定失败 |
| 新包安装/构建错误 | 每个 Sprint 独立测试 launch 可运行 |
| `sketch_runtime` 命名模糊 | Sprint 2 前决定最终命名 (`runtime_core` / `agent_runtime`)，用 remap 保持一致 |

---

## 8. 推荐 Git 分支策略

```bash
git checkout -b feature/robot_runtime_v1          # Sprint 总体分支
# 每个 Sprint 从 v1 分出
git checkout -b feature/runtime_task_id            # Sprint 1
git checkout -b feature/skill_runtime              # Sprint 2
git checkout -b feature/teleop_runtime             # Sprint 2b
git checkout -b feature/robotops_logger            # Sprint 3
git checkout -b feature/task_verification          # Sprint 4
```

当前稳定基线：`debug/llm_voice_agent`（冻结为旧架构参考）

---

## 9. Sprint 1 具体文件变更清单

| 变更类型 | 文件 | 说明 |
|----------|------|------|
| **Launch 修改** | `grounding/launch/ground_bringup.launch.py` | 增加 remap: `/world_model/roi_objects` → `/world_model/objects` |
| **新建包** | `src/sketch_runtime/` | 新建 package 骨架 |
| **新建文件** | `src/sketch_runtime/sketch_runtime/runtime_state_node.py` | RuntimeStateNode — 负责 task_id 生成、状态追踪、事件日志 |
| **新建文件** | `src/sketch_runtime/config/runtime_params.yaml` | 参数文件 |
| **新建文件** | `src/sketch_runtime/launch/runtime_bringup.launch.py` | launch 文件 |
| **修改** | `llm_executor/executor_node.py` | 最小注入：task_id 生成、`/runtime/log` 发布、执行结果发布（不改变动作序列） |
| **新建文件** | `src/sketch_runtime/sketch_runtime/skill_manager.py` | SkillManager 骨架（纯语意映射，不连接控制回路） |
| **新建文件** | `src/vision_interfaces/msg/TargetObject.msg` | 标准化目标对象消息（新增而不修改现有 DetectionResult.msg） |

---

## 10. 最终架构目标

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
