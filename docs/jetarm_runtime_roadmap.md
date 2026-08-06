# JetArm Robot Runtime v0.1 — 开发路线图

> 合并原始 Roadmap + 当前实施进度 + Sprint 5→10 计划
> 状态标记: ✅ COMPLETED | 🔶 IN_PROGRESS | ⬜ PLANNED | ⏸ DEFERRED
> 最后更新：2026-06-14
> **🎯 里程碑达成: Verification Runtime Dry-Run 闭环验证通过 · RobotOps Foundation COMPLETED**

---

## Project Strategic Shift (2026-06)

Phase A — the Runtime Platform — is largely complete.

The project is transitioning from:

**Fixed Arm Runtime Platform**

to:

**Autonomous Mobile Manipulation Robot**

Sprint 7A (RobotOps Foundation) marks the completion of the Runtime Platform foundation.

All future development focus moves to mobile robotics.

### Autonomous Mobile Manipulation Robot GOal

User: "Go to the kitchen and bring me a cup."

System flow:

NavigateSkill → SearchSkill → Grounding → PickSkill → NavigateSkill → PlaceSkill

The goal is an **Autonomous Mobile Manipulation Robot** — a robot that can:
- Navigate to a location using SLAM + Navigation2
- Perceive objects using cameras + lidar
- Ground natural language commands to physical objects
- Execute manipulation (pick, place, move) with verification
- Record all events for audit, replay, and VLA training
- Operate with safety arbitration between base and arm

---

## Phase A — Runtime Platform ✅ COMPLETED

**目标**: 建立固定机械臂的完整 Runtime 执行链路
**时间**: Sprint 1 — 7A


---

## 原始 Roadmap 回顾（来自 `robot_runtime/jetarm_robot_runtime_roadmap.md`）

> 以下为原始 Roadmap 的核心判断与方向，作为历史基线保留。

```text
P0：统一任务执行框架           ✅ 已进入 Sprint 4
P1：遥控输入 + 安全接管        ⬜ 计划 Sprint 7
P2：数据写入、监控、回传        ⬜ 计划 Sprint 6
P3：任务成功验证 / 失败检测     ⬜ 计划 Sprint 5
P4：IK / 抓取精度优化           ⬜ 计划 Sprint 8+
P5：语音系统重构                ⏸ 延后
P6：高级 IK / VLA / 数据集能力  ⬜ 计划 Sprint 9
```

---

## Sprint 1: sketch_runtime 包骨架 ✅ COMPLETED

**目标**: 新建 `sketch_runtime` ROS2 包，实现 Runtime 抽象层最小集合。

**完成内容**:
- `TaskContext` dataclass + `TaskState` enum (11 状态)
- `TargetObject` dataclass + 工厂方法 (`from_world_model`, `from_detection_result`)
- `ExecutionResult` dataclass
- `BaseSkill` ABC (precheck → execute → postcheck → cleanup)
- `SkillRegistry` 单例 + `SkillManager` (intent→skill_name 映射)
- `RuntimeAdapter` 骨架 (dry_run mock)
- `PickSkill` (5 步抓取序列: hover→open→approach→close→lift)
- 65 个单元测试全部通过

---

## Sprint 2: Grounding → Runtime 集成 ✅ COMPLETED

**目标**: 将 grounding 输出转换为 TaskContext。

**完成内容**:
- `TaskBuilder` 类 (6 个转换方法)
- `grounding_node.py` 新增可选 `publish_runtime` 参数 → `/grounded_task_context`
- `grounding_params.yaml` 默认 `publish_runtime: true`
- `executor_node.py` 新增 `/executor/done` Bool 发布
- `TargetObject.from_detection_result()` 工厂方法

---

## Sprint 3: 集成测试节点 ✅ COMPLETED

**目标**: 创建端到端干运行测试节点和 launch 文件。

**完成内容**:
- `runtime_test_node.py` — 模拟 parsed+grounded → 7 步执行链
- `runtime_test.launch.py` — 支持 `test_command` 参数
- Sprint 3.1: `run_once`/`interval_sec`/`dry_run` 参数 + ONCE/LOOP 模式

---

## Sprint 4.1: 真实 Runtime 桥接节点 ✅ COMPLETED

**目标**: 创建订阅驱动的真实 Runtime 节点 + 集成 launch。

**完成内容**:
- `real_grounded_runtime_node.py` — 订阅 `/grounded_task_context`，构建 TaskContext，执行 Skill
- `ground_runtime_bringup.launch.py` — 集成 grounding bringup + Runtime 桥接
- 支持所有 grounding 参数透传

---

## Sprint 4.2: Preview / Confirm 安全层 ✅ COMPLETED

**目标**: 添加执行前预览确认机制。

**完成内容**:
- `/runtime/preview` topic (task_id, intent, skill, target_object, summary)
- `/runtime/confirm` subscription (JSON `{task_id, confirm}` + 简单 "yes"/"no")
- `WAITING_CONFIRM` 状态添加到 TaskState
- 确认超时 (默认 300s) → 自动取消
- `require_confirm` 参数 (默认 true)
- 取消路径: `/executor/done` = false

---

## Sprint 4.3: PickSkill Fix + IK 安全分级 ✅ COMPLETED

**目标**: 修复 PickSkill 位姿读取 Bug，实现分级 IK 安全。

**完成内容**:
- PickSkill source_pose key 修复: `"pose"` → `"source_pose"` (回退到 [0,0,0] 的 bug 消除)
- RuntimeAdapter 安全分级: `dry_run` / `enable_real_ik` / `enable_real_servo` 独立控制
- `_call_ik_blocking()` — 每个 IK 调用创建临时 rclpy node，避免 executor 冲突
- 真实 IK 调用已验证通过 (pulse: `[476, 422, 313, 55, 504]`)
- IK 请求日志: position, pitch, pitch_range, resolution, success, pulse, rpy

---

## Sprint 4.4: Servo Message Adapter ✅ COMPLETED

**目标**: 修复 servo 消息格式，使 `enable_real_servo=true` 能正确发布。

**完成内容**:
- RuntimeAdapter `sp.position` 改为 `float()` 显式转换
- Servo imports 移至顶层 (`_HAS_SERVO_MSGS` guard)
- `gripper_set` 独立发布 (非通过 `servo_move`)
- 移除 silent exception swallowing

---

## Sprint 4.5: Hover-Only Safety Test ✅ COMPLETED

**目标**: 最小实物运动测试 — 仅移动到物体上方悬停。

**完成内容**:
- PickSkill 支持 `execution_stage: hover_only`
- 仅执行 1 个 MoveAction (不下降、不抓取)
- 实物验证通过

---

## Sprint 4.6: Action Sequence Architecture ✅ COMPLETED

**目标**: 从 PickSkill 硬编码中提取可组合的 Action 抽象。

**完成内容**:
- `BaseAction` ABC + `StepResult` dataclass
- `MoveAction` (ik_solve → servo_move)
- `GripperAction` (gripper_set)
- `ActionExecutor.run()` — 顺序执行，失败即停
- PickSkill 85 行委托给 ActionExecutor

---

## Sprint 4.7: Full Pick 真实硬件执行 ✅ COMPLETED

**目标**: 完成一次最小完整抓取 (不含放置)。

**完成内容**:
- ActionExecutor 时序同步 (`await asyncio.sleep(wait_after_sec)`)
- `wait_after_sec = duration_ms/1000.0` for MoveAction and GripperAction
- 完整 pick 序列在真实硬件上验证通过

---

## Sprint 4.8: 文档更新 + 里程碑标记 ✅ COMPLETED

**目标**: 更新文档反映当前状态。

**完成内容**:
- runtime_index.md / runtime_session_summary.md / jetarm_runtime_roadmap.md 更新
- 里程碑标记: "首次 Runtime 驱动的真实机械臂 pick 执行验证通过"
- Current Known Limitations 记录

---

## Sprint 5: Stable World Model / Perception Fusion ✅ COMPLETED

**目标**: YOLO + ROI 感知融合 → StableObjectTracker 时序稳定 → 供 Grounding 消费。

**Context**: ROI 形状分类不稳定，YOLO 语义稳定但无颜色/可靠位姿。Fusion 解决互补问题。ROI 颜色/类名不稳定仍为已知限制。

---

### Sprint 5.1 — Raw Detection Audit ✅ COMPLETED

**目标**: 量化当前感知不稳定程度。

**成果**:
- `roi_detection_audit_node` — 收集 100 帧 ROI 检测，量化 label stability
- 发现同一物体跨帧 class/color 跳变 (cup red → cup black → cylinder red)，label_stability_ratio ≈ 0.57
- 输出 `artifacts/perception_audit/roi_perception_audit.md`

**文件**: `src/app/app/roi_detection_audit_node.py`, `src/app/app/audit_utils.py`

---

### Sprint 5.2 — StableObjectTracker 🔶 IMPLEMENTED (integration pending)

**目标**: 新建 `StableObjectTracker` 节点。

**成果**:
- 空间追踪 + 20 帧多数投票 + EMA 置信度平滑 (`α=0.2`) + TTL 3 秒
- 输入 `/world_model/roi_objects` → 输出 `/world_model/stable_objects`
- 24 个单元测试通过
- ⚠ 输入目前为 raw ROI，待 Sprint 5.5 切换到 `/world_model/perception_objects`

**文件**: `src/app/app/stable_object_tracker_node.py`, `src/app/app/stable_tracker_utils.py`

---

### Sprint 5.3 — ROI Shape/Color Robustness ⏸ DEFERRED / PARTIAL

**目标**: 修复 ROI 颜色与形状分类不稳定。

**状态**: 部分调研完成，未正式完成。

- `right_angle_count` cube-before-cup 重排已实验性实现
- 但 ROI 颜色/类名不稳定仍未根本解决
- 战略决策：减少对 ROI class_name 的依赖，以 YOLO 语义为主，ROI 提供 pose/color/yaw
- 保留 `roi_only` 作为 YOLO 词汇外物体（如红色方块）的 fallback
- 后续 StableObjectTracker 在融合之后运行，过滤残留的不稳定

**文件**: `src/app/app/roi_color_detector_node.py` (实验性修改，未完成)

---

### Sprint 5.4 — Perception Fusion Node ✅ COMPLETED

**目标**: YOLO + ROI 融合节点。

**成果**:
- YOLO class_name + ROI color/pose/rpy → `/world_model/perception_objects`
- 空间距离 < 0.06m → `yolo_roi_fused`；无匹配 → `roi_only` / `yolo_only`
- 缓存 TTL 2 秒 + 空间去重 + 3 Hz 发布
- 19 个单元测试 (含去重逻辑)

**文件**: `src/app/app/perception_fusion_node.py`, `src/app/app/perception_fusion_utils.py`

---

### Sprint 5.5 — StableObjectTracker Input Switch ✅ COMPLETED

**目标**: StableObjectTracker 输入从 `/world_model/roi_objects` 切换到 `/world_model/perception_objects`。

**成果**:
- 默认 `input_topic` 改为 `/world_model/perception_objects`
- QoS 条件适配: perception_objects → reliable, roi_objects → BestEffort
- `extract_frame_data` 保留 source/yolo_class/roi_class/match_distance
- `build_stable_object` 输出 `source="stable"` + `source_votes`
- 26 个单元测试通过

---

### Sprint 5.6 — Grounding Switch to Stable ✅ COMPLETED

**目标**: grounding_node 从 `/world_model/roi_objects` 切换到 `/world_model/stable_objects`。

**完成内容**:
- perception_bringup.launch.py 添加为标准化感知启动入口
- camera startup 保持独立: `ros2 launch peripherals depth_camera.launch.py`
- StableObjectTracker → Grounding 链路验证通过
- `/grounded_goal` status=ok 验证通过 (from="cup")
- `/grounded_task_context` 验证通过
- 已知限制: ROI color detection 可能返回 unknown, blue_cup 匹配可能失败, 建议使用 class-only 匹配

---

### Sprint 5 验收标准 (更新)

- [x] ROI audit 完成，数据量化
- [x] StableObjectTracker 发布 `/world_model/stable_objects`
- [x] PerceptionFusionNode 发布 `/world_model/perception_objects`
- [x] YOLO 语义 + ROI 位姿/颜色融合规则确立
- [x] StableObjectTracker 输入切换到 perception_objects
- [x] grounding_node 切换到 stable_objects
- [ ] 不修改 ROI 检测逻辑、YOLO 检测逻辑

---

## Sprint 6: Verification Runtime ✅ COMPLETED

**目标**: 不只知道"执行了"，更要知道"成功了"。

> **前置条件**: Sprint 5 Stable World Model 完成 (`/world_model/stable_objects`)。

---

### Sprint 6.1 — Verification Architecture Plan ✅ COMPLETED

**目标**: 定义 VerificationResult 数据结构与验证阶段。

**VerificationResult 数据结构**:

```python
@dataclass
class VerificationResult:
    task_id: str
    stage: str              # "pre_pick" | "post_pick" | "post_place"
    success: bool
    confidence: float
    reason: str
    evidence: dict
    timestamp: float
```

**验证阶段**:

* precheck_target_exists
* after_pick_target_removed
* after_place_target_present

**新建文件**:

```text
src/sketch_runtime/sketch_runtime/verification_result.py
src/sketch_runtime/sketch_runtime/verification_result_node.py
src/sketch_runtime/test/test_verification.py
```

**完成内容**:

* VerificationResultNode 独立 sidecar 节点实现
* 订阅：

  * /grounded_task_context
  * /world_model/stable_objects
  * /executor/done
* 发布：

  * /runtime/verification_result
* Precheck:

  * 确认目标存在于源位置附近
* Postcheck:

  * 确认目标从源位置消失
* 软件仿真验证通过

  * Case A: precheck success
  * Case B: postcheck failure
  * Case C: postcheck success
* 101 tests passed
* 19 verification tests passed
* Observation-only 设计确认：

  * 无 IK
  * 无 servo
  * 无硬件控制
  * 无 Runtime 阻塞
* FIFO 匹配：

  * 仅支持顺序任务

---

### Sprint 6.2 — Runtime Integration ✅ COMPLETED

**功能**:

让 Runtime 正式消费 Verification 结果。

**流程**:

```text
TaskContext
↓
Executor
↓
VerificationResult
↓
Runtime State
```

**修改文件**:

```text
src/sketch_runtime/sketch_runtime/real_grounded_runtime_node.py
src/sketch_runtime/sketch_runtime/task_context.py
```

**完成目标**:

* 订阅：

  * /runtime/verification_result
* Runtime 接收 VerificationResult
* Runtime 保存验证状态
* Runtime 根据验证结果更新任务状态

---

### Sprint 6.3 — Runtime Verification State ✅ COMPLETED

**功能**:

增加 Runtime 生命周期中的验证状态。

**新增状态**:

```text
VERIFYING
VERIFIED
VERIFICATION_FAILED
```

**目标流程**:

```text
EXECUTING
↓
VERIFYING
↓
VERIFIED
```

或

```text
EXECUTING
↓
VERIFYING
↓
VERIFICATION_FAILED
```

**完成目标**:

* Runtime State 可观测
* /runtime/state 显示验证阶段

---

### Sprint 6.4 — After Place Verification ✅ COMPLETED

**流程**:

Place 完成后：

```text
place
↓
等待 0.5s
↓
验证目标是否出现在目标区域
```

**验证内容**:

* target exists near target pose
* target class matches
* target color matches（可选）

**保守策略**:

第一版仅记录 evidence。

不自动恢复。

不自动重试。

---

### Sprint 6.5 — Runtime Verification Logs ✅ COMPLETED

**Topic**:

```text
/runtime/verification_result
/runtime/event
```

**功能**:

* Verification 历史记录
* Runtime Event 记录
* RobotOps 审计基础

**完成目标**:

能够回放：

```text
Task
↓
Execution
↓
Verification
↓
Result
```

---

### Sprint 6.6 — Retry Plan ⏳ DEFERRED TO SPRINT 7

**方案**:

```text
offset_retry
ask_user_confirm
manual_recovery
```

Sprint 7 实现。

仅输出设计文档。

不开发代码。


## Sprint 7A: RobotOps Foundation ✅ COMPLETED

**目标**: 任务事件持久化 — 关闭后仍可复盘。

**完成内容**:
- ✅ 新建 `robotops` ROS2 包 (独立于 `sketch_runtime`)
- ✅ SQLite `events` 表 — 单一事件表存储所有 runtime 原始 JSON
- ✅ RobotOpsRecorderNode — 订阅 /runtime/state, /runtime/log, /runtime/execution_result, /runtime/verification_result
- ✅ Observer-only 设计 — 不修改任何 runtime 发布者
- ✅ 单表 schema + 任务级索引
- ✅ EventStore 持久化接口
- ✅ TaskHistory 查询接口
- ✅ 20 个单元测试全部通过
- ✅ PC-side real Runtime event stream validated: /runtime/state, /runtime/log, and /runtime/verification_result persisted into SQLite.

---

## Supporting Tracks

These tracks remain valuable and may be developed in parallel.
However they are not the primary strategic focus for 2026-06.

### Sprint 7B: RobotOps Dashboard ⬜ PLANNED

**目标**: 可视化任务执行与验证历史。

**任务**:
- 任务时间线浏览器 (task → events → state transitions)
- 验证时间线 (precheck → postcheck → post_place)
- 最小 API 或终端查看器: task 列表 + task 详情 + 回放
- `/api/tasks`, `/api/task/:id`

---

### Sprint 8A: Teleop Input ⬜ PLANNED

**目标**: 人类可通过键盘/手柄干预机器人。

**任务**:
- `keyboard_teleop_node` — 键盘 → 笛卡尔/关节速度
- `/control/request` topic
- 手动介入模式 (MANUAL)

---

### Sprint 8B: Teleop Safety Arbitration ⬜ PLANNED

**目标**: 多输入源下保证安全控制。

**任务**:
- `control_arbiter_node` — 控制所有权仲裁
- 控制状态: `AUTO` / `MANUAL` / `PAUSED` / `EMERGENCY_STOP`
- 优先级: EMERGENCY_STOP > MANUAL > AUTO
- 控制切换记录 → `/robotops/arbitration_log`

---

### Sprint 9: Data Logger / Demonstration Collection ⬜ PLANNED

**目标**: 采集真实操作数据供 VLA 训练。

**任务**:
- 图像快照 (pre-pick, post-pick, post-place)
- 动作日志 (IK 脉冲, servo 位置, 夹爪状态)
- Runtime 状态日志 (/runtime/state, /runtime/log → per-task file)
- 验证结果日志 (/runtime/verification_result → per-task file)
- 演示数据集收集 (human demo → trajectory → file)

---

### Sprint 10: VLA Readiness ⬜ PLANNED

**目标**: 为 VLA 模型集成准备数据和接口。

**任务**:
- 数据集构建器 (trajectory + image → training format)
- 轨迹导出 (task_id → file)
- 任务回放 (从 SQLite 重建执行链)
- VLA bridge 准备 (VLA command → Runtime input 适配)

---

### Sprint 11: Retry / Recovery ⬜ PLANNED

**目标**: 基于真实失败数据设计恢复策略。

**任务**:
- `stop-only` — 失败后停止，等待人工
- `ask_user_confirm` — 提示用户选择恢复方式
- `offset_retry` — 偏移位姿后重试
- `auto_retry` — 自动重试 (n 次限制)

---

## Phase B — Mobile Robot Foundation ⬜ CURRENT FOCUS

**Current Focus (2026-06)**

These are now the primary engineering priorities.

**架构变更**:

```
旧架构:  Fixed Arm + Depth Camera + Grounding + Runtime + RobotOps
新架构:  Mobile Base + Lidar + SLAM + Navigation2 + World Model + Grounding + Runtime + RobotOps + Manipulator
```

### Epic 8 — Mobile Base Integration

| Sprint | 状态 | 目标 | DoD |
|--------|------|------|-----|
| 8.1 | Done | Base Driver — ROS2 driver for mobile base chassis | `/cmd_vel` working |
| 8.2 | Done  | Odometry Validation — verify wheel encoders, IMU fusion, TF frames | `/odom` + TF working |
| 8.3 | Done  | Lidar — 2D/3D lidar integration, point cloud processing | `/scan` topic producing data |
| 8.4 | Done  | SLAM — Cartographer / FastSLAM, map building | map generation successful |
| 8.5 | Ongoing| Navigation2 — AMCL, NavFn, path planning, obstacle avoidance | Nav2 goal execution successful |

### Epic 9 — Mobile World Model

**目标**: 世界模型融合移动基座位姿数据，使物体定位不依赖固定的相机/基座坐标。

**任务**:
- 基座位姿 → 世界模型坐标转换 (odom → map → object poses)
- 移动中的感知稳定性 — 动态 TF 变换下的物体追踪
- Persistent Object Tracking — 物体身份在机器人移动时保持稳定
  - 例: `track_002 = blue_cup`，机器人移动后 `track_002` 仍为 `blue_cup`
- Lidar + camera fusion for larger workspace coverage
- World model supports moving observer (not just static camera)

### Epic 10 — Mobile Agent Runtime

**目标**: Runtime 技能系统扩展支持移动操作。

**任务**:
- `NavigateSkill` — move base to target pose (via Nav2)
- `SearchSkill` — autonomous object search with lidar + camera
- `DockSkill` — docking / undocking workflow
- Mobile manipulation coordination — base + arm joint task execution
- Runtime state machine extends with MOBILE_NAVIGATING, MOBILE_SEARCHING states
- RobotOps records navigation events alongside manipulator events
- Safety: base + arm collision zones, speed limits during manipulation

---

## Future Phase C — Agentic Mobile Manipulation ⬜ PLANNED

| Phase | 状态 | 目标 |
|-------|------|------|
| C.1 | PLANNED | Agentic Task Planning — multi-step autonomous task decomposition |
| C.2 | PLANNED | Multi-robot Coordination — fleet management |
| C.3 | PLANNED | VLA Integration — Vision-Language-Action model for natural language task execution |
| C.4 | PLANNED | Full Autonomy — unattended operation in dynamic environments |

---

## 路线图重排序说明

| Sprint | 原位置 | 新位置 | 调整理由 |
|--------|--------|--------|----------|
| Retry / Recovery | 7 | 11 | 重试/恢复应基于 RobotOps、Teleop、Data Logger 积累的真实失败数据设计，而非提前推测 |
| RobotOps | 8 | 7A/7B | 持久化 + 可观测性是一切后续工作的基础。没有日志和 API，后续每个 Sprint 的调试都只能靠终端 print |
| Teleop | 9 | 8A/8B | 遥控输入应在数据采集之前，因为人工演示是 VLA 训练数据的重要来源 |
| Data Logger | 10 (部分) | 9 | VLA 需要轨迹、图像、动作、状态、验证结果的完整记录 |
| VLA Readiness | 10 (部分) | 10 | VLA bridge 应在 Retry 之前，因为 VLA 和真实数据可能改变"恢复"的定义 |
| Retry | 7 | 11 | 此时已拥有: RobotOps 日志 → 知道失败模式; Teleop → 人类可介入; Data → 可分析分布 |

---

## 延后项

| 项 | 原因 |
|----|------|
| 语音系统重构 (P5) | 语音不是当前瓶颈；Keyboard/Web 输入优先 |
| 高级 IK 替换 | 继续使用 JetArm 原厂 `.so` IK；优先级低于验证/日志/安全 |
| social_robot 清理 | 不影响 Runtime 核心功能 |
| 旧 executor 直连 bus_servo 修复 | 新 Runtime 通过 /servo_controller 路径 |

---

## 不变规则

- **禁止修改**: `servo_controller`, `kinematics`(IK .so), `ros_robot_controller`, STM32 固件
- **安全默认**: dry_run=true, require_confirm=true, enable_real_servo=false
- **测试**: 每次变更后运行单元测试
- **预览确认**: 任何真实硬件运动前必须经过 Preview/Confirm
