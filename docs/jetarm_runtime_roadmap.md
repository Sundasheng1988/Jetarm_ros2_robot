# JetArm Robot Runtime v0.1 — 开发路线图

> 合并原始 Roadmap + 当前实施进度 + Sprint 4.4→9 计划
> 状态标记: ✅ COMPLETED | 🔶 IN_PROGRESS | ⬜ PLANNED | ⏸ DEFERRED
> 最后更新：2026-05-20
> **🎯 里程碑达成: 首次 Runtime 驱动的真实硬件 pick 执行验证通过**

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

## Sprint 5: Verification Runtime 🔶 IN_PROGRESS

**目标**: 不只知道"执行了"，更要知道"成功了"。

---

### Sprint 5.1 — Verification Architecture Plan ⬜ PLANNED

**目标**: 定义 VerificationResult 数据结构与验证阶段。

**VerificationResult 数据结构**:
```python
@dataclass
class VerificationResult:
    task_id: str
    stage: str              # "pre_pick" | "post_pick" | "post_place"
    success: bool
    confidence: float       # 0.0 ~ 1.0
    reason: str             # "target_found" | "target_missing" | "target_still_at_source" | ...
    evidence: dict          # {object_count_before, object_count_after, distance_m, ...}
    timestamp: float
```

**验证阶段**:
| 阶段 | 时机 | 检查内容 |
|------|------|----------|
| `precheck_target_exists` | Skill.execute 之前 | 目标物体是否在世界模型中存在 |
| `after_pick_target_removed` | 抓取 + lift 完成后 | 目标是否从原位置消失 |
| `after_place_target_present` | 放置完成后 | 目标是否出现在目标区域 |

**失败原因枚举**:
```python
class VerificationReason:
    TARGET_NOT_FOUND = "target_not_found"
    TARGET_STILL_AT_SOURCE = "target_still_at_source"
    TARGET_NOT_AT_DESTINATION = "target_not_at_destination"
    VISION_UNAVAILABLE = "vision_unavailable"
    VERIFICATION_TIMEOUT = "verification_timeout"
    VISION_CONFIDENCE_TOO_LOW = "vision_confidence_too_low"
    OK = "verification_passed"
```

**新建文件**: `src/sketch_runtime/sketch_runtime/verification_result.py`

**验收标准**:
- `VerificationResult` dataclass 可独立 import
- 与 `ExecutionResult` 不冲突
- 可与 `TaskContext` 关联

---

### Sprint 5.2 — World Model Reader ⬜ PLANNED

**目标**: 从 ROS2 topic 读取并解析世界模型 JSON。

**功能**:
1. 订阅 `/world_model/objects` 或 `/world_model/roi_objects`（通过参数）
2. 解析 JSON → `[TargetObject]` 列表
3. 查询接口: `find_by_class(class_name)` / `find_by_color(color)` / `find_by_id(object_id)`
4. 距离阈值: 判断物体是否在源位置附近（例如 ±0.05m 内认为"仍在原位"）

**新建文件**: `src/sketch_runtime/sketch_runtime/world_model_reader.py`

**ROS2 接口**:
- 订阅: `/world_model/objects` (可配置)
- 不发布 — 纯 reader

**验收标准**:
- 能从 JSON 解析出 `TargetObject` 列表
- 能按 class/color/id 查询
- 能判断物体是否仍在源位置（距离阈值）

---

### Sprint 5.3 — Precheck Verification ⬜ PLANNED

**目标**: 执行前确认目标存在。

**流程**:
```
PickSkill.precheck() 或 SkillManager 调用时
  → VerificationNode.precheck_target_exists(ctx)
    → WorldModelReader.query(class=ctx.target_object.class_name, color=...)
    → 目标存在? → VerificationResult(success=True, reason="target_found")
    → 目标不存在? → VerificationResult(success=False, reason="target_not_found")
```

**集成点**: 插入到 `real_grounded_runtime_node._execute_skill()` 的 precheck 阶段之后、execute 之前。

**参数**: `require_precheck_verification` (默认 `false` — 先默认关闭，逐步开启)

**验收标准**:
- 开启后，目标不存在时返回 `ExecutionResult(success=False, reason="target_not_found")`
- 目标存在时正常执行

---

### Sprint 5.4 — After Pick Verification ⬜ PLANNED

**目标**: 抓取后确认目标从原位置消失。

**流程**:
```
PickSkill 完成 lift 后
  → VerificationNode.after_pick(ctx)
    → 等待 0.5s (视觉刷新)
    → WorldModelReader.query(class=..., space=source_area)
    → 目标消失? → success=True, reason="target_removed_from_source"
    → 目标仍在? → success=False, reason="target_still_at_source", confidence=0.5
    → 视觉不可用? → success=None (不确定), reason="vision_unavailable"
```

**保守策略**:
- 第一版仅报告 evidence，不自动判定任务失败
- 记录 `object_count_before`, `object_count_after`, `distance_remaining`
- 在 `/runtime/verification` 发布结果但不阻断执行

**验收标准**:
- 抓取后发布 `/runtime/verification` 
- 真实硬件抓取成功后，日志显示 `target_removed_from_source`
- 视觉不可用时返回 `vision_unavailable` 而非崩溃

---

### Sprint 5.5 — Verification Logs ⬜ PLANNED

**目标**: 结构化发布验证结果。

**ROS2 Topic**: `/runtime/verification` (`String` JSON)

**消息格式**:
```json
{
  "task_id": "task_a1b2c3_1715900000",
  "stage": "after_pick",
  "success": true,
  "confidence": 0.85,
  "reason": "target_removed_from_source",
  "evidence": {
    "object_count_before": 3,
    "object_count_after": 2,
    "missing_class": "cube",
    "missing_color": "red",
    "source_distance_m": 0.0,
    "vision_age_ms": 320
  },
  "timestamp": 1715900010.0
}
```

**发布节点**: `real_grounded_runtime_node` 或独立 `verification_node`

**验收标准**:
- `/runtime/verification` topic 可用
- 至少 precheck 和 post_pick 两个阶段的消息格式一致

---

### Sprint 5.6 — Retry Plan ⬜ PLANNED (Planning Only)

**目标**: 设计 retry 机制但不在 Sprint 5 实现。

**设计方案**:
1. Verification 返回 `success=False` → Executor 决策 retry
2. Retry 策略:
   - `offset_retry`: 轻微偏移目标位姿重试 (最多 3 次)
   - `ask_user`: 发布 `/runtime/confirm_retry` 等待用户确认
3. `TaskContext.retry_count` 追踪重试次数
4. `max_retries` 默认为 2

**Sprint 5 不实现 retry** — 仅记录 evidence，供后续决策。

**验收标准**:
- `retry_count` / `max_retries` 字段已在 TaskContext 中
- 设计方案文档化供 Sprint 6 实现

---

### Sprint 5 总体验收标准

- [ ] VerificationResult 数据结构定义完成
- [ ] WorldModelReader 可读取 /world_model/objects 并解析
- [ ] precheck 可阻断"目标不存在"的任务
- [ ] after_pick 可发布 evidence（不阻断）
- [ ] `/runtime/verification` topic 发布标准化 JSON
- [ ] verification 默认关闭，通过参数开启
- [ ] 不影响现有 real pick flow
- [ ] 0 行修改到 servo_controller / kinematics / ros_robot_controller

---

## Sprint 6: RobotOps / Monitoring ⬜ PLANNED

**目标**: 每个任务可追踪、可复盘。

**任务**:
- `task_id` 索引日志
- 完整执行链路记录: user_command → parsed → grounded → skill → IK → servo → result
- Event timeline (state transitions)
- SQLite 持久化
- 最小 Dashboard API (`GET /api/tasks`, `GET /api/task/:id`)
- 可选: 图像快照

---

## Sprint 7: Teleop / Safety Control ⬜ PLANNED

**目标**: 遥控输入 + 控制权仲裁。

**任务**:
- 控制状态: `AUTO` / `MANUAL` / `PAUSED` / `EMERGENCY_STOP`
- 键盘/手柄/Web 遥控输入
- `control_arbiter_node` — 所有权仲裁
- Manual 模式下自动任务被阻塞
- 每次控制权切换记录日志

---

## Sprint 8: Skill Library Expansion ⬜ PLANNED

**目标**: 从单一 PickSkill 扩展到完整 Skill 库。

**计划 Skill**:
- `MoveToSkill` — 移动到指定位姿
- `OpenGripperSkill` — 张开夹爪
- `CloseGripperSkill` — 闭合夹爪
- `PlaceSkill` — 放置 (hover→approach→open→lift)
- `HomeSkill` — 回到初始位姿
- `ScanSkill` — 环境扫描 (未来)

---

## Sprint 9: Data Logger / VLA Readiness ⬜ PLANNED

**目标**: 为 VLA 数据集构建准备数据采集管道。

**任务**:
- 记录: 视觉帧 → target_pose → IK pulse → servo trajectory → 操作员干预
- 任务成功/失败标签
- 轨迹回放
- VLA 数据集格式导出
- 数据版本控制

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
- **测试**: 每次变更后运行 65 个单元测试
- **预览确认**: 任何真实硬件运动前必须经过 Preview/Confirm
