# JetArm Robot Runtime v0.1 — 开发路线图

> 合并原始 Roadmap + 当前实施进度 + Sprint 5→10 计划
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

## Sprint 5: Stable World Model / Perception Fusion 🔶 IN_PROGRESS

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

### Sprint 5.5 — StableObjectTracker Input Switch ⬜ NEXT

**目标**: StableObjectTracker 输入从 `/world_model/roi_objects` 切换到 `/world_model/perception_objects`。

---

### Sprint 5.6 — Grounding Switch to Stable ⬜ PLANNED

**目标**: grounding_node 从 `/world_model/roi_objects` 切换到 `/world_model/stable_objects`。

---

### Sprint 5 验收标准 (更新)

- [x] ROI audit 完成，数据量化
- [x] StableObjectTracker 发布 `/world_model/stable_objects`
- [x] PerceptionFusionNode 发布 `/world_model/perception_objects`
- [x] YOLO 语义 + ROI 位姿/颜色融合规则确立
- [ ] StableObjectTracker 输入切换到 perception_objects
- [ ] grounding_node 切换到 stable_objects
- [ ] 不修改 ROI 检测逻辑、YOLO 检测逻辑

---

## Sprint 6: Verification Runtime ⬜ PLANNED

**目标**: 不只知道"执行了"，更要知道"成功了"。

> **前置条件**: Sprint 5 Stable World Model 完成 (`/world_model/stable_objects`)。

---

### Sprint 6.1 — Verification Architecture Plan

**目标**: 定义 VerificationResult 数据结构与验证阶段。

**VerificationResult 数据结构**:
```python
@dataclass
class VerificationResult:
    task_id: str
    stage: str              # "pre_pick" | "post_pick" | "post_place"
    success: bool
    confidence: float       # 0.0 ~ 1.0
    reason: str
    evidence: dict
    timestamp: float
```

**验证阶段**: precheck_target_exists / after_pick_target_removed / after_place_target_present

**新建文件**: `src/sketch_runtime/sketch_runtime/verification_result.py`

---

### Sprint 6.2 — World Model Reader

**功能**: 订阅 `/world_model/stable_objects` → 解析 JSON → 按 class/color/id 查询 → 距离阈值判断

**新建文件**: `src/sketch_runtime/sketch_runtime/world_model_reader.py`

---

### Sprint 6.3 — Precheck Verification

**流程**: PickSkill 执行前 → verify target exists in stable_objects

---

### Sprint 6.4 — After Pick Verification

**流程**: lift 完成后 → 等待 0.5s → check target disappeared from source area

**保守策略**: 第一版仅报告 evidence，不自动阻断

---

### Sprint 6.5 — Verification Logs

**Topic**: `/runtime/verification` (String JSON)

---

### Sprint 6.6 — Retry Plan (Design Only)

**方案**: offset_retry / ask_user_confirm，Sprint 7 实现

---

## Sprint 7: Retry / Recovery ⬜ PLANNED

**目标**: 验证失败后自动或手动重试。

---

## Sprint 8: RobotOps / Monitoring ⬜ PLANNED

**目标**: 每个任务可追踪、可复盘。

**任务**:
- `task_id` 索引日志
- 完整执行链路记录: user_command → parsed → grounded → skill → IK → servo → result
- Event timeline (state transitions)
- SQLite 持久化
- 最小 Dashboard API (`GET /api/tasks`, `GET /api/task/:id`)
- 可选: 图像快照

---

## Sprint 9: Teleop / Safety Control ⬜ PLANNED

**目标**: 遥控输入 + 控制权仲裁。

**任务**:
- 控制状态: `AUTO` / `MANUAL` / `PAUSED` / `EMERGENCY_STOP`
- 键盘/手柄/Web 遥控输入
- `control_arbiter_node` — 所有权仲裁

---

## Sprint 10: Data Logger / VLA Readiness ⬜ PLANNED

**目标**: 为 VLA 数据集构建准备数据采集管道。

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
