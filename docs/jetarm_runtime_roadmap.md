# JetArm Robot Runtime v0.1 — 开发路线图

> 合并原始 Roadmap + 当前实施进度 + Sprint 4.4→9 计划
> 状态标记: ✅ COMPLETED | 🔶 IN_PROGRESS | ⬜ PLANNED | ⏸ DEFERRED

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

## Sprint 4.4: Servo Message Adapter 🔶 IN_PROGRESS

**目标**: 修复 servo 消息格式，使 `enable_real_servo=true` 能正确发布。

**未解决问题**: 发布到 `/servo_controller` 时报错 `"The 'position' field must be of type 'float'"`。

**任务**:
1. 检查旧代码中工作 servo 发布格式 (`executor_node.py`, `grasp.py`, `actions.py`)
2. 确认正确的 servo 消息类型和字段 (`servo_controller_msgs` vs `ros_robot_controller_msgs`)
3. 映射 arm 脉冲到正确 servo ID (1-5 for arm, 10 for gripper)
4. 修复消息字段类型
5. 发布失败应返回 task failure

---

## Sprint 4.5: Hover-Only Safety Test ⬜ PLANNED

**目标**: 最小实物运动测试 — 仅移动到物体上方悬停。

**任务**:
1. 启用 `enable_real_servo=true`
2. 仅执行 hover_source 步骤
3. 不下降、不抓取、不移动夹爪
4. 验证机械臂到达正确悬停位置
5. 保持 Preview/Confirm 启用

---

## Sprint 4.6: Gripper Standalone Test ⬜ PLANNED

**目标**: 夹爪独立测试 — 仅控制 ID10。

**任务**:
1. 发送仅含 servo ID 10 的命令
2. 验证张开/闭合脉冲方向和范围
3. 确认 200=开放, 700=闭合 或当前项目约定
4. 不移动 arm 关节

---

## Sprint 4.7: Full Pick Minimal Execution ⬜ PLANNED

**目标**: 完成一次最小完整抓取 — 不含放置。

**任务**:
1. 依次执行: hover → open gripper → approach → close gripper → lift
2. 每次运动间等待静止确认
3. Preview/Confirm 每个子步骤或整体流程
4. 记录完整执行轨迹

---

## Sprint 5: Verification Runtime ⬜ PLANNED

**目标**: 不只知道"执行了"，更要知道"成功了"。

**任务**:
- 抓取前: 目标是否存在 (vision query)
- 抓取后: 目标是否从原位置消失
- 夹爪状态: ID10 是否闭合到预期位置
- 放置后: 目标是否出现在目标区域
- 超时检测
- `VerificationResult`: task_id, success, confidence, reason, evidence

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
