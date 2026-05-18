# JetArm Robot Runtime v0.1 开发 Roadmap

## 0. 当前判断

当前系统的问题不再是单点技术问题，例如“IK 不够准”“语音不够流畅”“某个 ROS2 节点写得不够好”。

更核心的问题是：

```text
指令 → Grounding → Executor → Skill → 执行 → 验证 → 记录
```

这条闭环还没有形成稳定的 Runtime。

因此，当前阶段的目标不应定义为：

```text
把机械臂抓准
```

而应定义为：

```text
构建一个可控制、可监控、可回放、可验证、可远程接管的机器人执行系统。
```

这条路线是合理的，并且应作为后续开发主线。

---

## 1. 总体开发路线

推荐路线如下：

```text
P0：统一任务执行框架
P1：遥控输入 + 安全接管
P2：数据写入、监控、回传、RobotOps
P3：任务成功验证 / 失败检测
P4：IK / 抓取精度优化
P5：语音系统重构
P6：高级 IK / VLA / 数据集能力
```

其中，P0 到 P3 是 Runtime 基础能力，优先级高于单纯优化 IK 或语音体验。

---

## P0：建立统一任务执行框架

### 优先级

最高优先级。

### 目标

把当前写死逻辑改造成可扩展的任务执行链路。

当前问题是：

```text
用户指令只能触发固定动作
Grounding 与 Executor 写死
不同任务之间缺少统一状态
执行结果无法标准化返回
```

目标链路应变成：

```text
用户指令
↓
结构化任务
↓
Grounding 找到目标
↓
Executor 选择 Skill
↓
Skill 执行动作
↓
返回 success / failed / reason
```

### 需要重构的模块

```text
grounding
executor
skill 调度
任务状态管理
```

### 核心设计

建议引入统一任务对象：

```text
TaskContext
- task_id
- user_command
- parsed_command
- target_object
- selected_skill
- execution_state
- result
- error_reason
- timestamp
```

建议引入统一目标对象：

```text
TargetObject
- name
- class_name
- color
- confidence
- center_x
- center_y
- center_z
- world_x
- world_y
- world_z
- source
- timestamp
```

### 验收标准

完成后，系统至少能做到：

```text
输入：抓取红色杯子
↓
Parser 输出结构化命令
↓
Grounding 输出 TargetObject
↓
Executor 选择 pick_skill
↓
pick_skill 调用原厂 IK + servo_controller
↓
返回 success / failed / reason
```

---

## P1：遥控输入 + 安全接管

### 优先级

第二优先级。

### 原因

遥控输入不是附加功能，而是机器人系统的基础能力。

它同时承担：

```text
调试工具
安全接管工具
数据采集工具
部署运维工具
```

### 需要支持的输入

```text
键盘遥控
手柄遥控
Web 控制
远程控制指令
```

### 必须增加控制权仲裁

系统必须明确当前机械臂由谁控制。

建议定义状态：

```text
AUTO
MANUAL
PAUSED
EMERGENCY_STOP
```

### 为什么必须做仲裁

否则会出现：

```text
自动任务正在控制机械臂
遥控节点同时发舵机指令
视觉跟踪节点也在发控制指令
```

最终导致动作冲突，甚至机械臂异常运动。

### 验收标准

```text
1. 手动模式下，自动任务不能控制机械臂
2. 自动模式下，遥控可随时接管
3. Emergency Stop 可以立即阻断动作输出
4. 每次控制权切换都有日志记录
```

---

## P2：数据写入、监控、回传、RobotOps

### 优先级

第三优先级，但需要尽早做最小版本。

### 目标

让每一次任务都可追踪、可复盘、可调试。

### 需要记录的数据

```text
task_id
user_command
parsed_command
selected_skill
vision_result
target_pose
ik_result
servo_command
execution_status
error_reason
image_snapshot
timestamp
```

### 推荐模块结构

```text
robot_ops/
├── task_logger_node.py
├── state_monitor_node.py
├── event_recorder.py
├── dashboard_api.py
└── logs/
```

### 价值

RobotOps 不是为了展示，而是为了解决真实问题：

```text
为什么没抓到？
为什么 IK 算错？
为什么视觉识别错？
为什么放错位置？
什么时候开始失败？
失败前系统状态是什么？
```

### 验收标准

```text
1. 每个任务都有 task_id
2. 每个任务都有完整执行日志
3. 失败任务能看到 error_reason
4. 至少保存一次任务相关图像快照
5. 能从日志复盘一次抓取过程
```

---

## P3：任务成功验证 / 失败检测

### 优先级

第四优先级，建议与 P2 并行。

### 原因

即使 IK 不完美，系统也必须知道任务是否成功。

机器人系统不能只执行动作，还必须判断：

```text
有没有抓到？
有没有放对？
有没有撞到？
有没有超时？
```

### 最小验证逻辑

抓取任务可以先按以下方式验证：

```text
抓取前：目标是否存在
抓取后：目标是否从原位置消失
放置后：目标是否出现在目标区域
夹爪状态：ID10 是否闭合到合理位置
执行状态：servo 是否到位
任务超时：是否超过预设时间
```

### 验证结果格式

```text
VerificationResult
- task_id
- success
- confidence
- reason
- evidence
- timestamp
```

### 验收标准

```text
1. 抓取后能判断目标是否消失
2. 放置后能判断目标是否出现在目标区域
3. 执行失败能返回明确原因
4. Executor 能根据失败原因决定是否 retry
```

---

## P4：IK / 抓取精度优化

### 优先级

第五优先级。

### 当前策略

当前不建议立即重写 IK。

正确策略是：

```text
继续调用 JetArm 原厂封装的 .so IK 模块
```

重点不是先替换 IK，而是先排查抓不准的真正来源。

### 重点优化方向

```text
视觉坐标 → base 坐标
末端夹爪 gripper_tip 偏移补偿
抓取高度补偿
抓取姿态 pitch 调整
目标中心点选择误差
深度值滤波
相机标定误差验证
```

### 重要判断

机械臂抓不准，不一定是 IK 本身错误。

常见原因包括：

```text
相机标定误差
深度图噪声
目标中心点不等于可抓取点
gripper_tip 偏移没有补偿
抓取高度设置不合理
末端姿态 pitch 不适合当前目标
```

### 验收标准

```text
1. 同一目标重复抓取误差可测量
2. 每次抓取都记录 target_pose 和 ik_result
3. 能区分视觉误差、IK误差、执行误差
4. 抓取成功率有可量化统计
```

---

## P5：语音系统重构

### 优先级

第六优先级。

### 原因

语音体验差，不等于机器人执行系统差。

当前阶段可以先用：

```text
键盘输入
Web 输入
遥控输入
```

稳定调试机器人执行闭环。

语音系统后面再优化。

### 语音系统定位

`llm_voice_agent` 不应该承担机器人主脑职责。

它应该只是：

```text
语音入口
```

真正任务调度应由：

```text
executor / agent_runtime
```

负责。

### 后续重构内容

```text
ASR
wake word
LLM parser
TTS
interrupt
dialog state
语音播报节奏
任务状态反馈
```

### 验收标准

```text
1. 语音输入只负责生成用户指令
2. 任务调度不依赖 voice_agent 内部逻辑
3. TTS 支持打断
4. 机器人执行状态可以自然播报
```

---

## P6：高级 IK / VLA / 数据集能力

### 优先级

最后阶段。

### 前置条件

只有当以下能力稳定后，才适合进入 VLA 或高级学习阶段：

```text
任务执行闭环稳定
遥控输入稳定
数据记录稳定
任务验证稳定
基础抓取成功率可测量
```

### 后续方向

```text
遥控数据采集
轨迹记录
图像 + 动作对齐
任务成功标签
VLA 数据集格式
OpenVLA / π0 / GR00T 接入实验
```

### 关键判断

VLA 不是替代当前 Runtime。

VLA 应该接入：

```text
Robot Runtime
```

作为高级策略模块，而不是直接绕过 ROS2、IK、Servo、安全仲裁和 RobotOps。

---

## 2. 第一个 Sprint 建议

### Sprint 目标

建立 Robot Runtime v0.1 的最小闭环。

### Sprint 范围

只做四件事：

```text
1. executor_node 不再写死动作
2. grounding_node 输出统一 TargetObject
3. 新建 skill_manager_node
4. 每次任务生成 task_id 并记录执行结果
```

### 最小闭环

```text
输入：抓取红色杯子
↓
parser 输出 command
↓
grounding 找到目标
↓
executor 选择 pick_skill
↓
pick_skill 调用原厂 IK + servo_controller
↓
verify 判断是否成功
↓
logger 记录全过程
```

### 不做的事情

第一个 Sprint 不建议做：

```text
不重写 IK
不重写语音系统
不接 VLA
不做复杂 Web Dashboard
不大规模重构原厂程序
不修改 STM32 / 底层舵机控制
```

### 验收标准

```text
1. 可以从一条用户指令生成一个 task_id
2. 可以根据 parsed_command 选择一个 skill
3. skill 执行后返回 success / failed / reason
4. 日志中能看到完整任务过程
5. 执行失败不会让系统失控
```

---

## 3. 推荐分支策略

当前稳定分支：

```text
debug/llm_voice_agent
```

建议冻结为旧架构稳定基线。

新建开发分支：

```bash
git checkout -b feature/robot_runtime_v1
git push -u origin feature/robot_runtime_v1
```

后续建议分支：

```text
feature/grounding_runtime
feature/skill_manager
feature/teleop_runtime
feature/robotops_logger
feature/task_verification
feature/grasp_precision
feature/voice_runtime
```

---

## 4. 最终目标

该 Roadmap 的最终目标是把当前系统从：

```text
ROS2 节点拼接系统
```

升级为：

```text
Robot Runtime v0.1
```

进一步升级为：

```text
Embodied Agent Runtime
```

系统最终应具备：

```text
可执行
可监控
可验证
可回放
可远程接管
可持续优化
可采集数据
可接入更高级 VLA 模型
```

这条路线是当前阶段最合理的主线。
