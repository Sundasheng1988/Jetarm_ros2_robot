# Runtime Execution-Chain Audit — 2026-07-31

> 调查模式：READ-ONLY。全程未修改任何源码、launch、YAML、systemd、build/install/log；
> 未启动任何 ROS 2 launch；未运行 real_grounded_runtime_node / runtime_test_node；
> 未发布任何 ROS 2 控制消息；未调用真实 IK；未执行 Servo / Hover / Pick；
> 未触碰 Jetson；未执行 git add/commit/push/reset。
>
> 仅运行的“副作用”命令：纯 Python 单元测试（已确认不创建 ROS Node / Publisher / Client / Service）
> 与只读 ROS 2 图查询（`ros2 node list`）。
>
> 证据等级标签：**CONFIRMED**（有代码/命令直接证据）/ **UNCONFIRMED**（推断）/ **UNKNOWN**（未查）/ **NOT IMPLEMENTED**（代码中不存在）。
> 本报告不使用无证据的 SAFE / READY / COMPLETE / NO RISK / ALL TESTS PASS。

---

## 0. 前提偏差声明（必须先读）

任务指令称“之前已经存在 `docs/runtime_execution_chain_audit_20260731.md`，由较弱模型生成，只能作为问题清单和初稿”。
**实际核查结果（CONFIRMED）**：

- `docs/runtime_execution_chain_audit_20260731.md` 在工作区与 `git log --all` 中均**不存在**（`find` / `git log` 无命中）。
- 文档（`runtime_index.md`、dev log）反复引用的历史交付物名是 `runtime_execution_chain_audit_20260730.md`，该文件**同样不存在**。

因此本报告为**全新创建**，不存在可“修订”的较弱模型初稿；任务中“重新核对全部关键结论”的要求已按“从零核对”执行。`runtime_risks.md`（标注 2026-07-29）仍把 R31（重复控制栈）列为 Active P0，与 `2026-07-30_fixed_arm_runtime_reactivation_dev_log.md`“R31 已解决”的结论冲突；本审查以**运行时实测**为准（见第 3、13 节），R31 当前状态为**已关闭**。

---

## 1. Executive Summary

本轮对 JetArm 固定机械臂 Runtime 主执行链做了基于源码 / install / launch / 测试 / 运行图的只读审查。

核心结论：

1. **新 Runtime 的 dry_run 安全门是有效的（CONFIRMED）**。`dry_run=true` 时：IK 返回 mock `[500,500,500,500,500]`、**不创建** IK client、**不创建** `/servo_controller` publisher、**不发布**任何 Servo 消息。真实 Servo 需 `dry_run=false` **且** `enable_real_servo=true` 双条件；真实 IK 需 `dry_run=false` **且** `enable_real_ik=true`。pulse 在发布前钳位 `[0,1000]`。
2. **新 Runtime 不存在直接硬件总线旁路（CONFIRMED）**。`src/sketch_runtime` 全包仅 `runtime_adapter.py` 写 `/servo_controller`（经 controller_manager），**无** `/grasp`、**无** `/ros_robot_controller/bus_servo/set_position` 直写。
3. **当前正式 launch 不启动 legacy 执行器（CONFIRMED）**。legacy `ground_executor_node`（包 `llm_executor`，可执行 `executor_node`）源码存在、console_script 已注册、**可手动启动**，但**无任何 launch / 脚本引用它**。
4. **当前运行图未发现重复控制栈（CONFIRMED，最小复核）**。`ros2 node list`（2026-07-31 实测）显示 `/ros_robot_controller /controller_manager /servo_manager /grasp /kinematics` 各 ×1，与 dev log 一致。
5. **无活动 P0（CONFIRMED）**：在“当前正式 launch + 默认安全参数”配置下，dry_run / enable_real_servo 门控、确认门、并发互斥均不失效，legacy 也不会自动并行。
6. **进入安全 Dry-run 为 CONDITIONAL GO**：代码与门控层面满足，但**install 中的 `ground_runtime_bringup.launch.py` 过期**（不启动 verification sidecar），且**执行链 async 单元测试因缺 pytest-asyncio 被整体跳过**。这两项需先处理（均不涉及硬件）。

最大隐患（latent P0）：legacy `ground_executor_node` 的 `dry_run` 默认 `False`、`servo_topic` 默认 `/ros_robot_controller/bus_servo/set_position`（绕过 controller_manager 限位）。一旦被手动 `ros2 run llm_executor executor_node` 启动并与新 Runtime 并存，将直接向总线写脉冲。当前它未被任何 launch 启动，但代码层无任何机制阻止手动启动。

---

## 2. Audit Method and Evidence Rules

- **事实来源优先级**：当前源码 > install > launch > YAML > 当前 ROS 2 图 > 自动化测试 > 文档 > 历史日志。
- **实际对比证据**：第 4 节 src↔install 一致性均给出 `sha256sum` / `diff` 命令与结果，无“无漂移”的断言性结论。
- **只读运行证据**：`ros2 node list`（2026-07-31，`ROS_DOMAIN_ID=23`，PC 侧）。
- **测试证据**：`python3 -m pytest src/sketch_runtime/test src/robotops/test -rs --tb=short`，输出存档于 `/tmp/runtime_audit_pytest_20260731.log`。
- **未做**：源码/launch/YAML 修改、构建、服务启停、新 launch、ROS 2 消息发布、机械臂/底盘动作、IK/Servo/Nav2/AMCL 调试、Jetson 侧任何写操作、C++ 源码审计。
- **主机归属规则**：`ros2 node list` 是 DDS Domain 综合图，不能单独判定节点主机；本报告对主机归属结合 dev log 的进程/systemd 证据与当前图联合判定。

---

## 3. Repository / Branch / Install Baseline

| 项 | 值 | 证据 |
|----|----|------|
| 工作区 | `/home/sundasheng/ros2_ws` | `pwd` |
| 当前分支 | `feature/sketch_runtime_sprint3` | `git branch --show-current` |
| HEAD | `fc12932` | `git log -5` |
| 远端 | `origin/feature/sketch_runtime_sprint3` = `707b969` | `git rev-list --left-right --count` |
| 领先远端 | **4 commits**（均为 docs：`fc12932 7cef4eb 3f56071 8fc030c`） | `git log` |
| Runtime 包工作树 | **CLEAN**（sketch_runtime/grounding/robotops/app/vision_yolo 均无未提交改动） | `git status --short -- src/...` |

**为什么本地领先远端若干提交（CONFIRMED）**：领先的 4 个提交全部是文档提交（dev-log、runtime status、directional validation、localization debug）。Runtime 源码本身在 HEAD 已提交且工作树干净。**含义**：本次审计的源码 == HEAD == 可构建版本；这些 docs 提交尚未 push（非本审查范围，未执行 git 操作）。

**Runtime 包基线（CONFIRMED，`colcon list` + `ros2 pkg prefix`）**：

| 包 | 类型 | Source path | Install prefix | Expected host | Observed host（当前） |
|----|------|-------------|----------------|---------------|------------------------|
| sketch_runtime | ament_python | `src/sketch_runtime` | `install/sketch_runtime` | PC | **未运行**（PC） |
| grounding | ament_python | `src/grounding` | `install/grounding` | PC | **未运行**（PC） |
| robotops | ament_python | `src/robotops` | `install/robotops` | PC | **未运行**（PC） |
| app | ament_python | `src/app` | `install/app` | PC | 未运行 |
| vision_yolo | ament_python | `src/vision_yolo` | `install/vision_yolo` | PC | 未运行 |
| llm_executor（legacy） | ament_python | `src/llm_executor` | `install/llm_executor` | PC（legacy） | **未运行** |

**当前 ROS 2 图（CONFIRMED，2026-07-31 PC 侧 `ros2 node list`，12 节点）**：
```
/controller_manager  /depth_cam/camera_container  /depth_cam/depth_cam
/grasp  /kinematics  /launch_ros_1879  /rosapi  /rosapi_params
/rosbridge_websocket  /ros_robot_controller  /servo_manager  /web_video_server
```
- 上述全部为 **Jetson 侧硬件 / 基础设施节点**（与 2026-07-30 dev log 第 10 节基线一致）。
- 控制栈核心节点 `ros_robot_controller / controller_manager / servo_manager / grasp / kinematics` **各 ×1** → R31（重复控制栈）当前**已关闭**（最小复核，CONFIRMED）。
- `/buzzer_controller` 当前**未出现**（dev log 第 24 节残留 buzzer 重复已不在此图中；UNCONFIRMED 是否彻底修复，仅确认当前不可见）。
- **PC 侧 Runtime / grounding / legacy / robotops 节点全部未运行**（CONFIRMED）→ 当前为安全审查态。

---

## 4. Source vs Install Comparison

方法：对 Python 模块用 `sha256sum` 比对 `src/<pkg>/<mod>` 与 `install/<pkg>/lib/python3.10/site-packages/<mod>`；launch 用 `diff -u`。

### 4.1 sketch_runtime（CONFIRMED）

| 文件 | src↔install | 证据 |
|------|-------------|------|
| 14 个 Python 模块（real_grounded_runtime_node / runtime_adapter / action_executor / actions / base_action / base_skill / skill_registry / task_context / runtime_task_builder / target_object / execution_result / verification_result / verification_result_node / skills/pick_skill） | **全部 SAME** | `sha256sum` 逐文件相等 |
| `launch/runtime_test.launch.py` | **SAME** | `sha256sum` 相等 |
| `launch/ground_runtime_bringup.launch.py` | **DIFF** | `diff -u` rc=1 |

**⚠ install 中的 `ground_runtime_bringup.launch.py` 过期（CONFIRMED，P1）**：
- src 版 mtime `2026-06-07`，install 版 mtime `2026-05-19`（`install/sketch_runtime/share/.../package.xml` mtime `2026-05-18`）。
- `diff` 显示 install 版**缺少 verification sidecar 块**：
  ```diff
  -        # ── verification sidecar node ──
  -        Node( package="sketch_runtime",
  -              executable="verification_result_node", ... )
  ```
- **运行影响**：`ros2 launch sketch_runtime ground_runtime_bringup.launch.py` 解析的是 **install/share** 版（过期）→ **verification_result_node 不会被启动**。Python 模块虽一致，但 launch 数据文件未随 src 更新而重新构建。
- **含义**：在重新 `colcon build --packages-select sketch_runtime` 之前，从 install 启动该 launch 不会拉起验证 sidecar。

### 4.2 grounding / robotops（CONFIRMED）

| 文件 | src↔install | 证据 |
|------|-------------|------|
| `grounding/launch/ground_bringup.launch.py` | SAME | `sha256sum` |
| `robotops/launch/robotops_recorder.launch.py` | SAME | `sha256sum` |
| `grounding/grounding/grounding_node.py` | SAME | `sha256sum` |
| `robotops/robotops/robotops_recorder_node.py` | SAME | `sha256sum` |

### 4.3 消息包在 PC 的可用性（CONFIRMED）

`ros2 pkg prefix`：`kinematics_msgs`、`servo_controller_msgs`、`ros_robot_controller_msgs` **三个消息包均在 PC install 中存在**。
→ `runtime_adapter.py` 的 `try: from kinematics_msgs.srv ...` / `from servo_controller_msgs.msg ...` 在 PC 运行时 `_HAS_IK_SRV` / `_HAS_SERVO_MSGS` = **True**。**含义**：真实 IK / Servo 路径在 PC 上是“连通”的，dry_run / enable 门控是唯一防线（门控有效性见第 8 节，CONFIRMED 有效）。

### 4.4 Python 测试导入路径（CONFIRMED）

测试运行前 `source install/setup.bash`，`sketch_runtime` / `robotops` 解析自 **install** 的 `site-packages`。由于 4.1 已确认 14 模块 src↔install 字节一致，测试结果对 src 与 install 等价有效。

---

## 5. Current Runtime Execution Chain

逐层追踪（每层附 file:class:func + 入口 + IO + 部署主机 + 当前 launch 可达性）。除非标注，均为 **CONFIRMED**。

```
/parsed_command (std_msgs/String, JSON)
  └─[1] grounding_node : GroundingNode.on_cmd            src/grounding/grounding/grounding_node.py:177
        sub: /parsed_command ; sub: /world_model/stable_objects (param)
        pub: /grounded_goal            (line 90, 241)   ← legacy 用
        pub: /grounded_task_context    (line 108, 265)  ← 新 Runtime 用（仅 publish_runtime=true）
        【注】/grounded_task_context 不含 task_id（见第 10 节）
  → /grounded_task_context
  └─[2] real_grounded_runtime_node : RealGroundedRuntimeNode._on_grounded_task
        src/sketch_runtime/sketch_runtime/real_grounded_runtime_node.py:163
        console_script: real_grounded_runtime_node (setup.py:26)
        sub: /grounded_task_context, /runtime/confirm, /runtime/verification_result
        pub: /runtime/state, /runtime/log, /runtime/execution_result, /runtime/preview, /executor/done
  └─[3] TaskBuilder.from_parsed_command / from_grounded_goal  runtime_task_builder.py:22,32
        → TaskContext（task_id 在 TaskContext.__post_init__ 新生，task_context.py:57-65）
  └─[4] SkillManager.select(ctx) → INTENT_MAP        skill_registry.py:48
        intent→skill：pick/hold/grasp→pick_skill（已注册）；place/pour/move/home→未注册
  └─[5] SkillRegistry.instantiate → PickSkill         skill_registry.py:56 ; skills/pick_skill.py
  └─[6] RealGroundedRuntimeNode._execute_skill        real_grounded_runtime_node.py:335
        asyncio.new_event_loop().run_until_complete(skill.execute(ctx))  (阻塞 rclpy 主线程)
  └─[7] PickSkill.execute → ActionExecutor.run(actions, adapter, ctx)   pick_skill.py:25 ; action_executor.py:10
        full_pick 5 步：hover_source → gripper_open → approach_pick → gripper_close → lift_after_pick
        hover_only 1 步：hover_source（由 execution_stage 控制，pick_skill.py:49）
  └─[8] MoveAction.execute → adapter.ik_solve → adapter.servo_move   actions.py:26,41
        GripperAction.execute → adapter.gripper_set                   actions.py:61
  └─[9] RuntimeAdapter.ik_solve / servo_move / gripper_set   runtime_adapter.py:58,204,238
        ik_solve：dry_run→mock[500×5]；否则 enable_real_ik→ /kinematics/set_pose_target（临时节点，line 178）
        servo_move / gripper_set：dry_run or not enable_real_servo → return；否则 pub /servo_controller（line 221,236）
  → /servo_controller (servo_controller_msgs/ServosPosition)  +  /kinematics/set_pose_target (kinematics_msgs/SetRobotPose)
  └─[10] (Jetson) /servo_controller → /controller_manager → /servo_manager
         → /ros_robot_controller/bus_servo/set_position → /ros_robot_controller → STM32 → 舵机
```

**当前正式 launch 可达性**：
- `[1] grounding`、`[2] real_grounded_runtime_node`：在 `ground_runtime_bringup.launch.py`（**src 版**）中可达。
- `[9]` verification_result_node：**src 版 launch 可达；install 版 launch 不可达**（第 4.1 节漂移）。
- robotops_recorder：**不在** `ground_runtime_bringup.launch.py`，需单独 `ros2 launch robotops robotops_recorder.launch.py`（第 13 节）。

**未在链中的项（NOT IMPLEMENTED / 不连通）**：
- `RuntimeAdapter.get_joint_state`、`query_vision`：恒返回 None（runtime_adapter.py:271-284），**NOT IMPLEMENTED**。
- `place_skill / pour_skill / move_skill / home_skill`：INTENT_MAP 引用但未注册（第 6 节）。
- PickSkill 的 `target_pose`（放置点）**未被使用**——PickSkill 只用 `source_pose`，是“仅抓取并抬起”，无 place/return_home（pick_skill.py:25-72）。

---

## 6. Runtime / Skill / Action / Adapter Audit

| 组件 | 文件:类 | 状态 | 备注 |
|------|---------|------|------|
| TaskContext / TaskState | task_context.py | CONFIRMED | 14 个状态枚举；task_id 自动生成；max_retries=2（**未被任何逻辑使用**，RETRYING 状态从未进入） |
| TaskBuilder | runtime_task_builder.py | CONFIRMED | from_parsed_command / from_grounded_goal / build_full_task |
| TargetObject | target_object.py | CONFIRMED | to_skill_dict 输出 source_pose（frame/xyz/rpy） |
| SkillRegistry | skill_registry.py:7 | CONFIRMED | 类级 dict；运行时仅 `SkillRegistry.register(PickSkill)`（real_grounded_runtime_node.py:65） |
| SkillManager | skill_registry.py:33 | CONFIRMED | INTENT_MAP（见下） |
| PickSkill | skills/pick_skill.py | CONFIRMED | DEFAULTS 见第 7 节；execution_stage: hover_only / full_pick |
| MoveAction / GripperAction | actions.py | CONFIRMED | MoveAction: IK 失败→step 失败；GripperAction: 恒 success |
| ActionExecutor | action_executor.py | CONFIRMED | 顺序执行；首步失败即终止；**无 per-action 超时、无取消** |
| RuntimeAdapter | runtime_adapter.py | CONFIRMED | 唯一硬件写入点；dry_run/enable 门控；pulse 钳位 |
| BaseSkill.precheck | base_skill.py:14 | CONFIRMED | 仅检查 target_object 非空（不查世界模型） |

**INTENT_MAP 引用但未注册的 Skill（CONFIRMED，P2）**：`place→place_skill`、`pour→pour_skill`、`move→move_skill`、`release→place_skill`、`home→home_skill` 均未注册 → `instantiate` 返回 None → 节点记 `"skill instantiation returned None"` 后 return（real_grounded_runtime_node.py:228-230）。仅 `pick/hold/grasp` 实际可用。

---

## 7. Parameter Three-Layer Matrix

> “三层”= ① 源码 `declare_parameter` 默认 / ② launch `DeclareLaunchArgument` 默认 / ③ launch→Node 实参 + YAML 覆盖。
> 第 6 列“运行时值”：当前节点未运行，**无法运行时确认**，仅能确认默认会生效的值。

### 7.1 `real_grounded_runtime_node` 参数（CONFIRMED，real_grounded_runtime_node.py:23-39）

| 参数 | ① 源码默认 | ② launch 默认 | ③ launch→Node | YAML 覆盖 | 运行时值 |
|------|-----------|--------------|--------------|-----------|----------|
| dry_run | True | true | dry_run(LC) | 无 | 未运行；默认 True |
| run_once | True | true | run_once(LC) | 无 | 未运行；默认 True |
| input_topic | /grounded_task_context | —（硬编码） | "/grounded_task_context" | 无 | 未运行；默认该值 |
| require_confirm | True | true | require_confirm(LC) | 无 | 未运行；默认 True |
| confirm_timeout_sec | 300.0 | 300.0 | confirm_timeout_sec(LC) | 无 | 未运行；默认 300s |
| enable_real_ik | False | false | enable_real_ik(LC) | 无 | 未运行；默认 False |
| enable_real_servo | False | false | enable_real_servo(LC) | 无 | 未运行；默认 False |

launch 透传证据：ground_runtime_bringup.launch.py:93-101（src 版）。**三层一致，默认全安全（CONFIRMED）。**

### 7.2 运动参数（PickSkill.DEFAULTS，CONFIRMED，pick_skill.py:11-20）

**这些参数不是 ROS 参数**，是 PickSkill 类常量，经 `ctx.skill_params.get(key, DEFAULTS)` 读取（pick_skill.py:22-23）。

| 参数 | PickSkill.DEFAULTS | 可由生产节点配置？ | 证据 |
|------|--------------------|-------------------|------|
| hover_height | 0.08 | **否** | real_grounded_runtime_node 从不填充 skill_params |
| approach_z | 0.015 | 否 | 同上 |
| lift_height | 0.08 | 否 | 同上 |
| grip_open_pulse | 200 | 否 | 同上 |
| grip_close_pulse | 700 | 否 | 同上 |
| move_duration_ms | 2000 | 否 | 同上 |
| gripper_id | 10 | 否 | 同上 |
| execution_stage | "full_pick" | 否 | 同上（支持 "hover_only"） |

**关键（CONFIRMED，P1）**：`ctx.skill_params` 在生产链路中**从未被赋值**（`grep skill_params` 仅命中 pick_skill 读取处、task_context 默认 `{}`、与 **runtime_test_node.py:140** 的赋值）。即 `real_grounded_runtime_node` 路径下 PickSkill **恒用 DEFAULTS**，无法通过 launch/YAML 调整。仅测试驱动 `runtime_test_node` 会设置 skill_params（hover/approach/lift，runtime_test_node.py:140-144，且其自身 declare_parameter 这些值，setup 见 7.3）。

> 任务 Phase D 期望这些参数有“源码/launch/YAML/运行时”多层。**实际只有一层（代码常量）**，不可外部配置——这是与设计预期的偏差。

### 7.3 `runtime_test_node` 参数（CONFIRMED，runtime_test_node.py:22-29，仅测试驱动）

test_command="pick red cup to right side"、hover_height=0.08、approach_z=0.015、lift_height=0.08、auto_confirm=True、run_once=True、interval_sec=1.0、dry_run=True。其 `RuntimeAdapter(node=self, dry_run=self.dry_run)` **未传 enable_real_ik/enable_real_servo**（默认 False）→ 恒 dry，不会真实伺服。

### 7.4 `verification_result_node` 参数（CONFIRMED，verification_result_node.py:21-25）

dry_run=True、enable_precheck=True、enable_postcheck=True、distance_threshold=0.1、postcheck_delay_sec=1.0。src 版 launch 传 dry_run/enable_precheck=True/enable_postcheck=True（ground_runtime_bringup.launch.py:110-114）；**install 版 launch 不启动该节点**（第 4.1 节）。

---

## 8. Safety-Gate Control Flow（精确分支）

逐题回答（均 **CONFIRMED**，附行号）：

1. **dry_run=true 时是否创建 IK client？** → **否**。`ik_solve` 在 runtime_adapter.py:76-83 命中 `if self.dry_run: return [500,500,500,500,500]`，在到达 client 创建（line 177 `_call_ik_blocking`）之前返回。
2. **dry_run=true 时是否调用真实 IK service？** → **否**。同上，提前返回 mock。
3. **dry_run=true 时是否创建 /servo_controller publisher？** → **否**。`servo_move`（line 212）/`gripper_set`（line 246）首行 `if self.dry_run or not self.enable_real_servo: return`，在 `create_publisher`（line 220/254）之前返回。real_grounded_runtime_node 自身 `__init__` 也**不创建** `/servo_controller`（仅 /runtime/* 与 /executor/done，line 49-57）。
4. **dry_run=true 时是否发布 Servo 消息？** → **否**。同上，publish（line 236/269）不可达。
5. **dry_run=true 时 GripperAction 如何处理？** → `adapter.gripper_set` 同样提前 return（不发布）；`GripperAction.execute` 仍返回 success（actions.py:62-69），即“逻辑上完成、物理上不动”。
6. **enable_real_ik 与 dry_run 优先关系？** → **dry_run 优先**。`ik_solve` 先判 dry_run（line 76），再判 `enable_real_ik`（line 85）。dry_run=true 时无论 enable_real_ik 取值都返回 mock。
7. **enable_real_servo 与 dry_run 优先关系？** → **dry_run 优先**（OR 语义）。`if self.dry_run or not self.enable_real_servo: return`（line 212/246）。任一为“假动作”即不发布。
8. **require_confirm=true 时，未收到 /runtime/confirm 是否可能继续？** → **否**。`_on_grounded_task` 在 require_confirm 时进入 WAITING_CONFIRM 并存 `_pending_ctx`，只有 `_on_confirm`（line 255）收到匹配 confirm 才调 `_execute_skill`（line 297）。confirm 与 task_id 强匹配（line 267-270）。
9. **confirm 超时后进入什么状态？** → `_check_confirm_timeout`（line 304，0.5s 周期定时器）到期 → `_cancel_task` → `TaskState.CANCELLED`，result.reason="cancelled_by_user"，`/executor/done`=False（line 319-329）。
10. **run_once=false 时是否可能重复执行？** → **是（设计如此）**。run_once=False 时 `if self.run_once and self._has_run: return`（line 164）不触发，可处理后续 `/grounded_task_context`。但仍有 `_busy`/`_pending_ctx` 互斥（line 167），不会并发执行。**默认 run_once=True 不重复**。

**真实硬件发布的精确条件（CONFIRMED）**：
```
servo_move / gripper_set 实际 publish  ←必须满足→  (dry_run == False) AND (enable_real_servo == True)
ik_solve 实际调用 service             ←必须满足→  (dry_run == False) AND (enable_real_ik == True) AND _HAS_IK_SRV
publish 前 pulse 钳位：max(0, min(1000, int(p)))   (runtime_adapter.py:234, 267)
```

---

## 9. ActionExecutor and Concurrency Audit

| 项 | 状态 | 证据 / 说明 |
|----|------|-------------|
| Runtime busy 状态 | **CONFIRMED** | `_busy` 在 `_execute_skill` 入口置 True、finally 置 False（real_grounded_runtime_node.py:336,393） |
| 入口互斥 | **CONFIRMED** | `_on_grounded_task` 在 `_busy or _pending_ctx is not None` 时 skip（line 167-171） |
| ActionExecutor 自身并发保护 | **NOT IMPLEMENTED** | `ActionExecutor.run` 无锁；依赖上层单线程调用 |
| 同一 RuntimeAdapter 是否可能被并发调用 | **CONFIRMED 不会** | 单线程 rclpy spin + `_execute_skill` 用 `run_until_complete` **阻塞**主线程，期间无新回调进入；ActionExecutor 内 `await` 顺序执行 |
| per-action timeout | **NOT IMPLEMENTED** | ActionExecutor 无超时；仅 MoveAction 内 IK 有 timeout（`timeout_sec`，默认 8.0，actions.py:16,32） |
| IK timeout | **CONFIRMED** | `ik_solve` 用 `asyncio.wait_for(..., timeout+1.0)`（runtime_adapter.py:108-114）；临时节点内另有 deadline 轮询（line 192-198） |
| Servo timeout | **NOT IMPLEMENTED** | `servo_move` 发布后仅靠 `wait_after_sec=duration_ms/1000` sleep（actions.py:46,67），**无舵机到位确认**（get_joint_state 是 stub） |
| cancellation（任务取消） | **PARTIAL** | 仅 confirm 阶段可 cancel；**EXECUTING 中无法取消**（无取消通道） |
| task cancellation（运行中） | **NOT IMPLEMENTED** | 无中断 `run_until_complete` 的机制 |
| exception handling | **CONFIRMED** | `_execute_skill` 捕获异常→ExecutionResult(success=False, reason="exception")（line 345-351） |
| failure propagation | **CONFIRMED** | ActionExecutor 首步失败即返回失败（action_executor.py:23-29）；Runtime 据此转 FAILED |
| ActionResult 聚合 | **CONFIRMED** | 聚合 ik_calls/ik_failures/steps_completed/steps_total（action_executor.py:13-37） |
| shutdown 时在执行的 Action | **NOT IMPLEMENTED** | main() 仅 spin→destroy_node（line 437-443），**无对在飞 IK future / 临时节点线程的优雅取消** |
| retry | **NOT IMPLEMENTED** | max_retries=2 存在但无代码读取；RETRYING 状态从未进入 |

**并发安全结论（CONFIRMED）**：在单线程 spin + 阻塞式 `_execute_skill` + `_busy` 互斥的组合下，**不会出现并发下发 Servo**。代价是执行期间 rclpy 主线程被阻塞（回调、confirm、verification 均不处理），且 IK 通过临时节点 + 后台线程 + 手动 `spin_once` 规避 executor 冲突（runtime_adapter.py:105-202，注释自述）。

---

## 10. task_id End-to-End Trace

| 段 | 消息/字段 | 文件:类:函数 | task_id 来源 | 丢失？ | 测试？ |
|----|----------|-------------|-------------|--------|--------|
| /parsed_command | JSON | parser（未审查源码） | **无 task_id** | — | — |
| grounding → /grounded_task_context | JSON{intent,parsed_command,target_object,target_pose,status,detail} | grounding_node.py:257-265 | **无 task_id**（grounding 不生成） | 是（上游无） | 无 |
| TaskContext 生成 | task_id | task_context.py:57-65 `_generate_task_id` | **此处新生** `task_<uuid12>_<ts>` | — | test_task_context |
| /runtime/state | ctx.to_dict().task_id | real_grounded_runtime_node.py:114 | 来自 ctx | 否 | test_runtime_events |
| /runtime/log | event_id=`evt_<task_id>_<seq>`, task_id | real_grounded_runtime_node.py:120-134 | 来自 ctx | 否 | test_runtime_events（序列号） |
| /runtime/preview | task_id | real_grounded_runtime_node.py:136-155 | 来自 ctx | 否 | 无 |
| /runtime/execution_result | result.task_id | execution_result.py:17-26 | 来自 ctx | 否 | 无 |
| /runtime/verification_result（sidecar 发） | VerificationResult.task_id | verification_result_node.py（`_publish`） | **恒为默认 "verification"**（构造时从不赋值，verification_result.py:15） | **是（链路断裂）** | test_verification |
| runtime 内部 verification_complete log | task_id | real_grounded_runtime_node.py:430 | 来自 ctx | 否 | — |
| RobotOps SQLite events | task_id 列 | robotops_recorder_node.py:64-77；event_store.py:9 | 从各 topic payload 提取 | 见下 | test_event_store |

**关键结论（CONFIRMED，P1）**：
- **task_id 在 `real_grounded_runtime_node` 处诞生，不回溯 grounding/parser**。无法通过 task_id 把一条任务关联回 grounding 选物或 parser 解析阶段。
- **`/runtime/verification_result` 的 task_id 链路断裂**：sidecar 构造 `VerificationResult` 时**从不设置 task_id**（verification_result_node.py:129-143/162-168/260-266/311-317），持久化到 SQLite 时 task_id 列 = `"verification"`，**无法关联真实任务**。缓解：runtime 自身在收到 postcheck 后发的 `verification_complete` log（line 430）带真实 task_id——但原始 verification_result 行不可关联。
- **能否在 RobotOps 用 task_id 还原时间线**：从 `runtime/state`、`runtime/log`、`runtime/execution_result` 三路可还原（均带真实 task_id，且 log 有单调序列号 `evt_<task_id>_<seq>`，CONFIRMED 由 test_runtime_events 覆盖）；**verification_result 一路断裂**。**前提**：robotops_recorder 必须在运行（当前未运行，且不在 ground_runtime_bringup launch 内）。

---

## 11. Current Path vs Legacy Path

| 维度 | 新 Runtime 主路径 | Legacy 路径 |
|------|-------------------|-------------|
| 节点 | real_grounded_runtime_node | ground_executor_node |
| 包 / 可执行 | sketch_runtime / real_grounded_runtime_node | **llm_executor / executor_node**（setup.py:23） |
| 源码存在 | CONFIRMED | CONFIRMED（src/llm_executor/llm_executor/executor_node.py，660 行） |
| console_script 注册 | CONFIRMED | CONFIRMED |
| 可手动 `ros2 run` 启动 | 是 | **是** |
| **当前正式 launch 启动？** | 是（ground_runtime_bringup） | **否**（`grep` 全 src：无任何 launch/脚本引用 `llm_executor`/`executor_node`/`ground_executor`，除其自身 setup.py） |
| 输入 | /grounded_task_context | /grounded_goal（grounding 同时发布两者） |
| 硬件写入点 | /servo_controller（经 controller_manager） | **/ros_robot_controller/bus_servo/set_position**（默认 servo_topic，executor_node.py:78，**总线旁路**） |
| dry_run 默认 | True | **False**（executor_node.py:72，“真执行默认 False”） |
| dry_run 门控 | 有效（第 8 节） | 有效（executor_node.py:603,617 `if self.dry_run ...`） |
| task_id | 有（runtime 生成） | **无**（`grep task_id` 在 executor_node.py 无命中） |
| IK service | /kinematics/set_pose_target | /kinematics/set_pose_target（executor_node.py:96） |
| 当前是否运行 | 否 | 否 |

**四种状态严格区分（CONFIRMED）**：legacy `ground_executor_node` = **存在 + 可手动启动 + 默认不启动 + 当前未运行**。这四态不可混用：它**不是**“默认启动”，也**不是**“正在运行”，但**是**“可被手动启动且 dry_run 默认 False 且直写总线”。

**并行风险（latent P0）**：grounding 在 `publish_runtime=true` 时**同时**发 `/grounded_goal`（line 241）与 `/grounded_task_context`（line 265）。若有人手动启动 legacy executor，它将（dry_run=False 默认）直接向 `bus_servo/set_position` 写脉冲，绕过 controller_manager 限位，并与新 Runtime 并存。当前无仲裁层阻止。

---

## 12. Direct Hardware-Write Search

**搜索命令**：
```
grep -rnE "servo_controller|bus_servo/set_position|ServosPosition|/grasp\b" src/sketch_runtime src/grounding src/robotops
grep -rnE "create_publisher|\.publish\(" src/sketch_runtime/sketch_runtime/
```

**新 Runtime 命中（CONFIRMED）**：

| 文件:行 | 类/函数 | Topic | 消息类型 | 触发条件 | 经 RuntimeAdapter？ | launch 可达？ | legacy？ | 风险 |
|---------|---------|-------|---------|---------|--------------------|--------------|----------|------|
| runtime_adapter.py:220,236 | RuntimeAdapter.servo_move | /servo_controller | ServosPosition | dry_run=False AND enable_real_servo=True | 是（本体） | 是 | 否 | 低（双门控+钳位） |
| runtime_adapter.py:254,269 | RuntimeAdapter.gripper_set | /servo_controller | ServosPosition | 同上 | 是（本体） | 是 | 否 | 低 |

**新 Runtime 未命中（CONFIRMED）**：`/grasp`、`/ros_robot_controller/bus_servo/set_position`、`bus_servo/set_state`、`/joint_controller` 在 `src/sketch_runtime`、`src/grounding`、`src/robotops` **均无写入**。real_grounded_runtime_node 对 `/servo_controller` 的唯一引用是日志字符串（real_grounded_runtime_node.py:101）。

**Legacy 命中（CONFIRMED，仅记录，不在新 Runtime 内）**：

| 文件:行 | Topic | 触发 | 风险 |
|---------|-------|------|------|
| executor_node.py:78,193,612,630 | `/ros_robot_controller/bus_servo/set_position`（默认 servo_topic） | dry_run=False 时 `_publish_servos` 直发 | **高**（总线旁路，绕过 controller_manager 限位） |

**结论（CONFIRMED）**：新 Runtime 主路径**只经 RuntimeAdapter → /servo_controller**，无总线旁路。绕过路径仅存在于 legacy（手动启动）。

---

## 13. Verification Audit & RobotOps Audit

### 13.1 Verification（CONFIRMED）

| 维度 | 状态 |
|------|------|
| 代码存在 | Implemented（verification_result_node.py，462 行） |
| console_script | Implemented（verification_result_node:main） |
| **当前正式 launch 包含？** | src 版 launch：是；**install 版 launch：否**（第 4.1 节漂移） |
| 当前运行？ | 否 |
| 输入 | /grounded_task_context（precheck 触发）、/world_model/stable_objects（观测）、/executor/done（postcheck 触发） |
| 输出 | /runtime/verification_result（stage: precheck/postcheck/post_place） |
| observation-only？ | **CONFIRMED**：不调 IK、不发 Servo、不阻塞 Runtime（只发 verification_result；Runtime 侧 `_on_verification_result` 只在 success 路径后等 postcheck） |
| task_id 字段 | **缺陷**：VerificationResult.task_id 恒为 "verification"（第 10 节） |
| 单元测试 | Implemented（test_verification.py、test_verification_integration.py；本次 passed） |
| 集成测试 | 部分（test_verification_integration） |
| 被状态机调用？ | 是（real_grounded_runtime_node `_on_verification_result`，line 397；但**只处理 stage=="postcheck"**，忽略 post_place） |
| 实机验证？ | **未验证**（无硬件验收） |

**注意（CONFIRMED，P2）**：verification 的 precheck **不阻塞** Runtime 状态机——Runtime 自己的 `skill.precheck` 只查 target_object 存在（base_skill.py:14）；sidecar 的 precheck 结果发布后 Runtime 不消费用于 gating。即“precheck 失败仍可执行”在架构上成立（依赖 require_confirm 的人工确认兜底）。

### 13.2 RobotOps（CONFIRMED）

| 维度 | 状态 |
|------|------|
| 代码存在 | Implemented（robotops_recorder_node.py / event_store.py / db.py / task_history.py） |
| console_script / launch | Implemented（robotops_recorder.launch.py） |
| **当前正式 launch 包含？** | **否**——不在 ground_runtime_bringup；需单独启动 |
| 当前运行？ | 否 |
| 输入 | /runtime/state、/runtime/log、/runtime/execution_result、/runtime/verification_result |
| 输出 | SQLite `events` 表（id, source_topic, task_id, event_id, payload_json, received_at；WAL；3 索引） |
| task_id 字段 | 从 payload 提取，缺失记 "unknown_task"（继承第 10 节 verification 断链） |
| 单元测试 | Implemented（test_event_store.py、test_task_history.py；本次 passed） |
| 集成测试 | 未发现端到端集成测试 |
| 被状态机调用？ | 否（独立 sidecar，订阅式） |
| 实机验证？ | **未验证** |

**db_path 不一致（CONFIRMED，P2）**：节点默认 `~/.ros/robotops.db`（robotops_recorder_node.py:16），launch 覆盖为相对路径 `robotops.db`（robotops_recorder.launch.py:13，**CWD 依赖**），而 runtime_index.md 提到 `~/ros2_ws/robotops.db`。三者不一致。

### 13.3 状态分级（严格区分）

- **Implemented**：Verification 全链代码、RobotOps 全链代码、各自单元测试。
- **Integrated**：Verification 在 src launch 中集成（install 否）；RobotOps 不在主 launch。
- **Launched**：当前**均未运行**。
- **Tested**：单元测试 passed（但 async 执行链跳过，见第 14 节）。
- **Verified on real robot**：**均未实机验证**。

---

## 14. Test Inventory and Actual Results

**测试文件清单**：
- `src/sketch_runtime/test/`：test_actions、test_runtime_events、test_skill_registry、test_target_object、test_task_builder、test_task_context、test_verification_integration、test_verification
- `src/robotops/test/`：test_event_store、test_task_history
- `src/grounding/test/`：test_copyright、test_flake8、test_pep257（ament lint，未运行）

**ROS 依赖核查（CONFIRMED）**：对每个测试 `grep import rclpy|create_node|create_publisher|create_client|create_subscription|rclpy.init` → **rclpy_hits=0**。`test_runtime_events.py` 用 `sys.modules` 桩 rclpy/std_msgs 并以 `object.__new__(RealGroundedRuntimeNode)` 绕过 `__init__`（test_runtime_events.py:23-57），**不创建真节点、不调 rclpy.init** → 安全。

**实际运行命令与结果（CONFIRMED，存档 `/tmp/runtime_audit_pytest_20260731.log`）**：
```
python3 -m pytest src/sketch_runtime/test src/robotops/test -rs --tb=short -q
→ 130 passed, 23 skipped, 46 warnings in 0.18s   (pytest_exit=0)
```

**23 个 skipped（CONFIRMED，P1）**：
- 全部来自 `test_actions.py`（17）与 `test_skill_registry.py`（6）的 `async def` 测试。
- skip 原因：`async def function and no async plugin installed`——**PC 未安装 pytest-asyncio**（`import pytest_asyncio` 抛 ImportError）。
- **影响**：`PickSkill.execute`、`MoveAction.execute`（含 IK 失败分支）、`ActionExecutor.run`、`SkillManager` 异步路径这些**执行链核心测试被静默跳过**，未被实际执行。passing 的 130 个为同步数据类 / 状态机 / 事件格式 / SQLite 逻辑测试。

**导入路径（CONFIRMED）**：`source install/setup.bash` 后，sketch_runtime/robotops 解析自 install；与 src 字节一致（第 4 节），故结果对两者等价。

**严谨结论**：不得将“130 passed”推广为“Runtime 执行链已通过测试”——**执行链 async 测试实际未运行**。

---

## 15. Confirmed Findings（汇总）

1. dry_run 门控有效：dry_run=True 时不创建 IK client / servo publisher，不发布 Servo（runtime_adapter.py:76,212,246）。**CONFIRMED**
2. 真实硬件需 `dry_run=False AND enable_real_servo=True`（servo）/ `AND enable_real_ik=True`（IK）；publish 前 pulse 钳位 [0,1000]。**CONFIRMED**
3. 新 Runtime 无 `/grasp`、无 `bus_servo/set_position` 直写；唯一硬件写入点为 RuntimeAdapter→/servo_controller。**CONFIRMED**
4. legacy `ground_executor_node`（llm_executor:executor_node）存在、可手动启动、dry_run 默认 False、servo_topic 默认 bus_servo/set_position、无 task_id；但无 launch/脚本启动它。**CONFIRMED**
5. install `ground_runtime_bringup.launch.py` 过期（05-19），缺少 verification sidecar；Python 模块与其余 launch 无漂移。**CONFIRMED**
6. 当前 ROS 图控制栈各 ×1（R31 已关闭）；PC 侧 Runtime/grounding/legacy/robotops 均未运行。**CONFIRMED**
7. 默认参数三层一致且全安全（dry_run=true / require_confirm=true / enable_real_ik=false / enable_real_servo=false / run_once=true）。**CONFIRMED**
8. 并发互斥有效（_busy + 阻塞式 run_until_complete），不会并发下发 Servo。**CONFIRMED**
9. task_id 在 runtime 处诞生，不回溯 grounding/parser；`/runtime/verification_result` task_id 断链（恒 "verification"）。**CONFIRMED**
10. 运动参数（hover_height/grip_*_pulse 等）为 PickSkill.DEFAULTS 代码常量，生产路径不可经 launch/YAML 配置。**CONFIRMED**
11. 23 个 async 执行链测试因缺 pytest-asyncio 被跳过；130 同步测试 passed。**CONFIRMED**
12. 三个 msg 包（kinematics_msgs/servo_controller_msgs/ros_robot_controller_msgs）在 PC 已安装 → 真实路径连通。**CONFIRMED**

---

## 16. Unknowns

- **UNCONFIRMED**：`/buzzer_controller` 当前未在图中出现，但是否彻底修复（dev log 第 24 节的 finger_trace/joystick_control 寄生同名节点）未在本审查核实——本次不扩展到 app/peripherals。
- **UNKNOWN**：parser（llm_parser / llm_command_parser_node）源码本轮未读取；`/parsed_command` 的字段完整性、是否含 task_id 未确认。
- **UNKNOWN**：Jetson 侧 `start_app_node.service` 当前进程树本次未复核（沿用 dev log 2026-07-30 证据；当前 PC 图与该证据一致）。
- **UNKNOWN**：`servo_controller` 可执行文件（C++，controller_manager/servo_manager 内部）的限位实现未读 C++ 源码（超边界）。
- **UNKNOWN**：verification_result_node 的 task_id 断链是否有其他下游消费者依赖 "verification" 该值——未发现，但未穷举。
- **UNCONFIRMED**：`/joint_controller`、`/ros_robot_controller/bus_servo/set_state` 是否属于当前主路径——新 Runtime 不写它们（CONFIRMED 不属主路径），但是否被 Jetson 侧其他节点使用未查。

---

## 17. P0 / P1 / P2 Risks

### P0（latent —— 需人工手动触发，当前未自动发生）
- **P0-L1 legacy executor 并存即直写总线**：`ground_executor_node` dry_run 默认 False、默认 servo_topic=`/ros_robot_controller/bus_servo/set_position`（绕过 controller_manager 限位）、无 task_id、可手动启动。若与新 Runtime 并存将产生不可复现的双写。**当前无 launch 启动它（CONFIRMED），但代码层无机制阻止手动 `ros2 run llm_executor executor_node`。** 缓解建议见第 18 节。

> 说明：任务定义的“当前正式 launch 可能启动 legacy 与新 Runtime 两套执行器”这一 P0 **当前不成立**（legacy 不在任何 launch）。新 Runtime 自身的 dry_run/enable 门控、确认门、并发互斥均**未失效**（CONFIRMED）。故**当前配置下无活动 P0**。

### P1
- **P1-1 install launch 漂移**：从 install 启动 ground_runtime_bringup 不会拉起 verification sidecar。→ 重建 sketch_runtime。
- **P1-2 async 执行链测试被跳过**：缺 pytest-asyncio，PickSkill/MoveAction/ActionExecutor 测试未实际运行。→ 安装 pytest-asyncio 后重跑（需用户批准 pip）。
- **P1-3 verification_result task_id 断链**：SQLite 中 verification 行 task_id="verification"，无法关联任务。→ sidecar 构造 VerificationResult 时回填 task_id（从 /grounded_task_context 或 /executor/done 携带的 id）。
- **P1-4 运动参数不可配置**：生产路径 PickSkill 恒用 DEFAULTS（含 grip_close_pulse=700 等），无法经 launch/YAML 调整。→ 若需实机调参，需将 motion params 提升为 real_grounded_runtime_node 的 declare_parameter 并填入 skill_params。
- **P1-5 INTENT_MAP 引用未注册 Skill**：place/pour/move/home → instantiate 返回 None 静默 return。→ 注册或收窄映射。

### P2
- **P2-1 无 Servo 到位确认**：get_joint_state 是 stub，servo_move 仅靠 duration sleep（无闭环）。
- **P2-2 无运行中取消 / 无 per-action 超时 / 无 retry**：max_retries=2 形同虚设；RETRYING 状态未使用。
- **P2-3 grounding 子串匹配（R14 未修）**：`_select_object` 仍 `cls in class_name or class_name in cls`，空 cls 匹配全部（grounding_node.py:270）。
- **P2-4 yaw 丢失**：grounding 的 rto 不含 world_yaw → 运行时 source_pose rpy[2] 恒 0（real_grounded_runtime_node.py:209）。
- **P2-5 db_path 三处不一致**：节点默认 / launch 相对路径 / 文档路径。
- **P2-6 package.xml 未声明 msg 依赖**：sketch_runtime 仅 depend rclpy/std_msgs，kinematics_msgs/servo_controller_msgs 靠 try/except（功能可用但不规范）。
- **P2-7 precheck 不阻塞状态机**：sidecar precheck 失败不阻止执行（靠 require_confirm 兜底）。
- **P2-8 PlaceSkill 等未实现**：当前 Pick 主线不受其阻断（不列为 P1）。

---

## 18. Minimum Proposed Changes（仅建议，不执行）

1. **重建 sketch_runtime**（解决 P1-1）：`colcon build --packages-select sketch_runtime`，使 install 的 ground_runtime_bringup.launch.py 含 verification sidecar。**不涉及硬件，需用户批准构建。**
2. **隔离 / 弱化 legacy executor**（解决 P0-L1）：在未实现仲裁层前，至少（a）在文档与启动检查中明确禁止 `ros2 run llm_executor executor_node` 与新 Runtime 并存；或（b）将 legacy `servo_topic` 默认改为 `/servo_controller`、`dry_run` 默认改为 True（**属修改 Jetson/PC legacy 源码，需用户单独批准**）。本审查不执行。
3. **回填 verification task_id**（解决 P1-3）：verification_result_node 在构造 VerificationResult 时传入 context 中的 task_id。
4. **补装 pytest-asyncio 并重跑测试**（解决 P1-2）：需用户批准 `pip install pytest-asyncio`（CLAUDE.md 禁止 Agent 自行 pip install）。
5. **运动参数提升为 ROS 参数**（解决 P1-4，仅当需要实机调参时）。

---

## 19. Acceptance Criteria（Dry-run 准入检查清单）

进入安全 Dry-run 前**必须**满足：

- [x] 新 Runtime dry_run 门控经代码核实有效（第 8 节，CONFIRMED）
- [x] 新 Runtime 无总线旁路、无 /grasp 直写（第 12 节，CONFIRMED）
- [x] 默认参数三层一致且全安全（第 7 节，CONFIRMED）
- [x] 当前正式 launch 不启动 legacy、不启动第二套控制栈（第 11 节，CONFIRMED）
- [x] 当前 ROS 图控制栈单实例（第 3 节，CONFIRMED）
- [ ] **install launch 与 src 一致**（当前**不满足**：verification sidecar 缺失，P1-1）→ 需重建
- [ ] **执行链 async 测试实际运行**（当前**不满足**：23 skipped，P1-2）→ 需装 pytest-asyncio
- [ ] Dry-run 实际启动后观察：`/runtime/state` 出现 WAITING_CONFIRM、`/runtime/preview` 发布、且 `/servo_controller` **无** Runtime publisher（运行时确认，本轮节点未运行无法确认）

---

## 20. Dry-Run Readiness Decision

**CONDITIONAL GO（有条件放行）**。

- **代码与门控层面（CONFIRMED）**：dry_run 路径安全、无旁路、默认全安全参数、legacy 不自动启动、控制栈单实例。满足“可安全执行 Dry-run”的静态条件。
- **两个未决前置（非硬件）**：
  1. install launch 漂移（P1-1）——若不重建，从 install 启动会缺 verification sidecar（不影响安全，但影响验证链）。
  2. async 执行链测试未运行（P1-2）——执行链未被单元测试覆盖（逻辑已人工读核，但无自动回归保护）。
- **不放行**：真实 IK（enable_real_ik）、真实 Servo（enable_real_servo）、Hover、Pick——一律 NO-GO，与项目当前安全门一致。

---

## 21. Recommended Next Command（建议，不执行）

按顺序（每步均需用户批准 / 由用户执行）：

1. **重建 sketch_runtime（解决 install 漂移）**：
   ```
   colcon build --packages-select sketch_runtime
   ```
   （预计 < 1 min；不涉及硬件；CLAUDE.md 要求构建需用户批准）
2. **（可选）补装 async 测试插件并重跑**：
   ```
   pip install pytest-asyncio   # 需用户批准
   python3 -m pytest src/sketch_runtime/test src/robotops/test -rs
   ```
3. **Dry-run 启动（由用户执行；启用 dummy 世界模型，禁用真实 IK/Servo）**：
   ```
   ros2 launch sketch_runtime ground_runtime_bringup.launch.py \
     dry_run:=true require_confirm:=true run_once:=true use_dummy_wm:=true
   ```
   并另起 robotops recorder 以校验 task_id 链路：
   ```
   ros2 launch robotops robotops_recorder.launch.py
   ```

---

## 22. 声明

本次审查全程 READ-ONLY：未修改源码/launch/YAML/systemd/build/install/log；未启动 ROS 2 launch；未运行 real_grounded_runtime_node / runtime_test_node；未发布任何控制 Topic；未调用真实 IK；未执行 Servo/Hover/Pick；未触碰 Jetson；未执行 git add/commit/push/reset。唯一“副作用”：纯 Python 单元测试（已确认不创建 ROS Node/Publisher/Client/Service）与只读 `ros2 node list`。任务到此停止，不自动进入 Dry-run 或真实 IK 阶段。
