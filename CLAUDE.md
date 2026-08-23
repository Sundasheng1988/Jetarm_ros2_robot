# CLAUDE.md

> JetArm ROS 2 项目 Agent 工作规则  
> 当前基线：2026-08-22
> 文档更新：2026-08-23
> Voice 稳定基线：`c3228a3` / `voice-stabilization-stable-20260822`
> 目标：让 Agent 用最少上下文进入当前真实项目状态，同时严格限制真实机器人硬件风险。
> 启动命令速查：`docs/quick_start.md`

---

## 1. 每次会话先读取

开始任何项目任务前，先读取：

```text
docs/runtime_index.md
docs/runtime_debug_guide.md
docs/topic_service_map.md
```

职责：

```text
runtime_index.md
→ 当前阶段、当前 P0、稳定基线、工作顺序、安全边界

runtime_debug_guide.md
→ 启动方式、PC-only / Cross-host DDS、监听、Confirm、验收和故障排查

topic_service_map.md
→ 节点、Topic、Service、Action、Current / Legacy、Voice/Nav 与 Manipulation 控制路径
```

涉及系统架构、跨模块设计或新增节点时，再读取：

```text
docs/runtime_architecture.md
```

涉及具体代码时，必须继续核对：

```text
src / launch / YAML / tests
当前 ROS 2 graph / process / systemd / journal
```

文档不是代码事实。读取一次并形成摘要即可，不要在同一任务中反复全文读取。

---

## 2. 当前系统基线

### PC

```text
Role: Development / Intelligence / Project Runtime
Workspace: /home/sundasheng/ros2_ws
```

主要运行：

```text
Rebecca Voice / FunASR / TTS
LLM / Parser / Agent
YOLO / ROI / Fusion / StableObjectTracker
Grounding
Manipulation Runtime / Skill / Action
Navigation Executor
Place Manager 上层接口
Verification
RobotOps
RViz / rosbag / offline analysis
Git / docs / tests
```

### Orin

```text
Role: Robot-side Hardware Runtime
Workspace: /home/ubuntu/ros2_ws
```

主要运行：

```text
Gemini Camera
Kinematics
controller_manager
ServoManager
ros_robot_controller
STM32 / Servo bus
Mobile base
RPLidar
Robot-side TF
AMCL / Nav2 when explicitly enabled
start_app_node.service
```

PC 不得启动第二套 Orin 硬件驱动。

Jetson / Orin 工作区默认只读。未经用户明确批准，不得修改或构建 Jetson 代码。

---

## 3. ROS 2 / DDS 基线

通用：

```text
ROS 2 Humble
ROS_DOMAIN_ID=23
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### PC-only Voice / Navigation Regression

默认安全回归环境：

```bash
export ROS_DOMAIN_ID=23
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1
unset CYCLONEDDS_URI
```

适用于：

```text
ASR
TTS
Voice Agent
Parser
Navigation confirmation
Navigation Executor PC-only 状态机
```

### PC ↔ Orin Cross-host

需要 Camera / Kinematics / AMCL / Nav2 / Robot hardware 时：

```bash
export ROS_DOMAIN_ID=23
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
```

历史有线 Cyclone 配置：

```text
PC:
file:///home/sundasheng/ros2_ws/config/cyclonedds/pc_camera_eno1.xml

Orin:
file:///home/ubuntu/ros2_ws/config/cyclonedds/orin_camera_eth0.xml
```

固定有线地址：

```text
PC eno1:   192.168.100.2
Orin eth0: 192.168.100.1
```

重要：

> `CYCLONEDDS_URI` 必须与当前实际活动网卡一致。

如果 PC 使用 Wi-Fi / hotspot（例如 `wlo1`），不得机械复用绑定 `eno1` 的 XML。

DDS 发现异常时优先检查：

```bash
printenv ROS_DOMAIN_ID
printenv RMW_IMPLEMENTATION
printenv ROS_LOCALHOST_ONLY
printenv CYCLONEDDS_URI
ip link
```

必要时：

```bash
unset CYCLONEDDS_URI
```

再做最小 graph 验证。

修改 RMW / DDS 环境后，已经运行的 ROS 2 进程不会动态切换，必须重启相关进程。

---

## 4. 当前两条主线

### Track A — Rebecca Voice / Semantic Navigation

```text
Microphone
→ speech_dialog_funasr_node
→ /speech_query
→ llm_voice_agent_node
→ deterministic navigation confirmation
→ /voice_input/input
→ llm_command_parser_node
→ /parsed_command
→ navigation_executor_node
→ /goto_place / /cancel_navigation
→ Place Manager / goto_place_node
→ NavClient
→ Nav2 NavigateToPose
```

导航 action 在 `/parsed_command` 后与机械臂链路分流。

`grounding_node` 对：

```text
navigate_to_place
cancel_navigation
```

显式跳过，不进入机械臂 Grounding / Runtime。

### Track B — Fixed-Workspace Manipulation

```text
Camera
→ YOLO / ROI
→ Perception Fusion
→ StableObjectTracker
→ Parser
→ Grounding
→ /grounded_task_context
→ real_grounded_runtime_node
→ SkillManager / PickSkill / PlaceSkill
→ RuntimeAdapter
→ IK / Servo
→ Verification
→ RobotOps
```

两条主线共享部分输入和结果接口，但不得混淆执行职责。

---

## 5. 当前稳定基线

```text
Branch:
feature/voice_stabilization

Commit:
c3228a3

Tag:
voice-stabilization-stable-20260822
```

该基线已经完成真实语音回归：

```text
TTS self-echo control hardening
TTS → NORMAL utterance boundary isolation
TTS-time navigation confirmation
nav_wait_confirm short-confirm fast-listen
post-TTS residual mute bypass
nav_wait_confirm sensitive VAD context
TTS “停一下” barge-in
Robot STOP passthrough
L2 System Sleep / Wake
L3 Mute / Unmute
```

不要无理由回退。

Voice 修改发生回归时优先：

```bash
git show voice-stabilization-stable-20260822
git diff voice-stabilization-stable-20260822..HEAD
```

不要在 tag 的 detached HEAD 上继续长期开发。

---

## 6. 当前已实现能力

### Manipulation

```text
单一机械臂控制栈
PC / Orin Cyclone DDS
跨机相机 Topic
跨机 Kinematics Service
Dry-run
真实 IK
真实 Servo
首次真实视觉杯子抓取
```

夹爪实机基线：

```text
Servo ID10:
200 = open
700 = close
```

### Voice / Navigation

```text
Rebecca Voice / FunASR / TTS
L1 / L2 / L3 Voice Mode
TTS STOP
Robot STOP
Natural navigation request
Navigation second confirmation
Short “是的 / 确认” fast-listen
Semantic Locations
Place Manager
Navigation Intent
Navigation Executor
/goto_place
/cancel_navigation
/runtime/execution_result navigation result
Nav2 Interface
Real Nav2 Motion on physical robot
Voice → Semantic Navigation → Nav2 E2E
```

真实语音导航已完成：

```text
卧室
→ 餐厅
→ 客厅
→ 返回卧室
```

当前导航能力状态：

```text
Nav2 Interface             ✅ Connected
Real Nav2 Motion           ✅ Verified on physical robot
Voice → Semantic Nav E2E   ✅ Verified
AMCL / Localization        🔶 Real-navigation usable; robustness in validation
Navigation Pause / Resume  ⏳ Planned
```

Place Manager 核心接口：

```text
/save_place
/delete_place
/list_places
/get_place
/goto_place
/cancel_navigation
```

导航目标必须来自：

```text
places.yaml
```

---

## 7. 当前开发重点

Voice / Navigation：

```text
P0:
nav_wait_confirm stale pending protection / timeout

P1:
Navigation Pause / Resume
- pause target memory
- resume confirmation
- new goto_place from current pose

P1:
L3 mute negation guard
“不要静音”不能触发进入 MUTED

P1:
清理“暂停跟随”与 TTS “暂停”的词义竞争

P2:
PC-only regression
- 3 次确认 = 3 条命令
- 无 duplicate dispatch
- STOP always wins
- cancelled target cannot resume
- stale pending cannot be confirmed

P3:
后续任何 Voice / Navigation 状态机修改
→ 先 PC-only regression
→ 再做真实 Nav2 regression

当前真实 Nav2 E2E 已完成首次多地点验证。
```

并行 Manipulation 主线仍保留：

```text
Task MVP 1：整理蓝色积木
Task MVP 2：整理圆珠笔
Task MVP 3：搬运空茶杯
```

不要把 Manipulation 任务线误判为已废弃。

---

## 8. Voice / Navigation 语义边界

### L1 / L2 / L3

```text
L1:
瑞贝卡，暂停跟随
瑞贝卡，恢复跟随

L2:
瑞贝卡，系统休眠
瑞贝卡，启动系统

L3:
瑞贝卡，静音
瑞贝卡，取消静音
```

L3 wake 允许唤醒 / 恢复 L1 / face-follow 相关行为。不要自动把该联动当作 bug。

### TTS STOP

```text
停一下
别说了
```

含义：

```text
停止 Rebecca 当前播报
```

最终通道：

```text
/tts/interrupt
```

### Robot STOP / Navigation STOP

```text
停止移动
停止导航
取消导航
```

含义：

```text
停止实体机器人导航
```

TTS STOP 与 Robot STOP 必须严格分离。

### Navigation Confirmation

典型确认：

```text
是的
可以
确认
对的
没错
```

安全原则：

```text
STOP:
宁可误停，不能漏停

CONFIRM:
宁可不执行，不能误启动
```

确认属于执行侧命令，matcher 必须比停止 / 取消更严格。

---

## 9. Navigation Safety Boundary

LLM / Voice Agent 可以：

```text
理解语义
生成 deterministic intent
进入 confirmation state
产生规范文本命令
```

LLM / Voice Agent 不得：

```text
直接发布 /cmd_vel
直接发送 NavigateToPose goal
生成任意 map 坐标作为导航目标
接受任意语音坐标并直接执行
绕过 llm_command_parser_node
绕过 navigation_executor_node
绕过 place_manager
把语言回复当作导航成功证据
```

实体导航必须经过：

```text
Voice / CLI
→ Parser
→ Navigation Executor
→ Place Manager
→ Nav2
```

第一阶段只允许 `places.yaml` 中已保存的命名地点。

---

## 10. Navigation Pause / Resume 设计约束

截至当前，该能力尚未正式实现。

目标语义：

```text
“暂停导航 / 停止导航”
→ cancel 当前 Nav2 goal
→ 保存 paused_target

“继续导航 / 恢复导航”
→ “继续前往 <place> 吗？”
→ 用户确认
→ 从当前位置重新调用 goto_place(<place>)
→ Nav2 重新规划

“取消导航”
→ 停止当前 goal
→ 清空 paused_target
→ 后续“继续导航”不得恢复旧任务
```

不要依赖原 Nav2 goal 原地 resume。

推荐：

```text
pause = cancel current goal + remember semantic target
resume = create a new goto_place goal from current pose
```

任何导致机器人重新开始运动的 resume 默认保留 confirmation gate。

---

## 11. 谁执行什么

### 用户现场执行

```text
ros2 launch
长时间运行的 ros2 run
RViz
ros2 bag record

真实机械臂动作
真实底盘运动
真实 /goto_place
真实 Nav2 goal
/cmd_vel
机器人复位

systemd start / stop / restart
预计超过 30 秒的现场观察
```

### Agent 可以执行

```text
读取指定文件和源码
聚焦源码搜索
编辑用户要求的 PC 文件
语法检查
聚焦单元测试
指定软件包构建
离线分析

git status
git branch --show-current
git log
聚焦 git diff
git diff --check

只读 ROS 2 graph / process / systemd 调查
```

只读调查权限不得扩大解释为可以主动控制硬件。

---

## 12. 修改前要求

编辑任何文件前，先说明：

```text
任务目标
涉及文件
计划修改
验证方法
```

一次只处理一个明确任务。

不得顺带修改无关文件。

语音稳定化尤其禁止：

```text
为修一个小问题同时大改 ASR + TTS + Agent + Parser + Navigation
```

优先流程：

```text
稳定基线
→ 单问题
→ 单层定位
→ 最小 Patch
→ 分层测试
→ checkpoint
```

---

## 13. 硬件安全规则

未经用户明确批准，Agent 不得：

```text
发布 /cmd_vel
发布 /servo_controller
发布 /grasp
发布 /ros_robot_controller/bus_servo/set_position

调用会导致真实运动的 /goto_place
直接发送 Nav2 NavigateToPose goal
触发真实机械臂动作

启动或停止硬件 Bringup
修改 Jetson 运行时代码
修改网络、串口、udev 或 systemd
```

### Manipulation Real-Motion Gate

```text
单一控制栈
require_confirm=true
Stable Object 位姿稳定
无其他主动 /servo_controller 写者
工作区清空
用户现场观察
可立即断电
```

### Navigation Real-Motion Gate

真实 Nav2 测试前必须先 PC-only 证明：

```text
navigation request 正确
confirmation 正确
cancel 正确
Robot STOP 正确
无 duplicate dispatch
stale pending 已受控
目标来自 places.yaml
```

实车阶段还必须确认：

```text
底盘 / LiDAR / Odom / IMU / TF / AMCL 当前基线可用
用户现场观察
可立即停止机器人
```

---

## 14. 禁止操作

未经用户批准，不得执行：

```text
sudo
apt install
pip install
npm install
curl
wget

rm
rm -rf
chmod
chown
kill
pkill
reboot
shutdown

git reset
git clean
git restore
git rebase
git push

全工作区构建
无边界递归搜索
后台 Agent
并行 Agent
子 Agent
```

不得：

```text
清理 Git 状态
覆盖用户未确认的文件
删除历史日志
删除 archive
使用 git add .
自动 commit
自动 push
```

如果用户明确要求文档归档：

```text
优先 git mv
不覆盖 archive 中已有文件
归档后等待用户检查
```

---

## 15. Git 工作规则

任何核心修改前：

```bash
cd ~/ros2_ws

git branch --show-current
git status -sb
git log --oneline -5
git diff --check
```

禁止：

```bash
git add .
```

只 stage 明确目标文件。

Voice 稳定恢复点：

```text
c3228a3
voice-stabilization-stable-20260822
```

以下内容不要自动加入提交，除非用户明确要求：

```text
config/places.yaml.bak_20260809
voice_baseline_20260813_0002/
临时 test / debug 文件
```

commit 前检查：

```bash
git diff --cached --stat
git diff --cached --check
```

---

## 16. 调查与证据规则

所有结论必须区分：

```text
Confirmed Fact
Direct Observation
Inference
Rejected Hypothesis
Unknown
```

Source of Truth 顺序：

```text
当前 ROS 2 graph / process / systemd / journal
→ 当前 install
→ 当前 src / launch / YAML / tests
→ 当前文档
→ 历史日志
```

PC 上看到节点，不代表节点运行在 PC。节点主机必须通过 process / launch / systemd / journal 证据确认。

涉及机器人状态时：

```text
Rebecca / LLM 说了什么
≠
机器人真实完成了什么
```

必须检查：

```text
state transition
/voice_input/input
/parsed_command
/runtime/execution_result
Nav2 result
Verification / physical observation
```

---

## 17. Voice Debug 证据规则

遇到：

```text
“说了没反应”
```

不要直接归因于 FunASR。

按顺序定位：

```text
Audio frame captured?
→ VAD detected?
→ utterance formed?
→ utt_ms threshold?
→ TTS restricted?
→ mute_until?
→ ASR result?
→ control classifier?
→ Voice Agent state?
→ command published?
```

`min_utt_ms`：

> utterance 在送进 FunASR 前的最短语音时长门限。

当前不要全局降低 `min_utt_ms` 来修导航确认。

`nav_wait_confirm` 已使用专用 fast-listen。

---

## 18. 构建与测试

只构建用户指定的软件包：

```bash
colcon build --packages-select <package_name>
```

允许：

```text
python3 -m py_compile <file>
pytest <specific_test>
git diff -- <specific_file>
git diff --check
```

不默认构建整个工作区。

### Voice / Navigation 测试顺序

```text
Layer A — Voice only
wake
chat
sleep/wake
mute/unmute
TTS STOP
self-echo

Layer B — Voice Agent navigation
request
nav_wait_confirm
confirm
cancel
Robot STOP

Layer C — Parser
/voice_input/input
→ /parsed_command

Layer D — Navigation Executor / Place Manager
parsed_command
→ /goto_place / /cancel_navigation

Layer E — Real Nav2
最后才允许实体运动
```

不要一边修改 ASR/TTS，一边直接用真实 Nav2 判断语音是否正确。

---

## 19. 长输出规则

长输出必须写入日志文件。

终端只汇报：

```text
退出码
关键错误
关键结论
最多最后 80 行
```

不要输出完整大文件、完整源码或大段日志。

预计超过 2 分钟的命令，先说明预计耗时。  
预计超过 5 分钟的命令，必须先获得批准。

---

## 20. 文档维护规则

Current canonical 文档固定使用：

```text
docs/runtime_index.md
docs/runtime_debug_guide.md
docs/topic_service_map.md
docs/runtime_architecture.md
```

不要在 `docs/` 根目录长期并存：

```text
runtime_index.md
runtime_index_20260822.md
runtime_index_new.md
runtime_index_final.md
```

新版本验收后：

```text
旧 canonical
→ docs/archive/

新版本
→ canonical 文件名
```

历史文档不删除。

具体开发过程放在：

```text
docs/dev_log/
```

---

## 21. 任务完成格式

完成任务后汇报：

```text
读取了什么
修改了什么
执行了什么验证
退出码和关键结果
未完成或 Unknown
```

涉及 Git 时再补：

```text
git status
git diff --stat
git diff --check
```

然后停止。

不要自动开始下一项任务、commit、push 或继续修改其它文件。

---

## 22. 当前最重要原则

```text
真实机器人：
Safety > Convenience

语音确认：
False Positive Confirm 必须严格避免

停止：
STOP 永远优先

架构：
LLM 负责理解和计划
确定性节点负责执行边界

开发：
稳定基线
→ 单问题
→ 最小 Patch
→ 分层验证
→ checkpoint
```

当前 Voice 稳定基线：

```text
c3228a3
voice-stabilization-stable-20260822
```

不要无理由回退。
