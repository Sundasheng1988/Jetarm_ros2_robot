# 2026-08-24 Dev Log — Voice → Navigation Integration & DDS Environment Hardening

> **项目**：JetArm / Rebecca Voice / Mobile Navigation  
> **测试日期**：2026-08-24  
> **整理日期**：2026-08-25  
> **当前分支**：`feature/navigation_pause_resume`  
> **测试阶段**：PC-only Integration / Navigation Safety Regression  
> **实机状态**：本轮未启动真实 Nav2 / 未驱动 Eric 底盘

---

## 1. 本轮目标

本轮主要完成两类工作：

1. 验证 Rebecca 自然语言导航链路是否能够从 Voice 一直走到 Navigation Executor。
2. 解决 PC 多 ROS 节点并行时 CycloneDDS participant 不足的问题，并把 `rosvoice / rosrobot` 环境入口正式收口。

目标链路：

```text
Speech
→ speech_dialog_funasr_node
→ llm_voice_agent_node
→ /voice_input/input
→ llm_command_parser_node
→ /parsed_command
→ navigation_executor_node
→ /goto_place
→ goto_place_node
→ Nav2 /navigate_to_pose
→ Eric
```

本轮实际验证到：

```text
Speech
→ Rebecca
→ Parser
→ Navigation Executor
→ /goto_place boundary
```

由于测试时 `goto_place_node` 未成功保持运行，`/goto_place` Service 不存在，因此没有进入 Nav2 Action 层。

---

# 2. DDS Participant Exhaustion 问题

## 2.1 现象

最初尝试启动：

```bash
ros2 launch place_manager goto_place.launch.py
```

以及：

```bash
ros2 run place_manager navigation_executor_node
```

均失败。

核心错误：

```text
Failed to find a free participant index for domain 23
rmw_create_node: failed to create domain
rclpy._rclpy_pybind11.RCLError: error creating node
```

同时存在：

```text
selected interface "lo" is not multicast-capable: disabling multicast
```

后者不是主要故障。

### 判断

问题发生在：

```text
rclpy Node()
↓
rmw_cyclonedds_cpp
↓
CycloneDDS participant allocation
↓
participant index exhausted
```

节点甚至没有进入 `GotoPlaceNode` 或 `NavigationExecutorNode` 的业务逻辑。

因此，本问题与：

- `/goto_place` 业务逻辑
- Pause / Resume 状态机
- Nav2
- places.yaml
- 小车底盘

均无直接关系。

---

## 2.2 临时验证修复

使用：

```bash
export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
```

之后：

```bash
ros2 run place_manager navigation_executor_node
```

可以正常启动：

```text
NavigationExecutorNode 启动:
sub=/parsed_command
pub=/runtime/execution_result
goto=/goto_place
cancel=/cancel_navigation
```

说明 participant exhaustion 已被解除。

---

# 3. Voice → Parser → Executor 实际测试

## 3.1 Rebecca Voice Stack

启动 Rebecca 后，以下节点正常运行：

```text
speech_dialog_funasr_node
llm_voice_agent_node
tts_speaker_node
executor_done_sayer
```

LLM：

```text
backend=glm
model=glm-4.5-air
```

TTS：

```text
backend=cosyvoice
```

CosyVoice HTTP inference 正常返回 200，TTS 合成及播放正常。

---

## 3.2 自然语言导航请求

真实语音示例：

```text
“帮我去餐厅拿个杯子。”
```

Rebecca 正确进入部分任务协商：

```text
我目前还不能直接完成拿取，
不过能先让 Eric 去餐厅。
要让它过去吗？
```

状态进入：

```text
nav_wait_confirm
```

TTS 播放期间，自回声片段例如：

```text
“我目前还不能直接完成。”
“拿去，不过能先让。”
“Eric去餐厅。”
```

均被：

```text
🎧(interrupt-only) 丢弃非控制语音
```

没有形成 self-confirm / self-navigation。

---

## 3.3 Navigation Confirm

用户在 TTS 结束后说：

```text
“可以。”
```

系统识别：

```text
NAV_CONFIRM bypass L3 wake gate
```

Rebecca：

```text
好的，我让Eric去餐厅。
```

并真实发布：

```text
/voice_input/input <= 导航到餐厅
```

Parser → Executor 链路随后生效。

### 结论

以下链路已真实走通：

```text
真实麦克风
→ FunASR
→ Rebecca
→ Navigation confirmation
→ /voice_input/input
→ Parser
→ /parsed_command
→ NavigationExecutor
```

---

# 4. `/goto_place` Service 不存在时的安全失败

Navigation Executor 收到：

```text
navigate_to_place
place_name="餐厅"
```

但当时 `goto_place_node` 没有运行，因此：

```text
/goto_place 服务不可用，无法导航到 "餐厅"
```

结果：

```text
action=navigate_to_place
status=FAILED
place="餐厅"
```

随后 `executor_done_sayer` 播报：

```text
导航失败，请检查道路。
```

同样测试 `"客厅"`：

```text
/goto_place 服务不可用，无法导航到 "客厅"
status=FAILED
place="客厅"
```

### 结论

该负向路径行为正确：

```text
Service 不存在
→ 不伪造导航成功
→ 不产生真实运动
→ 明确 FAILED
```

---

# 5. Pause Navigation 负向路径测试

在原导航已经因为 `/goto_place` 不存在而 FAILED 后，用户说：

```text
“停止导航。”
```

ASR / Voice 层正确归一化为：

```text
停止移动
```

并下发到：

```text
/voice_input/input
```

Navigation Executor 发现当前没有 active navigation：

```text
暂停导航：当前无活动导航，无需暂停
```

发布：

```text
action=pause_navigation
status=PAUSED
place=""
```

再次收到 STOP：

```text
停止移动
```

仍返回：

```text
status=PAUSED
place=""
```

### 关键判断

这里：

```text
place=""
```

说明没有人为制造假的 paused target。

因此，本轮验证的是：

```text
无活动导航
→ STOP / Pause
→ 幂等 PAUSED
→ 不保存虚假的目标地点
```

这是安全行为。

但它 **不是**：

```text
ACTIVE(餐厅)
→ Pause
→ PAUSED(餐厅)
```

真实 Pause 生命周期仍需后续测试。

---

# 6. Resume Navigation 测试

## 6.1 Resume 需要重新确认

用户说：

```text
“恢复导航。”
```

Rebecca 没有直接下发恢复，而是：

```text
要继续刚才暂停的导航吗？
```

进入：

```text
nav_wait_confirm
```

TTS 自回声继续被 `interrupt-only` 丢弃。

用户正常确认：

```text
“是的。”
```

Rebecca：

```text
好的，我让Eric继续刚才的导航。
```

并发布：

```text
恢复导航
```

### 结论

恢复导航属于重新产生物理运动的动作，因此需要新的显式确认。

该确认门已真实验证。

---

## 6.2 没有 paused target 时拒绝恢复

因为此前没有真实 ACTIVE navigation 被暂停，Executor 内没有 `_paused_nav_meta`。

因此收到：

```text
resume_navigation
```

后结果：

```text
status=REJECTED
error_code=NO_PAUSED_NAVIGATION
place=""
```

语音反馈：

```text
当前没有可以恢复的导航任务。
```

### 结论

正确：

```text
没有 paused target
→ Resume
→ REJECTED
→ 不产生新导航
```

---

# 7. Resume Confirmation Timeout

另一轮测试中：

```text
恢复导航
→ Rebecca 询问确认
```

用户没有在期限内完成有效确认。

日志：

```text
⌛ 导航确认已超时，清除 pending navigation
```

之后出现的：

```text
“恢复的。是的。”
```

以及更晚的：

```text
“瑞贝卡是的。”
```

均没有重新触发旧的 Resume command。

### 结论

确认超时后：

```text
pending navigation
→ 清除
→ stale confirm 不生效
```

通过。

---

# 8. Robot STOP / TTS Self-Echo Regression

本轮持续观察到：

```text
停止导航
→ Robot STOP exact / prefix 命中
→ canonicalize 为 停止移动
→ /tts/interrupt
→ 下发停止移动
```

同时 Rebecca / done_sayer 自己播报的：

```text
好的，正在停止移动。
已暂停导航。
导航失败，请检查道路。
```

被 ASR 听到后均进入：

```text
🎧(interrupt-only) 丢弃非控制语音
```

没有产生递归命令。

说明此前 TTS self-echo hardening 未被本轮 Navigation 改动破坏。

---

# 9. 本轮测试中发现的重复 Executor 问题

调试过程中一度同时启动了两个：

```text
navigation_executor_node
```

结果两个 Executor 同时订阅：

```text
/parsed_command
```

并对同一命令分别生成结果。

表现为相同时间戳下重复：

```text
navigate_to_place FAILED
pause_navigation PAUSED
resume_navigation REJECTED
```

`executor_done_sayer` 因 task_id 去重而出现：

```text
跳过重复 execution_result
```

### 结论

正式启动必须避免：

```text
navigation.launch.py
+
手工 ros2 run navigation_executor_node
```

同时存在。

以后统一使用：

```bash
ros2 launch place_manager navigation.launch.py \
  launch_done_sayer:=false
```

该 launch 已包含：

```text
goto_place_node
navigation_executor_node
```

不再另外启动这两个节点。

---

# 10. `goto_place.launch.py` / `navigation.launch.py` / `place_manager.launch.py` 职责重新确认

## `goto_place.launch.py`

只启动：

```text
goto_place_node
```

提供：

```text
/goto_place
/cancel_navigation
```

内部：

```text
place name
→ PlaceStore
→ x/y/yaw
→ NavClient
→ Nav2 NavigateToPose
```

适合单独测试 GotoPlace Service。

---

## `navigation.launch.py`

正式自然语言导航语义层入口。

启动：

```text
goto_place_node
navigation_executor_node
executor_done_sayer（可选）
```

与 Rebecca Voice 同时运行时：

```bash
launch_done_sayer:=false
```

避免重复 done_sayer。

---

## `place_manager.launch.py`

仅用于命名地点管理：

```text
/save_place
/delete_place
/list_places
/get_place
```

不是小车执行导航的正式入口。

---

# 11. PC ROS Environment Hardening

昨天的测试暴露出：

```text
Voice Stack
+ Parser
+ Navigation Executor
+ GotoPlace
+ ros2 topic echo
+ ros2 CLI
```

并行后 CycloneDDS participant 搜索空间不足。

因此正式升级 PC DDS 环境。

---

## 11.1 `MaxAutoParticipantIndex`

以下配置统一为：

```xml
<ParticipantIndex>auto</ParticipantIndex>
<MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>
```

文件：

```text
config/cyclonedds/pc_camera_eno1.xml
config/cyclonedds/pc_camera_wlo1.xml
config/cyclonedds/pc_localhost.xml
```

原 eno1 / wlo1 为 30，现提升到 120。

---

## 11.2 新增 `pc_localhost.xml`

PC-only 模式正式使用：

```text
NetworkInterfaceAddress = lo
AllowMulticast = false
Peer = 127.0.0.1
ParticipantIndex = auto
MaxAutoParticipantIndex = 120
```

目标：

```text
rosvoice
→ PC-only
→ 不依赖 eno1 / wlo1
→ 不发现 Orin
→ 支持更多本机 ROS participant
```

---

# 12. `rosvoice / rosrobot / rosenv` 正式语义

## `rosvoice`

PC-only：

```text
ROS_DOMAIN_ID=23
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ROS_LOCALHOST_ONLY=1
ROS2CLI_DISABLE_DAEMON=1
CYCLONEDDS_URI=pc_localhost.xml
```

用途：

```text
Voice
Parser
PC-only Navigation
Fake navigation
Unit / integration regression
```

---

## `rosrobot`

PC ↔ Orin：

```text
ROS_DOMAIN_ID=23
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ROS_LOCALHOST_ONLY=0
ROS2CLI_DISABLE_DAEMON=1
```

自动选择：

```text
eno1 / 192.168.100.x
→ pc_camera_eno1.xml

否则：

wlo1 / 172.20.10.x
→ pc_camera_wlo1.xml
```

如果网络或 XML 不明确：

```text
ERROR
return 1
```

不回退 CycloneDDS 默认 discovery。

---

## `rosenv`

只查看当前环境，不改变模式。

---

# 13. `ros_env_pc.sh` Fail-Closed Hardening

环境脚本最终重构为：

```text
PHASE 1 — PREFLIGHT
↓
只检查
不 source
不 export
不 unset

全部通过
↓
PHASE 2 — COMMIT
↓
source ROS
source workspace
一次性 export 完整环境
```

### 解决的问题

此前：

```text
当前 shell = voice

执行 rosrobot
→ 中途设置 ROS_LOCALHOST_ONLY=0
→ 网络检查失败
→ CYCLONEDDS_URI 被 unset
→ return 1
```

可能留下：

```text
ROS_ENV_MODE=voice
ROS_LOCALHOST_ONLY=0
CYCLONEDDS_URI=<unset>
```

属于 inconsistent partial state。

现在：

```text
当前 shell = voice

rosrobot preflight failed
↓
return 1

当前 shell 仍完整保持 voice
```

---

# 14. Fail-Closed 验证结果

AGENT 使用 sentinel 做了非破坏性验证。

测试变量包括：

```text
ROS_ENV_MODE
ROS_DOMAIN_ID
RMW_IMPLEMENTATION
ROS_LOCALHOST_ONLY
CYCLONEDDS_URI
ROS2CLI_DISABLE_DAEMON
```

结果：

| Case | 结果 |
|---|---|
| `status` | PASS，环境无副作用 |
| invalid mode | PASS，失败后 sentinel 全部不变 |
| voice XML 缺失 | PASS，失败后环境不变 |
| robot XML 缺失 | PASS，失败后环境不变 |
| 无机器人网络 | PASS，失败后环境不变 |
| `rosvoice` success | PASS |
| `rosrobot` wlo1 success | PASS |

真实 `rosrobot` Wi-Fi 路径：

```text
mode=robot
domain=23
rmw=rmw_cyclonedds_cpp
localhost_only=0
ROS robot network: wlo1 / 172.20.10.x
CYCLONEDDS_URI=.../pc_camera_wlo1.xml
```

eno1 逻辑保留，但本轮没有实际有线环境验证。

---

# 15. Quick Start 收口

`docs/quick_start.md` 已重新整理启动规则。

主要明确：

1. `rosvoice` = PC-only。
2. `rosrobot` = PC ↔ Orin。
3. 每个 Terminal 都必须执行对应环境入口。
4. Voice-only 与 PC-only Navigation Integration 是两个不同层级。
5. 正式 Navigation semantic layer 使用：

```bash
ros2 launch place_manager navigation.launch.py \
  launch_done_sayer:=false
```

6. 使用 `navigation.launch.py` 后，不再额外启动：

```text
goto_place.launch.py
navigation_executor_node
```

7. 完整真实自然语言导航：

```text
Rebecca
+ Parser
+ navigation.launch.py
+ Orin nav_bringup
```

---

# 16. 当前验证矩阵

| 层级 | 状态 | 备注 |
|---|---|---|
| FunASR real microphone | ✅ PASS | 实际语音测试 |
| Rebecca / GLM | ✅ PASS | GLM online |
| CosyVoice | ✅ PASS | HTTP 200 / actual TTS |
| TTS self-echo isolation | ✅ PASS | interrupt-only 持续生效 |
| Navigation confirmation | ✅ PASS | 新导航 / Resume 均需确认 |
| Parser navigation intent | ✅ PASS | navigate / pause / resume |
| Voice → Parser → Executor | ✅ PASS | 实际链路 |
| `/goto_place` unavailable safety | ✅ PASS | FAILED，不产生运动 |
| Pause with no active navigation | ✅ PASS | `PAUSED place=""` |
| Resume with no paused target | ✅ PASS | `REJECTED / NO_PAUSED_NAVIGATION` |
| Resume confirmation timeout | ✅ PASS | stale confirm 无效 |
| DDS participant hardening | ✅ PASS | max=120 |
| `rosvoice` runtime env | ✅ PASS | localhost XML |
| `rosrobot` wlo1 env | ✅ PASS | 自动选择 Wi-Fi XML |
| ros_env fail-closed | ✅ PASS | sentinel 全通过 |
| Real `goto_place_node` → Nav2 OFF boundary | ⏳ 未完成 | 需启动真实 GotoPlace 后测试 |
| ACTIVE → PAUSED(place) → RESUME | ⏳ 未完成 | 建议 Fake `/goto_place` 验证 |
| Real Nav2 / Eric | ⏳ 未开始 | PC-only 完成后再进入实机 |

---

# 17. 尚未完成的核心测试

当前 Pause / Resume 功能还缺最重要的一层：

```text
navigate_to_place("餐厅")
↓
/goto_place ACTIVE
↓
停止导航
↓
/cancel_navigation accepted
↓
原 goto terminal
↓
PAUSED place="餐厅"
↓
恢复导航
↓
fresh confirmation
↓
resume_navigation
↓
新的 /goto_place("餐厅")
↓
新的 task_id
```

由于本轮 `/goto_place` Service 不存在，没有形成 active navigation，因此不能把当前结果标记为完整 Pause / Resume Integration PASS。

---

# 18. 下一步建议

下一阶段保持 PC-only，不启动 Eric：

### Layer D — Fake GotoPlace Integration

启动：

```text
Rebecca
Parser
NavigationExecutor
Fake /goto_place
Fake /cancel_navigation
```

Fake `/goto_place` 保持 active，直到收到 cancel。

验证：

```text
D1 navigate → ACTIVE
D2 pause → cancel accepted → PAUSED(place)
D3 repeated pause → idempotent PAUSED(place)
D4 resume → fresh confirm → new goto
D5 cancel → clear paused target
D6 resume after cancel → NO_PAUSED_NAVIGATION
```

Fake Integration 全部通过后，再进入：

```text
Real goto_place_node
→ Nav2 OFF boundary regression
→ Real Nav2 / Eric
```

---

# 19. 本轮结论

本轮没有发现新的机器人动作安全 blocker。

已经确认：

```text
Voice
→ Rebecca
→ Navigation confirmation
→ Parser
→ NavigationExecutor
```

真实链路成立。

同时：

```text
/goto_place 缺失
→ 明确失败
→ 不产生虚假导航状态

无 active navigation
→ Pause 幂等

无 paused target
→ Resume 拒绝

Resume
→ 必须 fresh confirmation

DDS participant exhaustion
→ 已通过统一 MaxAutoParticipantIndex=120 解决

rosvoice / rosrobot
→ 已形成统一、fail-closed 的正式 PC ROS 环境入口
```

当前不应继续重复 `/goto_place unavailable` 场景。

下一项有效测试是：

```text
ACTIVE → PAUSED(place) → RESUME
```

优先通过 Fake GotoPlace 在 PC-only 环境完成。
