# Duplicate Control-Stack Audit — 2026-07-29

> 调查模式：READ-ONLY。全程未修改任何源码、launch、YAML、systemd 状态、build/install/log；未构建工作区；未重启或停止任何服务；未启动新的 ROS 2 launch；未发布任何 ROS 2 控制消息；未执行机械臂或底盘动作。调查未扩展到 Runtime、感知、Nav2 或其他问题。

## 1. Executive Summary

机械臂控制栈双实例的根因已确认：

**`bringup.launch.py` 通过两条独立启动路径各启动一次同一个 `jetarm_sdk.launch.py`** —— 一次直接 include，一次经 `joystick_control.launch.py` 间接且无条件再次 include。两次 `jetarm_sdk.launch.py` 调用各产出一套完整控制栈（ros_robot_controller + servo_controller 可执行文件[内含 controller_manager/servo_manager/buzzer_controller] + grasp + kinematics），叠加后形成 6 类节点各 ×2，与实测完全一致。

根因分类：**B — 两个独立启动路径分别启动同一个 JetArm SDK 控制栈**（同时伴随 A：同一 `jetarm_sdk.launch.py` 被重复 include）。

补充发现：`joystick_control.launch.py` 传入的 `disable_servo_control: True` 是**死参数** —— `joystick_control` 节点源码从未声明或读取该参数，节点构造时无条件创建 `/servo_controller` publisher。因此该参数既不能禁止 joystick 自身发布，更无法阻止被 include 的第二套 SDK 启动。

推荐修复：方案 B（为 `joystick_control.launch.py` 增加 `include_sdk` 参数，默认 true 保持独立兼容，`bringup.launch.py` 调用时传 false，bringup 保持直接启动唯一 `jetarm_sdk.launch.py`）。本次仅提出，不执行。

## 2. Scope and Safety Boundary

调查范围：systemd 入口 → Linux 进程树 → ROS 2 图 → launch include 树 → install/src 对照 → `disable_servo_control` 源码作用确认。

未做：源码/launch/YAML 修改、构建、服务启停、新 launch、ROS 2 消息发布、机械臂/底盘动作、Runtime/感知/Nav2 调试、C++ 源码审计（servo_controller 可执行文件内部结构以进程/节点计数相关推断，未读 C++ 源码）。

## 3. Documents Read

PC 侧（只读）：

- `docs/runtime_index.md`（全文）
- `CLAUDE.md`（项目规则）
- `docs/runtime_risks.md` — R31、R3
- `docs/topic_service_map.md` — 1.6 底层控制链路、机械臂 topic/service 表
- `docs/runtime_architecture.md` — 1. 系统总体架构图与部署边界

Jetson 侧（只读 launch 与节点源码）：

- `install/bringup/share/bringup/launch/bringup.launch.py`（实际运行版）
- `src/bringup/launch/bringup.launch.py`
- `src/driver/sdk/launch/jetarm_sdk.launch.py`
- `src/peripherals/launch/joystick_control.launch.py`
- `src/peripherals/peripherals/joystick_control.py`（节点源码，仅构造函数与发布逻辑区间）
- `src/app/launch/start_app.launch.py`
- `src/driver/servo_controller/launch/servo_controller.launch.py`
- `src/driver/ros_robot_controller/launch/ros_robot_controller.launch.py`
- `src/driver/kinematics/launch/kinematics_node.launch.py`

## 4. Host and Network Evidence

PC：

- `eno1`: `192.168.100.2/24`（网线直连 Jetson）
- `wlo1`: `192.168.3.14/24`
- `192.168.100.0/24` 走 `eno1`；`ping 192.168.100.1` 3/3 通，RTT ≈ 0.24–0.55 ms

Jetson（SSH `ubuntu@192.168.100.1`）：

- `hostname`: ubuntu，`pwd`: /home/ubuntu
- `eth0`: `192.168.100.1/24`（DDS 走 eth0 直连）
- `wlan0`: `192.168.149.1/24`；`l4tbr0`: `192.168.55.1/24`（linkdown）

历史地址 `172.20.10.2` 未使用。

## 5. systemd Entry Point

`/etc/systemd/system/start_app_node.service`（enabled, active/running, since 2026-07-27 23:05:57 CST）：

- `ExecStart`: `/bin/zsh -c 'source /home/ubuntu/.zshrc && ros2 launch bringup bringup.launch.py'`
- `MainPID`: 561（`ros2 launch bringup bringup.launch.py`）
- `User`: ubuntu，`KillMode`: mixed
- service Environment 仅含 `PULSE_SERVER`；`need_compile` 由 `.zshrc` 提供（观测到 launch 走 src 路径分支，故其值非 `'True'`）

关键：systemd 只启动**一条** launch 命令、**一个**主进程。不存在第二个 service。

## 6. Linux Process Evidence

`ps -eo pid,ppid,lstart,cmd --forest` 与 `pgrep -af` 显示：所有相关进程的 `PPID` 均为 **561**。可执行文件均来自 `install/`。按 PID 升序分为两组：

### 第 1 组（来自 bringup 直接 `sdk_launch` → jetarm_sdk.launch.py）

| PID  | executable (install path)                          | 对应 ROS 2 Node                                              |
|------|----------------------------------------------------|--------------------------------------------------------------|
| 1337 | ros_robot_controller/lib/ros_robot_controller/ros_robot_controller | /ros_robot_controller                              |
| 1339 | servo_controller/lib/servo_controller/servo_controller | /controller_manager + /servo_manager + /buzzer_controller（单进程多节点，推断）|
| 1341 | servo_controller/lib/servo_controller/grasp        | /grasp                                                       |
| 1343 | kinematics/lib/kinematics/search_kinematics_solutions | /kinematics                                               |

伴随 app 节点（来自 `start_app.launch.py`）：1345 lab_manager、1349 tag_stackup、1353 calibration、1357 finger_trace、1361 object_tracking、1365 object_sortting、1377 waste_classification；1383 web_video_server。

### 第 2 组（来自 `joystick_control.launch.py` → 再次 include jetarm_sdk.launch.py）

| PID  | executable (install path)                          | 对应 ROS 2 Node                                              |
|------|----------------------------------------------------|--------------------------------------------------------------|
| 1385 | ros_robot_controller/lib/ros_robot_controller/ros_robot_controller | /ros_robot_controller                              |
| 1387 | servo_controller/lib/servo_controller/servo_controller | /controller_manager + /servo_manager + /buzzer_controller   |
| 1394 | servo_controller/lib/servo_controller/grasp        | /grasp                                                       |
| 1402 | kinematics/lib/kinematics/search_kinematics_solutions | /kinematics                                               |
| 1404 | peripherals/lib/peripherals/joystick_control       | /joystick_control                                            |

后续：1406 `ros2 launch rosbridge_server ...` → 1823 rosbridge_websocket、1825 rosapi。

证据等级：直接观察。

## 7. ROS 2 Graph Evidence

`ros2 node list | sort | uniq -c`（第二次采样，首次因 DDS 发现延迟只显示部分重复）：

```text
2 /buzzer_controller
2 /controller_manager
2 /grasp
2 /kinematics
2 /ros_robot_controller
2 /servo_manager
1 /joystick_control
1 /launch_ros_561
1 /web_video_server
1 /rosbridge_websocket, /rosapi, /rosapi_params
1 各 app 节点（lab_config_manager, tag_stackup, calibration, finger_trace, object_tracking, object_sortting, waste_classification）
1 /depth_cam/depth_cam, /depth_cam/camera_container
```

Topic 端点：

- `/servo_controller`：Publisher count = **10**，Subscription count = **2**（2 个 controller_manager 订阅）
- `/ros_robot_controller/bus_servo/set_position`：Publisher count = **2**（2 个 `servo_manager`），Subscription count = **2**（2 个 `ros_robot_controller`）
- `/controller_manager/joint_states`：Publisher count = **2**

`ros2 service list` 中同名服务（如 `/kinematics/set_pose_target`、`/controller_manager/init_finish`、`/ros_robot_controller/init_finish`）仅列出 1 条 —— 这是 ROS 2 按服务名寻址、重复同名节点服务解析不确定的表现（与 R31 风险一致），而非单实例。

证据等级：直接观察。

## 8. Source Launch Tree

### 精确文件路径与行号

**`src/bringup/launch/bringup.launch.py`**（install 与 src IDENTICAL）：

- 第 22–25 行：直接 include `jetarm_sdk.launch.py`
  - L22 `sdk_launch = IncludeLaunchDescription(`
  - L24 `os.path.join(sdk_package_path, 'launch/jetarm_sdk.launch.py')),`
- 第 57–59 行：include `joystick_control.launch.py`
  - L57 `joystick_control_launch = IncludeLaunchDescription(`
  - L58 `PythonLaunchDescriptionSource(os.path.join(peripherals_package_path, 'launch/joystick_control.launch.py')),`
- 第 75–83 行 `return [...]` 列表中同时含 `sdk_launch`（L77）与 `joystick_control_launch`（L81）——两者都被执行，无任何条件互斥

**`src/peripherals/launch/joystick_control.launch.py`**（install 与 src IDENTICAL）：

- 第 17–20 行：**无条件** include `jetarm_sdk.launch.py`
  - L17 `sdk_launch = IncludeLaunchDescription(`
  - L19 `os.path.join(sdk_package_path, 'launch/jetarm_sdk.launch.py')),`
- 第 23–30 行：`joystick_control_node`，参数含 `disable_servo_control: True`
  - L23 `joystick_control_node = Node(`
  - L27 `parameters=[`
  - L29 `'disable_servo_control': True}`
- 第 33–36 行 `return [...]`：`[sdk_launch, joystick_control_node]` —— sdk_launch 与 joystick 节点都被执行，无 IfCondition

**`src/driver/sdk/launch/jetarm_sdk.launch.py`**（install 与 src IDENTICAL）：

- 第 24–26 行：include `ros_robot_controller.launch.py`
- 第 29–31 行：include `servo_controller.launch.py`
- 第 33–35 行：include `kinematics_node.launch.py`
- 第 42–46 行 `return [...]`：三者均无条件执行

**`src/driver/servo_controller/launch/servo_controller.launch.py`**：

- 启动 `servo_controller` 可执行文件（单进程内创建 controller_manager / servo_manager / buzzer_controller 节点）
- 启动 `grasp` 可执行文件

**`src/driver/ros_robot_controller/launch/ros_robot_controller.launch.py`**：启动 `ros_robot_controller` 可执行文件
**`src/driver/kinematics/launch/kinematics_node.launch.py`**：启动 `search_kinematics_solutions` 可执行文件

### 完整启动树

```text
start_app_node.service
└── ros2 launch bringup bringup.launch.py            (PID 561)
    ├── direct: jetarm_sdk.launch.py                 ★ 第 1 套
    │     ├── ros_robot_controller.launch.py  → ros_robot_controller          (PID 1337)
    │     ├── servo_controller.launch.py
    │     │     ├── servo_controller exec → controller_manager / servo_manager / buzzer_controller (PID 1339)
    │     │     └── grasp exec             → grasp                               (PID 1341)
    │     └── kinematics_node.launch.py     → kinematics (search_kinematics_solutions) (PID 1343)
    │
    ├── GroupAction
    │     ├── start_app.launch.py  → app 节点（lab_manager / tag_stackup / calibration /
    │     │                           finger_trace / object_tracking / object_sortting /
    │     │                           shape_recognition / waste_classification）
    │     └── TimerAction(16.0s) → depth_camera.launch.py
    ├── startup_check_node
    ├── web_video_server_node                                                            (PID 1383)
    │
    ├── joystick_control.launch.py
    │     ├── indirect: jetarm_sdk.launch.py          ★ 第 2 套（重复）
    │     │     ├── ros_robot_controller.launch.py  → ros_robot_controller     (PID 1385)
    │     │     ├── servo_controller.launch.py
    │     │     │     ├── servo_controller exec → controller_manager / servo_manager / buzzer (PID 1387)
    │     │     │     └── grasp exec             → grasp                        (PID 1394)
    │     │     └── kinematics_node.launch.py     → kinematics                   (PID 1402)
    │     └── joystick_control node (disable_servo_control:=True，死参数)         (PID 1404)
    │
    └── rosbridge_websocket_launch (ExecuteProcess)                                      (PID 1406)
```

PID/PPID 与节点数量对照：两条 jetarm_sdk 路径各产出 rrc ×1 + servo_controller exec ×1（→ controller_manager/servo_manager/buzzer_controller 各 ×1）+ grasp ×1 + kinematics ×1；两路径叠加 = 6 类节点各 ×2，与第 7 节 `ros2 node list` 完全一致。

## 9. Installed Launch Tree

systemd 运行 `ros2 launch bringup bringup.launch.py`，`bringup` 经 ament 索引解析到 `install/bringup/share/bringup/launch/bringup.launch.py`。因 `need_compile != 'True'`，bringup 内部将 `sdk_package_path` 等指向 **src** 路径 —— 即实际 launch 逻辑来自 src，Node 可执行文件来自 install。

install 与 src 对照（`diff`）：

- `bringup.launch.py`：IDENTICAL
- `joystick_control.launch.py`：IDENTICAL
- `jetarm_sdk.launch.py`：IDENTICAL

结论：install 与 src 无偏差。launch 内容来源为 src（因 `need_compile` 取值），但与 install 一致，不影响根因。

## 10. Duplicate Node and Process Mapping

| ROS 2 Node              | 实例 A (PID, 启动路径)                  | 实例 B (PID, 启动路径)                  | 重复来源 |
|-------------------------|-----------------------------------------|-----------------------------------------|----------|
| /ros_robot_controller   | 1337, 直接 sdk_launch                   | 1385, joystick_control→jetarm_sdk       | 是       |
| /controller_manager     | 1339 内, 直接 sdk_launch                | 1387 内, joystick_control→jetarm_sdk    | 是       |
| /servo_manager          | 1339 内, 直接 sdk_launch                | 1387 内, joystick_control→jetarm_sdk    | 是       |
| /buzzer_controller      | 1339 内, 直接 sdk_launch                | 1387 内, joystick_control→jetarm_sdk    | 是       |
| /grasp                  | 1341, 直接 sdk_launch                   | 1394, joystick_control→jetarm_sdk       | 是       |
| /kinematics             | 1343, 直接 sdk_launch                   | 1402, joystick_control→jetarm_sdk       | 是       |

`/ros_robot_controller/bus_servo/set_position`：2 publisher（servo_manager ×2）+ 2 subscription（ros_robot_controller ×2），确认存在两套硬件写入栈。

## 11. disable_servo_control Analysis

源码证据（`src/peripherals/peripherals/joystick_control.py`，309 行）：

- 第 59 行：`self.servos_pub = self.create_publisher(ServosPosition, '/servo_controller', 1)` —— 构造函数中**无条件**创建 `/servo_controller` publisher
- 全文件 `grep -nE "declare_parameter|get_parameter|disable_servo_control"`：**无任何命中**（`disable_servo_control` 仅出现在 launch 文件 L29，节点源码从未声明或读取该参数）
- 第 107/112/118/124/130/136/140/144/148/183/190/196/202 行：多处 `bus_servo_control.set_servo_position(self.servos_pub, ...)` 在 joystick 输入处理中发布舵机命令

结论（Confirmed Fact）：

1. `disable_servo_control: True` 是**死参数**，`joystick_control` 节点完全不消费它；
2. 该参数**不能**禁止 joystick_control 节点自身向 `/servo_controller` 发布（节点构造即创建 publisher，joystick 事件即发布）；
3. 该参数更**不能**阻止 `joystick_control.launch.py` 无条件 include `jetarm_sdk.launch.py`（include 是 launch 层行为，与节点参数无关）；
4. 因此第二套 `ros_robot_controller / servo_controller executable / controller_manager / servo_manager / buzzer_controller / grasp / kinematics` 仍被完整启动。

即任务第二节判断成立，且比预期更强：`disable_servo_control` 连 joystick 自身发布都未禁用。

## 12. Confirmed Root Cause

**`bringup.launch.py` 直接启动一次 `jetarm_sdk.launch.py`，同时通过 `joystick_control.launch.py` 间接且无条件再次启动一次 `jetarm_sdk.launch.py`。**

两次 `jetarm_sdk.launch.py` 调用各产出一套完整机械臂控制栈，叠加形成 6 类节点各 ×2、`bus_servo/set_position` 双发布者双订阅者的重复运行状态。`disable_servo_control: True` 是死参数，对重复启动无任何抑制效果。

## 13. Root-Cause Classification

**B — 两个独立启动路径分别启动同一个 JetArm SDK 控制栈。**

（伴随 A：同一 `jetarm_sdk.launch.py` 被重复 include。B 是主导分类，A 是其机制表现。）

## 14. Confirmed Facts

1. systemd 只启动一条命令 `ros2 launch bringup bringup.launch.py`，MainPID 561（第 5 节）。
2. 所有重复控制栈进程 PPID 均为 561，无第二棵进程树（第 6 节，`ps --forest` 直接观察）。
3. `bringup.launch.py` 同时 include `jetarm_sdk.launch.py`（L22–25）与 `joystick_control.launch.py`（L57–59），二者在 return 列表中均无条件执行（L77, L81）。
4. `joystick_control.launch.py` 无条件 include `jetarm_sdk.launch.py`（L17–20，无 IfCondition，L33–36 return 中含 sdk_launch）。
5. 两次 jetarm_sdk 调用产出 6 类节点各 ×2，与 `ros2 node list` 一致（第 7、10 节）。
6. `bus_servo/set_position` 有 2 publisher（servo_manager）与 2 subscription（ros_robot_controller），两套硬件写入栈并存（第 7 节）。
7. install 与 src 三个关键 launch 文件 IDENTICAL（第 9 节，`diff` 证据）。
8. `disable_servo_control: True` 是死参数，节点源码未声明/读取，不影响 include 与自身发布（第 11 节，`grep` 与源码直接观察）。

## 15. Direct Observations

- 进程 PID/PPID/exec 路径（第 6 节 `ps`/`pgrep` 输出）。
- `ros2 node list`、`ros2 topic info -v` 端点计数与节点名（第 7 节）。
- launch 文件 include 语句原文与行号（第 8 节）。
- `joystick_control.py` L59 publisher 创建与多处 `set_servo_position` 发布（第 11 节）。
- 首次 `ros2 node list` 采样因 DDS 发现延迟仅显示 `/ros_robot_controller ×2`，第二次显示全部 ×2 —— 延迟本身为观察现象，不改变结论。

## 16. Inferences

- `servo_controller` 单可执行文件内部创建 controller_manager / servo_manager / buzzer_controller 多个 Node。证据：2 个 servo_controller 进程（PID 1339、1387）精确对应 2 个 controller_manager + 2 个 servo_manager + 2 个 buzzer_controller；未读其 C++ 源码（超出本次边界），故标记为推断（强相关）。
- `need_compile` 值非 `'True'`：基于 launch 走 src 路径分支的观测推断，未读 `.zshrc`。不影响根因。
- 修复方向：消除 `jetarm_sdk.launch.py` 的二次 include。具体方案见第 19、20 节，需人工批准。

## 17. Rejected Hypotheses

1. **第二个 systemd 服务启动第二套控制栈** — Rejected。证据：`systemctl status` 只有一个 `start_app_node.service`；`ps --forest` 显示所有进程 PPID 均为 561，无独立 service 子树。
2. **用户手动启动与 systemd 同时存在** — Rejected。证据：所有控制栈进程 PPID 链均回到 561（systemd 主进程），无 PPID 为 shell/用户会话的独立控制栈进程。
3. **`start_app.launch.py` 再次启动 SDK** — Rejected。证据：已完整读取 `start_app.launch.py`，其只 include 8 个 app 子 launch（lab_manager/calibration/object_tracking/finger_trace/object_sortting/tag_stackup/shape_recognition/waste_classification），无任何 sdk/servo_controller/ros_robot_controller/kinematics include。
4. **单一可执行进程内部意外创建两套完整控制栈** — Rejected。证据：重复来自两个不同 PID 的可执行文件（如 ros_robot_controller PID 1337 与 1385、servo_controller PID 1339 与 1387），非单进程多节点。每套各由独立 jetarm_sdk 调用产出。
5. **install 与 src 的 launch 内容不一致导致运行旧逻辑** — Rejected。证据：`diff` 显示 bringup/joystick_control/jetarm_sdk 三个 launch 文件 install 与 src IDENTICAL。
6. **ROS 2 DDS 仅仅虚假显示了重复节点** — Rejected。证据：Linux 进程层有真实对应的两个可执行文件进程（PID 1337/1385 等）与两条独立启动路径，非 DDS 幻象。DDS 发现延迟仅影响首次采样的显示完整性，第二次采样与进程证据一致。

> 说明：以上否定均基于直接证据（systemctl/ps/launch 源码/diff），非"未搜索到即否定"。

## 18. Unknowns

- `servo_controller` 可执行文件内部确切的 Node 创建结构未读 C++ 源码确认（第 16 节推断）。本次不扩展到 C++ 源码审计。
- `.zshrc` 中 `need_compile` 的具体取值未读取（推断非 `'True'`）。不影响根因。
- `shape_recognition` 在 `start_app.launch.py` 中被 include 但进程树未见对应进程，是否启动失败未调查（与重复控制栈根因无关）。
- 第二套 SDK 中 `ros_robot_controller`（PID 1385）是否成功打开 `/dev/ttyUSB0`（串口可能被第一套独占）未核实；即便其串口打开失败，ROS 2 Node 与 topic endpoint 仍已注册并计入重复。本次不触碰设备节点，不调查串口占用。
- 修复后 `/servo_controller` 的 Publisher count 具体下降到何值未预测（joystick_control、grasp、各 app demo 节点仍持有 publisher），属后续"多控制源最小化与仲裁"问题。

## 19. Repair Options

> 仅提出方案，不执行修改。两方案均需人工批准后另起任务执行。

### 方案 A：从 `bringup.launch.py` 删除直接的 `jetarm_sdk.launch.py` include

由 `joystick_control.launch.py` 间接提供唯一 SDK 栈。

- 优点：bringup 改动小（删 sdk_launch 一项）；joystick_control.launch.py 不动，保持原厂独立运行兼容。
- 缺点：SDK 栈的启动所有权隐藏在 joystick_control.launch.py 内部，启动来源不清晰；SDK 生命周期与 joystick 绑定（停 joystick 即停 SDK）；若后续不再需要 joystick，SDK 也会丢失；app 节点与 depth_camera 的 16s TimerAction 依赖 SDK 就绪，需重新评估时序。

### 方案 B：为 `joystick_control.launch.py` 增加 `include_sdk`（或 `start_sdk`）参数

- 默认 `true`，保持 `joystick_control.launch.py` 独立启动时的兼容性（独立运行仍自带 SDK）；
- `bringup.launch.py` 调用 `joystick_control.launch.py` 时传 `include_sdk:=false`；
- `bringup.launch.py` 保持直接启动唯一 `jetarm_sdk.launch.py`。

### 比较

| 维度 | 方案 A | 方案 B |
|------|--------|--------|
| 启动所有权清晰度 | 差（SDK 藏在 joystick 内） | 好（bringup 直接持有唯一 SDK） |
| joystick 与硬件驱动生命周期解耦 | 差（绑定） | 好（解耦） |
| 独立启动兼容性 | 好（joystick 不变） | 好（默认 true） |
| 对原厂代码侵入 | 小（仅改 bringup） | 中（改 joystick_control.launch.py + bringup 传参） |
| 后续最小机械臂 bringup | 难（需先理顺 joystick 依赖） | 易（bringup 直接控制 SDK 启停） |
| 后续 AUTO/MANUAL 控制仲裁 | 不利（SDK 归属混乱） | 有利（SDK 启动入口单一明确） |
| 风险 | 中（时序/依赖回归） | 低（行为可参数化、可回退） |
| 回退方式 | 恢复 bringup 的 sdk_launch | include_sdk 默认 true，传 false 改回 true |

## 20. Recommended Minimum Change

**推荐方案 B。**

理由：启动所有权清晰、joystick 与硬件驱动生命周期解耦、独立运行兼容、对后续控制仲裁友好、可参数化回退。`bringup.launch.py` 保持唯一且直接的 `jetarm_sdk.launch.py` 启动入口，符合"单一硬件写入入口"的安全方向。

实现要点（待人工批准）：

1. `joystick_control.launch.py`：为 `sdk_launch` 的 IncludeLaunchDescription 增加 `condition=IfCondition(LaunchConfiguration('include_sdk', default='true'))`，并 `DeclareLaunchArgument('include_sdk', default_value='true')`。
2. `bringup.launch.py`：调用 joystick_control.launch.py 时传入 `include_sdk='false'`（通过 `launch_arguments`）。
3. 保持 `bringup.launch.py` 直接的 `sdk_launch` 不变。
4. 不改 `disable_servo_control`（死参数，可在后续清理中移除，但与本次根因修复无关）。

## 21. Post-Fix Acceptance Criteria

> 报告写入，本次不执行。

1. 核心节点唯一性：

```text
ros_robot_controller ×1
controller_manager     ×1
servo_manager          ×1
buzzer_controller      ×1
grasp                  ×1
kinematics             ×1
```

2. 底层总线 `/ros_robot_controller/bus_servo/set_position`：

```text
Publisher count: 1
Subscription count: 1
```

3. 上层控制入口 `/servo_controller`：期望 `controller_manager` Subscriber count = 1。

> Publisher count 可能仍 >1，因为原厂 demo、grasp、joystick_control 或其他应用节点可能持有 Publisher（本次确认 joystick_control 第 59 行无条件创建 publisher）。这属于后续"多控制源启动最小化与仲裁"问题，**不得将 Publisher count >1 误判为本次重复控制栈修复失败**。

4. 进程证据：每个底层 executable 只存在一个进程；所有进程属于预期的单一 systemd / launch 树（PID 561 子树）。

## 22. Commands Executed

PC：`ip -4 addr` / `ip route` / `ping -c 3 192.168.100.1`；`rg` / `sed -n` 读取 PC 文档区间。

Jetson（经 SSH，全部只读）：`hostname`/`pwd`/`ip -4 addr`/`ip route`；`systemctl status|cat|show start_app_node.service`；`pgrep -af`/`ps -eo pid,ppid,lstart,cmd --forest`；`ros2 node list | sort | uniq -c`（×2）；`ros2 topic info -v /servo_controller`、`/ros_robot_controller/bus_servo/set_position`、`/controller_manager/joint_states`；`ros2 service list | grep`；`find ... -name '*.launch.py'`；`grep -nE`/`sed -n`/`cat` 读取 launch 与节点源码区间；`diff` install vs src。

未执行：任何 `kill`/`pkill`/`systemctl start|stop|restart`/`ros2 launch`/`ros2 topic pub`/构建/`rm`/设备节点访问。

## 23. Files Created or Modified

- 创建（PC）：`docs/dev_log/duplicate_control_stack_audit_20260729.md`（本报告）
- 未修改任何 Jetson 源码、launch、YAML、systemd、build/install/log。
- 未在 Jetson 写入任何文件。

---

**声明**：本次调查全程 READ-ONLY。未修改源码、未修改 launch、未修改 YAML、未重启或停止任何服务、未启动新 launch、未发布任何 ROS 2 控制消息、未执行机械臂或底盘动作。任务到此停止，不执行修复，不自动进入下一阶段。

---

## 24. Buzzer Controller Duplicate Follow-up — 2026-07-30

> 只读补充调查。前置已确认项无需重查：ros_robot_controller/controller_manager/servo_manager/grasp/kinematics 各 ×1，`bus_servo/set_position` 1 Publisher / 1 Subscriber。本节仅针对残留的 `/buzzer_controller ×2` 与 `/ros_robot_controller/set_buzzer` 2 Publisher / 1 Subscriber。

### 24.1 Runtime Evidence

`ros2 topic info -v /ros_robot_controller/set_buzzer`：

```text
Publisher count: 2
  Node name: buzzer_controller   (namespace /, PUBLISHER)
  Node name: buzzer_controller   (namespace /, PUBLISHER)
Subscription count: 1
  Node name: ros_robot_controller (namespace /, SUBSCRIPTION)
```

`ros2 node list` 中 `/buzzer_controller ×2`。

进程证据（`pgrep -af 'finger_trace|joystick_control|buzzer_controller'`）：

```text
PID 6035  /home/ubuntu/ros2_ws/install/app/lib/app/finger_trace --ros-args
PID 6056  /home/ubuntu/ros2_ws/install/peripherals/lib/peripherals/joystick_control --ros-args ...
```

不存在独立的 `buzzer_controller` 可执行进程。两个 `/buzzer_controller` 节点均为辅助子节点，分别寄生于 `finger_trace` 与 `joystick_control` 进程内部。

### 24.2 Source Files and Line Numbers

公共工具类 `src/driver/sdk/sdk/buzzer.py`（全文 35 行）：

- L6 `class BuzzerController(Node):`
- L8 `def __init__(self, name='buzzer_controller'):` —— 默认节点名 `buzzer_controller`
- L9 `super().__init__(name)` —— 创建名为 `buzzer_controller` 的 ROS 2 Node
- L10 `self.pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 1)` —— 发布到 `/ros_robot_controller/set_buzzer`
- L12–18 `set_buzzer(...)` —— 构造并 `self.pub.publish(msg)`

实例化点（两处均以**默认参数**调用，故节点名均为 `buzzer_controller`）：

- `src/app/app/finger_trace.py`
  - L13 `import sdk.buzzer as buzzer`
  - L137 `self.buzzer = buzzer.BuzzerController()` —— 在 `finger_trace` 进程内创建第 1 个 `buzzer_controller` 节点
  - L242 `self.buzzer.set_buzzer(500, 0.1, 0.5, 1)`（L240 `buzzer_task`，L290 线程启动）
- `src/peripherals/peripherals/joystick_control.py`
  - L17 `from sdk import common, buzzer`
  - L57 `self.buzzer_pub = buzzer.BuzzerController()` —— 在 `joystick_control` 进程内创建第 2 个 `buzzer_controller` 节点
  - L210 / L213 `self.buzzer_pub.set_buzzer(1000, ...)`

### 24.3 Confirmed Root Cause

两个 `/buzzer_controller` 节点由**同一个公共工具类** `sdk/buzzer.py:BuzzerController` 创建：

- `finger_trace` 进程（PID 6035）在 `finger_trace.py:137` 实例化 `BuzzerController()`，创建第 1 个 `buzzer_controller` 节点；
- `joystick_control` 进程（PID 6056）在 `joystick_control.py:57` 实例化 `BuzzerController()`，创建第 2 个 `buzzer_controller` 节点。

`BuzzerController.__init__` 默认 `name='buzzer_controller'`，两处调用均未传入自定义名称，故产生两个同名节点。两者都在构造时（L10）创建到 `/ros_robot_controller/set_buzzer` 的 publisher，叠加为 2 Publisher；唯一订阅者为硬件驱动 `ros_robot_controller`（1 Subscriber）。

这与第 12 节的 jetarm_sdk 双 include 是**不同的重复模式**：此处不是 launch 重复 include，而是两个独立应用节点各自把同一个公共 buzzer 工具类当作子节点实例化，造成同名辅助节点重复。任务前置给出的 GID 疑似（finger_trace / joystick_control）已由源码确认，非仅 GID 推断。

### 24.4 Recommended Handling

> 仅提出，不执行。三种方向比较：

| 方向 | 说明 | 评价 |
|------|------|------|
| A. 给两个辅助 Node 使用不同名称 | 调用 `BuzzerController(name='finger_trace_buzzer')` / `(name='joystick_buzzer')` | 消除同名，但仍有两个 buzzer publisher 并存，未解决多写者；命名需逐调用点维护 |
| B. 由 finger_trace / joystick_control 自身直接创建 buzzer Publisher | 不再实例化 `BuzzerController` 子节点，而在各自节点内 `create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 1)` | 去掉寄生子节点，节点图更干净；但仍多 publisher，且各应用节点各自重复 publisher 逻辑 |
| C. 后续建立唯一 buzzer_controller | 由 bringup 启动单一 `buzzer_controller` 节点，提供 buzzer 服务/话题，应用节点改为请求而非各自发布 | 与 R3 控制仲裁方向一致，长期最干净；改动较大，属后续"多控制源最小化与仲裁"范畴 |

推荐：短期可先 A（最小改动消除同名节点歧义），长期随控制仲裁推进 C。B 为中间态可选。本节不执行任何修改。

### 24.5 Unknowns

- 两个 `buzzer_controller` publisher 的 GID 与进程的精确对应未通过 DDS GID↔PID 映射逐一绑定；但源码确认仅 `finger_trace` 与 `joystick_control` 两进程实例化 `BuzzerController()`，且 `shape_recognition.py:88` 虽直接创建 `/ros_robot_controller/set_buzzer` publisher，但 (a) 其从 `shape_recognition` 节点自身发布、不创建 `buzzer_controller` 子节点，(b) `shape_recognition` 当前未运行（不在 pgrep 与进程树中），故不构成当前 2 个 `buzzer_controller` 节点的来源。综合排除法，两节点归属已确认。
- `shape_recognition` 若未来被启动，会向 `/ros_robot_controller/set_buzzer` 再增加第 3 个 publisher（非 `buzzer_controller` 同名节点），属潜在附加多写源，留待后续多控制源治理。
- 未读 `BuzzerController` 是否在其他未审计的包中被实例化（本次仅扫 app/peripherals，符合任务边界）。

### 24.6 本节声明

本次补充调查全程 READ-ONLY：未修改 Jetson 源码、未构建、未重启或停止服务、未发布任何 ROS 2 消息、未操作蜂鸣器、手柄或机械臂。仅执行 `pgrep`、`grep`、`cat -n`/`sed -n`、`ros2 topic info -v`（只读）。未继续调查 `/servo_controller` 的其他 Publisher。
