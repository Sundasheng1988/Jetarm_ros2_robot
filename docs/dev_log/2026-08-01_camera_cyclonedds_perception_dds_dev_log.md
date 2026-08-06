# 2026-08-01 Camera Cyclone DDS Integration and Perception DDS Topology Dev Log

> 调查/修改模式：本日志整理 2026-08-01 当天已取得的证据、当前停止点与后续测试计划。
> Agent 在本会话中：未启动或停止任何 ROS 2 launch；未启动或停止 systemd service；未 SSH 到 Orin 执行任何命令；
> 未发布任何 ROS 2 消息；未修改 DDS / 视觉 / Runtime 源码；未构建；未执行 git add/commit/push/stash/reset/restore/clean。
>
> 证据等级标签：
> - **CONFIRMED FACT**：源码、进程环境、动态库、Git、日志或实测直接证明；
> - **DIRECT OBSERVATION**：用户在终端或 GUI 中直接观察到；
> - **INFERENCE**：基于证据的推断，必须明确标注；
> - **UNKNOWN / NOT YET TESTED**：尚未验证；
> - **REJECTED HYPOTHESIS**：已被实验证据否定。
>
> **范围声明（重要）**：Orin 侧全部事实（IP、Git 基线、提交、标签、service 状态、/proc 环境与动态库、30 Hz、停/启恢复）
> 来自用户当天在 Orin 设备上的实测与终端输出，本会话 Agent 未 SSH Orin 独立复核。
> PC 侧文件存在性、Git 状态、`cmp`/XML 语义/`bash -n`、源码拓扑等由 Agent 在本会话只读确认（CONFIRMED FACT）。
>
> 本日志**不**使用“所有 DDS 问题已解决 / Fast 与 Cyclone 一定不能互通 / 视觉闭环已完成 / Kinematics 已可用 / Tracker→Grounding 已打通”等无证据表述。

---

## 1. Executive Summary

今天完成的工作范围（按证据严格限定）：

- **初始问题**：Fast DDS 跨主机传输 Gemini 图像时出现周期性停顿，并导致 Orin 图像源端有效帧率下降；问题集中在跨主机 DDS 图像传输链路（根因未精确定位到某个 Fast DDS 内部模块）。
- **已验证方案**：使用 Cyclone DDS + 静态单播 Peer + 指定有线网卡后，图像链路恢复至约 30 Hz（DIRECT OBSERVATION）。
- **Orin 永久集成**：Orin 相机 Cyclone DDS 已永久集成进现有**单一** `start_app_node.service`（主栈仍 Fast DDS，相机走独立 Cyclone 子进程）。
- **PC 永久化**：PC 相机查看与四节点感知流水线已可通过 Cyclone DDS 启动脚本启动（CONFIRMED：文件/脚本就绪、静态检查通过；运行启动为 DIRECT OBSERVATION）。
- **未完成**：完整系统仍存在 **Cyclone 感知侧 ↔ Fast DDS Runtime/硬件服务侧** 的跨 DDS 边界；Kinematics 服务实际调用当前**未打通**；`stable_objects → Grounding/Verification` 尚未完成实测。
- **停止原因**：用户离开设备，无法继续 Orin/PC 联调；今日在该停止点结束。

当前总体状态：

| 子系统 | 状态 |
|--------|------|
| Camera transport | **VERIFIED** |
| Orin service integration | **VERIFIED** |
| PC image viewing | **VERIFIED** |
| PC perception startup | **VERIFIED** |
| Cross-DDS service/topic integration | **INCOMPLETE** |
| Full task execution loop | **NOT YET VERIFIED** |

---

## 2. Initial Problem

原始问题记录：

- Fast DDS 环境下，PC 订阅 Orin Gemini 图像后出现约 2～3 秒的周期性停顿（DIRECT OBSERVATION）。
- PC 订阅会导致 Orin 图像**源端有效帧率下降**（DIRECT OBSERVATION）。
- Fast DDS asynchronous publication 模式未解决问题（DIRECT OBSERVATION）。
- Orin 本地 JPEG 压缩本身正常（DIRECT OBSERVATION）。
- 问题集中在**跨主机 DDS 图像传输链路**。

> **INFERENCE 边界**：以上观察将问题定位在“跨主机 DDS 大消息传输链路”，但**未**将根因精确定位到某个 Fast DDS 内部模块。不写“根因 = 某 Fast DDS 子系统”。

---

## 3. Verified Cyclone DDS Experiment

已验证的 Cyclone DDS 配置（DIRECT OBSERVATION）：

**Orin**：
- IP：`192.168.100.1`
- 接口：`eth0`
- `ROS_DOMAIN_ID=23`
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- `AllowMulticast=false`
- Peer：`192.168.100.1`、`192.168.100.2`

**PC**：
- IP：`192.168.100.2`
- 接口：`eno1`
- `ROS_DOMAIN_ID=23`
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- 同样的静态单播 Peer（`192.168.100.1`、`192.168.100.2`）

验证结果（DIRECT OBSERVATION）：

- Orin `/depth_cam/rgb/image_raw` ≈ 30.1 Hz；
- PC `/depth_cam/rgb/image_raw` ≈ 30.1 Hz；
- `rqt_image_view` 正常显示；
- 连续收到数百帧；
- `NetworkInterfaceAddress` deprecated 仅是警告，**未**影响当前验证结果。

> 该验证使用的临时配置为 `/tmp/cyclonedds_eno1.xml`（非永久路径），今天已逐字固化到 PC 永久路径（见第 7 节）。

---

## 4. Orin Git Baseline

（以下为用户在 Orin 设备上的操作记录，DIRECT OBSERVATION；本会话 Agent 未 SSH 复核。）

**备份**：
- `/home/ubuntu/ros2_ws_pre_cyclone_20260801.tar.gz`
- SHA256：`e8983d3f549a05f93ef2b202ebf6fd9702b589eda25185e5defbb681c1331221`

**稳定分支**：
- `snapshot/orin-stable-pre-cyclone-20260801`

**稳定提交**：
- `7d56820` snapshot: preserve current mature JetArm runtime
- `fd096a9` snapshot: add mobile base and localization sources
- `3932cdf` snapshot: track RPLIDAR ROS 2 driver as submodule

**RPLIDAR submodule**：
- 路径：`src/rplidar_ros`
- commit：`24cc9b6dea97e045bda1408eaa867ce730fd3fc3`

**稳定标签**：
- `orin-stable-pre-cyclone-20260801`

**开发分支**：
- `feature/camera-cyclonedds`

> 说明：`smart_robot` 旧资源文件已删除，不进入基线。

---

## 5. Orin Permanent Implementation

（用户在 Orin 上的实现记录，DIRECT OBSERVATION。）

**修改文件**：
- `config/cyclonedds/orin_camera_eth0.xml`
- `src/bringup/launch/bringup.launch.py`

**架构**：

```
start_app_node.service
  → 主 bringup 继续使用原有 Fast DDS
  → 16 秒 TimerAction
  → ExecuteProcess 启动独立子进程：
       ros2 launch peripherals depth_camera.launch.py
  → 该子进程只覆盖：
       RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
       CYCLONEDDS_URI=file:///home/ubuntu/ros2_ws/config/cyclonedds/orin_camera_eth0.xml
```

**记录**：
- `camera_use_cyclone` 默认 `true`；
- `true`：使用 Cyclone 子进程；
- `false`：回退原 Fast DDS `IncludeLaunchDescription`；
- `IfCondition` / `UnlessCondition` 严格互斥；
- 保留原有 16 秒延迟；
- **未**创建第二个 systemd service；
- **未**修改 `start_app_node.service`；
- **未**修改相机 topic 名称或消息类型。

---

## 6. Orin Static and Runtime Verification

（用户在 Orin 上的验证记录，DIRECT OBSERVATION。）

**静态验证**：
- Python AST / `py_compile` 通过；
- XML namespace-aware 语义检查通过；
- `colcon build --packages-select bringup` 成功；
- `src` 与 `install` 的 `bringup.launch.py` 完全一致；
- Git 仅有两个目标文件改动。

**运行验证**：
- `start_app_node.service` 为 `active`；
- 只有一个 `depth_camera.launch.py`；
- 只有一个 `camera_container`；
- 相机进程实际环境：
  - `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
  - `CYCLONEDDS_URI=file:///home/ubuntu/ros2_ws/config/cyclonedds/orin_camera_eth0.xml`
  - `ROS_DOMAIN_ID=23`
  - `CAMERA_TYPE=GEMINI`
- 实际加载动态库：
  - `librmw_cyclonedds_cpp.so`
  - `libddsc.so.0.10.4`
- Orin RGB 约 30 Hz；
- 停止 service 后 PC 图像停止；
- 重新启动 service 后 PC 图像恢复；
- 子进程生命周期受主 service 管理。

**提交与标签**：
- 提交：`bc7f302 fix: isolate Gemini camera on Cyclone DDS`
- 验证标签：`orin-camera-cyclonedds-verified-20260801`
- Orin 工作树在提交后 **clean**。

---

## 7. PC Camera Configuration Persistence

PC 侧新建 / 修改文件（Agent 本会话只读确认存在性与静态属性：CONFIRMED FACT）：

- `config/cyclonedds/pc_camera_eno1.xml`（新建）
- `scripts/with_cyclone_camera.sh`（新建）
- `scripts/start_cyclone_camera_view.sh`（修改：改为调用通用启动器）
- `scripts/start_cyclone_perception.sh`（新建）
- `src/app/launch/perception_bringup.launch.py`（现有未提交 diff 增加 `start_grounding` 开关）

**记录（CONFIRMED FACT）**：
- `pc_camera_eno1.xml` 与已验证的 `/tmp/cyclonedds_eno1.xml` **字节一致**（`cmp` 通过，sha256 `5fd319e1a71cd968eb87ce28b386c87e11e3e7ff3a5d6f792c2b8f3a94b410a0` 一致）；
- XML namespace-aware 语义检查通过（Domain@id=any、NetworkInterfaceAddress=eno1、AllowMulticast=false、ParticipantIndex=auto、MaxAutoParticipantIndex=30、Peers=[192.168.100.1, 192.168.100.2]）；
- 三个脚本 `bash -n` 通过；
- 脚本具有执行权限（`775`）；
- PC `~/.bashrc` 默认仍为 `rmw_fastrtps_cpp`，**未**全局切换；
- Cyclone 仅通过启动脚本注入到特定进程；
- **未**在 Python/C++ 视觉算法代码中加入 Cyclone 专用 API。

**通用启动器 `with_cyclone_camera.sh` 职责**：注入

```
ROS_DOMAIN_ID=23
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CYCLONEDDS_URI=file:///home/sundasheng/ros2_ws/config/cyclonedds/pc_camera_eno1.xml
```

随后 `exec "$@"`（在子进程调用 `rclpy.init()` / `rclcpp::init()` 之前完成环境注入）。

---

## 8. PC Perception Topology Audit

源码确认的节点 DDS 归属（CONFIRMED FACT，来自源码 `create_subscription` / `create_publisher` / `declare_parameter` 与 launch 参数）：

**Cyclone DDS 感知侧**（由 `scripts/start_cyclone_perception.sh` 启动）：
- `simple_yolo_node`
- `roi_color_detector_node`
- `perception_fusion_node`
- `stable_object_tracker_node`

**Fast DDS Runtime 侧**（由 `ground_runtime_bringup.launch.py` 启动，继承 `~/.bashrc` 默认）：
- `grounding_node`
- `real_grounded_runtime_node`
- `verification_result_node`
- parser / runtime / executor 相关节点

**启动链（CONFIRMED FACT）**：

```
scripts/start_cyclone_perception.sh
  → scripts/with_cyclone_camera.sh
  → ros2 launch app perception_bringup.launch.py start_grounding:=false
```

**说明**：
- 四个感知节点是**独立进程**，但继承同一 Cyclone 环境；
- `simple_yolo_node` 与 `roi_color_detector_node` 直接订阅相机；
- `fusion` 和 `tracker` 不直接订阅视频，但处于同一内部 pipeline；
- `grounding` **不**由感知 Launch 启动；
- `grounding` 由 `ground_runtime_bringup.launch.py` **间接** `IncludeLaunchDescription` 引入 `grounding/launch/ground_bringup.launch.py` 启动（CONFIRMED FACT，`ground_runtime_bringup.launch.py:15-19,70-85`；`ground_bringup.launch.py:66-72` 声明 `grounding_node`）；
- `start_grounding:=false` 已实测生效（DIRECT OBSERVATION；源码层 `IfCondition(LaunchConfiguration('start_grounding'))` CONFIRMED FACT）。

**内部 Cyclone pipeline（同一 DDS，无跨边界）**：

```
simple_yolo_node          → /world_model/objects
roi_color_detector_node   → /world_model/roi_objects
perception_fusion_node    → /world_model/perception_objects
stable_object_tracker_node → /world_model/stable_objects
```

---

## 9. PC Perception Runtime Evidence

实际启动结果（DIRECT OBSERVATION，用户今天在 PC 上启动并观察）：

**启动节点**：
- `/simple_yolo_node`
- `/roi_color_detector_node`
- `/perception_fusion_node`
- `/stable_object_tracker_node`

**未启动**：
- `/grounding_node`（`start_grounding:=false` 生效）

**日志 / GUI 证据**：
- 四节点均读取持久化 `pc_camera_eno1.xml`；
- YOLOv8 模型成功加载：`/home/sundasheng/ros2_ws/src/vision_yolo/models/yolov8x.pt`；
- YOLO 窗口有实时图像与检测结果；
- ROI 窗口有实时图像；
- fusion 节点启动；
- tracker 节点启动；
- `start_grounding:=false` 生效。

> **证据边界（必须保留）**：
> - 四节点的**实际 `/proc` RMW 动态库检查尚未形成完整存档**（UNKNOWN / NOT YET TESTED）；
> - 启动日志明确来自 Cyclone 配置（DIRECT OBSERVATION），但**不**把未完成的动态库检查写成已完成。

---

## 10. use_dummy_wm Decision

`ground_runtime_bringup.launch.py` 默认（CONFIRMED FACT，`:54`）：

- `use_dummy_wm=true`

此时 Fast DDS 侧 `wm_dummy_pub`（`ground_bringup.launch.py:75-81`，`IfCondition(use_dummy_wm)`）会发布：

- `/world_model/objects`

这会与真实 Cyclone YOLO 的**同名发布者竞争**，并可能污染（INFERENCE）：

- `fusion`
- `tracker`
- `grounding`
- `verification`

**结论**：
- 独立 Runtime dummy dry-run 可以使用 `use_dummy_wm=true`；
- **真实相机感知闭环测试必须使用 `use_dummy_wm:=false`**。

> 这是**运行参数决策**，当前**未**修改默认源码或 Launch 默认值。

---

## 11. Complete DDS Boundary Table

至少包含以下正式边界（源码拓扑 CONFIRMED FACT；实际跨 DDS 消息收发 NOT YET TESTED）：

**正式边界（必须实测）**：

1. `simple_yolo_node`（Cyclone）→ `/kinematics/get_current_pose` → Orin Fast DDS 服务端
2. `stable_object_tracker_node`（Cyclone）→ `/world_model/stable_objects` → `grounding_node`（Fast）
3. `stable_object_tracker_node`（Cyclone）→ `/world_model/stable_objects` → `verification_result_node`（Fast）

**条件性边界**：

4. `simple_yolo_node`（Cyclone）→ `/vision_target` → `executor_node`（Fast，仅单独启动 executor 时）

**非正式演示节点**（`social_robot` 的 `env_scan_node` / `static_env_report_node` / `face_follow_node`）单独列出，**不**纳入当前主运行链。

> **互操作性原则（重要）**：Fast DDS 与 Cyclone DDS 底层都使用 DDS/RTPS，**理论上可能互操作**；是否可用必须按具体 Topic、Service、QoS 与发现配置**实测**，**不能仅根据 RMW 名称推断**成功或失败。本日志不预先判定上述边界为“必通”或“必断”。

---

## 12. Kinematics Service Failure

今天必须完整记录的未解决问题。

**源码与接口（CONFIRMED FACT）**：
- 服务：`/kinematics/get_current_pose`
- 类型：`kinematics_msgs/srv/GetRobotPose`
- 请求为空；
- 响应包含 `success`、`solution`、`geometry_msgs/Pose`。

**观察（DIRECT OBSERVATION）**：
- `simple_yolo_node` 启动后警告：`/kinematics/get_current_pose 暂不可用`（对应 `yolo_node.py:139-140`，2 s `wait_for_service` 超时仅 warn，graceful-degrade）；
- Cyclone 环境下 `ros2 service list` / `ros2 service type` **曾显示**服务名称和类型；
- 但实际 `ros2 service call ... "{}"` **一直等待** service available，未成功返回；
- timeout 结束时出现 `context invalid`，是终止后的清理提示，**不是根因**。

**重要判断（INFERENCE，明确标注）**：
- `service list` / `type` 结果**可能受 ROS 2 daemon 缓存影响**，不代表真实可调用；
- 实际 `service call` 没有成功；
- 当前应记录为：**“Cyclone → Fast DDS Kinematics 服务调用未打通”**；
- **不**写成“已确认 Fast/Cyclone 永远无法服务互操作”；
- 还缺少**无 daemon** 的 Fast/Cyclone 对照测试（UNKNOWN / NOT YET TESTED）。

---

## 13. stable_objects Cross-DDS Status

**发布端（CONFIRMED FACT）**：
- `stable_object_tracker_node`（Cyclone）
- topic：`/world_model/stable_objects`

**正式订阅端（CONFIRMED FACT）**：
- `grounding_node`（Fast）
- `verification_result_node`（Fast）

**当前状态**：
- 源码拓扑已确认（CONFIRMED FACT）；
- **真实跨 DDS 消息接收尚未测试**（NOT YET TESTED）；
- **不得**标记为通过或失败。

**后续需要**：
- Cyclone 侧确认 tracker 确实发布**真实对象**（非空）；
- Fast 侧使用**无 daemon** `ros2 topic echo` 接收；
- 确认 Grounding 和 Verification **真实收到**；
- 必须使用可识别物体与 `use_dummy_wm:=false`。

---

## 14. Git Status at Stop Point

**Orin**（用户记录，DIRECT OBSERVATION）：
- 修改已提交（`bc7f302`）；
- 标签已创建（`orin-camera-cyclonedds-verified-20260801`）；
- 工作树 clean；
- **尚未 push 远端**。

**PC**（Agent 本会话只读确认，CONFIRMED FACT）：
- 分支：`feature/sketch_runtime_sprint3`；
- 工作树原本已有**与 DDS 无关**的历史未提交改动；
- DDS 目标文件当前**尚未提交**；
- 最终 DDS 路径限定提交范围应包含 **5 个文件**：

  ```
  config/cyclonedds/pc_camera_eno1.xml      (?? untracked)
  scripts/with_cyclone_camera.sh            (?? untracked)
  scripts/start_cyclone_camera_view.sh      (?? untracked)
  scripts/start_cyclone_perception.sh       (?? untracked)
  src/app/launch/perception_bringup.launch.py ( M modified)
  ```

**说明**：
- `perception_bringup.launch.py` diff **仅**包含 `start_grounding` 参数及其 `IfCondition`（CONFIRMED FACT，见第 7 节）；
- **不**能使用 `git add .`、`git add -A` 或 `commit -a`；
- 当前**不**执行提交。

---

## 15. Confirmed / Unknown / Rejected

**CONFIRMED**：
- Orin 相机使用 Cyclone（env / 动态库 / 30 Hz）；
- Orin 本地约 30 Hz；
- PC 跨机约 30 Hz；
- rqt 正常；
- Orin service 生命周期正常（停/启恢复）；
- PC 永久 XML 有效（cmp / XML 语义）；
- 四节点感知 Launch 能够启动；
- YOLO 和 ROI 能够接收图像；
- `start_grounding=false` 生效；
- Kinematics 实际调用当前**未成功**。

**UNKNOWN / NOT YET TESTED**：
- Tracker 是否持续产生有效 `stable_objects`；
- Cyclone `stable_objects` 能否到 Fast Grounding；
- Cyclone `stable_objects` 能否到 Fast Verification；
- executor 启动时能否收到 `/vision_target`；
- Fast/Cyclone **无 daemon** 对照结果；
- 四节点 `/proc` RMW 动态库完整存档；
- 完整自然语言 → 感知 → grounding → runtime 闭环。

**REJECTED HYPOTHESES**：
- “永久化后 Orin 相机仍在使用 Fast DDS” —— 被 `/proc` 环境与动态库证据否定；
- “PC 无法收到永久化后的 Cyclone 图像” —— 被 30 Hz 与 rqt 否定；
- “必须修改 YOLO 业务源码才能使用 Cyclone” —— 启动环境继承已证明不需要；
- “整个 PC 必须全局切换 Cyclone” —— 当前采用**按进程注入**方案。

---

## 16. Current Stop Point

2026-08-01 结束时：

- Orin 相机与 PC 视频链路已稳定；
- PC 感知四节点已能运行；
- 跨 DDS 服务与 Runtime 侧消息边界**未完成**；
- 用户离开设备，**无法**继续 Orin/PC 联调；
- 今日**不再**进行任何运行测试；
- PC 侧文件**暂不提交**。

---

## 17. Next Session Checklist

按顺序执行（长时 launch / rqt / 现场观察由用户执行；Agent 仅给命令、读进程/日志、查 RMW 动态库、分析结果）：

1. 确认 Orin `start_app_node.service` active；
2. 启动 PC Cyclone 感知：`./scripts/start_cyclone_perception.sh`；
3. 检查 YOLO/Tracker 实际 PID 及 `/proc` 动态库；
4. 分别使用 `ROS2CLI_DISABLE_DAEMON=1` 做 Cyclone 与 Fast 节点/服务对照；
5. 使用 **Fast DDS 客户端**实际调用 Kinematics 服务；
6. 使用 **Cyclone 客户端**再次实际调用 Kinematics 服务；
7. 启动 Fast Runtime：`use_dummy_wm:=false`；
8. Cyclone 侧确认 `/world_model/stable_objects` 实际发布；
9. Fast 侧确认 Grounding 和 Verification 实际接收；
10. 条件性验证 `/vision_target → executor`；
11. 根据结果决定：保持当前边界 / 调整 DDS 配置 / 调整进程边界 / **最后才考虑 bridge**；
12. 全部验证通过后，**仅路径限定** `git add` 上述 5 个 PC 文件；
13. 审查 cached diff 后再 `commit`；
14. 最后决定是否 push Orin 与 PC 远端。

> **不**在报告中直接决定 bridge 方案。

---

## 18. Final Status Matrix

| 子系统 | 状态 | 证据 | 下一步 |
|--------|------|------|--------|
| Orin Cyclone camera | **VERIFIED** | env / maps / 30 Hz | 保持 |
| Orin service integration | **VERIFIED** | start / stop / restart | 保持 |
| PC persistent Cyclone config | **VERIFIED** | cmp / XML / rqt | 待提交 |
| PC YOLO/ROI video input | **VERIFIED** | GUI / log | 保持 |
| Fusion/Tracker startup | **VERIFIED** | node / log | 验证真实输出 |
| Kinematics service | **FAILED/UNRESOLVED** | call 等待 | 无 daemon 对照 |
| stable_objects → Grounding | **NOT TESTED** | 仅源码拓扑 | 实测 |
| stable_objects → Verification | **NOT TESTED** | 仅源码拓扑 | 实测 |
| Full task loop | **NOT VERIFIED** | — | 后续联调 |

> “FAILED/UNRESOLVED” **仅**用于当前 Kinematics 调用结果，**不**推广到所有 DDS 互操作。
