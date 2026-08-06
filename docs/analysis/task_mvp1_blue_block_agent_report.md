# Task MVP 1 Agent Report

> 日期：2026-08-02 ｜ 分支：feature/sketch_runtime_sprint3
> 范围：只读源码与文档审查。未运行 ROS、未操作机械臂、未修改任何生产代码、未执行 Git 命令。
> 证据等级：CF = Confirmed Fact（源码/文档直接佐证）｜ INF = Inference ｜ UNK = Unknown（PC 源码不可见或需实机观察）。
> 主线参考：`docs/runtime_index.md`、`docs/topic_service_map.md`、`docs/runtime_debug_guide.md`、`docs/analysis/vision_agent_checkpoint.md`。

---

## Current Goal

Task MVP 1：在固定工作区内识别、抓取并收纳**蓝色积木**。本轮只回答两个问题：

一、蓝色积木的视觉与抓取能力现状（ROI / YOLO / Fusion / Stable World Model）。
二、机械臂为什么不能实时反馈末端状态（`/controller_manager/joint_states`、`/controller_manager/servo_states`、`/kinematics/get_current_pose`）。

目标不是重新设计视觉架构，也不是讨论 Depth 方案（见 `vision_agent_checkpoint.md` 的 gate 约束），而是定位“当前代码已经具备什么、抓取位姿还差什么、末端反馈为何不实时”。

---

## Blue Block Existing Capability

### 结论：蓝色积木只能由 ROI 节点识别

| 节点 | 能否识别蓝色积木 | 证据 |
|---|---|---|
| `roi_color_detector_node`（`RoiColorDetectorNode`） | **能** | 默认 `color_keys=['red','blue']`（`roi_color_detector_node.py:89`）；`lab_config.yaml:13` 含 `blue` 范围；按色 mask 后做形状分类 |
| `simple_yolo_node`（`AppCompatibleYoloNode`） | **不能** | YOLO 类白名单无 block/cube（`yolo_node.py:82-83`，仅 sports ball/cup/bottle/...）；其兜底 `_find_cubes_topdown` 被 `only_red` 门控为**只找红色**（`yolo_node.py:76` `fallback_only_red=True` + `yolo_node.py:421`） |

→ 因此蓝色积木进入 World Model 时只能走 **roi_only** 路径，YOLO 对蓝色积木无贡献。CF。

### ROI 对蓝色积木已经计算的全部几何量

在 `roi_color_detector_node.py` 的 `on_rgb()` 中，每个颜色先做 LAB `inRange` 得到 mask，再做形态学开闭，再 `findContours`，逐轮廓计算：

| 量 | 是否计算 | 位置 | 是否发布 |
|---|---|---|---|
| mask / 二值图 | 是（每色一份，瞬时） | `roi_color_detector_node.py:332` | **否**（仅用于内部检测） |
| contour | 是 | `:337` `cv2.findContours` | **否** |
| centroid | 是（取 `minAreaRect` 中心 `cx,cy`） | `:343-347` | 是（`DetectionResult.center_x/y` `:400-401`；并经 `_pix_to_world` 转成世界 xyz） |
| `minAreaRect`（含 `theta`） | 是 | `:343` | **否** |
| 长宽（`w,h`） | 是 | `:344` | **否**（仅用于过滤与形状判定） |
| 方向 / yaw | 是（box 角点投到白区平面算 `yaw_world`） | `:386-397` | 是（`objects[].pose.rpy` `:412`；并转四元数进 PoseArray/PoseStamped） |
| 颜色 | 是（`_dominant_color_key`） | `:234-243`,`:378` | 是（`objects[].color` `:412`） |

ROI 发布的全部 topic（`roi_color_detector_node.py:134-138`）：

```text
/roi_color_detector/image_result   Image            调试图（含 box/centroid 叠加）
/roi_vision_target                 DetectionResult  class_name/conf/center_x/y/z（无 color、无 yaw、无 w/h）
/world_model/roi_objects           String(JSON)     id/class_name/color/pose{frame,xyz,rpy}/confidence
/roi_objects/poses                 PoseArray        pose(position+orientation 四元数，yaw 已烤进四元数)
/roi_target_pose                   PoseStamped      置信度最高的一条 pose
```

关键结论：**centroid/xyz、yaw、color 已发布；mask、contour、minAreaRect、长宽 w/h 只在节点内部存在，不下发。** CF。

### 形状分类对“积木”的潜在过滤（需实机确认）

`roi_color_detector_node.py:354-376` 的分类逻辑：

- `right_cnt >= 3 and 0.70 <= ar <= 1.35` → `cube`（`:359`）
- 否则按 circularity 走 `cup` / `ball` / `cylinder`，都不命中则 `target_cls=None` → `continue`（`:375-376`，**直接丢弃**）

含义：只有**接近正方形**（长宽比 0.70–1.35）的蓝色块会被标成 `cube`。若是长条形蓝色积木（长宽比 > 1.35），即使颜色命中也会被丢弃。YOLO 的兜底 cube 检测同样有 `0.75 <= ar <= 1.25` 的门控（`yolo_node.py:457`），且只对红色生效。

→ INF：若实际蓝色积木是长条形，当前两条路径都可能漏检。实际形状 UNK，需实机观察 `/world_model/roi_objects` 是否输出该块。

---

## Current Perception and World Model Flow

数据流（CF，源码 `perception_fusion_node.py`、`perception_fusion_utils.py`、`stable_object_tracker_node.py`）：

```text
/depth_cam/rgb/image_raw ──► simple_yolo_node   ──► /world_model/objects      (yolo)
                        └──► roi_color_detector ──► /world_model/roi_objects   (roi)

/world_model/objects + /world_model/roi_objects
   └► perception_fusion_node ──► /world_model/perception_objects
        └► stable_object_tracker_node ──► /world_model/stable_objects
             └► grounding_node ──► /grounded_task_context
```

### Fusion 用谁的坐标？（CF，`perception_fusion_utils.py`）

| 分支 | class_name | color | pose.xyz/rpy | 证据 |
|---|---|---|---|---|
| `yolo_roi_fused`（位置匹配 < 0.06m） | YOLO | ROI | **ROI** | `build_fused_object`：`pose=roi_entry`（`:125`）、`color=roi_entry`（`:124`） |
| `roi_only`（YOLO 未检出） | ROI | ROI | **ROI** | `build_roi_only_object`（`:135-144`） |
| `yolo_only`（ROI 未检出） | YOLO | `unknown` | YOLO | `build_yolo_only_object`（`:147-156`，color 固定 `unknown`） |

### World Model 最终用 YOLO 还是 ROI 坐标？

- **蓝色积木**：YOLO 无法检出 → 走 `roi_only` → World Model 用 **ROI 坐标**（xyz + yaw + color 全部来自 ROI）。CF。
- **杯子（2026-08-02 已抓）**：当时 Stable 为 `yolo_only`、`color=unknown`（见 `topic_service_map.md` §3.5 实测）→ 用 **YOLO 坐标**。这也解释了“拿起蓝色杯子”会 `no_match`（color 为 unknown，见 `runtime_debug_guide.md` §11.1）。
- **匹配成功时**：用 ROI 坐标（`build_fused_object` 取 `roi_entry.pose`）。

两条坐标链差异（CF）：

- ROI 坐标链是**静态**的：用 `transform.yaml` 的 `white_area_pose_cam`（相机固定位姿假设）做平面投影（`roi_color_detector_node.py:164-175`,`245-274`）。
- YOLO 坐标链是**动态**的：每 5 Hz 调 `/kinematics/get_current_pose` 取当前末端位姿，与 `hand2cam` 合成相机位姿再投影（`yolo_node.py:138`,`202-222`,`236-244`）。
- → INF：因为 YOLO 用动态链、ROI 用静态链，两者投影基准不同；这正是 `vision_agent_checkpoint.md` 所指“统一权威坐标链”待解决的根因之一。但本报告不展开 Depth/统一链方案。

---

## Missing Grasp Information

当前蓝色积木（roi_only）已经带到 World Model 的信息：**物体中心 xyz + yaw + color**。

要生成“任务级抓取位姿”，最少还缺（按从近到远）：

1. **yaw 无法下达给机械臂（最关键）**
   - `RuntimeAdapter.ik_solve()` 接收 `rpy` 参数，但 `_call_ik_blocking` 构造 `SetRobotPose.Request` 时**只填 position/pitch/pitch_range/resolution，完全丢弃 rpy**（`runtime_adapter.py:185-189`）。
   - 且 `/kinematics/set_pose_target` 的请求接口本身**没有 rpy/yaw 字段**（`topic_service_map.md` §11.1：request = `position[]/pitch/pitch_range/resolution`）。
   - → 即使 ROI 算出了 yaw，当前运动链无法据此设定抓取朝向；末端朝向只能由 `pitch`（俯仰）单自由度控制。这与历史现象“Grounding rpy=[0,0,1.57] 但 Runtime IK rpy=[0,0,0]”一致（`runtime_debug_guide.md` §11.7、`topic_service_map.md` §12.4）。CF。

2. **缺少“抓取几何”输出（w/h/centroid/contour 未发布）**
   - 块的长宽 `w,h`、`minAreaRect`、contour 只在 ROI 内部，未进入 world-model 流。CF。
   - 由此无法据块宽推算**所需夹爪开度**（当前 PickSkill 夹爪是固定 pulse：开 200 / 关 700，`runtime_index.md` 夹爪基线）。

3. **缺少抓取点偏移 / approach 规划**
   - 当前 `PickSkill` 用固定 hover/approach 高度偏移（见 `runtime_index.md` §Runtime Capabilities），不随物体形状/厚度变化。CF（动作序列固定）。

4. **可能根本没检出（见上节形状门控）**
   - 长条形蓝色块可能被 ROI/YOLO 双双丢弃 → 需实机确认。INF/UNK。

**最小缺口小结**：对蓝色积木而言，xyz 已有；真正卡住“生成抓取位姿”的是 (a) 块的几何（w/h/yaw）没有以结构化形式发布给下游 grasp 估计器；(b) 即使有 yaw，运动接口也无法下达 yaw。两者都解决前，“object pose ≠ grasp pose”无法闭合。

---

## Mechanical Arm Feedback Chain

涉及的源码节点（CF）：

| 接口 | 发布/服务端节点 | 位置（PC 源码） |
|---|---|---|
| `/controller_manager/joint_states`（`sensor_msgs/JointState`） | `controller_manager` | `servo_controller/controller_manager.py:47,97-115` |
| `/controller_manager/servo_states`（`ServoStateList`） | `controller_manager` | `servo_controller/controller_manager.py:48,97-115` |
| `/kinematics/get_current_pose`（`GetRobotPose`） | `kinematics`（**Jetson 独有，PC 无源码**） | 仅客户端在 PC：`yolo_node.py:138`、`object_detection_node.py:27`、`calibration_node.py:168`、`working_calibration_pose_node.py:93` |
| `/kinematics/set_pose_target`（`SetRobotPose`） | `kinematics`（**Jetson 独有**） | 客户端：`runtime_adapter.py:177-179`、`grasp.py:18` |

### 1. 哪些数据是真实舵机反馈？

- `/controller_manager/joint_states`、`/servo_states` 的数据源是 `ServoManager.get_position()`（`controller_manager.py:102`）。
- `ServoManager.get_position()` 直接 `return self.servos`（`servo_controller.py:68-71`）。
- `self.servos[i].position` 的取值只有两个来源：
  1. **启动初始化**：从参数 `<joint>.init` 读取（`servo_controller.py:26-39`）；
  2. **每次 `set_position()` 调用**：覆盖为**刚下发的命令值** `self.servos[str(i.id)].position = position`（`servo_controller.py:106`，注释原文“记录发送的位置”）。
- 底层**真实读回**服务 `/ros_robot_controller/bus_servo/get_state`（`GetBusServoState`）的客户端虽然已创建（`servo_controller.py:41`），但只被 `get_servo_id()` 用于存在性检查（`servo_controller.py:82-97`），`connect()` 是空实现（`servo_controller.py:47-51`）。**位置回读从未接入发布环路。**

→ CF：`joint_states`/`servo_states` 是**命令值/初值**，不是真实舵机编码器反馈。真实硬件读回能力存在（`bus_servo/get_state`），但未接进反馈链。

### 2. joint_states 是否真实随机械臂运动变化？

- **有命令下达时**：会变化——但变化方式是“在 `set_position()` 发令瞬间直接跳到目标 pulse”，随后 `publish_joint_states`（50 Hz，`controller_manager.py:97-115`）读到该目标值。即它反映“命令目标”，不反映“伺服过程/到位”。
- **无命令时**（手动扳动、重力下垂、外力扰动）：**不变**，因为没有任何回读刷新 `self.servos`。

→ CF：`joint_states` 不真实反映机械臂物理运动；它是命令镜像。

### 3. get_current_pose 是 FK 实时、固定值、缓存值还是命令值？

- `get_current_pose` 的服务端是 Jetson 的 `kinematics` 节点，**其源码不在 PC 工作区**（`src/` 下只有 `kinematics_msgs`，无 `kinematics` 包；全树无 `create_service(GetRobotPose)`）。→ UNK：无法从 PC 源码确认其内部实现。
- 强 INF：基于同源 vendor 栈 `servo_controller`（`ServoManager` 命令镜像、无回读）的架构一致性，以及用户观察到的“末端状态不实时变化”症状，`get_current_pose` 很可能也是基于**命令/初值关节角**做 FK，而非真实反馈关节角；其末端位姿会在发令后“跳变到目标”，空闲或外力下保持命令值。具体机制待 Jetson 源码或实机判别。

### 4. 当前末端坐标无法实时变化的直接原因

直接原因（CF）：**反馈链缺少硬件回读闭环**。`controller_manager` 发布的关节状态取自 `ServoManager`，而 `ServoManager` 只记录下发命令、从不读取真实舵机位置；真实回读服务 `bus_servo/get_state` 存在但未接入。因此末端位姿（无论由 joint_states 做 FK 还是由 `get_current_pose` 提供）只能跟随“命令”，不能跟随“真实运动”。

加重因素（CF）：Current Runtime 自身**根本没有实时位姿查询能力**——`RuntimeAdapter.get_joint_state()` 是返回 `None` 的桩（`runtime_adapter.py:271-275`），`query_vision()` 同为桩（`:277-284`），且 RuntimeAdapter 全程不调用 `get_current_pose`。

### 5. 最小修复方案

按代价从低到高：

1. **接通真实回读（最小且治本）**：在 `ServoManager` 中用已存在的 `bus_servo/get_state` 客户端周期性回读各舵机真实 pulse，写入 `self.servos[i].position`（替换/补充当前的命令镜像）。这样 `controller_manager` 发布的 `joint_states`/`servo_states` 立即变为真实反馈；依赖它们的 FK/`get_current_pose` 同步变实时。涉及：`servo_controller.py`（`ServoManager`），属 Jetson 运行时代码——**按 CLAUDE.md 必须先经用户明确批准**。
2. **Runtime 增加末端查询**：在 `RuntimeAdapter` 用 `/controller_manager/joint_states`（修复后）或 `get_current_pose` 实现 `get_joint_state()`，替换当前 `return None` 桩。属 PC 代码。
3. （可选）让 `get_current_pose` 服务端显式订阅 `joint_states` 做实时 FK——属 Jetson 源码，UNK，需先确认其现状。

> 注：上述任何一项都属“修改生产/Jetson 代码”，本轮仅作建议，不实施。

### 6. 不移动机械臂先验证反馈链

目的是用零风险手段判别“joint_states 是命令值还是真实反馈”：

1. **源码判别（已完成，零风险）**：见上文 §1/§4，CF 为命令镜像。
2. **运行期对照（不动臂）**：对照两个数据源
   - 真实回读：`ros2 service call /ros_robot_controller/bus_servo/get_state ...`（查 STM32 真实 pulse）
   - 发布值：`ros2 topic echo /controller_manager/servo_states --once`
   - 在**长时间空闲**后比较两者；若存在重力下垂/微动，真实回读会偏离命令值，而 `servo_states` 保持命令值不动 → 即证明发布值非反馈。
3. **静默观察**：`ros2 topic echo /controller_manager/joint_states` 在不发任何 `/servo_controller` 命令时持续观察——若数值恒为 home/上次命令值且对外力无响应，则与源码结论一致。
4. （判别性更强但属“动臂”）：轻推某一关节，看 `servo_states` 是否变化——不变即坐实无回读。若严格“不动臂”，则以 2+3 为准。

---

## Why End-Effector Pose Is Not Real-Time

汇总（CF 除特别标注）：

1. `ServoManager.get_position()` 返回初值/命令值，不回读硬件（`servo_controller.py:68-71,26-39,106`）。
2. `controller_manager.publish_joint_states()` 以该值发布 `joint_states`/`servo_states`（`controller_manager.py:97-115`）。
3. 真实回读服务 `bus_servo/get_state` 存在但未接入位置发布（`servo_controller.py:41,47-51,82-97`）。
4. `get_current_pose` 服务端在 Jetson，PC 无源码（UNK）；INF 为基于命令角的 FK。
5. Current Runtime 的 `get_joint_state()` 返回 `None`（`runtime_adapter.py:271-275`），且不调用 `get_current_pose`。

→ 末端位姿“不实时”的根因是**反馈链无硬件回读闭环**，叠加 Runtime 自身无位姿查询。

---

## Confirmed Facts

1. 蓝色积木仅 ROI 可识别（YOLO 无 block 类，且 cube 兜底为红色专属）。`roi_color_detector_node.py:89,332`；`yolo_node.py:76,82-83,421`。
2. ROI 已计算 mask/contour/centroid/minAreaRect/w-h/yaw/color；其中 centroid/xyz、yaw、color 已发布，mask/contour/minAreaRect/w-h 仅内部。`roi_color_detector_node.py:332,337,343-344,386-397,399-415`。
3. ROI 形状分类仅放行接近正方形的块（`right_cnt>=3 and 0.70<=ar<=1.35` → cube，否则丢弃）。`roi_color_detector_node.py:359,375-376`。
4. Fusion：匹配用 ROI pose/color；roi_only 用 ROI；yolo_only color=unknown。`perception_fusion_utils.py:118-156`。
5. 蓝色积木走 roi_only → World Model 用 ROI 坐标。`perception_fusion_utils.py:135-144`。
6. ROI 用静态 `transform.yaml` 投影；YOLO 用动态 `get_current_pose`+`hand2cam` 投影。`roi_color_detector_node.py:164-175`；`yolo_node.py:138,202-244`。
7. RuntimeAdapter 丢弃 rpy，不传入 IK；`SetRobotPose` 请求无 rpy 字段。`runtime_adapter.py:185-189`；`topic_service_map.md` §11.1。
8. `joint_states`/`servo_states` 来自 `ServoManager.get_position()` = 初值/命令值，非真实回读。`controller_manager.py:97-115`；`servo_controller.py:68-71,26-39,106`。
9. 真实回读 `bus_servo/get_state` 存在但未接入发布环路。`servo_controller.py:41,47-51,82-97`。
10. RuntimeAdapter `get_joint_state()` 为 `None` 桩，且不调用 `get_current_pose`。`runtime_adapter.py:271-284`。
11. `get_current_pose` 服务端为 Jetson 独有，PC 工作区无其源码（`src/` 仅 `kinematics_msgs`）。
12. `transform.yaml` 同时含 `hand2cam_transformation_matrix`、`white_area_pose_cam`、`white_area_pose_world`（`transform.yaml:14,31,48`）。

## Inferences

1. 长条形蓝色积木可能被 ROI+YOLO 双双漏检（受 `0.70-1.35` / `0.75-1.25` 长宽比门控）。需实机确认。
2. `get_current_pose` 很可能基于命令/初值关节角做 FK（与同源 vendor 栈无回读一致 + 用户症状）。待 Jetson 源码或实机判别。
3. ROI 静态链与 YOLO 动态链投影基准不同，是坐标一致性问题的来源之一（不展开 Depth/统一链）。
4. 即便补全蓝色积木 yaw，在当前运动接口（无 rpy 字段）下也无法下达抓取朝向。

## Unknowns

1. 实际“蓝色积木”的形状/长宽比 → 决定是否能被当前分类器检出。
2. Jetson 端 `kinematics` 节点 `get_current_pose`/`set_pose_target` 的确切实现（FK 关节角来源、是否回读、yaw 是否内部处理）。
3. `transform.yaml` 中静态 `white_area_pose_cam` 相对当前真实相机位姿的误差量（与 `vision_agent_checkpoint.md` U-8 一致）。
4. 真实回读 `bus_servo/get_state` 的延迟/频率上限是否满足 50 Hz 发布需求。

---

## Recommended First Coding Task

**任务：把蓝色积木的抓取几何从 ROI 内部“暴露”到 world-model 流，并用现有 audit 工具离线核对。**

理由（最小性 + 安全性 + 解锁下游）：

- 它是当前阻塞“object pose → grasp pose”的最小数据缺口：xyz 已有，缺的是块的长宽 w/h、yaw、centroid 的结构化输出（现仅在节点内部）。
- 纯感知侧数据管道，**不碰机械臂、不碰 Jetson、不改 srv**，可离线/单元测试，符合 CLAUDE.md 受限改动边界。
- 复用已有 `roi_detection_audit_node`（`runtime_debug_guide.md` §14.3）即可核对真实蓝色块是否被检出及其几何值，顺带回答 UNK-1（形状门控）。

具体内容（**仅建议，本轮不实施**）：

1. 在 `roi_color_detector_node.on_rgb()` 构造 `objects[]` 时，增加结构化字段：`length_px/width_px`（`w,h`）、`yaw_rad`（`yaw_world`）、`centroid_px`、可选 `box`/`contour`（压缩或采样后）。位置：`roi_color_detector_node.py:408-415`。
2. 同步让 `perception_fusion_utils.build_roi_only_object`/`build_fused_object` 透传这些几何字段，使 `/world_model/stable_objects` 携带块几何。位置：`perception_fusion_utils.py:118-144`。
3. 用 `roi_detection_audit_node` 在真实蓝色块上采样，确认检出率与 w/h/yaw 数值稳定性。

前置依赖（后续任务，非本轮）：

- 运动接口需支持 yaw 下达（`SetRobotPose` 无 rpy 字段，`runtime_adapter.py:185-189` 丢弃 rpy）——这是把 yaw 变成真实抓取朝向的前提，但代价更大，排在几何暴露之后。
- 与 `runtime_index.md` 的“统一权威坐标链 / AprilTag”P0 并行，不冲突：本任务只补几何数据，不改坐标来源。

## Recommended Verification

蓝色积木视觉（不动臂）：

```bash
# 1. 确认蓝色块是否被检出 + 几何值
timeout 15 ros2 topic echo /world_model/roi_objects --once --full-length
# 2. 走完整链看 stable 输出（应 source=roi_only, color=blue, pose.rpy 非零）
timeout 15 ros2 topic echo /world_model/stable_objects --once --full-length
# 3. 离线 audit
ros2 run app roi_detection_audit_node --ros-args -p sample_count:=200
```

机械臂反馈链（不动臂）：

```bash
# A. 真实硬件回读（参照真值）
ros2 service call /ros_robot_controller/bus_servo/get_state <type> "{}"
# B. 发布值（疑似命令镜像）
timeout 5 ros2 topic echo /controller_manager/servo_states --once
# C. 静默观察 joint_states 是否对外力/空闲无响应
ros2 topic echo /controller_manager/joint_states
```

判据：B 与 A 在长时间空闲后出现偏差（重力下垂等）且 C 无相应变化 → 坐实“joint_states 为命令镜像、无真实回读”。

---

> 本报告仅为只读审查结论与建议，未修改任何生产代码、未运行 ROS、未操作机械臂、未执行 Git 命令。等待用户审查。
