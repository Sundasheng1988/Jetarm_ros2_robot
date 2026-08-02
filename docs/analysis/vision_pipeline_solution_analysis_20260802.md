# JetArm 视觉抓取链路专项审查与解决方案（v3）

> **报告类型**：调查 + 方案设计（本轮不修改生产代码/参数、不启动 ROS、不发消息、不操作硬件）
> **基线日期**：2026-08-02 ｜ **分支**：feature/sketch_runtime_sprint3
> **证据等级**：Confirmed Fact (CF) / High-Confidence Inference (HCI) / Direct Observation (DO) / Unknown
> **v3 重点**：以 7 条核心证据为骨架；方案主线为 Phase 1A（离线基线测量，不改生产代码）→ Phase 1B（候选链经 1A 证明更优后才改生产节点）→ Phase 2（bbox 颜色 + 平面/depth 对照实验，不预设 depth 更优）→ Phase 3（seg mask + grasp estimator）。「不融合/class gate/距离 gate」仅作临时安全保护。
> **引用**：`相对路径:行号`

---

## 1. Executive Summary

当前生产视觉链 **不是真正的 RGB-D 3D 感知**：没有任何生产节点订阅深度图，YOLO 与 ROI 都把目标投影到一个固定工作平面、假设 Z=0.030；且 YOLO 用的是**代码默认的理想化 hand2cam**（launch 与启动脚本均未覆盖），而 `transform.yaml` 里**标定后的 hand2cam_transformation_matrix 仅被标定工具读取**，未被任何生产感知节点使用。与此同时 ROI 用**静态 white_area_pose_cam**。两条链独立漂移，再由 Fusion 用 6 cm 距离把两个「完整对象」拼起来——这是类别/颜色/坐标不准、模块结果无法对应同一对象的共同结构根源。

**主解决方案主线**（详见 §10 与测量计划 §15/§16）：
- **Phase 1A（Baseline Measurement）**：**不改生产代码**；离线计算候选统一链（calibrated hand2cam estimate + 动态链 + 一致 K/y_flip），用 Test A/B 建立多点误差基线；保持当前平面投影。
- **Phase 1B（Coordinate-Chain Implementation）**：**仅当 Phase 1A 证明候选链更优**后，才修改生产节点统一坐标链；仍**不改消息架构**。`hand2cam_transformation_matrix` 为手眼标定结果（calibrated estimate），**实测前不作 ground truth**。
- **Phase 2**：**不直接替换为 depth**，而是建立对照实验（A 统一平面 / B bbox 中心单点 depth / C bbox 区域鲁棒 depth / D 平面+depth 混合）；仅当实测证明 depth 方案优于 A 才改生产位姿来源。
- **Phase 3**：升级 segmentation mask；mask 内计算颜色/深度/目标表面；增加独立 grasp pose estimator。

> **User Historical Observation（用户历史观察，非 CF，不可忽略）**：此前曾实现 RGB+Depth 定位，发现深度坐标非常不准，才改用当前平面投影（旧 RGB-D 版本以 `yolo_node.py.bak` 保留于 git）。故**不预设「真实 depth 一定比平面投影更准」**。详见测量计划 §15。

**临时安全保护（非最终方案）**：Fusion class/置信度/距离 gate、Tracker pose gate、Verification 多帧/模式 gate——只止血，不解决识别与坐标根本问题。

---

## 2. Core Evidence — 7 条核心证据（每条含 文件/函数或参数/行号/配置来源/分级）

> 本节为报告骨架。证据基于对 **launch、启动脚本、节点订阅、源码引用** 的检查。

| # | 结论 | 文件 | 函数 / 参数 | 关键行号 | 配置来源 | 分级 |
|---|---|---|---|---|---|---|
| **E1** | 生产视觉链**未真正使用 depth image** 计算目标三维坐标 | `yolo_node.py`、`roi_color_detector_node.py` | 订阅：`rgb_topic`、`camera_info_topic` | YOLO `:48-49,133-134`（仅 rgb 图 + depth **camera_info**）；ROI `:69-70,140-141`（仅 rgb 图 + rgb camera_info） | `perception_bringup.launch.py`（无 depth remap）；`start_cyclone_perception.sh`（仅 `start_grounding:=false`） | **CF** |
| **E2** | YOLO 与 ROI 本质上基于**固定工作平面**算 XY、**假设 Z** | `yolo_node.py`、`roi_color_detector_node.py` | `_pix_to_world_on_plane` / `_pix_to_world` | YOLO `:255-279`，`z=self.pick_plane_z` `:268`；ROI `:245-274`，`pz=self.z_up` `:267` | 代码默认 `pick_plane_z=0.030`(YOLO `:87`)、`z_up=0.030`(ROI `:80`)；launch 不传 | **CF** |
| **E3** | YOLO 当前 `hand2cam` 是**代码默认值**，launch/脚本**未覆盖** | `yolo_node.py` | 参数 `hand2cam_matrix`（声明+默认） | 声明默认 `:60-65`；读取 `:121-122`；使用 `:239`(`base_T_cam=T_hand@hand2cam`) | `perception_bringup.launch.py:37-42`（YOLO Node **无 parameters 块**）；`start_cyclone_perception.sh`（仅 `start_grounding:=false`） | **CF** |
| **E4** | `transform.yaml` 标定 `hand2cam_transformation_matrix` **未被生产视觉节点使用** | `transform.yaml`、`yolo_node.py`、`roi_color_detector_node.py` | （搜索 `hand2cam_transformation_matrix`） | 定义 `transform.yaml:14`；**唯一真正从 yaml 读取者为** `working_calibration_node.py:39`（标定工具） | 生产 YOLO/ROI 全源码搜索**无**该字段引用（`working_calibration_pose_node.py:33`、`test_calculation.py:17` 为**同名内联字面量**，非 yaml 读取） | **CF** |
| **E5** | ROI 使用**静态** `white_area_pose_cam` | `roi_color_detector_node.py` | `_load_transform`、`_pix_to_world` | 加载 `:171-172`；使用 `:252-261`(`white_T_cam=inv(white_area_pose_cam)`) | `transform.yaml: white_area_pose_cam`（标定时刻冻结，不随机械臂运动更新） | **CF**（静态本身=CF；「已漂移」=HCI，见 §6） |
| **E6** | YOLO 使用**动态** `/kinematics/get_current_pose`，但结合的是**默认 hand2cam** | `yolo_node.py` | `_refresh_hand_pose`/`_on_pose_result`/`_build_roi_and_homography` | client `:138`；查询 `:202-222`；`base_T_cam=T_hand@hand2cam` `:239`；`pose_query_hz=5` `:67,146` | `hand2cam` 默认值 `:60-65`；`/kinematics/get_current_pose`（Orin 服务） | **CF** |
| **E7** | 当前视觉链**不能称为真正的 RGB-D 3D perception** | （E1+E2 复合） | 无深度图订阅 + Z 固定平面 | 见 E1、E2 | — | **CF**（由 E1+E2 直接导出） |

### 2.1 支撑性代码搜索证据（响应「必须基于 launch/订阅/源码检查」）

- **「生产未用 depth」审计（E1）**：全 `src` 搜索 `/depth_cam/depth/image_raw` 订阅者 → **唯一** `working_calibration_node.py:30`（标定工具）。生产节点（`yolo_node.py`、`roi_color_detector_node.py`、`object_detection_node.py`）均只订阅 `/depth_cam/rgb/image_raw`。`yolo_node.py:134` 订阅的是 `depth/camera_info`（仅取内参 K），非深度图。
- **「标定 hand2cam 未用」审计（E4）**：`grep hand2cam_transformation_matrix src` → 命中 `transform.yaml:14`（定义）、`working_calibration_node.py:39`（**唯一** `data['hand2cam_transformation_matrix']` 读取）、`working_calibration_pose_node.py:33` 与 `test_calculation.py:17`（均为同名**内联字面量**赋值，非读 yaml）。生产 YOLO/ROI **零命中**。
- **「launch 未覆盖 hand2cam」审计（E3）**：`perception_bringup.launch.py:37-42` 的 YOLO Node 无 `parameters`；`start_cyclone_perception.sh` 仅追加 `start_grounding:=false`。故运行态用 `yolo_node.py:60-65` 默认值。

---

## 3. Current Pipeline

```
/depth_cam/rgb/image_raw
   ├─→ simple_yolo_node            → /world_model/objects      (class_name + 平面 pose; y_flip=True; depth camera_info 的 K; 无深度图)
   └─→ roi_color_detector_node     → /world_model/roi_objects  (几何 class + color + 平面 pose; y_flip=False; rgb camera_info; 无深度图)
两路「完整对象」 ──6 cm 距离匹配──→ perception_fusion_node → /world_model/perception_objects
   → stable_object_tracker_node → /world_model/stable_objects
   → grounding_node → /grounded_task_context → real_grounded_runtime_node → verification_result_node
```
结构问题：**两路各自输出完整对象**，对应关系外包给一个距离阈值——这是 E1–E7 之外、使问题放大的架构因素。

---

## 4. Confirmed Facts（补充）

| 编号 | 事实 | 证据 |
|---|---|---|
| CF-1 | Fusion 仅欧氏距离匹配 `distance_threshold=0.06`，无 class 检查 | `perception_fusion_node.py:37`；`perception_fusion_utils.py:84-115` |
| CF-2 | 融合 pose 无条件取 ROI | `perception_fusion_utils.py:125` |
| CF-3 | 置信度 `min(yolo,roi)`，低置信 ROI 仍覆盖 | `perception_fusion_utils.py:126` |
| CF-4 | 两路 `std_msgs/String` JSON 无 header；TTL wall-clock `cache_ttl_sec=2.0` | `perception_fusion_node.py:38,76,93` |
| CF-8 | YOLO `y_flip=True`、depth camera_info；ROI `y_flip=False`、rgb camera_info | `yolo_node.py:88,49`；`roi_color_detector_node.py:81,70` |
| CF-9 | YOLO 用 `getOptimalNewCameraMatrix` 去畸变 K；ROI 用原始 `msg.k` | `yolo_node.py:227-230`；`roi_color_detector_node.py:195` |
| CF-10 | `white_area_pose_world` 被 YOLO+ROI 共享为最终 base 变换 | `yolo_node.py:177-180`；`roi_color_detector_node.py:171` |
| CF-14 | `lab_config.yaml` 含 `white`：L[111,255],a[100,155],b[100,155]（OpenCV LAB，a/b≈128 中性） | `lab_config.yaml` |
| CF-15 | ROI 对**整张白区裁剪图**逐色 `inRange`，非仅在物体轮廓内 | `roi_color_detector_node.py:318-337` |
| CF-16 | ROI 颜色=`_dominant_color_key`；形状分类 cube/cup/ball/cylinder；置信度 clamp[0.50,0.99] | `roi_color_detector_node.py:234-243,354-376,403` |
| CF-18 | Tracker 发布 `xyz_latest`，每帧无条件覆盖，pose 无平滑/离群门 | `stable_tracker_utils.py:16,156-166`；`stable_object_tracker_node.py:90` |
| CF-19 | Verification postcheck 单快照 `success=not object_still_at_source`，阈值 0.1；`dry_run` 声明未用 | `verification_result_node.py:21,24,223-266` |
| CF-20 | Servo OFF 时 servo/gripper return 但 skill 完成→发 `/executor/done`→触发 postcheck | `runtime_adapter.py:212,246` |
| CF-21 | Grounding 精确 `o["color"]==color`；unknown≠blue→no_match | `grounding_node.py:267-279` |
| CF-22 | IK 请求 `SetRobotPose.Request` 无 rpy→yaw 结构性丢失 | `runtime_adapter.py:185-189` |

---

## 5. Direct Observations

| 编号 | 观察 | 来源 |
|---|---|---|
| DO-1 | ROI `cube/white [0.185,-0.011,0.034]` 同时 YOLO `cup [0.237,-0.007,0.041]`，真实杯子未动 | `runtime_debug_guide.md §11.2` |
| DO-2 | 两点世界距离 ≈ 5.26 cm | 计算 |
| DO-4 | Servo OFF 时 Verification `object_no_longer_at_source/verified` | `runtime_debug_guide.md §11.6` |
| DO-5 | Grounding rpy=[0,0,1.57]，IK 请求 rpy=[0,0,0] | `runtime_debug_guide.md §11.7` |

---

## 6. Root-Cause：5 cm 偏差的**多因素**分析（不归因单一因素）

> DO-1/DO-2 的 5 cm 偏差是**以下多因素叠加**的结果，**非单一原因**。各因素的量化分离需 Phase 1A 多点坐标误差测试（§10）。

| 因素 | 影响节点 | 机理 | 证据/量级 | 分级 |
|---|---|---|---|---|
| **F1. 默认 hand2cam vs 标定矩阵** | YOLO | YOLO 用理想化默认（纯 90° 轴置换），标定矩阵有小角度耦合 | 平移差：tx≈4.7 mm、ty≈20.8 mm（**且符号翻转**）、tz≈34 mm；旋转耦合项达 ~0.105（~6°）。`yolo_node.py:60-65` vs `transform.yaml:14` | CF（矩阵差异）；「这是 5 cm 主因」=HCI |
| **F2. ROI 静态 transform** | ROI | `white_area_pose_cam` 标定时刻冻结，不随臂/相机运动更新 | `roi_color_detector_node.py:171-172,252-261` | CF（静态）；「已漂移」=HCI |
| **F3. y_flip 不一致** | 两者 | YOLO `y_flip=True`、ROI `y_flip=False`，Y 轴约定不同 | `yolo_node.py:88`；`roi_color_detector_node.py:81` | CF |
| **F4. RGB / depth 内参差异** | 两者 | YOLO 用 depth camera_info 的 K 且去畸变；ROI 用 rgb camera_info 原始 K | `yolo_node.py:49,227-230`；`roi_color_detector_node.py:70,195` | CF |
| **F5. 平面 Z 假设** | 两者 | 物体不在 z=0.030 平面时，斜投影使 XY 偏移随高度/相机倾角放大 | `yolo_node.py:268`；`roi_color_detector_node.py:267` | CF |
| **F6. 轮廓中心 vs 真实目标中心** | 两者 | ROI 用轮廓质心；YOLO 用 bbox 中心；杯体/杯口/把手致 (u,v) 不同→平面 XY 不同 | `roi_color_detector_node.py:347`；`yolo_node.py:351` | CF |

> 注：F1 仅影响 YOLO（ROI 不用 hand2cam）；F2 仅影响 ROI（YOLO 不用静态外参）。两链各自受不同主因影响，叠加 F3/F4/F5/F6 共同形成 5 cm。**单点重标定只能短期缓解 F2，不能消除 F1/F3/F4**——故需统一标定链（Phase 1B，须先经 Phase 1A 证明）。

### 颜色/类别根因（补充）
- **blue cup→white（CF 机制）**：`white` 区间 a/b≈128 捕获**低饱和/中性**像素（CF-14）；blue cup 的高光 + 白板背景（CF-15）落入该区间，`_dominant_color_key`（CF-16）取多数→white。
- **blue cup→cube（CF 机制）**：杯轮廓在顶视投影满足 `right_cnt>=3 且 ar∈[0.70,1.35]`（CF-16）。

---

## 7. Code Findings — 坐标参数流转

```
calibration_node.py:225-275  → 生成 transform.yaml
  AprilTag → hand2base(/kinematics) → hand2cam(标定) ─┬─→ white_area_pose_cam  (静态; ROI 生产用 → F2 漂移源)
                                                       └─→ white_area_pose_world(YOLO+ROI 共享)
生产链(现状):
  YOLO : 像素 → depth K(去畸变) → 白区平面 → z=0.030 → white_area_pose_world → base ; hand2cam=默认 ; y_flip=True
  ROI  : 像素 → rgb K(原始)     → 白区平面 → z=0.030 → white_area_pose_world → base ; 外参=静态   ; y_flip=False
未用 : hand2cam_transformation_matrix(标定, 仅 calibration 工具读) ; depth image(仅 calibration 工具订阅)
```
- eye-in-hand 结构（相机随臂动）：`base_T_cam = T_hand @ hand2cam`（`yolo_node.py:239`）→ 相机固连手部。**HCI**（由数学结构推断，可 TF 核实）。

---

## 8. 临时安全保护（⚠ 非最终视觉方案，仅 Phase 1 前止血）

| 措施 | 位置 | 作用 | 定位 |
|---|---|---|---|
| Fusion class gate | `perception_fusion_utils.py:match_objects` | 阻 cup+cube 错融 | 临时 |
| Fusion 置信度 gate | 同上 + `min_roi_conf` | 低置信 ROI 不覆盖 pose | 临时 |
| Fusion 距离/新鲜度 gate | 同上 + `max_age_sec≈0.4` | 旧 ROI 不参与 | 临时 |
| Tracker pose gate+EMA | `stable_tracker_utils.py` | 单帧跳变不改发布 pose | 临时 |
| Verification 多帧确认 | `verification_result_node.py:_run_postcheck` | 单帧漏检不判成功 | 临时 |
| Verification 执行模式 gate | 同上 + `require_real_motion` | Servo OFF 不返回真实成功 | 临时 |

> 这些 gate 在 Phase 1/2 落地后会被「统一观测模型」自然取代或弱化，**不得作为长期视觉架构**。

---

## 9. 风险排序

| ID | 风险 | 等级 |
|---|---|---|
| R-Coord | 双坐标链 + 默认 hand2cam + 静态 ROI 外参（F1–F6） | **P0** |
| R-HeightModel | 固定平面 + 假设 Z=0.030 的**适用范围**风险（高物/非平面物体 Z 不准）（E2/E7） | P1（depth 为**候选**解，非「根治」；由 Phase 2 对照实验判定） |
| R-B | Fusion 无 class gate、ROI 覆盖 YOLO pose | **P0** |
| R-E1/E2 | Verification 单帧 + 无模式门控 | **P0** |
| R-D | Tracker pose 不平滑/无离群门 | **P0** |
| R-Color | 全图 LAB + white 区间 | P1 |
| R-Class | ROI 几何覆盖语义 | P1 |
| R-Y | yaw 结构性丢失 | P1 |

---

## 10. 主解决方案主线（Phase 1 / 2 / 3）

### Phase 1A：Baseline Measurement（**不改生产代码**，离线计算候选统一链）

> 目标：保持当前平面投影生产链不变；离线计算候选统一链并与旧链对比，**先量化**现状漂移。本阶段不新增/不改 topic 结构、不引入 detection_id、不做颜色改造。

| 项 | 内容 |
|---|---|
| **修改文件** | **无生产代码改动**；新增**离线分析工具**（录制 + 离线统一链重算 + 误差表） |
| **数据流变化** | **无**（仅离线分析；`/world_model/*` 不变） |
| **参数** | 离线工具参数：`calib_points`、`max_pose_lag_sec`；候选链用 `transform.yaml:hand2cam_transformation_matrix`（**calibrated estimate，非 ground truth**） |
| **单元测试** | 离线统一链重算的正确性测试（矩阵方向、y_flip 统一、K 一致） |
| **离线验证** | **多点坐标误差表（核心交付）**：≥6 个已知 base 坐标点（标定件），并排测「候选统一链」vs「旧 ROI 链」vs「旧 YOLO 链」，量化 F1–F6 各因素贡献 |
| **ROS Topic 验证** | 仅录制（静止标定件下 `/world_model/objects`、`/world_model/roi_objects`、`/kinematics/get_current_pose`、图像、camera_info） |
| **坐标精度验收** | 候选链相对旧链的改善量（bias/跨链差）；**最终阈值以基线 + 测量不确定度确定，不预设 RMSE<1cm** |
| **是否需要实机动作** | Test A 否；Test B 需用户低速换姿态、不抓取、到位静态采集 |

### Phase 1B：Coordinate-Chain Implementation（**仅当 1A 证明候选链更优**后才改生产节点）

> 门控：只有 Phase 1A 误差表证明候选统一链显著优于旧链，才进入 1B。仍**不改消息架构**。

| 项 | 内容 |
|---|---|
| **修改文件** | `yolo_node.py`（`hand2cam` 改读 `transform.yaml:hand2cam_transformation_matrix`；统一 y_flip 到矩阵约定；仍用平面 Z 过渡）；`roi_color_detector_node.py`（pose 改走同一动态链 `T_hand@hand2cam_calibrated`，停用静态 `white_area_pose_cam` 作生产位姿源——保留为自检）；`perception_bringup.launch.py`（传 `transform_yaml` 给 YOLO） |
| **数据流变化** | **无消息架构变化**；仅 pose 计算内部统一到标定动态链。`/world_model/*` topic 与字段不变 |
| **参数** | `hand2cam_source=transform.yaml`；统一 `y_flip`（矩阵内，代码移除 flip 分支） |
| **单元测试** | (1) YOLO 读取 transform.yaml hand2cam 而非默认；(2) y_flip 统一后两链方向一致；(3) 静态 `white_area_pose_cam` 不再进入生产 pose |
| **离线验证** | 用 1A 录制数据回归：1B 改动后离线 pose 与 1A 候选链一致 |
| **ROS Topic 验证** | 静止标定件下 `/world_model/objects`、`/world_model/roi_objects`、`/world_model/stable_objects` 的 pose 稳定且两链一致 |
| **坐标精度验收** | 达到 1A 设定的最终阈值（基线 + 不确定度校准后） |
| **是否需要实机动作** | **否**（静止标定件 + rosbag；机械臂保持观测姿态，无 Pick） |

### Phase 2：YOLO bbox 内颜色识别 + 同 bbox 内 aligned depth 鲁棒深度 → class+color+pose

> 目标：在统一坐标链基础上，对「平面 vs depth」做**对照实验**（A 统一平面 / B 中心点 depth / C 区域鲁棒 depth / D 混合），并完成 bbox 内颜色识别。**前置**：Orin 提供 aligned depth（U-1）。**不直接替换生产位姿源**——仅当 C/D 实测优于 A 才切换（详见测量计划 §16）。

| 项 | 内容 |
|---|---|
| **修改文件** | `yolo_node.py`（订阅 aligned depth；输出 bbox+timestamp）；新增 color estimator（bbox 内估色）+ pose estimator（bbox 内鲁棒深度）；`perception_fusion_utils.py`（pose 来源切到 pose estimator，移除 ROI pose 覆盖） |
| **数据流变化** | YOLO 输出 `{class_name, bbox, timestamp, conf}`；color 在 bbox 内；depth 在 bbox 内中位数→真 3D；Z 不再固定平面。Fusion 合并 class+color+pose（同 bbox） |
| **参数** | `depth.aligned_topic`；`color.min_color_pixel_ratio`、`color.erode_px`；`depth.erode_px`、`depth.min_valid_ratio` |
| **单元测试** | (1) bbox 内颜色剔除低饱和/高光/背景；(2) bbox 内深度中位数剔除空洞/边缘；(3) Z 随物体高度变化；(4) ROI 不再覆盖 pose |
| **离线验证** | rosbag：已知高度物体 Z 误差 < 1 cm；blue cup 不再判 white |
| **ROS Topic 验证** | `/world_model/perception_objects` 含 class+color+pose 且 color 稳定、Z 合理 |
| **坐标精度验收** | Z 误差均值 < 1 cm，std < 0.5 cm；XYZ RMSE < 1.5 cm |
| **是否需要实机动作** | **否**（前置 U-1：Orin aligned depth 可用性） |

### Phase 3：segmentation mask + mask 内颜色/深度/目标表面 + 独立 grasp pose estimator

| 项 | 内容 |
|---|---|
| **修改文件** | `yolo_node.py`（YOLOv8-seg，输出 mask）；color/depth estimator 改用 mask；新增 `grasp_pose_estimator_node`；`runtime_adapter.py`（IK 携带 yaw，修复 R-Y） |
| **数据流变化** | mask 驱动颜色/深度/表面点；grasp pose（区分 center/surface/rim/grasp）→ IK |
| **参数** | `grasp.approach_offset`、`grasp.yaw_strategy`、shape_hint 权重 |
| **单元测试** | (1) mask 内颜色/深度更准；(2) 杯口/方块/球产出不同 grasp pose；(3) yaw 正确传入 IK |
| **离线验证** | 多类物体 grasp pose 与人工标注对比 |
| **ROS Topic 验证** | `/world_model/grasp_targets`；`/runtime/preview` 含 rpy |
| **坐标精度验收** | grasp pose 与真实抓取点 < 1.5 cm；yaw 误差 < 10° |
| **是否需要实机动作** | **是（最终验收）**——用户现场批准，单一控制栈/require_confirm/可断电 |

---

## 11. Validation Matrix（本轮不执行真实动作）
- **A 离线单测**：见各 Phase。
- **B ROS Topic dry-run**：观察 `/world_model/*`，pose 不跳、color 稳定。
- **C IK-only**：统一 pose + IK 脉冲；Verification 报 skipped/非成功。
- **D 真实动作**：仅 Phase 3 验收，用户批准后。

---

## 12. Files Proposed for Modification（按 Phase，本轮不执行）

| Phase | 文件 | 要点 |
|---|---|---|
| 临时 | `perception_fusion_utils.py`/`_node.py`、`stable_tracker_utils.py`/`_node.py`、`verification_result_node.py`/launch | 各 gate（过渡） |
| **P1** | `yolo_node.py`、`roi_color_detector_node.py`、`perception_bringup.launch.py` | 统一标定动态链；新增多点误差测量工具（离线，不改消息） |
| **P2** | `yolo_node.py`、新增 color/pose estimator、`perception_fusion_utils.py` | bbox 颜色 + aligned depth |
| **P3** | `yolo_node.py`(seg)、新增 `grasp_pose_estimator_node`、`runtime_adapter.py` | mask + grasp pose + yaw |

> transform.yaml/lab_config.yaml：Phase 1 不改结构；按 §7 标注 `white_area_pose_cam`（停用生产）、理想 hand2cam 默认（停用）为弃用项。

---

## 13. Unknowns

| 编号 | 未知 | 确认手段 | 分级 |
|---|---|---|---|
| U-1 | Orin Gemini 是否发布 aligned depth（Phase 2 前置） | Orin `ros2 topic list`（用户执行，Jetson 只读） | Unknown |
| U-2 | rgb 与 depth 是否已像素对齐 | 同 U-1 + 驱动配置 | Unknown |
| U-3 | ROI 静态外参自标定以来漂移量 | Phase 1 多点误差表 | Unknown（待测） |
| U-4 | 5 cm 各因素（F1–F6）的量化占比 | Phase 1 多点误差表逐项隔离 | Unknown（待测） |
| U-5 | 相机为 eye-in-hand | TF 核实 | HCI→待确认 |
| U-6 | Runtime 发 `/executor/done` 逐行确认 | 读 `real_grounded_runtime_node` 或实测 | HCI→强 |

---

## 14. Recommended First Implementation Task

**Phase 1 第一步：建立多点坐标误差测试（不改任何生产代码/消息架构）。**
- 做法：用 ≥6 个已知 base 坐标的静止标定件，并排采集「YOLO 旧链 pose」「ROI 旧链 pose」「统一标定链 pose」，制误差表，量化 F1–F6。
- 产出：误差基线 → 作为 Phase 1「统一标定链」改动的验收门槛（平面 RMSE<1 cm）。
- 价值：把 5 cm 偏差从「归因争议」变成「可量化、可分因素」的证据，再决定 hand2cam 替换与 y_flip 统一的具体改动。
- 无实机动作；仅静止观测 + rosbag。

**停止点**：本轮为调查 + 方案设计。已完成 4 份必读文档 + 全量 transform.yaml + launch/脚本/订阅/源码审计 + 测试；未改生产代码/参数；未启动 ROS；未发消息；未触硬件/Jetson。等待用户审查与批准。

---

### 附录 A：15 个必答问题速查
1. YOLO pose：动态 `T_hand@hand2cam`(默认)→白区平面，y_flip=True，depth K，无深度图（E1/E2/E3/E6）。
2. ROI pose：静态 `white_area_pose_cam`→白区平面，y_flip=False，rgb K，无深度图（E1/E2/E5）。
3. 坐标系**不一致**（§7）。
4. transform 配置**不一致**：YOLO 用 white_area_pose_world+默认 hand2cam；ROI 用 white_area_pose_cam；标定 hand2cam 未被生产用（E4）。
5. 消息**无** ROS 时间戳；仅 wall-clock TTL（CF-4）。
6. Fusion：6 cm 距离匹配两套完整对象（CF-1）。
7. class 冲突不处理；低置信 min() 覆盖（CF-1/3）。
8. ROI 无条件覆盖 YOLO pose（CF-2）。
9. Tracker：投票/EMA，pose=latest 不平滑（CF-18）。
10. 无 pose 离群门（CF-18）。
11. Grounding unknown 精确匹配→no_match（CF-21）。
12. Verification 单快照，10 cm 内无候选即「移除」（CF-19）。
13. 单帧漏检**可**触发成功（CF-19）。
14. Servo OFF 仍 postcheck：dry_run 未用 + /executor/done 触发（CF-19/20）。
15. 归类：A=算法+配置+坐标；B=算法+数据同步；C=算法；D=算法；E=状态机+算法+测试。

### 附录 B：7 条核心证据速查（见 §2）
E1 无深度图 / E2 平面+假设Z / E3 hand2cam 默认未覆盖 / E4 标定 hand2cam 未被生产用 / E5 ROI 静态外参 / E6 YOLO 动态+默认 hand2cam / E7 非真正 RGB-D 3D。
