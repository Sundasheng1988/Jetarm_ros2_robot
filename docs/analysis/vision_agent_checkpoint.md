# Vision Pipeline Agent Checkpoint

> **日期**：2026-08-02 ｜ **分支**：feature/sketch_runtime_sprint3 ｜ **状态**：调查+设计；未改生产代码/参数/launch，未启 ROS。
> 主报告：`vision_pipeline_solution_analysis_20260802.md` ｜ 测量计划：`vision_phase1_measurement_plan_20260802.md`

## User Historical Observation（用户历史观察，非 CF，不可忽略）
- **此前曾实现 RGB+Depth 定位，发现深度坐标非常不准，才改用当前固定工作平面投影。**
- **不得预设「真实 depth 一定比平面投影更准」。** Phase 2 必须用对照实验证明后才允许改生产位姿来源。
- 代码佐证：`src/vision_yolo/vision_yolo/yolo_node.py.bak`（`YoloToWorldNode`）为**已提交的旧 RGB-D 版本**；当前已提交 `yolo_node.py`（`AppCompatibleYoloNode`，平面版）已替换它；两者均在 git 中。详见测量计划 §15。

## 关键状态（Handoff）
- Production perception currently uses planar projection, not true depth-based 3D. → CF
- Calibrated hand2cam matrix (`transform.yaml:hand2cam_transformation_matrix`) exists but is not used by the active YOLO path (only `working_calibration_node.py:39` reads it). → CF
- Primary next investigation is a unified calibrated coordinate chain. RGB-D is a candidate, not the assumed final solution.
- Do not repeat the previous fusion-only solution discussion（gate 仅临时保护）。

## 方案主线（已按历史观察修订）
- **Phase 1A（Baseline Measurement）**：不改生产代码；离线计算候选统一链（calibrated hand2cam estimate + 动态链 + 一致 K/y_flip），建立多点误差基线，保持平面投影。
- **Phase 1B（Coordinate-Chain Implementation）**：仅当 1A 证明候选链更优，才改生产节点；不改消息架构。
- **Phase 2**：**不直接替换为 depth**，而是对照实验 A 统一平面 / B bbox 中心单点 depth / C bbox 区域鲁棒 depth / D 平面+depth 混合；仅当 B/C/D 实测优于 A 才改生产位姿源。
- **Phase 3**：seg mask + grasp pose estimator。

## 旧 RGB-D 实现要点（.bak，详见测量计划 §15）
bbox 中心 5×5 中位数深度；假定 rgb-depth 已对齐（未校验）；用 depth K；mm→m；仅过滤 0/NaN（无边缘/背景/空洞处理）；无测试数据或决策记录留存于 repo。

## Unknowns
U-1 aligned depth topic ｜ U-2 rgb-depth 同 optical frame ｜ U-3 eye-in-hand（HCI，待 TF）｜ U-4 EE 帧身份 ｜ U-5 深度编码/分辨率 ｜ U-6 Orbbec align 参数 ｜ U-7 base frame 名 ｜ U-8 white_area_pose_world 携带的理想化 hand2cam 误差量。

## 下一步
Phase 1A 测量准备（不改生产代码）：用户先用只读 ROS 命令补全 U-1~U-7；制作 AprilTag 网格板（注意 board→base 锚定误差）；编写离线录制+离线统一链计算脚本（新分析工具，非生产改动）；跑 Test A/B 制误差基线。仅当 1A 证明候选链更优才进入 1B。
