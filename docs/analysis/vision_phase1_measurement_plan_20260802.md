# Phase 1 实施前审查与测量设计

> **类型**：测量设计（不改生产代码/参数/launch，不启动 ROS，不发消息，不操作硬件）
> **日期**：2026-08-02 ｜ **分支**：feature/sketch_runtime_sprint3
> **证据等级**：Confirmed Fact (CF) / High-Confidence Inference (HCI) / Unknown
> **目的**：在改动任何生产坐标链之前，先把 5 cm 偏差**量化、分因素**，并定义统一标定链的验收方法。
> **约定**：`T_A_B` = B 在 A 中的位姿 = 把 B 系坐标变换到 A 系（`p_A = T_A_B @ p_B`）。

---

## 1. Coordinate Frame Table（矩阵方向审计，结合乘法与代码确认）

| 矩阵 | 语义 (=) | source→target | 需 inverse? | 生成 (file:line) | 读取 (file:line) | 用于计算式 (file:line) | 生产用? | 分级 |
|---|---|---|---|---|---|---|---|---|
| **T_base_hand** | EE 在 base 中的位姿 | hand→base | 否（直接用） | Orin kinematics 服务（不在 PC src） | `/kinematics/get_current_pose` 的 `pose` | YOLO `yolo_node.py:239` `base_T_cam=_last_T_hand@hand2cam`；calib `calibration_node.py:226` `pose_world=endpoint@pose_end` | **是（YOLO）** | CF（方向：乘法链 + `working_calibration_node.py:130-133` 注释「末端到基座」）；EE 物理帧身份 = HCI |
| **hand2cam（默认）** | T_hand_cam，cam 在 hand 中的位姿 | cam→hand | 否 | 代码硬编码（理想化轴置换） | YOLO 参数 `hand2cam_matrix` 默认 `yolo_node.py:60-65`；calib 类属性 `calibration_node.py:83-88`（**两者数值相同**） | YOLO `:239`；calib `:225` `hand2cam_tf_matrix@avg_pose` | **是（YOLO）** | CF |
| **hand2cam_transformation_matrix（标定）** | T_hand_cam（**calibrated estimate / 手眼标定结果；实测前不作 ground truth**） | cam→hand | 否（语义） | 外部手眼标定写入 transform.yaml（当前 `calibration_node` **不重新生成**，它用理想化值） | **仅** `working_calibration_node.py:39`（`data['hand2cam_transformation_matrix']`）；`working_calibration_pose_node.py:33`、`test_calculation.py:17` 为同名**内联字面量**，非 yaml 读取 | 仅标定工具 | **否** | CF |
| **white_area_pose_cam** | T_cam_white，白板在 cam 中的位姿 | white→cam | ROI 取 inv（`:260`） | `calibration_node.py:228`（`=avg_pose`，即检测到的 tag 在 cam 中的位姿） | `roi_color_detector_node.py:172`；calib `:158` | ROI `:252-261`（ray∩平面：`n=[:3,2]`、`p0=[:3,3]` 为 cam 系量；`inv`→white） | **是（ROI）** | CF（方向：ROI 用法 + 生成式双确认） |
| **white_area_pose_world** | T_base_white，白板在 base 中的位姿 | white→base | 否 | `calibration_node.py:229`（`=endpoint@hand2cam_tf_matrix@avg_pose`，**用理想化 hand2cam**） | YOLO `yolo_node.py:177-180`；ROI `roi_color_detector_node.py:171` | YOLO `:241,264-265`；ROI `:265-266` | **是（两链共享）** | CF；**注意：其本身用理想化 hand2cam 生成，可能携带 F1 误差** |
| **extristric** | solvePnP 的 (rmat,tvec)，T_cam_tag（tag 在 cam 中的位姿），按 `[tvec;rmat]` 堆叠为 4×3 | tag→cam | N/A | `calibration_node.py:259` | calib `:156`（仅加载） | 仅 calib 内部（`extristric_plane_shift`） | **否** | CF（solvePnP 约定），生产中惰性 |

> 内参 K（非刚体矩阵，但属坐标链）：YOLO 用 **depth camera_info** 的 K 且 `getOptimalNewCameraMatrix` 去畸变（`yolo_node.py:49,227-230`）；ROI 用 **rgb camera_info** 原始 `msg.k`（`roi_color_detector_node.py:70,195`）。两者 K 来源不同 → F4。

---

## 2. Camera Mounting（相机安装方式）

**结论：eye-in-hand（HCI，非 CF）。**

依据（均指向「相机随末端运动」）：
- 所有 cam→base 转换都使用**实时** `T_base_hand`：`base_T_cam = T_base_hand @ hand2cam`（`yolo_node.py:239`、`object_detection_node.py:96` `hand2base@hand2cam@target_in_cam`、`working_calibration_pose_node.py:121`）。eye-to-hand（相机固定于世界）不需要手部位姿来定位相机。
- `working_calibration_node.py:130-133` 注释明确「hand2base（末端到基座的变换矩阵）」。
- `hand2cam` 为「手→相机固定外参」（`calibration_node.py:82`），即相机固连末端。

> 按要求：仅凭 `T_hand@hand2cam` 记为 **HCI**，不视为物理事实。需用 TF/URDF/结构确认（见 §12）。真实机器人 TF 由 Orin 发布（runtime_index），PC 无法离线确认。

---

## 3. Current YOLO Equation（`yolo_node.py:236-279`）

```
T_base_hand  = /kinematics/get_current_pose           # 动态 ~5Hz，未与图像时间同步（HCI：时间错配）
hand2cam     = hand2cam_matrix 默认（理想化）          # :60-65
base_T_cam   = T_base_hand @ hand2cam                  # :239
cam_T_base   = inv(base_T_cam)                         # :240
cam_T_white  = cam_T_base @ white_area_pose_world      # :241   (white_area_pose_world = T_base_white)
H_w2i        = K_depth_undistort @ [Rcw[:,0:2] | tcw]  # :243   (Rcw,tcw 来自 cam_T_white)
H_i2w        = inv(H_w2i)                              # :244
[X,Y]        = H_i2w @ [u,v,1]（归一化）               # :259-261   (u,v)=bbox 中心 :351
if y_flip: Y = -Y                                      # :262   y_flip=True
p_world_xy   = (Rw @ [X,Y,0]) + tw                     # :264-265  (Rw,tw 来自 white_area_pose_world)
z            = pick_plane_z = 0.030（+ droop）          # :268,275-276
→ pose.xyz = [p_world_xy.x, p_world_xy.y, z]
```
特征：depth K（去畸变）｜y_flip=True｜理想化 hand2cam｜动态 T_base_hand｜**Z 固定平面**。

## 4. Current ROI Equation（`roi_color_detector_node.py:164-274`）

```
white_area_pose_cam = T_cam_white（静态，transform.yaml）  # :172
K_rgb               = msg.k（原始，rgb camera_info）        # :195
(u,v)               = 轮廓质心                              # :347
ray                 = [(u-cx)/fx, (v-cy)/fy, 1]            # :250
n                   = white_area_pose_cam[:3,2]（cam 系法向量）
p0                  = white_area_pose_cam[:3,3]（cam 系平面点）
t                   = dot(n,p0)/dot(n,ray)                 # :257
p_cam               = t*ray                                # :258
white_T_cam         = inv(white_area_pose_cam)             # :260
[X,Y]               = white_T_cam[:3,:3]@p_cam + white_T_cam[:3,3]  # :261   y_flip=False
p_world_xy          = (Rw @ [X,Y,0]) + tw                  # :265-266  (Rw,tw 来自 white_area_pose_world)
z                   = z_up = 0.030（+ droop）               # :267,272-273
→ pose.xyz = [p_world_xy.x, p_world_xy.y, z]
```
特征：rgb K（原始）｜y_flip=False｜**静态 white_area_pose_cam（不用手部位姿）**｜Z 固定平面。

> YOLO vs ROI 差异（即 5 cm 偏差来源）：外参（动态理想化 vs 静态）｜K（depth 去畸变 vs rgb 原始）｜y_flip（True vs False）｜中心（bbox vs 轮廓质心）。共享 white_area_pose_world 与 z=0.030。

## 5. Candidate Unified Equation（Phase 1A 离线参考实现；Phase 2 加 depth 对照）

```
T_base_hand(t_img) = /kinematics/get_current_pose，采样于【图像时间戳】（插值/最近邻 + lag 门控）
hand2cam_cal       = transform.yaml:hand2cam_transformation_matrix（T_hand_cam，calibrated estimate，非 ground truth）
base_T_cam         = T_base_hand(t_img) @ hand2cam_cal
p_cam              = (Phase 2) depth 反投影(K_rgb, aligned_depth, mask)
                     (Phase 1 过渡) 平面相交(K_rgb, ray, base_T_cam, white_plane)   # 仅用于对照，不上线
p_base             = base_T_cam @ [p_cam; 1]
→ pose.xyz=p_base.xyz ; frame_id=base ; timestamp=t_img ; pose_confidence=f(valid_ratio, var, lag)
```
Phase 1A 关键：统一链为**离线计算**（从录制的 image/T_base_hand 流/K 计算），**不改生产代码、不替换运行节点、不改消息架构**。它把 hand2cam 换为 calibrated estimate、K 与 y_flip 统一，从而**单独隔离 F1/F3/F4**。候选链是否更优由 1A 误差表判定；**仅证明更优后才进入 Phase 1B 改生产节点**。

> ⚠ **不预设「真实 depth 一定比平面投影更准」**：历史上曾用 RGB-D（见 §15）发现深度不准才改平面。Phase 1 保持平面投影；depth 是否回归须由 §16 对照实验证明。

---

## 6. Ground-Truth Method（真实坐标基准）

| 方法 | 目标点物理定义 | 测量误差 | 复现性 | 评价 |
|---|---|---|---|---|
| 人工尺测/卡尺 | 标记点距 base 原点 | 尺~5mm；卡尺~1mm | 中 | 简单、与视觉链独立；适合锚定 |
| 打印网格板 | 网格交点（已知间距） | ~1-2mm（打印机） | 高 | 相对位置准，需锚定 base 原点 |
| **AprilTag 标定板（推荐）** | tag 几何中心（亚像素检测） | <1mm（检测）+ 板锚定 | **高** | 既给亚像素图像定位，又给已知几何；可与尺测锚定组合 |
| 固定孔位板 | 孔中心（机械加工） | <1mm | 高 | 需定制加工 |
| 已知机械定位点 | 机械结构特征点 | 取决于结构 | 中 | 受可触及性限制 |

**base frame 原点**：以机械臂 base 安装点为原点（与 `/kinematics/get_current_pose` 一致；待 TF 确认 base frame 名称）。

**避免语义中心不一致（关键）**：YOLO 返回 bbox 中心、ROI 返回轮廓质心、物体有物理中心——三者**不是同一点**。处理：
- 每个目标**物理标记唯一参考点**（如贴 AprilTag / 标记点于物体几何中心）。
- 数据 schema 同时记录 `bbox_center`、`contour_centroid`、`physical_center` 三列，使 F6（中心差异）**被量化而非被隐藏**。
- 以 AprilTag 中心为基准时，tag 中心 = 精确定义的物理点，直接消除歧义。
- ⚠ **亚像素精度边界**：AprilTag 的亚像素精度**仅针对图像中心定位与板内 tag 间相对坐标**；**base-frame ground truth 仍依赖 board→base 的机械锚定与尺测误差**（尺~5mm / 卡尺~1mm）。即 tag 检测噪声 ≠ base 系真值噪声；两量须分开记录。

**推荐可复现方案**：打印 **3×3 AprilTag 网格板**（tag36h11，与 `calibration_node.py:137` 一致），固定于工作区；用尺测锚定网格原点到 base 原点；tag 中心提供亚像素 ground truth。物体类目标（杯）在顶部贴 tag 或置于已知 tag 格点，统一参考点。

---

## 7. Test A — Fixed Camera / Workspace Grid

- 相机与机械臂**保持固定观测姿态**（与生产观测姿态一致）。
- 工作区布置 **≥3×3 = 9 个已知 base 坐标点**（AprilTag 网格板）：覆盖 左/中/右 × 近/中/远。
- 每点采集 **≥30 帧**。
- 每帧分别记录 **YOLO pose、ROI pose、候选统一链 pose**（统一链由录制数据离线计算）。
- 样本数：**9 点 × 30 帧 = 270 帧 × 3 链**。
- 目的：量化各链在工作区的**系统性偏差（bias）与面内精度**，分离 F1/F2/F3/F4。

## 8. Test B — Fixed Target / Multiple Camera Poses

- **同一目标**保持在同一 base 坐标（不动）。
- 使用 **≥3 个不同机械臂/相机观测姿态**（改变 T_base_hand）。
- ⚠ **需用户执行真实机械臂低速换姿态**；**不进行抓取**；每个姿态**到位停止后静态采集**，避免首轮引入运动-时间同步问题。
- 每姿态 **≥30 帧**（静态）。
- 将每姿态估计转换到 base frame，检查**同一固定目标的 base-frame 坐标是否跨姿态一致**。
- 样本数：**1 目标 × ≥3 姿态 × 30 帧 = ≥90 帧 × 3 链**。
- 目的：验证 hand-eye（hand2cam）与**动态坐标链**——若 eye-in-hand 且 hand2cam 正确，不同姿态下 base-frame 坐标应一致；不一致则定量 hand2cam 误差。直接检验 F1（默认 vs 标定 hand2cam estimate）与 F2（ROI 静态链在臂移动后是否漂移）。

## 9. Optional Test C — Height Test

- 同一 XY 位置，使用 **0 / 20 / 40 mm 已知高度**垫高目标（不抓取）。
- 每高度 **≥30 帧**。
- 量化**固定平面 Z=0.030 假设**造成的 XY/Z 误差（F5）。
- 样本数：**1 XY × ≥3 高度 × 30 帧 = ≥90 帧**。
- 目的：单独量化平面 Z 假设误差；为 Phase 2 引入真实深度提供动机与基线。

---

## 10. Data Schema（每条样本字段）

| 字段 | 说明 |
|---|---|
| sample_id | 唯一样本 ID |
| test_type | A / B / C |
| target_id | 目标/tag ID |
| arm_pose_id | 机械臂观测姿态 ID（Test B） |
| frame_index | 帧序号 |
| image_timestamp | 图像 header.stamp |
| hand_pose_timestamp | T_base_hand 采样时间 |
| pose_age_ms | 图像与手位姿时间差（同步质量） |
| expected_x/y/z | ground-truth base 坐标 |
| yolo_x/y/z | YOLO 估计 |
| roi_x/y/z | ROI 估计 |
| unified_candidate_x/y/z | 候选统一链估计（离线） |
| bbox_center_u/v | YOLO bbox 中心像素 |
| contour_centroid_u/v | ROI 轮廓质心像素 |
| physical_center_u/v | 物理参考点像素（标记/tag） |
| class_name | 类别 |
| color | 颜色 |
| confidence | 置信度 |
| source | yolo_only / roi_only / yolo_roi_fused / unified |
| frame_id | base |

---

## 11. Metrics

**Accuracy（vs ground truth）**：bias_x/y/z、MAE_x/y/z、RMSE_xyz、max_error（分链、分轴）。
**Repeatability（同点同姿态）**：std_x/y/z、range_x/y/z。
**Cross-chain**：d(YOLO,ROI)、d(YOLO,unified)、d(ROI,unified)。
**Pose consistency（Test B）**：同一固定目标跨机械臂姿态的 base-frame 坐标差异（均值/范围）。

**验收阈值分层（不预先硬结 RMSE<1cm 为结论）**：
- **当前基线**：由 Test A/B/C 实测得到（含尺测不确定度、时间同步抖动、tag 检测噪声）。
- **建议目标**：统一链相对旧链改善量（如 bias 减半、跨链差 < 当前 1/2）。
- **最终验收阈值**：在确认**测量不确定度**（尺/卡尺分辨率、pose_age 抖动、tag 噪声）后设定；候选参考平面 RMSE ≈ 1 cm 量级，但需以基线+不确定度校准后最终确定。

---

## 12. Commands Requiring User Execution（只读 ROS，用户执行；本轮不自启 ROS）

> 目的：补全 §13 Unknowns（多为 Orin 相机驱动/TF，PC 无法离线确认）。Jetson 只读。

```bash
# 1. 列出所有 depth_cam topic，确认是否存在 aligned depth
ros2 topic list | grep -i depth_cam

# 2. RGB / depth 分辨率、内参、frame_id、optical frame
ros2 topic echo /depth_cam/rgb/camera_info --once
ros2 topic echo /depth_cam/depth/camera_info --once

# 3. 深度编码（type/encoding）
ros2 topic echo /depth_cam/depth/image_raw --no-arr --once | head -20

# 4. 频率
ros2 topic hz /depth_cam/rgb/image_raw
ros2 topic hz /depth_cam/depth/image_raw

# 5. 确认 eye-in-hand：相机 link 是否为机械臂末端的子帧（随臂动）
ros2 run tf2_tools view_frames        # 生成 frames.pdf，查 camera_*_link 的父帧
ros2 run tf2_ros tf2_echo base_link <camera_link>

# 6. base frame 名称与 EE frame 名称（核对 T_base_hand 的真实帧）
ros2 topic echo /controller_manager/joint_states --once
ros2 run tf2_ros tf2_echo base_link <end_effector_link>

# 7. Orbbec 驱动是否启用 align/registration（在 Orin 查 launch/参数）
#    （Gemini = OrbbecSDK/orbbec_camera；查 align_depth / depth_align 参数）
```

---

## 13. Unknowns

| 编号 | 未知 | 确认手段 | 影响 |
|---|---|---|---|
| U-1 | 是否存在 aligned depth topic（Phase 2 前置） | §12 命令 1/3 | Phase 2 |
| U-2 | RGB 与 depth 是否同一 optical frame / 已注册 | §12 命令 2/5 | Phase 2 像素对应 |
| U-3 | 相机是否 eye-in-hand（物理事实） | §12 命令 5 | 动态链前提（现 HCI） |
| U-4 | T_base_hand 的 EE 物理帧身份 | §12 命令 6 | hand2cam 标定一致性 |
| U-5 | 深度编码/分辨率/频率 | §12 命令 2/3/4 | Phase 2 |
| U-6 | Orbbec 是否启用 align/registration | §12 命令 7 | Phase 2 |
| U-7 | base frame 真实名称 | §12 命令 6 | ground-truth 锚定 |
| U-8 | white_area_pose_world 因用理想化 hand2cam 生成而携带的误差量 | Test A/B | 统一链是否需重标 |

---

## 14. Recommended Next Task

**Phase 1A — Baseline Measurement（不改生产代码）：**
1. 用户先用 §12 命令补全 U-1~U-7（相机/TF/帧名）。
2. 制作 3×3 AprilTag 网格板，尺测锚定 base 原点（注意 §6 的 board→base 锚定误差边界）。
3. 编写**离线录制 + 离线统一链计算**脚本（录制 `/world_model/objects`、`/world_model/roi_objects`、`/kinematics/get_current_pose`、图像、camera_info；离线重算候选统一链）——此为**新分析工具**，非生产代码改动。
4. 跑 Test A/B（可选 C），制误差表，量化 F1–F6，得到当前基线。
5. 据基线 + 不确定度，定最终验收阈值。

**Phase 1B — Coordinate-Chain Implementation（仅在 1A 证明候选链更优后）：**
6. 仅当 1A 误差表证明候选统一链（calibrated hand2cam estimate + 一致 K/y_flip）显著优于旧链，才修改生产节点；仍**不改消息架构**。`hand2cam_transformation_matrix` 为 calibrated estimate，**实测前不作 ground truth**。

**停止点**：本轮为测量设计。未改生产代码/参数/launch；未启动 ROS；未发消息；未操作硬件。等待用户审查与批准。

---

## 15. Historical RGB-D Investigation（历史 RGB-D 调查）

> 触发：用户指出「此前试过 RGB+Depth，深度坐标不准，才改用平面」。本节用源码 + git（只读）核实。

**发现**：`src/vision_yolo/vision_yolo/yolo_node.py.bak`（类 `YoloToWorldNode`，节点名 `simple_yolo_node`）是**已提交的旧 RGB-D 版本**；当前已提交 `yolo_node.py`（`AppCompatibleYoloNode`，平面版，0 处 `depth/image_raw`）已替换它。两者均被 git 跟踪（`git ls-files` 确认）。即：**生产 RGB-D 定位实现曾存在，已被平面投影取代，旧代码以 `.bak` 保留。** 这与用户历史观察一致。
- git 证据：`git log -- src/vision_yolo/vision_yolo/yolo_node.py` 仅 2 次提交；`git show HEAD:...yolo_node.py` 为平面版（`_pix_to_world_on_plane`），`grep depth/image_raw` 计数=0；`git diff --stat` 对当前文件为空（已提交）。
- 仓库/docs 中**未发现** depth 误差测试数据或「放弃 depth」的决策记录（`grep dev_log` 无命中）→ 决策仅存于代码 + 用户记忆，**建议本轮补记**。

**对历史 RGB-D 实现的逐项回答（基于 `.bak` 源码）**：

| 问题 | 回答 | 证据（`yolo_node.py.bak`） |
|---|---|---|
| 历史上是否真有生产 RGB-D 实现？ | **是**。`.bak` 为完整 RGB-D→world 节点，曾为 `simple_yolo_node` 入口 | `:41-57,147` |
| 单点深度还是区域统计？ | **bbox 中心 + 5×5 中位数**（`median_kernel=5`）；整 bbox depth 仅用于形状(bulge)分类，不用于 pose | `_depth_at :346-360`；`:258-263,399-410` |
| RGB 与 Depth 是否对齐？ | **假定对齐但未校验**；rgb 与 depth 来自不同 optical frame，无时间同步（用缓存 depth） | `:147,202-203,261,295` |
| RGB 内参还是 Depth 内参？ | **Depth K**（`/depth_cam/depth/camera_info`），与 depth 图一致，但与 rgb 检测像素未必同帧 | `:62,196,300-304` |
| 深度单位转换？ | **有**：假定 16UC1 mm，`img/1000.0`（mm→m），`depth_in_meters=False` | `:66,203` |
| 是否处理 0/NaN/边缘/背景？ | **仅中心 patch 过滤 `isfinite & >0`**；无边缘剔除、无背景分离、无空洞填补 | `:351-357` |
| 误差类型？ | **混合**：系统（rgb-depth 光学帧不对齐 + depth K 用于 rgb 像素）+ 随机（反光/透明面深度噪声）+ 语义（bbox 中心落空心杯口/背景） | 推断（HCI），与用户「深度不准」一致 |
| 平面投影解决了哪类误差？ | 消除：深度噪声、rgb-depth 对齐依赖、空心/反光面深度失败。代价：Z 假设（高物不准）+ 依赖 white_area_pose 标定 | 对比 `.bak` 与当前 `_pix_to_world_on_plane` |
| 是否保留测试数据/误差结果？ | **未在 repo/docs 发现** | `grep` dev_log/docs 无命中 |

> 关键教训：旧实现的失败可能**部分来自实现缺陷**（假定对齐、中心点落空洞、无背景分离），而非「深度本身不可用」。故 Phase 2 须用**正确实现**的对照（C 区域鲁棒 depth）再下结论。

---

## 16. Phase 2 重构 — 对照实验（不直接替换为 depth）

> **原则**：只有实测证明 depth 方案优于统一平面方案，才允许改生产位姿来源。

**对照方案**：
- **A. 统一后的平面投影**（Phase 1 产物，标定 hand2cam + 一致 K/y_flip）— 基准。
- **B. bbox 中心单点 depth**（复现旧 `.bak` 思路，用于定位旧失败原因）。
- **C. bbox 区域鲁棒 depth**（mask/bbox 内中位数 + 边缘剔除 + 空洞填补 + 背景分离）。
- **D. 平面 + depth 高度混合**（XY 用平面，Z 用 depth；或平面约束 + depth 修正）。

**前置**：U-1/U-2/U-6（aligned depth、rgb-depth 同帧、Orbbec align）必须先确认；无 aligned depth 则 B/C/D 无法公平实现。

**每方案评估维度**：

| 维度 | A 平面 | B 中心点 depth | C 区域鲁棒 depth | D 混合 |
|---|---|---|---|---|
| XY accuracy | 依赖标定（基线） | 受对齐/噪声影响 | 较稳（待测） | 平面基础 |
| Z accuracy | 假设值（高物差） | 空洞/反光易错 | 待测 | depth 修正 |
| 静态抖动 | 低 | 高（单点） | 待测 | 中 |
| 反光/透明材质 | 不受影响（Z 假设） | 严重 | 待测 | 部分 |
| 空心杯（杯口/内部） | 不受影响 | 中心落空洞→错 | 边缘/表面采样待测 | 待测 |
| 物体边缘 | 中心点 | 含背景风险 | erode 后改善 | — |
| 不同工作距离 | 平面标定范围内稳 | 随距离噪声↑ | 待测 | 待测 |
| RGB-depth 对齐误差 | 不依赖 | **关键依赖** | 关键依赖 | 部分依赖 |
| 计算延迟 | 低 | 低 | 中（统计） | 中 |
| 对抓取成功率影响 | 当前基线 | 待测 | 待测 | 待测 |

**判定规则**：在同一 Test A/B/C 数据集上，C（或 D）需在 XY RMSE、Z 误差、静态抖动、反光/空心场景上**全面不劣于 A 且至少一维显著更优**，方可提议替换；否则保留 A。最终阈值以 Phase 1 基线 + 测量不确定度确定（不预设 RMSE<1cm）。

> 本节**取代**主报告/早期表述中「Phase 2 = 直接用 aligned depth」的任何措辞。
