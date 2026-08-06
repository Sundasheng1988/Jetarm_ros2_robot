# 2026-08-04 Toy Block Perception Dev Log

> **项目**：机器人开发 / JetArm 真实任务闭环  
> **开发主题**：积木离线分割、颜色建模、ROS 2 实时相机识别  
> **日期**：2026-08-04  
> **环境**：Ubuntu 22.04 / ROS 2 Humble / PC + Orin / Cyclone DDS / `ROS_DOMAIN_ID=23`

---

## 1. 当日目标

围绕“固定工作区内的真实积木识别”完成一条可验证的视觉链路：

```text
RGB 图像
→ 自动定位 bbox
→ 生成 core seed
→ HSV 轮廓分割
→ 颜色分类
→ ROS 2 实时发布
```

当日任务限定为视觉识别，不接入机械臂动作、底盘控制或抓取执行。

---

## 2. 数据与工作区

主要数据目录：

```text
/home/sundasheng/ros2_ws/vision_data/task_mvp1_toy_blocks/
```

漫射光单体数据：

```text
diffuse_light/objects/
├── B01 blue：5 张
├── B02 green：5 张
├── B03 yellow：5 张
├── B04 cyan：5 张
├── B04 cyan no-glare：1 张
└── B05 purple：5 张
```

合计 26 张图片，分辨率统一为 `640 × 480`。

旧实验目录和失败输出已完成审查，并提出“保留原始数据、旧结果先归档、不直接删除”的清理方案；本日志不确认是否已经实际执行归档移动。

---

## 3. B04 单图分割原型

### 3.1 初始候选方案

对 B04 青色无强反光图片测试了三种方法：

1. GrabCut
2. LAB Delta-E
3. HSV Hue + Saturation

### 3.2 结果

| 方法 | 结果 |
|---|---|
| GrabCut | 吞入右侧背景和底部阴影，拒绝 |
| LAB Delta-E | 右侧背景泄漏，拒绝 |
| HSV H+S | 主体轮廓最完整，选为主方案 |

最终采用：

```text
core seed
+ HSV Hue/Saturation
+ object gate
+ 与 core seed 重叠最大的连通域
+ 连通域选定后填孔
```

### 3.3 Core seed 定义

`core seed` 是“高可信的物体内部像素”，不是最终物体轮廓。

用途：

- 估计 Hue 中心；
- 估计 Saturation 下限；
- 锚定目标连通域；
- 避免选择背景或 AprilTag。

其紫色可视化区域不要求贴合物体边缘，排除凸点、暗孔、高光和不稳定边缘属于预期行为。

### 3.4 B04 gated 结果

B04 HSV mask 已能：

- 包住积木主体；
- 排除大部分背景和 AprilTag；
- 输出 bbox、centroid、solidity、extent；
- 作为几何定位候选使用。

当日确认：

```text
单图离线分割：通过
像素级高质量训练标签：仍需人工精修底边
```

---

## 4. 26 张自动批处理

生成通用批处理程序：

```text
annotation_pipeline/batch_block_hsv_test.py
```

每张图片独立执行：

```text
自动 locate
→ 自动 bbox
→ outer ROI / object gate
→ core seed
→ HSV mask
→ metrics
→ overlay
```

输出包括：

```text
summary.csv
summary.json
all_hsv_overlays_contact_sheet.png
group_contact_sheets/
items/<image_id>/
```

### 4.1 首轮结果

```text
AUTO_OK：25
REVIEW：1
FAILED：0
```

唯一 REVIEW：

```text
B01_blue_2x2_diffuse_center_4.png
reason = touches_object_gate_edge:bottom
```

### 4.2 B01 frame 4 对照试验

将 object gate margin 从 5 px 改为 8 px 单独重跑。

结果：

- mask 像素数不变；
- bbox 不变；
- centroid 不变；
- solidity、extent 不变；
- 不再触碰 object gate；
- 状态由 `REVIEW` 变为 `AUTO_OK`。

最终离线批次结论：

```text
26 / 26 自动定位成功
26 / 26 分割通过
0 FAILED
```

该结果适用于：

```text
固定相机
漫射光
单个积木
中心工作区
```

---

## 5. 五种颜色建模

从合格 core seed 统计 HSV 与 LAB，获得当前数据集的颜色中心：

| 颜色 | OpenCV Hue 中心均值 | 核心像素 H 范围 |
|---|---:|---:|
| yellow | 21.289 | 21–22 |
| green | 68.509 | 66–70 |
| cyan | 98.132 | 96–100 |
| blue | 105.712 | 104–108 |
| purple | 138.289 | 135–142 |

### 5.1 离线分类验证

使用最近 Hue 中心分类：

```text
26 / 26 正确
```

使用留一法验证：

```text
26 / 26 正确
```

最小留一法置信间隔约：

```text
6.46 OpenCV Hue units
```

### 5.2 蓝色与青色

当前数据中：

```text
cyan H center ≈ 98.13
blue H center ≈ 105.71
临时边界 ≈ 101.9
```

LAB 可作为蓝青二次确认：

```text
cyan: a ≈ 115–118, b ≈ 107–109
blue: a ≈ 129–130, b ≈ 101–102
```

当日生成分析报告：

```text
color_profile_analysis.xlsx
```

### 5.3 限制

这些 26 张图片主要为同位置连续帧，相关性较高，尚未覆盖：

- 多位置；
- 多角度；
- 多距离；
- 不同曝光；
- 手部遮挡；
- 多积木场景。

因此颜色中心只能作为当前固定场景的离线原型参数。

---

## 6. ROS 2 实时识别包

创建 ROS 2 Python 包：

```text
~/ros2_ws/src/toy_block_perception/
```

核心文件：

```text
toy_block_perception/
├── package.xml
├── setup.py
├── setup.cfg
├── config/toy_block_color_detector.yaml
├── launch/toy_block_color_detector.launch.py
└── toy_block_perception/
    ├── classifier_core.py
    └── color_detector_node.py
```

### 6.1 话题

订阅：

```text
/depth_cam/rgb/image_raw
```

发布：

```text
/toy_block/detection
/toy_block/debug_image
/toy_block/mask
```

默认配置：

```text
process_hz = 5.0
expected image = 640 × 480
gate_margin_px = 8
stable_frames = 3
```

节点只发布视觉结果，不发布：

```text
/cmd_vel
/servo_controller
/grasp
```

---

## 7. 构建与启动

### 7.1 构建问题

首次使用：

```bash
colcon build \
  --packages-select toy_block_perception \
  --symlink-install
```

失败：

```text
error: option --editable not recognized
```

当前环境采用普通安装构建：

```bash
cd ~/ros2_ws

source /opt/ros/humble/setup.bash

rm -rf \
  build/toy_block_perception \
  install/toy_block_perception

colcon build \
  --packages-select toy_block_perception

source install/setup.bash
```

### 7.2 启动

```bash
ros2 launch toy_block_perception \
  toy_block_color_detector.launch.py
```

节点成功启动，订阅相机 RGB 图像并以约 5 Hz 处理。

Cyclone DDS 提示：

```text
NetworkInterfaceAddress: deprecated element
```

属于配置弃用警告，没有阻止通信和识别。

---

## 8. 实时单目标测试

### 8.1 青色积木

青色表现最好，典型结果：

```text
color = cyan
confidence ≈ 0.82–0.91
center ≈ [330, 198]
angle ≈ -20° ～ -24°
status = AUTO_OK
stable = true
```

验证通过：

- 相机订阅；
- 自动 bbox；
- 实时 mask；
- cyan 分类；
- centroid；
- minAreaRect 角度；
- 多帧稳定判定。

### 8.2 Debug image

`/toy_block/debug_image` 验证：

```text
encoding = bgr8
width = 640
```

`image_tools showimage` 可连续接收并正常显示。

`rqt_image_view` 偶发灰屏，判定为查看器显示问题，不是 publisher 或识别节点故障。

---

## 9. 实时压力测试结果

进一步进行了：

- 移动积木；
- 改变位置和角度；
- 更换五种颜色；
- 手套遮挡；
- 多积木同时进入画面；
- 积木位于工作区边缘。

结果表明实时架构可运行，但离线规则不能直接作为生产感知算法。

### 9.1 已验证能力

| 项目 | 状态 |
|---|---|
| ROS 2 相机订阅与图像发布 | 通过 |
| 中心区域静止单积木分割 | 基本通过 |
| 青色实时识别 | 通过，表现最好 |
| 黄色静止识别 | 可进入 AUTO_OK + STABLE |
| 蓝色识别 | 多数可识别，但 blue/unknown 闪烁 |
| 紫色识别 | 部分可识别，角度/曝光变化后易 UNKNOWN |
| 绿色识别 | 受 core seed 规则影响严重 |

### 9.2 主要故障：Core seed 过严

日志大量出现：

```text
Core seed too small: opened=0, raw=0
Core seed too small: opened=0, raw=3
Core seed too small: opened=0, raw=16
```

累计失败数达到数百次。

当前 seed 来自：

```text
bbox 内缩矩形
+ 固定 L/chroma/local-std 阈值
```

问题：

- 暗绿色内部像素被大量过滤；
- 实时曝光变化后固定阈值失效；
- bbox 内可能含背景、暗面、凸点和阴影；
- 移动或遮挡期间更容易 seed 为空。

### 9.3 Locator 范围过窄

当前 locator 本质上只选择：

```text
中心附近最可信的一个彩色连通域
```

不具备：

- 多实例输出；
- 手部排除；
- 物体身份跟踪；
- 固定工作区 polygon 约束；
- 任务目标选择。

因此手套、画面边缘积木或多个积木会成为干扰候选。

### 9.4 实时颜色模型泛化不足

紫色离线中心：

```text
H ≈ 138
```

实时部分帧：

```text
H ≈ 130
```

超过固定最大 Hue 距离，导致同一紫色积木在 `purple` 和 `unknown` 间切换。

结论：

```text
单一颜色中心点不足
需要区间/方差模型及更多真实姿态数据
```

### 9.5 稳定性逻辑漏洞

日志出现过：

```text
color=yellow
confidence=0.35
status=AUTO_OK
stable=True
```

说明当前 AUTO_OK/stable 未设置最低分类置信度门槛。

应增加：

```text
confidence >= 0.65
```

否则低质量分类可能被标记为稳定。

### 9.6 UNKNOWN 置信度语义错误

出现：

```text
color=unknown
confidence=0.88
```

原因是候选匹配分数与最终接受状态未分离。

建议输出：

```json
{
  "candidate_color": "blue",
  "match_score": 0.88,
  "accepted": false,
  "color": "unknown",
  "reject_reasons": []
}
```

只有 `accepted=true` 才允许作为正式颜色结果。

### 9.7 正方形积木角度存在 90° 二义性

B01/B05 等正方形积木出现：

```text
-78°
+82°
+84°
```

这是 `minAreaRect` 在接近正方形时切换长短边导致，并不一定代表物体真实旋转。

后续应增加：

```text
angle_valid = false
orientation_symmetry_deg = 90
```

长方形积木的角度结果才具有较稳定的唯一性。

---

## 10. 当日技术结论

### 10.1 已完成

```text
✅ B04 单图 HSV 分割
✅ 26 张自动 bbox 与 mask 批处理
✅ 五种颜色离线 profile
✅ 26/26 离线颜色分类验证
✅ ROS 2 实时相机订阅
✅ mask / debug image / detection 发布
✅ 青色静止单目标实时识别
✅ centroid、bbox、角度和多帧状态发布
```

### 10.2 尚未完成

```text
🔶 暗色和曝光变化下的自适应 core seed
🔶 位置和角度泛化
🔶 稳定可靠的五色实时分类
🔶 固定工作区 polygon
🔶 多积木实例输出
🔶 遮挡与手部干扰处理
🔶 正方形角度有效性定义
❌ 作为机器人抓取输入的生产级感知
```

### 10.3 当前阶段定位

当前系统应定义为：

> **固定相机、固定工作区、单个静止彩色积木场景下可运行的 ROS 2 视觉原型。**

不能定义为：

> **支持任意位置、多物体、遮挡和机器人执行的生产感知节点。**

---

## 11. 下一步 P0

按优先级执行：

### P0-1：重构 core seed

改为：

```text
selected_component_mask
→ 腐蚀或距离变换
→ 获取目标内部区域
→ 基于内部像素自身的 S/chroma 分位数自适应筛选
→ core seed
```

不再只从 bbox 内缩矩形生成 seed。

### P0-2：增加固定工作区 mask

人工标定黑色矩形四角，生成 polygon mask：

```text
只在工作区内部参与 locate
```

后续再考虑由 AprilTag 或平面标定自动生成。

### P0-3：修正接受与稳定逻辑

必须要求：

```text
classification_status == CLASSIFIED
confidence >= 0.65
review_status == AUTO_OK
```

才能进入 stable。

### P0-4：修正 UNKNOWN 输出

区分：

```text
candidate_color
match_score
accepted
final color
reject_reasons
```

### P0-5：处理角度二义性

对于接近正方形的 mask：

```text
angle_valid = false
orientation_symmetry_deg = 90
```

### P0-6：补采真实数据

每种颜色至少采集：

```text
3 个位置
× 3 个方向
× 2 个亮度条件
```

保存稳定帧用于重新拟合颜色 profile。

### P1：多目标实例化

在单目标规则稳定后，再扩展：

```text
所有合法连通域
→ 每个实例独立 seed / mask / color / centroid / angle / id
→ 任务层选择目标
```

---

## 12. 推荐下一次开发入口

下一次 session 首先阅读：

```text
~/ros2_ws/src/toy_block_perception/
├── toy_block_perception/classifier_core.py
├── toy_block_perception/color_detector_node.py
├── config/toy_block_color_detector.yaml
└── launch/toy_block_color_detector.launch.py
```

同时保留以下验证材料：

```text
vision_data/task_mvp1_toy_blocks/
├── diffuse_light/objects/
├── annotation_pipeline/batch_hsv_results_auto_v1/
└── color_profile_analysis.xlsx
```

下一次首要任务：

> **先修复 component-based adaptive core seed，再重新进行绿色、紫色和多位置静止测试。**
