# Robot Runtime v0.1 — TargetObject

> 设计目标：定义统一的物体描述数据结构，屏蔽视觉源差异
> 关联：`runtime_task_schema.md`、`runtime_skill_interface.md`

---

## 1. 问题背景

当前系统使用通用 JSON dict 描述目标物体，各视觉源输出格式不一致：

| 来源包 | 话题 | 字段 | 差异 |
|--------|------|------|------|
| `vision_yolo` | `/world_model/objects` | `id, class_name, pose, confidence, updated_at` | 无 `color`；`rpy` 固定 `[0,0,1.57]` |
| `roi_color_detector` | `/world_model/roi_objects` | `id, class_name, color, pose, confidence, updated_at, angle_deg` | 多 `color`、`angle_deg` |
| `wm_dummy_pub` | `/world_model/objects` | `id, class_name, color, pose, confidence, updated_at` | 硬编码固定值 |
| `wm_from_tf` | `/world_model/objects` | `id, class_name, color, pose, confidence, updated_at` | TF frame 推断 class/color |

Skill / Executor / Verification 不应关心数据来自哪个视觉源。需要一个统一的 `TargetObject` 结构。

---

## 2. TargetObject 数据结构

```python
# sketch_runtime/target_object.py
from dataclasses import dataclass, field
from typing import Optional, Dict
import time

@dataclass
class TargetObject:
    # ── 标识 ──
    object_id: str = ""                       # "obj_a1b2c3" 全局唯一，带前缀（"yolo_"/"roi_"/"dummy_"）

    # ── 语义描述 ──
    class_name: str = ""                      # "cup", "ball", "cube", "bottle", "box"
    color: str = ""                           # "red", "blue", "yellow", "green", "black", "white"
    name: str = ""                            # 可读名称，例："红色杯子"

    # ── 图像坐标（像素） ──
    center_u: float = 0.0                     # 图像中心 u (columns)
    center_v: float = 0.0                     # 图像中心 v (rows)
    center_z: float = -1.0                    # 深度 z (米), -1.0 = 未知

    # ── 世界坐标（米 + 弧度） ──
    world_frame: str = "base"                 # 参考坐标系名
    world_x: float = 0.0
    world_y: float = 0.0
    world_z: float = 0.0
    world_roll: float = 0.0
    world_pitch: float = 0.0
    world_yaw: float = 0.0                    # 朝向角（用于方块放置对正）

    # ── 置信度 ──
    confidence: float = 0.0                   # 0.0 ~ 1.0

    # ── 来源标记 ──
    source: str = ""                          # "yolo_v8" | "roi_color" | "tf_bridge" | "dummy" | "manual"
    source_node: str = ""                     # "app_compatible_yolo_node" 等

    # ── 时间 ──
    detected_at: float = field(default_factory=time.time)

    # ── 扩展 ──
    metadata: dict = field(default_factory=dict)
    # 例: {"bbox": [x1,y1,x2,y2], "angle_deg": 15.3, "yolo_conf": 0.85}
```

---

## 3. 核心方法

```python
    # (续 TargetObject 类)
    def to_pose_dict(self) -> dict:
        """转为 grounding_node 兼容的 pose 格式 → 放入 /grounded_goal.source_pose"""
        return {
            "frame": self.world_frame,
            "xyz": [round(self.world_x, 4),
                    round(self.world_y, 4),
                    round(self.world_z, 4)],
            "rpy": [round(self.world_roll, 3),
                    round(self.world_pitch, 3),
                    round(self.world_yaw, 3)]
        }

    def to_skill_dict(self) -> dict:
        """转为 Skill 可直接使用的 dict"""
        return {
            "object_id": self.object_id,
            "class_name": self.class_name,
            "color": self.color,
            "source_pose": self.to_pose_dict(),
            "confidence": self.confidence,
            "source": self.source,
        }

    # ── 工厂方法 ──

    @classmethod
    def from_world_model(cls, obj: dict, source: str = "wm_json") -> "TargetObject":
        """从世界模型通用 JSON 构造（兼容现有 /world_model/* 话题）"""
        pose = obj.get("pose", {})
        xyz = pose.get("xyz", [0.0, 0.0, 0.0])
        rpy = pose.get("rpy", [0.0, 0.0, 0.0])
        return cls(
            object_id=f"{source}_{obj.get('id', 0)}",
            class_name=(obj.get("class_name") or "").lower(),
            color=(obj.get("color") or "").lower(),
            name=(obj.get("name") or f"{obj.get('color','')}{obj.get('class_name','')}").strip(),
            world_frame=pose.get("frame", "base"),
            world_x=float(xyz[0]), world_y=float(xyz[1]), world_z=float(xyz[2]),
            world_roll=float(rpy[0]), world_pitch=float(rpy[1]), world_yaw=float(rpy[2]),
            confidence=float(obj.get("confidence", 0.0)),
            source=source,
            source_node="",
            detected_at=float(obj.get("updated_at", time.time())),
            metadata={"angle_deg": obj.get("angle_deg"), "raw": obj}
        )

    @classmethod
    def from_detection_result(cls, det, idx: int,
                               world_objects: list = None) -> "TargetObject":
        """从 vision_interfaces/DetectionResult 构造（兼容 YOLO）"""
        xyz = [0.0, 0.0, 0.0]
        if world_objects and idx < len(world_objects):
            xyz = world_objects[idx].get("pose", {}).get("xyz", [0, 0, 0])
        cls_name = det.class_name[idx] if idx < len(det.class_name) else ""
        return cls(
            object_id=f"yolo_{idx}_{int(time.time())}",
            class_name=cls_name.lower(),
            center_u=float(det.center_x[idx]) if idx < len(det.center_x) else 0.0,
            center_v=float(det.center_y[idx]) if idx < len(det.center_y) else 0.0,
            center_z=float(det.center_z[idx]) if idx < len(det.center_z) else -1.0,
            world_x=float(xyz[0]), world_y=float(xyz[1]), world_z=float(xyz[2]),
            world_yaw=1.57,  # 默认朝下
            confidence=float(det.confidence[idx]) if idx < len(det.confidence) else 0.0,
            source="yolo_v8",
            source_node="app_compatible_yolo_node",
            metadata={
                "image_w": det.image_width[0] if det.image_width else 0,
                "image_h": det.image_height[0] if det.image_height else 0
            }
        )
```

---

## 4. ROS2 消息定义

```msg
# vision_interfaces/msg/TargetObject.msg
# 新增消息 — 不修改现有 DetectionResult.msg 保持向后兼容

string object_id           # 全局唯一对象 ID，例 "yolo_3_1715900000"

string class_name          # 语义类别: "cup", "ball", "cube", "bottle", "box"
string color               # 颜色: "red", "blue", "yellow", "green", "black", "white"
string name                # 可读名称: "红色杯子"

float32 center_u           # 图像坐标 u (pixel columns)
float32 center_v           # 图像坐标 v (pixel rows)
float32 center_z           # 深度 z (meters), -1.0 = unknown

string world_frame         # 世界坐标系: "base" | "table"
float32 world_x            # 世界 x (m)
float32 world_y            # 世界 y (m)
float32 world_z            # 世界 z (m)
float32 world_roll         # 姿态 roll (rad)
float32 world_pitch        # 姿态 pitch (rad)
float32 world_yaw          # 姿态 yaw (rad)

float32 confidence         # 0.0 ~ 1.0

string source              # 来源: "yolo_v8" | "roi_color" | "tf_bridge" | "dummy" | "manual"
string source_node         # 来源节点名: "app_compatible_yolo_node"

float64 detected_at        # 检测 unix timestamp

string metadata            # JSON 扩展字符串，例 '{"bbox":[100,200,300,400],"angle_deg":15.3}'
```

---

## 5. 视觉源映射表

### 5.1 vision_yolo (app_compatible_yolo_node) → TargetObject

```
DetectionResult.msg (并行数组)  →  TargetObject

class_name[idx]   →  class_name
confidence[idx]   →  confidence
center_x[idx]     →  center_u
center_y[idx]     →  center_v
center_z[idx]     →  center_z
world_objects[idx].pose.xyz  →  world_x/y/z (如果需要世界坐标)
source            →  "yolo_v8"
source_node       →  "app_compatible_yolo_node"
```

### 5.2 roi_color_detector_node → TargetObject

```
/world_model/roi_objects JSON  →  TargetObject

obj["class_name"]      →  class_name
obj["color"]           →  color
obj["pose"]["xyz"][0:3] →  world_x/y/z
obj["pose"]["rpy"][0:3] →  world_roll/pitch/yaw
obj["confidence"]      →  confidence
obj["angle_deg"]       →  metadata["angle_deg"]
source                 →  "roi_color"
source_node            →  "roi_color_detector_node"
```

### 5.3 wm_dummy_pub → TargetObject

```
/world_model/objects JSON  →  TargetObject

通过 from_world_model() 工厂方法
source → "dummy"
```

### 5.4 wm_from_tf → TargetObject

```
/tf + /tf_static → /world_model/objects JSON  →  TargetObject

通过 from_world_model() 工厂方法
source → "tf_bridge"
```

### 5.5 手动指定 (teleop / 调试)

```python
TargetObject(
    object_id="manual_001",
    class_name="cup",
    color="red",
    world_x=0.15, world_y=-0.10, world_z=0.03,
    source="manual",
    source_node="keyboard_input_node"
)
```

---

## 6. 与现有 pipeline 的兼容过渡

Sprint 1 阶段不强制所有视觉源输出 `TargetObject`。过渡策略：

```
现有状态 (兼容):
  vision_yolo           → /world_model/objects       (通用 JSON)
  roi_color_detector    → /world_model/roi_objects   (通用 JSON)
  grounding_node        → /grounded_goal             (含 source_pose)

Sprint 1 新增:
  runtime_state_node    订阅 /grounded_goal → 内部转 TargetObject → 记录日志

Sprint 2 目标:
  所有视觉节点           → /world_model/target_objects  (TargetObject[])
  grounding_node        ← /world_model/target_objects  (统一消费)
  executor / skill       ← TargetObject (通过 TaskContext)
```

---

## 7. Skill 使用 TargetObject 示例

```python
# pick_skill.py
from sketch_runtime.target_object import TargetObject

class PickSkill(BaseSkill):
    async def execute(self, ctx: TaskContext):
        obj = TargetObject.from_world_model(ctx.target_object)
        # obj.to_pose_dict() → 用于 IK
        # obj.class_name → 用于视觉确认
        # obj.confidence → 用于决定接近策略
        # obj.source     → 用于日志记录

        source_pose = obj.to_pose_dict()
        pulses = await self.adapter.ik_solve(
            [source_pose["xyz"][0], source_pose["xyz"][1], source_pose["xyz"][2] + 0.08],
            source_pose["rpy"]
        )
        # ...
```

---

## 8. 与 Teleop / RobotOps / Verification 的兼容

| 层 | 兼容点 |
|---|--------|
| **Teleop** | 手动输入 `TargetObject(source="manual")` 可复用同一 Skill 链路 |
| **Verification** | `object_id` 用于关联"抓取前/后"同一物体是否消失；`confidence` 用于设定验证阈值 |
| **RobotOps** | `source` + `source_node` 记录检测来源；`detected_at` 用于时序分析；`metadata` 含原始检测详情 |
| **未来扩展** | `metadata: dict` 可扩展：物体质量估计、抓取点候选列表、3D bounding box、点云 cluster id |
