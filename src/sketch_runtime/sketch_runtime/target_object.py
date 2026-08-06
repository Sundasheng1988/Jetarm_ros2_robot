import time
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class TargetObject:
    object_id: str = ""
    class_name: str = ""
    color: str = ""
    name: str = ""

    center_u: float = 0.0
    center_v: float = 0.0
    center_z: float = -1.0

    world_frame: str = "base"
    world_x: float = 0.0
    world_y: float = 0.0
    world_z: float = 0.0
    world_roll: float = 0.0
    world_pitch: float = 0.0
    world_yaw: float = 0.0

    confidence: float = 0.0

    source: str = ""
    source_node: str = ""

    detected_at: float = field(default_factory=time.time)

    metadata: dict = field(default_factory=dict)

    def __post_init__(self):
        if not self.object_id:
            self.object_id = f"{self.source or 'unknown'}_{int(time.time())}"
        if not self.name and (self.color or self.class_name):
            parts = [p for p in [self.color, self.class_name] if p]
            self.name = "".join(parts)

    def to_pose_dict(self) -> dict:
        return {
            "frame": self.world_frame,
            "xyz": [
                round(self.world_x, 4),
                round(self.world_y, 4),
                round(self.world_z, 4),
            ],
            "rpy": [
                round(self.world_roll, 3),
                round(self.world_pitch, 3),
                round(self.world_yaw, 3),
            ],
        }

    def to_skill_dict(self) -> dict:
        return {
            "object_id": self.object_id,
            "class_name": self.class_name,
            "color": self.color,
            "name": self.name,
            "source_pose": self.to_pose_dict(),
            "confidence": self.confidence,
            "source": self.source,
        }

    @classmethod
    def from_world_model(cls, obj: dict, source: str = "wm_json") -> "TargetObject":
        pose = obj.get("pose", {})
        xyz = pose.get("xyz", [0.0, 0.0, 0.0])
        rpy = pose.get("rpy", [0.0, 0.0, 0.0])
        return cls(
            object_id=f"{source}_{obj.get('id', 0)}",
            class_name=(obj.get("class_name") or "").lower(),
            color=(obj.get("color") or "").lower(),
            name=(obj.get("name") or "").strip(),
            world_frame=pose.get("frame", "base"),
            world_x=float(xyz[0]) if len(xyz) > 0 else 0.0,
            world_y=float(xyz[1]) if len(xyz) > 1 else 0.0,
            world_z=float(xyz[2]) if len(xyz) > 2 else 0.0,
            world_roll=float(rpy[0]) if len(rpy) > 0 else 0.0,
            world_pitch=float(rpy[1]) if len(rpy) > 1 else 0.0,
            world_yaw=float(rpy[2]) if len(rpy) > 2 else 0.0,
            confidence=float(obj.get("confidence", 0.0)),
            source=source,
            source_node=obj.get("source_node", ""),
            detected_at=float(obj.get("updated_at", time.time())),
            metadata={"raw": obj},
        )

    @classmethod
    def from_detection_result(cls, det, idx: int,
                               world_objects: list = None) -> "TargetObject":
        cls_name = ""
        conf = 0.0
        cu = 0.0
        cv = 0.0
        cz = -1.0
        iw = 0
        ih = 0

        if isinstance(det, dict):
            cls_name = (det.get("class_name", []) or [])
            conf = (det.get("confidence", []) or [])
            cu = (det.get("center_x", []) or [])
            cv = (det.get("center_y", []) or [])
            cz_arr = (det.get("center_z", []) or [])
            iw = (det.get("image_width", []) or [0])[0] if det.get("image_width") else 0
            ih = (det.get("image_height", []) or [0])[0] if det.get("image_height") else 0
        else:
            cls_name = getattr(det, "class_name", []) or []
            conf = getattr(det, "confidence", []) or []
            cu = getattr(det, "center_x", []) or []
            cv = getattr(det, "center_y", []) or []
            cz_arr = getattr(det, "center_z", []) or []
            iw = (getattr(det, "image_width", []) or [0])[0]
            ih = (getattr(det, "image_height", []) or [0])[0]

        if idx < len(cls_name):
            cls_name = str(cls_name[idx]).lower()
        else:
            cls_name = ""
        if idx < len(conf):
            conf = float(conf[idx])
        else:
            conf = 0.0
        if idx < len(cu):
            cu = float(cu[idx])
        else:
            cu = 0.0
        if idx < len(cv):
            cv = float(cv[idx])
        else:
            cv = 0.0
        if "cz_arr" in dir() and idx < len(cz_arr):
            cz = float(cz_arr[idx])
        elif isinstance(det, dict) and idx < len(det.get("center_z", []) or []):
            cz = float((det.get("center_z", []) or [])[idx])

        xyz = [0.0, 0.0, 0.0]
        if world_objects and idx < len(world_objects):
            wm_pose = world_objects[idx].get("pose", {})
            xyz = wm_pose.get("xyz", [0.0, 0.0, 0.0])

        return cls(
            object_id=f"yolo_{idx}_{int(time.time())}",
            class_name=cls_name,
            center_u=cu,
            center_v=cv,
            center_z=cz,
            world_x=float(xyz[0]),
            world_y=float(xyz[1]),
            world_z=float(xyz[2]),
            world_yaw=1.57,
            confidence=conf,
            source="yolo_v8",
            source_node="app_compatible_yolo_node",
            metadata={"image_w": iw, "image_h": ih, "det_idx": idx},
        )
