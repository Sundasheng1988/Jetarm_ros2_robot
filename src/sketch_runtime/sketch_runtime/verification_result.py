import time
from dataclasses import dataclass, field
from typing import Optional, Dict, Any, List


@dataclass
class VerificationResult:
    """Structured verification output from verification_result_node.

    Observation-only — does not control hardware, call IK, or block runtime.
    Matching priority: object_id > class_name > position proximity.
    Color is optional — verification works when color == "unknown".
    """

    task_id: str = "verification"
    stage: str = ""                   # "precheck" | "postcheck"
    success: bool = False
    confidence: float = 0.0
    reason: str = ""
    evidence: Dict[str, Any] = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)

    def to_dict(self) -> dict:
        return {
            "task_id": self.task_id,
            "stage": self.stage,
            "success": self.success,
            "confidence": round(self.confidence, 3),
            "reason": self.reason,
            "evidence": self.evidence,
            "timestamp": self.timestamp,
        }


def extract_source_xyz(target_object: dict) -> List[float]:
    """Extract source position from grounded_task_context target_object.

    Supports two formats:
    - target_object.world_x / world_y / world_z (primary)
    - target_object.pose.xyz (fallback)
    """
    wx = target_object.get("world_x")
    wy = target_object.get("world_y")
    wz = target_object.get("world_z")
    if wx is not None and wy is not None and wz is not None:
        return [float(wx), float(wy), float(wz)]
    pose = target_object.get("pose", {})
    xyz = pose.get("xyz", [0.0, 0.0, 0.0])
    if isinstance(xyz, (list, tuple)) and len(xyz) >= 3:
        return [float(xyz[0]), float(xyz[1]), float(xyz[2])]
    return [0.0, 0.0, 0.0]
