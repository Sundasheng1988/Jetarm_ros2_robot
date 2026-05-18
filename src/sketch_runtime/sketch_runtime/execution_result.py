import time
from dataclasses import dataclass, field
from typing import Optional, Dict, Any


@dataclass
class ExecutionResult:
    task_id: str = ""
    success: bool = False
    reason: str = ""
    confidence: float = 0.0

    evidence: Dict[str, Any] = field(default_factory=dict)
    error_detail: Optional[str] = None
    timestamp: float = field(default_factory=time.time)

    def to_dict(self) -> dict:
        return {
            "task_id": self.task_id,
            "success": self.success,
            "reason": self.reason,
            "confidence": round(self.confidence, 3),
            "evidence": self.evidence,
            "error_detail": self.error_detail,
            "timestamp": self.timestamp,
        }
