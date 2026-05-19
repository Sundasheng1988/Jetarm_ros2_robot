import uuid
import time
from enum import Enum
from dataclasses import dataclass, field
from typing import Optional, Dict, Any, List


class TaskState(Enum):
    CREATED = "created"
    PARSED = "parsed"
    GROUNDED = "grounded"
    SKILL_SELECTED = "skill_selected"
    WAITING_CONFIRM = "waiting_confirm"
    EXECUTING = "executing"
    DONE = "done"
    FAILED = "failed"
    CANCELLED = "cancelled"
    PAUSED = "paused"
    RETRYING = "retrying"


@dataclass
class TaskContext:
    task_id: str = ""

    user_command: str = ""
    source: str = "keyboard"

    parsed_command: Optional[Dict] = None

    target_object: Optional[Dict] = None
    target_pose: Optional[Dict] = None

    selected_skill: str = ""
    skill_params: Dict = field(default_factory=dict)

    state: TaskState = TaskState.CREATED
    state_history: List[Dict] = field(default_factory=list)

    result: Optional[Dict] = None

    created_at: float = field(default_factory=time.time)
    parsed_at: Optional[float] = None
    grounded_at: Optional[float] = None
    started_at: Optional[float] = None
    completed_at: Optional[float] = None

    priority: int = 0
    retry_count: int = 0
    max_retries: int = 2

    metadata: Dict = field(default_factory=dict)

    def __post_init__(self):
        if not self.task_id:
            self.task_id = self._generate_task_id()

    @staticmethod
    def _generate_task_id() -> str:
        short_uuid = uuid.uuid4().hex[:12]
        ts = int(time.time())
        return f"task_{short_uuid}_{ts}"

    def transition(self, new_state: TaskState, detail: str = ""):
        old = self.state
        self.state = new_state
        self.state_history.append({
            "from": old.value,
            "to": new_state.value,
            "timestamp": time.time(),
            "detail": detail,
        })
        if new_state == TaskState.PARSED:
            self.parsed_at = time.time()
        elif new_state == TaskState.GROUNDED:
            self.grounded_at = time.time()
        elif new_state == TaskState.EXECUTING:
            self.started_at = time.time()
        elif new_state in (TaskState.DONE, TaskState.FAILED, TaskState.CANCELLED):
            self.completed_at = time.time()

    @property
    def elapsed_ms(self) -> float:
        end = self.completed_at or time.time()
        return (end - self.created_at) * 1000.0

    def to_dict(self) -> dict:
        return {
            "task_id": self.task_id,
            "state": self.state.value,
            "user_command": self.user_command,
            "source": self.source,
            "parsed_command": self.parsed_command,
            "selected_skill": self.selected_skill,
            "priority": self.priority,
            "retry_count": self.retry_count,
            "max_retries": self.max_retries,
            "created_at": self.created_at,
            "elapsed_ms": round(self.elapsed_ms, 1),
            "state_history": self.state_history,
            "result": self.result,
        }
