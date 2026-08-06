from dataclasses import dataclass, field
from typing import Any, Dict
from abc import ABC, abstractmethod

from sketch_runtime.task_context import TaskContext


@dataclass
class StepResult:
    step_name: str = ""
    success: bool = False
    reason: str = ""
    wait_after_sec: float = 0.0
    evidence: Dict[str, Any] = field(default_factory=dict)


class BaseAction(ABC):
    name: str = "base_action"

    def __init__(self, step_name: str = ""):
        self.step_name = step_name or self.name

    @abstractmethod
    async def execute(self, adapter, ctx: TaskContext) -> StepResult:
        ...
