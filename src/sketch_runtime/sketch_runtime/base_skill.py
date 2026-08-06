from abc import ABC, abstractmethod
from typing import Optional

from sketch_runtime.task_context import TaskContext
from sketch_runtime.execution_result import ExecutionResult


class BaseSkill(ABC):
    name: str = "base_skill"

    def __init__(self, adapter):
        self.adapter = adapter

    def precheck(self, ctx: TaskContext) -> Optional[ExecutionResult]:
        obj = ctx.target_object
        if not obj:
            return ExecutionResult(
                task_id=ctx.task_id,
                success=False,
                reason="no_target_object",
                error_detail="TaskContext.target_object is None",
            )
        return None

    @abstractmethod
    async def execute(self, ctx: TaskContext) -> ExecutionResult:
        ...

    def postcheck(self, ctx: TaskContext, result: ExecutionResult) -> ExecutionResult:
        return result

    def cleanup(self, ctx: TaskContext):
        pass
