from typing import List
import asyncio

from sketch_runtime.base_action import BaseAction
from sketch_runtime.task_context import TaskContext
from sketch_runtime.execution_result import ExecutionResult


class ActionExecutor:
    @staticmethod
    async def run(actions: List[BaseAction], adapter,
                  ctx: TaskContext) -> ExecutionResult:
        ev = {
            "ik_calls": 0,
            "ik_failures": 0,
            "steps_completed": 0,
            "steps_total": len(actions),
        }
        for action in actions:
            step = await action.execute(adapter, ctx)
            ev["ik_calls"] += step.evidence.get("ik_calls", 0)
            ev["ik_failures"] += step.evidence.get("ik_failures", 0)
            if not step.success:
                return ExecutionResult(
                    task_id=ctx.task_id,
                    success=False,
                    reason=step.reason,
                    evidence=ev,
                )
            ev["steps_completed"] += 1
            if step.wait_after_sec > 0:
                await asyncio.sleep(step.wait_after_sec)
        return ExecutionResult(
            task_id=ctx.task_id,
            success=True,
            reason="all_steps_completed",
            evidence=ev,
        )
