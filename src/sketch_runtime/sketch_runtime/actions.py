from typing import List, Optional

from sketch_runtime.base_action import BaseAction, StepResult
from sketch_runtime.task_context import TaskContext


class MoveAction(BaseAction):
    name = "move"

    def __init__(self, step_name: str, position: List[float],
                 rpy: Optional[List[float]] = None,
                 duration_ms: int = 2000,
                 pitch: float = 80.0,
                 pitch_range: List[float] = None,
                 resolution: float = 1.0,
                 timeout_sec: float = 8.0):
        super().__init__(step_name)
        self.position = position
        self.rpy = rpy or [0.0, 0.0, 0.0]
        self.duration_ms = duration_ms
        self.pitch = pitch
        self.pitch_range = pitch_range
        self.resolution = resolution
        self.timeout_sec = timeout_sec

    async def execute(self, adapter, ctx: TaskContext) -> StepResult:
        pulses = await adapter.ik_solve(
            self.position, self.rpy,
            pitch=self.pitch,
            pitch_range=self.pitch_range,
            resolution=self.resolution,
            timeout_sec=self.timeout_sec,
        )
        if pulses is None:
            return StepResult(
                step_name=self.step_name,
                success=False,
                reason="ik_failed",
                evidence={"ik_calls": 1, "ik_failures": 1},
            )
        await adapter.servo_move(pulses, self.duration_ms)
        return StepResult(
            step_name=self.step_name,
            success=True,
            reason="move_completed",
            evidence={"ik_calls": 1, "ik_failures": 0},
        )


class GripperAction(BaseAction):
    name = "gripper"

    def __init__(self, step_name: str, servo_id: int = 10,
                 pulse: int = 200, duration_ms: int = 300):
        super().__init__(step_name)
        self.servo_id = servo_id
        self.pulse = pulse
        self.duration_ms = duration_ms

    async def execute(self, adapter, ctx: TaskContext) -> StepResult:
        await adapter.gripper_set(self.servo_id, self.pulse, self.duration_ms)
        return StepResult(
            step_name=self.step_name,
            success=True,
            reason="gripper_completed",
            evidence={},
        )
