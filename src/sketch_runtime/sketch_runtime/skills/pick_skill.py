from sketch_runtime.base_skill import BaseSkill
from sketch_runtime.task_context import TaskContext
from sketch_runtime.execution_result import ExecutionResult


class PickSkill(BaseSkill):
    name = "pick_skill"

    DEFAULTS = {
        "hover_height": 0.08,
        "approach_z": 0.015,
        "lift_height": 0.08,
        "grip_open_pulse": 200,
        "grip_close_pulse": 700,
        "move_duration_ms": 2000,
        "gripper_id": 10,
    }

    def _param(self, ctx: TaskContext, key: str):
        return ctx.skill_params.get(key, self.DEFAULTS.get(key))

    async def execute(self, ctx: TaskContext) -> ExecutionResult:
        src = ctx.target_object or {}
        src_pose = src.get("source_pose", {}) if isinstance(src, dict) else {}

        if not src_pose or not src_pose.get("xyz"):
            return ExecutionResult(
                task_id=ctx.task_id,
                success=False,
                reason="missing_source_pose",
                error_detail="target_object has no source_pose with xyz — cannot compute pick trajectory",
            )

        s_xyz = list(map(float, src_pose["xyz"]))
        s_rpy = list(map(float, src_pose.get("rpy", [0.0, 0.0, 0.0])))

        hover_h = self._param(ctx, "hover_height")
        approach_z = self._param(ctx, "approach_z")
        lift_h = self._param(ctx, "lift_height")
        grip_open = self._param(ctx, "grip_open_pulse")
        grip_close = self._param(ctx, "grip_close_pulse")
        gripper_id = self._param(ctx, "gripper_id")
        move_dur = self._param(ctx, "move_duration_ms")

        ev = {
            "ik_calls": 0,
            "ik_failures": 0,
            "steps_completed": 0,
            "steps_total": 5,
        }

        try:
            pulses = await self.adapter.ik_solve(
                [s_xyz[0], s_xyz[1], s_xyz[2] + hover_h], s_rpy
            )
            ev["ik_calls"] += 1
            if pulses is None:
                ev["ik_failures"] += 1
                return ExecutionResult(
                    task_id=ctx.task_id,
                    success=False,
                    reason="ik_failed_hover",
                    evidence=ev,
                )
            await self.adapter.servo_move(pulses, move_dur)
            ev["steps_completed"] += 1

            await self.adapter.gripper_set(gripper_id, grip_open, 300)
            ev["steps_completed"] += 1

            pulses = await self.adapter.ik_solve(
                [s_xyz[0], s_xyz[1], approach_z], s_rpy
            )
            ev["ik_calls"] += 1
            if pulses is None:
                ev["ik_failures"] += 1
                return ExecutionResult(
                    task_id=ctx.task_id,
                    success=False,
                    reason="ik_failed_approach",
                    evidence=ev,
                )
            await self.adapter.servo_move(pulses, move_dur)
            ev["steps_completed"] += 1

            await self.adapter.gripper_set(gripper_id, grip_close, 300)
            ev["steps_completed"] += 1

            pulses = await self.adapter.ik_solve(
                [s_xyz[0], s_xyz[1], s_xyz[2] + lift_h], s_rpy
            )
            ev["ik_calls"] += 1
            if pulses is None:
                ev["ik_failures"] += 1
                return ExecutionResult(
                    task_id=ctx.task_id,
                    success=False,
                    reason="ik_failed_lift",
                    evidence=ev,
                    error_detail="Pick succeeded but lift failed, object may be dropped",
                )
            await self.adapter.servo_move(pulses, move_dur)
            ev["steps_completed"] += 1

            return ExecutionResult(
                task_id=ctx.task_id,
                success=True,
                reason="pick_completed",
                confidence=0.95,
                evidence=ev,
            )

        except Exception as e:
            return ExecutionResult(
                task_id=ctx.task_id,
                success=False,
                reason="exception",
                error_detail=str(e),
                evidence=ev,
            )
