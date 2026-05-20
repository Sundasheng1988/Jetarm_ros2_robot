from sketch_runtime.base_skill import BaseSkill
from sketch_runtime.task_context import TaskContext
from sketch_runtime.execution_result import ExecutionResult
from sketch_runtime.actions import MoveAction, GripperAction
from sketch_runtime.action_executor import ActionExecutor


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
        "execution_stage": "full_pick",
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
        stage = self._param(ctx, "execution_stage")

        if stage == "hover_only":
            actions = [
                MoveAction("hover_source",
                           [s_xyz[0], s_xyz[1], s_xyz[2] + hover_h],
                           s_rpy, duration_ms=move_dur),
            ]
        else:  # full_pick (default)
            actions = [
                MoveAction("hover_source",
                           [s_xyz[0], s_xyz[1], s_xyz[2] + hover_h],
                           s_rpy, duration_ms=move_dur),
                GripperAction("gripper_open", servo_id=gripper_id,
                              pulse=grip_open),
                MoveAction("approach_pick",
                           [s_xyz[0], s_xyz[1], approach_z],
                           s_rpy, duration_ms=move_dur),
                GripperAction("gripper_close", servo_id=gripper_id,
                              pulse=grip_close),
                MoveAction("lift_after_pick",
                           [s_xyz[0], s_xyz[1], s_xyz[2] + lift_h],
                           s_rpy, duration_ms=move_dur),
            ]

        return await ActionExecutor.run(actions, self.adapter, ctx)
