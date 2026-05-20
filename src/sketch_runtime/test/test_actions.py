import pytest

from sketch_runtime.runtime_adapter import RuntimeAdapter
from sketch_runtime.task_context import TaskContext
from sketch_runtime.actions import MoveAction, GripperAction
from sketch_runtime.action_executor import ActionExecutor
from sketch_runtime.base_action import StepResult


class TestMoveAction:
    @pytest.mark.asyncio
    async def test_move_action_ik_solve_then_servo(self):
        adapter = RuntimeAdapter(dry_run=True)
        action = MoveAction("test_move", [0.15, -0.10, 0.11], [0, 0, 1.57],
                            duration_ms=2000)
        ctx = TaskContext()
        result = await action.execute(adapter, ctx)
        assert result.success is True
        assert result.reason == "move_completed"
        assert result.evidence["ik_calls"] == 1
        assert result.evidence["ik_failures"] == 0
        # adapter should have recorded ik_solve + servo_move
        assert len(adapter._call_log) == 2
        assert adapter._call_log[0]["method"] == "ik_solve"
        assert adapter._call_log[1]["method"] == "servo_move"

    @pytest.mark.asyncio
    async def test_move_action_position_is_used(self):
        adapter = RuntimeAdapter(dry_run=True)
        action = MoveAction("test", [1.0, 2.0, 3.0], duration_ms=1000)
        ctx = TaskContext()
        await action.execute(adapter, ctx)
        ik_args = adapter._call_log[0]["args"]
        assert ik_args["position"] == [1.0, 2.0, 3.0]

    @pytest.mark.asyncio
    async def test_move_action_default_duration(self):
        adapter = RuntimeAdapter(dry_run=True)
        action = MoveAction("test", [0, 0, 0])
        await action.execute(adapter, ctx=TaskContext())
        assert adapter._call_log[1]["args"]["duration_ms"] == 2000


class TestGripperAction:
    @pytest.mark.asyncio
    async def test_gripper_action_calls_gripper_set(self):
        adapter = RuntimeAdapter(dry_run=True)
        action = GripperAction("open", servo_id=10, pulse=200)
        ctx = TaskContext()
        result = await action.execute(adapter, ctx)
        assert result.success is True
        assert result.reason == "gripper_completed"
        assert len(adapter._call_log) == 1
        assert adapter._call_log[0]["method"] == "gripper_set"
        assert adapter._call_log[0]["args"]["servo_id"] == 10
        assert adapter._call_log[0]["args"]["pulse"] == 200

    @pytest.mark.asyncio
    async def test_gripper_close(self):
        adapter = RuntimeAdapter(dry_run=True)
        action = GripperAction("close", pulse=700)
        ctx = TaskContext()
        await action.execute(adapter, ctx)
        assert adapter._call_log[0]["args"]["pulse"] == 700


class TestActionExecutor:
    @pytest.mark.asyncio
    async def test_runs_all_actions(self):
        adapter = RuntimeAdapter(dry_run=True)
        actions = [
            MoveAction("step1", [0, 0, 0.08], duration_ms=1000),
            GripperAction("step2", pulse=200),
            MoveAction("step3", [0, 0, 0.015], duration_ms=1000),
        ]
        ctx = TaskContext()
        result = await ActionExecutor.run(actions, adapter, ctx)
        assert result.success is True
        assert result.reason == "all_steps_completed"
        assert result.evidence["steps_completed"] == 3
        assert result.evidence["ik_calls"] == 2
        # 2 move actions (each ik_solve + servo_move) + 1 gripper = 5 calls
        assert len(adapter._call_log) == 5

    @pytest.mark.asyncio
    async def test_stops_on_failed_action(self):
        # Use a subclass that simulates failing ik_solve
        class FailingAdapter:
            dry_run = False
            enable_real_ik = False
            _call_log = []

            async def ik_solve(self, *args, **kwargs):
                self._call_log.append(
                    {"method": "ik_solve", "args": {"position": list(args[0])}}
                )
                return None

            async def servo_move(self, *args, **kwargs):
                self._call_log.append(
                    {"method": "servo_move", "args": {}}
                )

            async def gripper_set(self, *args, **kwargs):
                self._call_log.append(
                    {"method": "gripper_set", "args": {}}
                )

        adapter = FailingAdapter()
        actions = [
            MoveAction("step1", [0, 0, 0.08]),
            GripperAction("step2", pulse=200),
        ]
        ctx = TaskContext()
        result = await ActionExecutor.run(actions, adapter, ctx)
        assert result.success is False
        assert result.reason == "ik_failed"
        assert result.evidence["steps_completed"] == 0
        assert result.evidence["ik_failures"] == 1

    @pytest.mark.asyncio
    async def test_empty_actions_returns_success(self):
        adapter = RuntimeAdapter(dry_run=True)
        ctx = TaskContext()
        result = await ActionExecutor.run([], adapter, ctx)
        assert result.success is True
        assert result.evidence["steps_total"] == 0


class TestPickSkillActionSequence:
    @pytest.mark.asyncio
    async def test_full_pick_action_order(self):
        from sketch_runtime.skills.pick_skill import PickSkill

        adapter = RuntimeAdapter(dry_run=True)
        skill = PickSkill(adapter)
        ctx = TaskContext(
            parsed_command={"action": "pick"},
            target_object={
                "source_pose": {
                    "xyz": [0.15, -0.10, 0.03],
                    "rpy": [0.0, 0.0, 1.57],
                }
            },
        )
        result = await skill.execute(ctx)
        assert result.success is True
        assert result.evidence["steps_completed"] == 5
        assert result.evidence["ik_calls"] == 3

        methods = [c["method"] for c in adapter._call_log]
        expected = [
            "ik_solve", "servo_move",       # hover
            "gripper_set",                   # open
            "ik_solve", "servo_move",       # approach
            "gripper_set",                   # close
            "ik_solve", "servo_move",       # lift
        ]
        assert methods == expected

    @pytest.mark.asyncio
    async def test_hover_only_stage(self):
        from sketch_runtime.skills.pick_skill import PickSkill

        adapter = RuntimeAdapter(dry_run=True)
        skill = PickSkill(adapter)
        ctx = TaskContext(
            parsed_command={"action": "pick"},
            target_object={
                "source_pose": {
                    "xyz": [0.15, -0.10, 0.03],
                    "rpy": [0.0, 0.0, 1.57],
                }
            },
            skill_params={"execution_stage": "hover_only"},
        )
        result = await skill.execute(ctx)
        assert result.success is True
        assert result.evidence["steps_completed"] == 1
        assert result.evidence["ik_calls"] == 1

        methods = [c["method"] for c in adapter._call_log]
        assert methods == ["ik_solve", "servo_move"]

    @pytest.mark.asyncio
    async def test_missing_source_pose_returns_error(self):
        from sketch_runtime.skills.pick_skill import PickSkill

        adapter = RuntimeAdapter(dry_run=True)
        skill = PickSkill(adapter)
        ctx = TaskContext(target_object={"no_pose": True})
        result = await skill.execute(ctx)
        assert result.success is False
        assert result.reason == "missing_source_pose"
