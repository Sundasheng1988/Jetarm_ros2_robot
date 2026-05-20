import pytest

from sketch_runtime.skill_registry import SkillRegistry, SkillManager
from sketch_runtime.task_context import TaskState, TaskContext
from sketch_runtime.execution_result import ExecutionResult
from sketch_runtime.base_skill import BaseSkill
from sketch_runtime.runtime_adapter import RuntimeAdapter


class _DummySkill(BaseSkill):
    name = "dummy_skill"

    async def execute(self, ctx: TaskContext) -> ExecutionResult:
        return ExecutionResult(
            task_id=ctx.task_id, success=True, reason="dummy_done"
        )


class _DummySkillNoName(BaseSkill):
    name = ""  # Explicitly empty name — should raise ValueError on register

    async def execute(self, ctx: TaskContext) -> ExecutionResult:
        return ExecutionResult(task_id=ctx.task_id, success=True)


class TestSkillRegistry:
    def setup_method(self):
        SkillRegistry.clear()

    def test_register_and_get(self):
        SkillRegistry.register(_DummySkill)
        assert SkillRegistry.get("dummy_skill") is _DummySkill

    def test_register_missing_name_raises(self):
        with pytest.raises(ValueError):
            SkillRegistry.register(_DummySkillNoName)

    def test_get_unknown_returns_none(self):
        assert SkillRegistry.get("nonexistent") is None

    def test_has(self):
        SkillRegistry.register(_DummySkill)
        assert SkillRegistry.has("dummy_skill")
        assert not SkillRegistry.has("nonexistent")

    def test_list_all(self):
        SkillRegistry.register(_DummySkill)
        names = SkillRegistry.list_all()
        assert "dummy_skill" in names

    def test_clear(self):
        SkillRegistry.register(_DummySkill)
        SkillRegistry.clear()
        assert SkillRegistry.list_all() == []


class TestSkillManager:
    def setup_method(self):
        SkillRegistry.clear()

    def test_select_pick_intent(self):
        mgr = SkillManager()
        ctx = TaskContext(parsed_command={"action": "pick"})
        assert mgr.select(ctx) == "pick_skill"

    def test_select_grasp_intent(self):
        mgr = SkillManager()
        ctx = TaskContext(parsed_command={"action": "grasp"})
        assert mgr.select(ctx) == "pick_skill"

    def test_select_hold_intent(self):
        mgr = SkillManager()
        ctx = TaskContext(parsed_command={"action": "hold"})
        assert mgr.select(ctx) == "pick_skill"

    def test_select_place_intent(self):
        mgr = SkillManager()
        ctx = TaskContext(parsed_command={"action": "place"})
        assert mgr.select(ctx) == "place_skill"

    def test_select_move_intent(self):
        mgr = SkillManager()
        ctx = TaskContext(parsed_command={"action": "move"})
        assert mgr.select(ctx) == "move_skill"

    def test_select_unknown_intent(self):
        mgr = SkillManager()
        ctx = TaskContext(parsed_command={"action": "dance"})
        assert mgr.select(ctx) == "unknown_skill"

    def test_select_no_parsed_command(self):
        mgr = SkillManager()
        ctx = TaskContext()
        assert mgr.select(ctx) == "unknown_skill"

    def test_instantiate_registered_skill(self):
        SkillRegistry.register(_DummySkill)
        adapter = RuntimeAdapter(dry_run=True)
        mgr = SkillManager(adapter=adapter)
        skill = mgr.instantiate("dummy_skill")
        assert skill is not None
        assert skill.name == "dummy_skill"

    def test_instantiate_unknown_skill(self):
        mgr = SkillManager()
        skill = mgr.instantiate("nonexistent")
        assert skill is None


class TestBaseSkillLifecycle:
    def setup_method(self):
        SkillRegistry.clear()
        SkillRegistry.register(_DummySkill)

    def test_precheck_no_target_object(self):
        adapter = RuntimeAdapter(dry_run=True)
        mgr = SkillManager(adapter=adapter)
        skill = mgr.instantiate("dummy_skill")
        ctx = TaskContext(target_object=None)
        result = skill.precheck(ctx)
        assert result is not None
        assert result.success is False
        assert result.reason == "no_target_object"

    def test_precheck_with_target_object(self):
        adapter = RuntimeAdapter(dry_run=True)
        mgr = SkillManager(adapter=adapter)
        skill = mgr.instantiate("dummy_skill")
        ctx = TaskContext(target_object={"source_pose": {"xyz": [0, 0, 0]}})
        result = skill.precheck(ctx)
        assert result is None

    def test_postcheck_passthrough(self):
        adapter = RuntimeAdapter(dry_run=True)
        mgr = SkillManager(adapter=adapter)
        skill = mgr.instantiate("dummy_skill")
        ctx = TaskContext()
        original = ExecutionResult(task_id=ctx.task_id, success=True)
        result = skill.postcheck(ctx, original)
        assert result is original
        assert result.success is True


class TestRuntimeAdapter:
    def test_dry_run_default(self):
        adapter = RuntimeAdapter()
        assert adapter.dry_run is True

    def test_call_log_empty_initially(self):
        adapter = RuntimeAdapter()
        assert adapter._call_log == []

    @pytest.mark.asyncio
    async def test_ik_solve_dry_run(self):
        adapter = RuntimeAdapter(dry_run=True)
        pulses = await adapter.ik_solve([0.15, -0.10, 0.08])
        assert pulses == [500, 500, 500, 500, 500]
        assert len(adapter._call_log) == 1
        assert adapter._call_log[0]["method"] == "ik_solve"

    @pytest.mark.asyncio
    async def test_servo_move_dry_run(self):
        adapter = RuntimeAdapter(dry_run=True)
        await adapter.servo_move([500, 560, 130, 115, 500], 2000)
        assert len(adapter._call_log) == 1
        assert adapter._call_log[0]["method"] == "servo_move"

    @pytest.mark.asyncio
    async def test_gripper_set_dry_run(self):
        adapter = RuntimeAdapter(dry_run=True)
        await adapter.gripper_set(10, 200, 300)
        assert len(adapter._call_log) == 1
        assert adapter._call_log[0]["method"] == "gripper_set"


class TestPickSkill:
    def setup_method(self):
        SkillRegistry.clear()

    @pytest.mark.asyncio
    async def test_pick_skill_dry_run_completes(self):
        from sketch_runtime.skills.pick_skill import PickSkill

        adapter = RuntimeAdapter(dry_run=True)
        skill = PickSkill(adapter)
        ctx = TaskContext(
            user_command="pick red cup",
            parsed_command={"action": "pick", "from": "red_cup"},
            target_object={
                "source_pose": {
                    "frame": "table",
                    "xyz": [0.15, -0.10, 0.03],
                    "rpy": [0.0, 0.0, 1.57],
                }
            },
        )

        result = await skill.execute(ctx)
        assert result.success is True
        assert result.reason == "all_steps_completed"
        assert result.evidence["steps_completed"] == 5
        assert result.evidence["ik_calls"] == 3
        assert result.evidence["ik_failures"] == 0

    @pytest.mark.asyncio
    async def test_pick_skill_precheck_missing_target(self):
        from sketch_runtime.skills.pick_skill import PickSkill

        adapter = RuntimeAdapter(dry_run=True)
        skill = PickSkill(adapter)
        ctx = TaskContext(target_object=None)
        result = skill.precheck(ctx)
        assert result is not None
        assert result.success is False

    @pytest.mark.asyncio
    async def test_pick_skill_with_custom_params(self):
        from sketch_runtime.skills.pick_skill import PickSkill

        adapter = RuntimeAdapter(dry_run=True)
        skill = PickSkill(adapter)
        ctx = TaskContext(
            parsed_command={"action": "pick"},
            target_object={
                "source_pose": {"xyz": [1, 2, 3], "rpy": [0, 0, 0]}
            },
            skill_params={
                "hover_height": 0.12,
                "grip_open_pulse": 150,
                "grip_close_pulse": 800,
            },
        )

        result = await skill.execute(ctx)
        assert result.success is True
        # Verify adapter recorded the custom params
        ik_calls = [
            c for c in adapter._call_log if c["method"] == "ik_solve"
        ]
        # First hover: z=3+0.12=3.12
        assert ik_calls[0]["args"]["position"][2] == 3.12
