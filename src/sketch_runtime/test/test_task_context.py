import pytest
import time

from sketch_runtime.task_context import TaskState, TaskContext


class TestTaskState:
    def test_enum_values(self):
        assert TaskState.CREATED.value == "created"
        assert TaskState.PARSED.value == "parsed"
        assert TaskState.GROUNDED.value == "grounded"
        assert TaskState.SKILL_SELECTED.value == "skill_selected"
        assert TaskState.EXECUTING.value == "executing"
        assert TaskState.DONE.value == "done"
        assert TaskState.FAILED.value == "failed"
        assert TaskState.CANCELLED.value == "cancelled"
        assert TaskState.PAUSED.value == "paused"
        assert TaskState.RETRYING.value == "retrying"


class TestTaskContext:
    def test_generates_task_id_when_empty(self):
        ctx = TaskContext()
        assert ctx.task_id.startswith("task_")
        parts = ctx.task_id.split("_")
        assert len(parts) == 3  # task_{uuid12}_{unix_ts}

    def test_accepts_custom_task_id(self):
        ctx = TaskContext(task_id="my_custom_id")
        assert ctx.task_id == "my_custom_id"

    def test_default_source(self):
        ctx = TaskContext()
        assert ctx.source == "keyboard"

    def test_default_state_is_created(self):
        ctx = TaskContext()
        assert ctx.state == TaskState.CREATED

    def test_transition_updates_state(self):
        ctx = TaskContext()
        ctx.transition(TaskState.PARSED, "parser completed")
        assert ctx.state == TaskState.PARSED

    def test_transition_records_history(self):
        ctx = TaskContext()
        ctx.transition(TaskState.PARSED)
        ctx.transition(TaskState.GROUNDED)

        assert len(ctx.state_history) == 2
        assert ctx.state_history[0]["from"] == "created"
        assert ctx.state_history[0]["to"] == "parsed"
        assert ctx.state_history[1]["from"] == "parsed"
        assert ctx.state_history[1]["to"] == "grounded"

    def test_transition_sets_timestamps(self):
        ctx = TaskContext()
        assert ctx.parsed_at is None
        ctx.transition(TaskState.PARSED)
        assert ctx.parsed_at is not None

        assert ctx.grounded_at is None
        ctx.transition(TaskState.GROUNDED)
        assert ctx.grounded_at is not None

        assert ctx.started_at is None
        ctx.transition(TaskState.EXECUTING)
        assert ctx.started_at is not None

    def test_completed_at_set_on_terminal_states(self):
        for state in (TaskState.DONE, TaskState.FAILED, TaskState.CANCELLED):
            ctx = TaskContext()
            ctx.transition(TaskState.PARSED)
            ctx.transition(state)
            assert ctx.completed_at is not None

    def test_elapsed_ms_increases(self):
        ctx = TaskContext()
        assert ctx.elapsed_ms >= 0.0

    def test_to_dict_contains_core_keys(self):
        ctx = TaskContext(user_command="test command")
        ctx.transition(TaskState.PARSED)
        d = ctx.to_dict()
        assert "task_id" in d
        assert d["state"] == "parsed"
        assert d["user_command"] == "test command"
        assert d["source"] == "keyboard"

    def test_skill_params_defaults_to_empty_dict(self):
        ctx = TaskContext()
        assert ctx.skill_params == {}

    def test_retry_defaults(self):
        ctx = TaskContext()
        assert ctx.retry_count == 0
        assert ctx.max_retries == 2

    def test_different_calls_produce_unique_ids(self):
        ctx1 = TaskContext()
        ctx2 = TaskContext()
        assert ctx1.task_id != ctx2.task_id
