import pytest
import time

from sketch_runtime.task_context import TaskState, TaskContext


class TestTaskStateNewValues:
    def test_verify_new_states_exist(self):
        assert TaskState.VERIFYING.value == "verifying"
        assert TaskState.VERIFIED.value == "verified"
        assert TaskState.VERIFICATION_FAILED.value == "verification_failed"

    def test_state_history_records_transitions(self):
        ctx = TaskContext()
        ctx.transition(TaskState.PARSED)
        ctx.transition(TaskState.EXECUTING)
        ctx.transition(TaskState.VERIFYING)
        ctx.transition(TaskState.VERIFIED)

        assert len(ctx.state_history) == 4
        assert ctx.state_history[2]["from"] == "executing"
        assert ctx.state_history[2]["to"] == "verifying"
        assert ctx.state_history[3]["from"] == "verifying"
        assert ctx.state_history[3]["to"] == "verified"


class TestTaskContextVerification:
    def test_completed_at_not_set_on_verifying(self):
        ctx = TaskContext()
        ctx.transition(TaskState.EXECUTING)
        ctx.transition(TaskState.VERIFYING)
        assert ctx.completed_at is None

    def test_completed_at_set_on_verified(self):
        ctx = TaskContext()
        ctx.transition(TaskState.EXECUTING)
        ctx.transition(TaskState.VERIFYING)
        ctx.transition(TaskState.VERIFIED)
        assert ctx.completed_at is not None

    def test_completed_at_set_on_verification_failed(self):
        ctx = TaskContext()
        ctx.transition(TaskState.EXECUTING)
        ctx.transition(TaskState.VERIFYING)
        ctx.transition(TaskState.VERIFICATION_FAILED)
        assert ctx.completed_at is not None

    def test_result_contains_verification_on_success(self):
        ctx = TaskContext()
        ctx.transition(TaskState.EXECUTING)
        ctx.transition(TaskState.VERIFYING)
        if ctx.result is None:
            ctx.result = {}
        ctx.result["verification"] = {"success": True, "reason": "object_gone"}
        ctx.transition(TaskState.VERIFIED)
        assert ctx.result["verification"]["success"] is True
        assert ctx.result["verification"]["reason"] == "object_gone"

    def test_result_contains_verification_on_failure(self):
        ctx = TaskContext()
        ctx.transition(TaskState.EXECUTING)
        ctx.transition(TaskState.VERIFYING)
        if ctx.result is None:
            ctx.result = {}
        ctx.result["verification"] = {"success": False, "reason": "object_still"}
        ctx.transition(TaskState.VERIFICATION_FAILED)
        assert ctx.result["verification"]["success"] is False
        assert ctx.result["verification"]["reason"] == "object_still"

    def test_to_dict_includes_verification(self):
        ctx = TaskContext()
        ctx.transition(TaskState.EXECUTING)
        ctx.transition(TaskState.VERIFYING)
        if ctx.result is None:
            ctx.result = {}
        ctx.result["verification"] = {
            "success": True,
            "stage": "postcheck",
            "reason": "object_gone",
        }
        ctx.transition(TaskState.VERIFIED)
        d = ctx.to_dict()
        assert "verification" in d["result"]
        assert d["result"]["verification"]["success"] is True
        assert d["result"]["verification"]["stage"] == "postcheck"

    def test_executing_to_failed_skips_verification(self):
        ctx = TaskContext()
        ctx.transition(TaskState.EXECUTING)
        ctx.transition(TaskState.FAILED)
        assert ctx.state == TaskState.FAILED
        assert ctx.completed_at is not None
        assert TaskState.VERIFYING not in [s["to"] for s in ctx.state_history]

    def test_result_none_guard_before_verification(self):
        ctx = TaskContext()
        ctx.transition(TaskState.EXECUTING)
        ctx.transition(TaskState.VERIFYING)
        assert ctx.result is None
        if ctx.result is None:
            ctx.result = {}
        ctx.result["verification"] = {"success": True, "reason": "test"}
        ctx.transition(TaskState.VERIFIED)
        assert ctx.result is not None
        assert "verification" in ctx.result
