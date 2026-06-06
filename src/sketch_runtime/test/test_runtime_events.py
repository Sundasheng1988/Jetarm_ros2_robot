import sys
import types


class _FakeNode:
    """Stand-in for rclpy.node.Node so RealGroundedRuntimeNode is a real class."""
    pass


class _FakeString:
    """Stand-in for std_msgs.msg.String."""
    def __init__(self, data=""):
        self.data = data


class _FakeBool:
    """Stand-in for std_msgs.msg.Bool."""
    def __init__(self, data=False):
        self.data = data


# Build fake rclpy module tree
rclpy_mod = types.ModuleType('rclpy')
rclpy_node_mod = types.ModuleType('rclpy.node')
rclpy_node_mod.Node = _FakeNode
rclpy_mod.node = rclpy_node_mod
sys.modules['rclpy'] = rclpy_mod
sys.modules['rclpy.node'] = rclpy_node_mod

# Build fake std_msgs module tree
std_msgs_mod = types.ModuleType('std_msgs')
std_msgs_msg_mod = types.ModuleType('std_msgs.msg')
std_msgs_msg_mod.String = _FakeString
std_msgs_msg_mod.Bool = _FakeBool
std_msgs_mod.msg = std_msgs_msg_mod
sys.modules['std_msgs'] = std_msgs_mod
sys.modules['std_msgs.msg'] = std_msgs_msg_mod

import json

from sketch_runtime.task_context import TaskContext, TaskState
from sketch_runtime.real_grounded_runtime_node import RealGroundedRuntimeNode


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def make_runtime_node():
    node = object.__new__(RealGroundedRuntimeNode)
    node._event_counter = {}
    node.pub_log = FakePublisher()
    return node


def latest_payload(node):
    assert node.pub_log.messages
    return json.loads(node.pub_log.messages[-1].data)


class TestEventPayloadFormat:
    def test_emit_log_has_required_fields(self):
        node = make_runtime_node()
        ctx = TaskContext(task_id="task_test")
        node._emit_log(ctx, "test_event", {"key": "value"})

        payload = latest_payload(node)

        assert payload["event_id"] == "evt_task_test_0001"
        assert payload["task_id"] == "task_test"
        assert payload["event"] == "test_event"
        assert payload["state"] == ctx.state.value
        assert "timestamp" in payload
        assert payload["data"] == {"key": "value"}

    def test_emit_log_data_defaults_to_empty_dict(self):
        node = make_runtime_node()
        ctx = TaskContext(task_id="task_test")

        node._emit_log(ctx, "test_event")

        payload = latest_payload(node)
        assert payload["data"] == {}

    def test_emit_log_data_preserved(self):
        node = make_runtime_node()
        ctx = TaskContext(task_id="task_data")

        data = {"skill": "pick_skill", "attempts": 3}
        node._emit_log(ctx, "execution_started", data)

        payload = latest_payload(node)
        assert payload["data"]["skill"] == "pick_skill"
        assert payload["data"]["attempts"] == 3


class TestEventSequence:
    def test_emit_log_sequence_monotonic_for_same_task(self):
        node = make_runtime_node()
        ctx = TaskContext(task_id="task_seq")

        node._emit_log(ctx, "event_1")
        node._emit_log(ctx, "event_2")
        node._emit_log(ctx, "event_3")

        payloads = [json.loads(msg.data) for msg in node.pub_log.messages]

        assert payloads[0]["event_id"] == "evt_task_seq_0001"
        assert payloads[1]["event_id"] == "evt_task_seq_0002"
        assert payloads[2]["event_id"] == "evt_task_seq_0003"

    def test_emit_log_sequence_independent_per_task(self):
        node = make_runtime_node()

        ctx_a = TaskContext(task_id="task_a")
        ctx_b = TaskContext(task_id="task_b")

        node._emit_log(ctx_a, "a1")
        node._emit_log(ctx_b, "b1")
        node._emit_log(ctx_a, "a2")
        node._emit_log(ctx_b, "b2")

        payloads = [json.loads(msg.data) for msg in node.pub_log.messages]

        assert payloads[0]["event_id"] == "evt_task_a_0001"
        assert payloads[1]["event_id"] == "evt_task_b_0001"
        assert payloads[2]["event_id"] == "evt_task_a_0002"
        assert payloads[3]["event_id"] == "evt_task_b_0002"


class TestStateAtEmission:
    def test_emit_log_state_matches_current_context_state(self):
        node = make_runtime_node()
        ctx = TaskContext(task_id="task_state")

        ctx.transition(TaskState.EXECUTING)
        node._emit_log(ctx, "execution_started")
        assert latest_payload(node)["state"] == "executing"

        ctx.transition(TaskState.VERIFYING)
        node._emit_log(ctx, "verification_started")
        assert latest_payload(node)["state"] == "verifying"

        ctx.transition(TaskState.VERIFIED)
        node._emit_log(ctx, "verification_complete", {"success": True})
        assert latest_payload(node)["state"] == "verified"

    def test_emit_log_failed_state(self):
        node = make_runtime_node()
        ctx = TaskContext(task_id="task_fail")

        ctx.transition(TaskState.FAILED)
        node._emit_log(ctx, "execution_result", {"success": False})
        assert latest_payload(node)["state"] == "failed"

    def test_emit_log_verification_failed_state(self):
        node = make_runtime_node()
        ctx = TaskContext(task_id="task_vfail")

        ctx.transition(TaskState.VERIFICATION_FAILED)
        node._emit_log(ctx, "verification_complete", {"success": False})
        assert latest_payload(node)["state"] == "verification_failed"
