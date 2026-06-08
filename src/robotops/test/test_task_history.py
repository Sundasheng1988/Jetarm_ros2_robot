import json
import pytest

from robotops.db import DatabaseConnection
from robotops.event_store import EventStore
from robotops import task_history


@pytest.fixture
def store_with_data():
    s = EventStore(":memory:")

    s.save_event("/runtime/state", "task_a",
                 json.dumps({"task_id": "task_a", "state": "created"}))
    s.save_event("/runtime/log", "task_a",
                 '{"task_id":"task_a","event":"grounded_task_received","state":"grounded"}',
                 event_id="evt_task_a_0001")
    s.save_event("/runtime/log", "task_a",
                 '{"task_id":"task_a","event":"execution_started","state":"executing"}',
                 event_id="evt_task_a_0002")
    s.save_event("/runtime/execution_result", "task_a",
                 json.dumps({"task_id": "task_a", "success": True}))
    s.save_event("/runtime/verification_result", "task_a",
                 json.dumps({"task_id": "task_a", "stage": "postcheck", "success": True}))
    s.save_event("/runtime/state", "task_a",
                 json.dumps({"task_id": "task_a", "state": "verified"}))

    s.save_event("/runtime/state", "task_b",
                 json.dumps({"task_id": "task_b", "state": "created"}))
    s.save_event("/runtime/log", "task_b",
                 '{"task_id":"task_b","event":"execution_started","state":"executing"}',
                 event_id="evt_task_b_0001")
    s.save_event("/runtime/execution_result", "task_b",
                 json.dumps({"task_id": "task_b", "success": False, "reason": "grip_failed"}))

    yield s
    s.close()


class TestTaskHistory:
    def test_get_events_for_task_all(self, store_with_data):
        results = task_history.get_events_for_task(
            store_with_data.db, "task_a"
        )
        assert len(results) == 6

    def test_get_events_for_task_filter_topic(self, store_with_data):
        results = task_history.get_events_for_task(
            store_with_data.db, "task_a", source_topic="/runtime/state"
        )
        assert len(results) == 2
        for r in results:
            assert r["source_topic"] == "/runtime/state"

    def test_get_events_for_task_ordered(self, store_with_data):
        results = task_history.get_events_for_task(
            store_with_data.db, "task_a"
        )
        timestamps = [r["received_at"] for r in results]
        assert timestamps == sorted(timestamps)

    def test_get_events_for_nonexistent_task(self, store_with_data):
        results = task_history.get_events_for_task(
            store_with_data.db, "nonexistent"
        )
        assert results == []

    def test_get_task_ids(self, store_with_data):
        ids = task_history.get_task_ids(store_with_data.db)
        assert sorted(ids) == ["task_a", "task_b"]

    def test_get_tasks_by_state(self, store_with_data):
        ids = task_history.get_tasks_by_state(store_with_data.db, "executing")
        assert "task_b" in ids or "task_a" in ids

    def test_get_tasks_by_state_verified(self, store_with_data):
        ids = task_history.get_tasks_by_state(store_with_data.db, "verified")
        assert ids == ["task_a"]

    def test_get_event_count(self, store_with_data):
        assert task_history.get_event_count(store_with_data.db) == 9

    def test_get_event_count_for_task(self, store_with_data):
        assert task_history.get_event_count_for_task(store_with_data.db, "task_a") == 6
        assert task_history.get_event_count_for_task(store_with_data.db, "task_b") == 3

    def test_payload_is_parsed_json(self, store_with_data):
        results = task_history.get_events_for_task(
            store_with_data.db, "task_a", source_topic="/runtime/execution_result"
        )
        assert len(results) == 1
        payload = results[0]["payload"]
        assert isinstance(payload, dict)
        assert payload["success"] is True
