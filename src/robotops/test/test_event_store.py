import os
import json
import tempfile
import pytest

from robotops.db import DatabaseConnection
from robotops.event_store import EventStore


@pytest.fixture
def db():
    conn = DatabaseConnection(":memory:")
    conn.init_schema()
    yield conn
    conn.close()


@pytest.fixture
def store():
    s = EventStore(":memory:")
    yield s
    s.close()


class TestDatabaseConnection:
    def test_init_creates_in_memory_db(self, db):
        cursor = db.execute(
            "SELECT name FROM sqlite_master WHERE type='table' AND name='events'"
        )
        assert cursor.fetchone() is not None

    def test_init_creates_indexes(self, db):
        indexes = db.execute(
            "SELECT name FROM sqlite_master WHERE type='index'"
        ).fetchall()
        index_names = [r[0] for r in indexes]
        assert "idx_events_task" in index_names
        assert "idx_events_topic" in index_names
        assert "idx_events_ts" in index_names

    def test_file_based_db(self):
        with tempfile.NamedTemporaryFile(suffix=".db", delete=False) as f:
            db_path = f.name
        try:
            db_conn = DatabaseConnection(db_path)
            db_conn.init_schema()
            db_conn.execute(
                "INSERT INTO events (source_topic, task_id, payload_json) "
                "VALUES (?, ?, ?)",
                ("/runtime/state", "task_test", '{"state":"created"}'),
            )
            cursor = db_conn.execute("SELECT COUNT(*) FROM events")
            assert cursor.fetchone()[0] == 1
            db_conn.close()
            assert os.path.exists(db_path)
        finally:
            os.unlink(db_path)


class TestEventStore:
    def test_save_event_inserts_row(self, store):
        payload = json.dumps({"task_id": "task_abc", "state": "executing"})
        store.save_event("/runtime/state", "task_abc", payload)
        cursor = store.db.execute("SELECT COUNT(*) FROM events")
        assert cursor.fetchone()[0] == 1

    def test_save_event_stores_all_fields(self, store):
        payload = json.dumps({"task_id": "task_xyz", "state": "grounded", "key": "val"})
        store.save_event(
            "/runtime/log", "task_xyz", payload,
            event_id="evt_task_xyz_0001",
        )
        row = store.db.execute(
            "SELECT source_topic, task_id, event_id, payload_json FROM events"
        ).fetchone()
        assert row[0] == "/runtime/log"
        assert row[1] == "task_xyz"
        assert row[2] == "evt_task_xyz_0001"
        assert json.loads(row[3])["state"] == "grounded"

    def test_save_event_nullable_event_id(self, store):
        payload = json.dumps({"task_id": "task_no_event"})
        store.save_event("/runtime/state", "task_no_event", payload)
        row = store.db.execute(
            "SELECT event_id FROM events"
        ).fetchone()
        assert row[0] is None

    def test_save_multiple_events(self, store):
        for i in range(5):
            payload = json.dumps({"task_id": "task_multi", "seq": i})
            store.save_event(
                "/runtime/log", "task_multi", payload,
                event_id=f"evt_task_multi_{i:04d}",
            )
        cursor = store.db.execute("SELECT COUNT(*) FROM events")
        assert cursor.fetchone()[0] == 5

    def test_save_events_different_topics(self, store):
        topics = [
            "/runtime/state",
            "/runtime/log",
            "/runtime/execution_result",
            "/runtime/verification_result",
        ]
        for i, topic in enumerate(topics):
            payload = json.dumps({"task_id": f"task_{i}", "topic": topic})
            store.save_event(topic, f"task_{i}", payload)
        for topic in topics:
            cursor = store.db.execute(
                "SELECT COUNT(*) FROM events WHERE source_topic=?", (topic,)
            )
            assert cursor.fetchone()[0] == 1

    def test_save_event_payload_preserved(self, store):
        payload_dict = {
            "task_id": "task_detail",
            "state": "verified",
            "result": {"success": True, "reason": "pick_ok"},
            "evidence": {"gripper_closed": True},
        }
        store.save_event(
            "/runtime/state", "task_detail",
            json.dumps(payload_dict, ensure_ascii=False),
        )
        row = store.db.execute(
            "SELECT payload_json FROM events"
        ).fetchone()
        stored = json.loads(row[0])
        assert stored["state"] == "verified"
        assert stored["result"]["success"] is True

    def test_event_received_at_is_set(self, store):
        store.save_event("/runtime/state", "task_ts", '{"task_id":"task_ts"}')
        row = store.db.execute(
            "SELECT received_at FROM events"
        ).fetchone()
        assert row[0] is not None
        assert row[0] > 0
