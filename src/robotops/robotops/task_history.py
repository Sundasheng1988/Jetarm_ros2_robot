import json
from robotops.db import DatabaseConnection


def get_events_for_task(db: DatabaseConnection, task_id: str,
                        source_topic: str = None) -> list:
    if source_topic:
        cursor = db.execute(
            "SELECT id, source_topic, task_id, event_id, payload_json, received_at "
            "FROM events WHERE task_id=? AND source_topic=? ORDER BY received_at ASC",
            (task_id, source_topic),
        )
    else:
        cursor = db.execute(
            "SELECT id, source_topic, task_id, event_id, payload_json, received_at "
            "FROM events WHERE task_id=? ORDER BY received_at ASC",
            (task_id,),
        )
    return [_row_to_dict(row) for row in cursor.fetchall()]


def get_task_ids(db: DatabaseConnection) -> list:
    cursor = db.execute(
        "SELECT DISTINCT task_id FROM events ORDER BY task_id"
    )
    return [row[0] for row in cursor.fetchall()]


def get_tasks_by_state(db: DatabaseConnection, state: str) -> list:
    cursor = db.execute(
        "SELECT DISTINCT task_id FROM events "
        "WHERE payload_json LIKE ?"
        " ORDER BY task_id",
        ('%"' + state + '"%',),
    )
    return [row[0] for row in cursor.fetchall()]


def get_event_count(db: DatabaseConnection) -> int:
    cursor = db.execute("SELECT COUNT(*) FROM events")
    return cursor.fetchone()[0]


def get_event_count_for_task(db: DatabaseConnection, task_id: str) -> int:
    cursor = db.execute(
        "SELECT COUNT(*) FROM events WHERE task_id=?", (task_id,)
    )
    return cursor.fetchone()[0]


def _row_to_dict(row) -> dict:
    return {
        "id": row[0],
        "source_topic": row[1],
        "task_id": row[2],
        "event_id": row[3],
        "payload": json.loads(row[4]),
        "received_at": row[5],
    }
