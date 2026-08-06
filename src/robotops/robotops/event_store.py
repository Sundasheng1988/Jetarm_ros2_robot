from robotops.db import DatabaseConnection


class EventStore:
    def __init__(self, db_path: str):
        self.db = DatabaseConnection(db_path)
        self.db.init_schema()

    def save_event(self, source_topic: str, task_id: str,
                   payload_json: str, event_id: str = None):
        self.db.execute(
            "INSERT INTO events (source_topic, task_id, event_id, payload_json) "
            "VALUES (?, ?, ?, ?)",
            (source_topic, task_id, event_id, payload_json),
        )

    def close(self):
        self.db.close()
