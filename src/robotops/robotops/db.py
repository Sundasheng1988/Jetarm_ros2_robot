import sqlite3
from pathlib import Path


SCHEMA_DDL = """
CREATE TABLE IF NOT EXISTS events (
    id            INTEGER PRIMARY KEY AUTOINCREMENT,
    source_topic  TEXT NOT NULL,
    task_id       TEXT NOT NULL,
    event_id      TEXT,
    payload_json  TEXT NOT NULL,
    received_at   REAL DEFAULT (strftime('%s', 'now'))
);

CREATE INDEX IF NOT EXISTS idx_events_task ON events(task_id);
CREATE INDEX IF NOT EXISTS idx_events_topic ON events(source_topic);
CREATE INDEX IF NOT EXISTS idx_events_ts ON events(received_at);
"""


class DatabaseConnection:
    def __init__(self, db_path: str):
        self.db_path = db_path
        self.conn = None
        self._open()

    def _open(self):
        parent = Path(self.db_path).parent
        parent.mkdir(parents=True, exist_ok=True)
        self.conn = sqlite3.connect(self.db_path)
        if self.db_path != ":memory:":
            self.conn.execute("PRAGMA journal_mode=WAL")
            self.conn.execute("PRAGMA synchronous=NORMAL")

    def init_schema(self):
        self.conn.executescript(SCHEMA_DDL)
        self.conn.commit()

    def execute(self, sql: str, params=None):
        cursor = self.conn.execute(sql, params or ())
        self.conn.commit()
        return cursor

    def close(self):
        if self.conn:
            self.conn.close()
            self.conn = None

    def __del__(self):
        self.close()
