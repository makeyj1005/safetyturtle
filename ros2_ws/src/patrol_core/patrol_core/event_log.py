"""event_log.py — 감지 노드들이 공용으로 쓰는 SQLite 이벤트 기록.

Power BI 등 외부 분석 도구에서 바로 읽을 수 있도록 단순한 한 테이블 구조로 둔다.
여러 노드(restricted_node, 나중에 helmet_node·fire_node)가 같은 DB 파일에
각자 node 이름을 남기며 기록한다 — 노드마다 파일을 나누면 나중에 합쳐 보기 번거롭다.

테이블 events:
    id            INTEGER PRIMARY KEY
    ts            TEXT     ISO8601 문자열 (예: 2026-08-29T14:30:00) — 사람이 보기 위함
    ts_epoch      REAL     time.time() — 계산·정렬용
    node          TEXT     예: "restricted_node"
    event_type    TEXT     예: "alert" | "clear"
    detail        TEXT     사람이 읽는 설명
    person_count  INTEGER  해당 시점 감지된 사람 수 (없으면 NULL)
    zone          TEXT     구역 이름 (구역 제한 붙이기 전까지는 NULL)
"""
import os
import sqlite3
import time

DEFAULT_DB = os.path.join(os.path.expanduser("~"), "vibe", "ex1", "logs", "events.sqlite")

_SCHEMA = """
CREATE TABLE IF NOT EXISTS events (
    id            INTEGER PRIMARY KEY AUTOINCREMENT,
    ts            TEXT NOT NULL,
    ts_epoch      REAL NOT NULL,
    node          TEXT NOT NULL,
    event_type    TEXT NOT NULL,
    detail        TEXT,
    person_count  INTEGER,
    zone          TEXT
);
CREATE INDEX IF NOT EXISTS idx_events_ts ON events(ts_epoch);
CREATE INDEX IF NOT EXISTS idx_events_node ON events(node);
"""


def _ensure_schema(db_path):
    os.makedirs(os.path.dirname(db_path), exist_ok=True)
    con = sqlite3.connect(db_path, timeout=5.0)
    try:
        con.executescript(_SCHEMA)
        con.commit()
    finally:
        con.close()


def log_event(node, event_type, detail="", person_count=None, zone=None,
             db_path=DEFAULT_DB):
    """이벤트 한 줄을 기록한다. 실패해도 예외를 던지지 않는다(로깅 실패로 노드가
    죽으면 안 된다 — 호출부는 그냥 warn 로그만 남기고 계속 돈다)."""
    try:
        _ensure_schema(db_path)
        now = time.time()
        ts = time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(now))
        con = sqlite3.connect(db_path, timeout=5.0)
        try:
            con.execute(
                "INSERT INTO events (ts, ts_epoch, node, event_type, detail, "
                "person_count, zone) VALUES (?, ?, ?, ?, ?, ?, ?)",
                (ts, now, node, event_type, detail, person_count, zone),
            )
            con.commit()
        finally:
            con.close()
        return True
    except sqlite3.Error:
        return False
