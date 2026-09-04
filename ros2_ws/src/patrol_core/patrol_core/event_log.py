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
    image         TEXT     증거 사진 파일명 (logs/shots_web/ 안, 없으면 NULL)
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
    zone          TEXT,
    image         TEXT
);
CREATE INDEX IF NOT EXISTS idx_events_ts ON events(ts_epoch);
CREATE INDEX IF NOT EXISTS idx_events_node ON events(node);

-- 소화기 점검 기록. events 와 따로 두는 이유: events 는 "경고가 났다"는 사건
-- 흐름이고, 이쪽은 "언제 무엇을 점검해서 어떤 판정이 나왔다"는 대장이다.
-- 웹에서 소화기별 최근 점검 이력을 보여주고, 나중에 Power BI 로 점검 주기를
-- 분석하려면 한 줄이 한 번의 점검이어야 한다.
CREATE TABLE IF NOT EXISTS inspections (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    ts          TEXT NOT NULL,
    ts_epoch    REAL NOT NULL,
    name        TEXT NOT NULL,   -- 소화기 이름 (extinguisher_info.yaml 의 name)
    qr_id       TEXT,            -- QR 로 읽은 식별자
    verdict     TEXT NOT NULL,   -- 정상 | 이상 | 판정불가 | 부재
    detail      TEXT,            -- 판정 근거(변화량·정합점수 등)
    mfg_date    TEXT,            -- 그 시점 대장의 제조년월
    expiry_date TEXT,            -- 그 시점 대장의 교체년월
    manager     TEXT,            -- 책임자
    days_left   INTEGER,         -- 교체년월까지 남은 일수(음수면 지남)
    image       TEXT             -- 점검 사진 파일명 (logs/shots_web/)
);
CREATE INDEX IF NOT EXISTS idx_insp_ts ON inspections(ts_epoch);
CREATE INDEX IF NOT EXISTS idx_insp_name ON inspections(name);
"""

# 증거 사진을 두는 곳. 대시보드가 이 폴더만 웹으로 서빙한다(다른 로그 폴더를
# 통째로 노출하지 않으려고 따로 뒀다).
SHOT_DIR = os.path.join(os.path.expanduser("~"), "vibe", "ex1", "logs", "shots_web")


def _ensure_schema(db_path):
    os.makedirs(os.path.dirname(db_path), exist_ok=True)
    con = sqlite3.connect(db_path, timeout=5.0)
    try:
        con.executescript(_SCHEMA)
        # image 칸은 나중에 추가했다 — 이미 있는 DB 를 쓰던 사람도 그대로 쓰게
        # 마이그레이션한다(2026-09-02). 없으면 붙이고, 있으면 조용히 넘어간다.
        cols = {r[1] for r in con.execute("PRAGMA table_info(events)")}
        if "image" not in cols:
            con.execute("ALTER TABLE events ADD COLUMN image TEXT")
        con.commit()
    finally:
        con.close()


def log_event(node, event_type, detail="", person_count=None, zone=None,
             image=None, db_path=DEFAULT_DB):
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
                "person_count, zone, image) VALUES (?, ?, ?, ?, ?, ?, ?, ?)",
                (ts, now, node, event_type, detail, person_count, zone, image),
            )
            con.commit()
        finally:
            con.close()
        return True
    except sqlite3.Error:
        return False


def log_inspection(name, verdict, qr_id=None, detail="", mfg_date=None,
                  expiry_date=None, manager=None, days_left=None, image=None,
                  db_path=DEFAULT_DB):
    """소화기 점검 한 건을 기록한다. 실패해도 예외를 던지지 않는다(log_event 와 같은 정책)."""
    try:
        _ensure_schema(db_path)
        now = time.time()
        ts = time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(now))
        con = sqlite3.connect(db_path, timeout=5.0)
        try:
            con.execute(
                "INSERT INTO inspections (ts, ts_epoch, name, qr_id, verdict, detail, "
                "mfg_date, expiry_date, manager, days_left, image) "
                "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                (ts, now, name, qr_id, verdict, detail, mfg_date, expiry_date,
                 manager, days_left, image),
            )
            con.commit()
        finally:
            con.close()
        return True
    except sqlite3.Error:
        return False


def save_shot(frame, prefix="intrusion", shot_dir=SHOT_DIR):
    """증거 사진을 저장하고 파일명을 돌려준다(실패하면 None).

    cv2 를 이 모듈에서 import 하지 않으려고 프레임 인코딩은 호출부에서 하지 않고,
    여기서 지연 import 한다 — event_log 는 원래 ROS·cv2 의존이 없는 순수 모듈이고
    사진 저장은 부가 기능이라 필요할 때만 불러온다.
    """
    try:
        import cv2
        os.makedirs(shot_dir, exist_ok=True)
        name = f"{prefix}_{time.strftime('%Y%m%d_%H%M%S', time.localtime())}.jpg"
        path = os.path.join(shot_dir, name)
        if cv2.imwrite(path, frame):
            return name
        return None
    except Exception:                                          # noqa: BLE001
        return None
