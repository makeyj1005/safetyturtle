#!/usr/bin/env python3
"""dashboard_server.py — 로컬 웹 대시보드. 순찰 로봇 상태를 브라우저로 본다.

[VM에서 실행]
  python3 ~/vibe/ex1/tools/dashboard_server.py
  브라우저: http://localhost:8080  (같은 team1 네트워크의 다른 기기에서는
  http://<이 노트북 IP>:8080)

[역할]
  - /restricted/status, /webcam/image_raw/compressed 를 구독해 메모리에 최신값만 들고 있는다
  - events.sqlite 를 읽기 전용으로 조회해 최근 이벤트 표를 보여준다
  - Flask 등 외부 패키지 없이 표준 라이브러리(http.server)만 쓴다 — 설치 불필요

[구성만 먼저, 디자인은 나중]
  지금은 정보 배치(실시간 영상 / 현재 상태 / 최근 이벤트 표)만 잡아둔 것이다.
  꾸미기는 나중에 — 구조가 맞는지 먼저 확인한다.
"""
import json
import os
import sqlite3
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote

sys.path.insert(0, os.path.join(os.path.expanduser("~"), "vibe", "ex1",
                                "ros2_ws", "src", "patrol_core"))

import rclpy                                    # noqa: E402
from geometry_msgs.msg import Twist             # noqa: E402
from rclpy.node import Node                     # noqa: E402
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import CompressedImage     # noqa: E402
from std_msgs.msg import Bool, String           # noqa: E402

from patrol_core import event_log               # noqa: E402

PORT = 8080

# cmd_vel_mux.py 의 최우선 입력(/cmd_vel_teleop)으로 낸다 — 절대 규칙(HANDOFF2 3.4):
# 어떤 노드도 /cmd_vel 에 직접 발행하지 않는다. mux 가 0.4초 안에 이 토픽으로 신호가
# 안 오면 자동으로 정지시킨다(데드맨 스위치) — 브라우저 탭이 닫히거나 키를 떼면
# 이 서버가 별도 타이머 없이도 안전하게 멈춘다.
TELEOP_TOPIC = "/cmd_vel_teleop"
# mux 의 max_linear=0.15, max_angular=1.2 보다 낮게 잡아 처음 테스트는 더 보수적으로.
MAX_LINEAR = 0.10
MAX_ANGULAR = 0.6

# 여러 스레드(ROS 콜백, HTTP 핸들러)가 같이 건드리므로 락으로 보호한다.
state_lock = threading.Lock()
state = {
    "frame": None,           # 최신 jpeg 바이트
    "frame_at": 0.0,
    "restricted_status": "(아직 없음)",
    "restricted_at": 0.0,
    "fire_status": "(아직 없음)",
    "fire_at": 0.0,
    "helmet_status": "(아직 없음)",
    "helmet_at": 0.0,
    "extinguisher_status": "(아직 없음)",
    "extinguisher_at": 0.0,
}

# 상태가 이 시간(초) 넘게 갱신되지 않으면 "그 경고는 끝났다"고 본다.
# 각 노드가 상태를 바뀔 때만 내는 경우가 있어서, 화면이 옛 경고에 붙어있지 않게
# 하는 안전장치다(lcd_node 의 STALE_SEC 과 같은 이유).
STALE_SEC = 15.0


class DashboardNode(Node):
    def __init__(self):
        super().__init__("dashboard_server")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(CompressedImage, "/webcam/image_raw/compressed",
                                 self.on_frame, qos_profile_sensor_data)
        self.create_subscription(String, "/restricted/status", self.on_status, qos)
        self.create_subscription(String, "/fire/status", self.on_fire, qos)
        self.create_subscription(String, "/helmet/status", self.on_helmet, qos)
        self.create_subscription(String, "/extinguisher/status",
                                 self.on_extinguisher, qos)
        self.pub_teleop = self.create_publisher(Twist, TELEOP_TOPIC, qos)
        # 작업자가 웹에서 작업금지 감시 모드를 바꾼다(auto | on | off)
        self.pub_mode = self.create_publisher(String, "/restricted/mode", qos)
        # 화재 시연/수동 해제용
        self.pub_fire_trigger = self.create_publisher(Bool, "/fire/trigger", qos)

    def on_frame(self, msg):
        with state_lock:
            state["frame"] = bytes(msg.data)
            state["frame_at"] = time.time()

    def on_status(self, msg):
        with state_lock:
            state["restricted_status"] = msg.data
            state["restricted_at"] = time.time()

    def on_fire(self, msg):
        with state_lock:
            state["fire_status"] = msg.data
            state["fire_at"] = time.time()

    def on_helmet(self, msg):
        with state_lock:
            state["helmet_status"] = msg.data
            state["helmet_at"] = time.time()

    def on_extinguisher(self, msg):
        with state_lock:
            state["extinguisher_status"] = msg.data
            state["extinguisher_at"] = time.time()

    def send_teleop(self, linear, angular):
        t = Twist()
        t.linear.x = max(-MAX_LINEAR, min(float(linear), MAX_LINEAR))
        t.angular.z = max(-MAX_ANGULAR, min(float(angular), MAX_ANGULAR))
        self.pub_teleop.publish(t)

    def send_mode(self, mode):
        m = String()
        m.data = str(mode)
        self.pub_mode.publish(m)

    def send_fire_trigger(self, on):
        m = Bool()
        m.data = bool(on)
        self.pub_fire_trigger.publish(m)


def build_status():
    """대시보드가 화면을 그리는 데 필요한 모든 상태를 한 번에 모아 준다.

    경고 여부를 서버에서 판정해 내려보낸다 — 브라우저에서 문자열을 파싱하게 두면
    노드가 문구를 바꿀 때마다 화면이 조용히 깨진다(문구는 사람 읽는 용도다).
    """
    now = time.time()
    with state_lock:
        snap = dict(state)

    def fresh(key):
        ts = snap.get(f"{key}_at", 0.0)
        return ts > 0.0 and (now - ts) < STALE_SEC

    def age(key):
        ts = snap.get(f"{key}_at", 0.0)
        return (now - ts) if ts else -1

    fire_txt = snap["fire_status"]
    restricted_txt = snap["restricted_status"]
    helmet_txt = snap["helmet_status"]

    # 각 노드의 상태 문구에서 "지금 경고 중인가"를 뽑는다.
    fire_alert = fresh("fire") and "FIRE" in fire_txt.upper()
    intrusion_alert = fresh("restricted") and "ALERT" in restricted_txt.upper()
    helmet_alert = fresh("helmet") and helmet_txt.startswith("hold")

    # restricted_node 가 idle 일 때 내는 "idle mode=auto watching=yes window=00:00-06:00"
    # 에서 모드·감시여부를 뽑아 화면에 보여준다(작업자가 지금 설정을 알 수 있게).
    mode, watching, window = "?", None, ""
    for tok in restricted_txt.split():
        if tok.startswith("mode="):
            mode = tok[5:]
        elif tok.startswith("watching="):
            watching = tok[9:] == "yes"
        elif tok.startswith("window="):
            window = tok[7:]

    return {
        "now": time.strftime("%H:%M:%S", time.localtime(now)),
        "fire": {"alert": fire_alert, "text": fire_txt, "age_sec": age("fire")},
        "intrusion": {"alert": intrusion_alert, "text": restricted_txt,
                      "age_sec": age("restricted"), "mode": mode,
                      "watching": watching, "window": window},
        "helmet": {"alert": helmet_alert, "text": helmet_txt, "age_sec": age("helmet")},
        "extinguisher": {"text": snap["extinguisher_status"],
                         "age_sec": age("extinguisher")},
        "camera_age_sec": age("frame"),
    }


def fetch_photos(limit=24):
    """저장된 증거 사진 목록(최신순). DB 에 기록된 것 위주로 보여준다."""
    db_path = event_log.DEFAULT_DB
    rows = []
    if os.path.exists(db_path):
        con = sqlite3.connect(db_path, timeout=3.0)
        try:
            con.row_factory = sqlite3.Row
            rows = [dict(r) for r in con.execute(
                "SELECT ts, node, detail, person_count, image FROM events "
                "WHERE image IS NOT NULL ORDER BY ts_epoch DESC LIMIT ?", (limit,)
            ).fetchall()]
        except sqlite3.Error:
            rows = []
        finally:
            con.close()
    # 파일이 실제로 남아있는 것만 준다(지운 사진의 깨진 썸네일을 안 보이게).
    out = []
    for r in rows:
        if os.path.exists(os.path.join(event_log.SHOT_DIR, r["image"])):
            out.append(r)
    return out


def fetch_events(limit=30):
    db_path = event_log.DEFAULT_DB
    if not os.path.exists(db_path):
        return []
    con = sqlite3.connect(db_path, timeout=3.0)
    try:
        con.row_factory = sqlite3.Row
        # image 는 나중에 추가한 칸이라, 아직 없는 옛 DB 에서도 죽지 않게 확인한다.
        cols = {r[1] for r in con.execute("PRAGMA table_info(events)")}
        img = "image" if "image" in cols else "NULL AS image"
        rows = con.execute(
            f"SELECT ts, node, event_type, detail, person_count, zone, {img} "
            "FROM events ORDER BY ts_epoch DESC LIMIT ?", (limit,)
        ).fetchall()
        return [dict(r) for r in rows]
    except sqlite3.Error:
        return []
    finally:
        con.close()


PAGE_TMPL = """<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>순찰 로봇 관제</title>
<style>
  * { box-sizing: border-box; }
  body { font-family: system-ui, -apple-system, sans-serif; margin: 0;
         background: #eef0f3; color: #1f2328; }
  header { background: #16191d; color: #fff; padding: 12px 20px;
           display: flex; align-items: center; justify-content: space-between; }
  header h1 { margin: 0; font-size: 17px; letter-spacing: -0.2px; }
  header .clock { font-variant-numeric: tabular-nums; opacity: 0.75; font-size: 14px; }

  /* ---------- 경고 구역: 화면 맨 위에서 한눈에 보이게 ---------- */
  #alertzone { display: grid; gap: 10px; padding: 12px 16px 0;
               grid-template-columns: repeat(auto-fit, minmax(260px, 1fr)); }
  .alert { border-radius: 10px; padding: 14px 16px; border: 2px solid transparent;
           background: #fff; display: flex; align-items: center; gap: 12px; }
  .alert .icon { font-size: 26px; line-height: 1; }
  .alert .body { flex: 1; min-width: 0; }
  .alert .title { font-weight: 700; font-size: 15px; }
  .alert .sub { font-size: 12px; opacity: 0.7; margin-top: 3px;
                overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
  /* 평상시 = 조용한 초록. 경고 = 강한 빨강/주황 + 깜빡임 */
  .alert.ok { border-color: #cfe8d5; background: #f2fbf5; color: #14532d; }
  .alert.fire { border-color: #d1242f; background: #ffe3e3; color: #8b0f16;
                animation: flash 0.9s infinite; }
  .alert.warn { border-color: #d97706; background: #fff4e0; color: #8a4b00;
                animation: flash 1.3s infinite; }
  @keyframes flash { 50% { filter: brightness(0.9); } }

  .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 14px; padding: 14px 16px; }
  .card { background: #fff; border-radius: 10px; padding: 15px;
          box-shadow: 0 1px 3px rgba(0,0,0,0.09); }
  .card h2 { margin: 0 0 10px; font-size: 14px; color: #57606a;
             text-transform: uppercase; letter-spacing: 0.4px; }
  .wide { grid-column: 1 / -1; }
  #camimg { width: 100%; border-radius: 6px; background: #ddd; display: block; }

  /* ---------- 작업금지 모드 제어 ---------- */
  .moderow { display: flex; gap: 8px; margin-bottom: 10px; }
  .modebtn { flex: 1; padding: 11px 8px; border: 1px solid #d0d7de; background: #f6f8fa;
             border-radius: 8px; cursor: pointer; font-size: 13px; font-weight: 600; }
  .modebtn.active { background: #16191d; color: #fff; border-color: #16191d; }
  .modehint { font-size: 12px; color: #57606a; line-height: 1.6; }
  .badge { display: inline-block; padding: 2px 8px; border-radius: 99px;
           font-size: 11px; font-weight: 700; }
  .badge.on { background: #ffe3e3; color: #8b0f16; }
  .badge.off { background: #eaeef2; color: #57606a; }

  /* ---------- 주행 조작 ---------- */
  .keys { display: grid; grid-template-columns: repeat(3, 54px);
          grid-template-rows: repeat(2, 46px); gap: 6px;
          justify-content: center; margin: 10px 0; }
  .keys button { font-size: 17px; border: 1px solid #d0d7de; border-radius: 7px;
                 background: #f6f8fa; cursor: pointer; }
  .keys button:active, .keys button.active { background: #16191d; color: #fff; }
  #kf { grid-column: 2; grid-row: 1; }
  #kl { grid-column: 1; grid-row: 2; }
  #ks { grid-column: 2; grid-row: 2; }
  #kr { grid-column: 3; grid-row: 2; }
  #stopbtn { display: block; margin: 6px auto 0; padding: 8px 22px; background: #d1242f;
             color: #fff; border: none; border-radius: 7px; font-weight: 700; cursor: pointer; }
  .note { font-size: 11px; color: #8b949e; text-align: center; margin-top: 8px; }

  /* ---------- 증거 사진 ---------- */
  #photos { display: grid; gap: 10px;
            grid-template-columns: repeat(auto-fill, minmax(150px, 1fr)); }
  .shot { border: 1px solid #e2e6ea; border-radius: 8px; overflow: hidden; background: #fafbfc; }
  .shot img { width: 100%; display: block; cursor: pointer; }
  .shot .cap { font-size: 11px; padding: 6px 7px; color: #57606a; }

  table { width: 100%; border-collapse: collapse; font-size: 13px; }
  th, td { text-align: left; padding: 6px 8px; border-bottom: 1px solid #eee; }
  th { color: #57606a; font-weight: 600; }
  td.thumb img { height: 34px; border-radius: 3px; display: block; cursor: pointer; }

  /* 사진 크게 보기 */
  #viewer { position: fixed; inset: 0; background: rgba(0,0,0,0.85); display: none;
            align-items: center; justify-content: center; z-index: 50; padding: 20px; }
  #viewer img { max-width: 100%; max-height: 100%; border-radius: 6px; }
  @media (max-width: 720px) { .grid { grid-template-columns: 1fr; } }
</style>
</head>
<body>
<header>
  <h1>순찰 로봇 관제</h1>
  <div class="clock" id="clock">--:--:--</div>
</header>

<!-- 경고 구역 — 화재/침입/안전모를 눈으로 바로 확인 -->
<div id="alertzone">
  <div class="alert ok" id="a-fire">
    <div class="icon">🔥</div>
    <div class="body"><div class="title">화재 감지</div><div class="sub">확인 중...</div></div>
  </div>
  <div class="alert ok" id="a-intrusion">
    <div class="icon">🚷</div>
    <div class="body"><div class="title">작업금지구역</div><div class="sub">확인 중...</div></div>
  </div>
  <div class="alert ok" id="a-helmet">
    <div class="icon">⛑️</div>
    <div class="body"><div class="title">안전모</div><div class="sub">확인 중...</div></div>
  </div>
</div>

<div class="grid">
  <div class="card">
    <h2>실시간 카메라</h2>
    <img id="camimg" src="/snapshot.jpg" alt="카메라 로딩 중...">
    <div class="note" id="camnote"></div>
  </div>

  <div class="card">
    <h2>작업금지구역 모드</h2>
    <div class="moderow">
      <button class="modebtn" data-mode="auto" id="m-auto">자동 (시간표)</button>
      <button class="modebtn" data-mode="on" id="m-on">강제 감시</button>
      <button class="modebtn" data-mode="off" id="m-off">감시 해제</button>
    </div>
    <div class="modehint" id="modehint">불러오는 중...</div>
    <hr style="border:none;border-top:1px solid #eee;margin:14px 0">
    <h2>화재 시연 (수동)</h2>
    <div class="moderow">
      <button class="modebtn" id="f-on">화재 발생</button>
      <button class="modebtn" id="f-off">화재 해제</button>
    </div>
    <div class="note">불꽃센서와 별개로, 시연용으로 화재 상황을 만들 수 있습니다</div>
  </div>

  <div class="card">
    <h2>주행 조작 (↑↓←→ 또는 WASD)</h2>
    <div class="keys">
      <button id="kf">▲</button><button id="kl">◀</button>
      <button id="ks">▼</button><button id="kr">▶</button>
    </div>
    <button id="stopbtn">정지 (Space)</button>
    <div class="note">키를 떼거나 0.4초간 신호가 없으면 자동 정지합니다</div>
  </div>

  <div class="card">
    <h2>소화기 점검</h2>
    <div class="modehint" id="extstatus">불러오는 중...</div>
  </div>

  <div class="card wide">
    <h2>증거 사진 (침입 감지 시 자동 촬영)</h2>
    <div id="photos"><div class="modehint">불러오는 중...</div></div>
  </div>

  <div class="card wide">
    <h2>이벤트 기록</h2>
    <table>
      <thead><tr><th>시각</th><th>노드</th><th>종류</th><th>내용</th><th>인원</th><th>사진</th></tr></thead>
      <tbody id="eventsbody"><tr><td colspan="6">불러오는 중...</td></tr></tbody>
    </table>
  </div>
</div>

<div id="viewer"><img id="viewerimg" src=""></div>

<script>
// ---------------- 상태 갱신 ----------------
function paint(el, level, title, sub) {
  el.className = 'alert ' + level;
  el.querySelector('.title').textContent = title;
  el.querySelector('.sub').textContent = sub;
}

function refreshStatus() {
  fetch('/api/status').then(r => r.json()).then(d => {
    document.getElementById('clock').textContent = d.now;

    // 화재 — 24시간 감시
    paint(document.getElementById('a-fire'),
          d.fire.alert ? 'fire' : 'ok',
          d.fire.alert ? '화재 감지! 대피 안내 중' : '화재 감지 — 정상 (24시간 감시)',
          d.fire.text);

    // 작업금지구역
    const iv = d.intrusion;
    let sub = iv.text;
    if (iv.mode && iv.mode !== '?') {
      const modeLabel = {auto: '자동', on: '강제 감시', off: '감시 해제'}[iv.mode] || iv.mode;
      sub = '모드: ' + modeLabel + (iv.window ? ' / 금지시간 ' + iv.window : '')
            + ' / 지금 ' + (iv.watching ? '감시 중' : '감시 안 함');
    }
    paint(document.getElementById('a-intrusion'),
          iv.alert ? 'warn' : 'ok',
          iv.alert ? '작업금지구역 인원 감지!' : '작업금지구역 — 이상 없음',
          sub);

    // 안전모
    paint(document.getElementById('a-helmet'),
          d.helmet.alert ? 'warn' : 'ok',
          d.helmet.alert ? '안전모 미착용 감지!' : '안전모 — 이상 없음',
          d.helmet.text);

    // 모드 버튼 활성 표시
    document.querySelectorAll('.modebtn[data-mode]').forEach(b => {
      b.classList.toggle('active', b.dataset.mode === iv.mode);
    });
    document.getElementById('modehint').innerHTML =
      '<b>자동</b>: 금지시간(' + (iv.window || '00:00-06:00') + ')에만 감시<br>' +
      '<b>강제 감시</b>: 시간 무관하게 항상 감시<br>' +
      '<b>감시 해제</b>: 감시 중지 (주간 작업 중)<br>' +
      '현재: <span class="badge ' + (iv.watching ? 'on' : 'off') + '">' +
      (iv.watching ? '감시 중' : '감시 안 함') + '</span>';

    document.getElementById('extstatus').textContent = d.extinguisher.text;
    document.getElementById('camnote').textContent =
      d.camera_age_sec >= 0 ? ('영상 ' + d.camera_age_sec.toFixed(1) + '초 전') : '영상 없음';
  }).catch(() => {});
}

function refreshCam() {
  document.getElementById('camimg').src = '/snapshot.jpg?t=' + Date.now();
}

function refreshPhotos() {
  fetch('/api/photos').then(r => r.json()).then(rows => {
    const box = document.getElementById('photos');
    if (!rows.length) {
      box.innerHTML = '<div class="modehint">아직 저장된 사진이 없습니다</div>';
      return;
    }
    box.innerHTML = rows.map(r =>
      '<div class="shot"><img src="/photo/' + encodeURIComponent(r.image) +
      '" onclick="showPhoto(this.src)">' +
      '<div class="cap">' + r.ts + '<br>' + (r.person_count ?? '?') + '명</div></div>'
    ).join('');
  }).catch(() => {});
}

function refreshEvents() {
  fetch('/api/events').then(r => r.json()).then(rows => {
    const body = document.getElementById('eventsbody');
    if (!rows.length) { body.innerHTML = '<tr><td colspan="6">기록 없음</td></tr>'; return; }
    body.innerHTML = rows.map(r => {
      const thumb = r.image
        ? '<td class="thumb"><img src="/photo/' + encodeURIComponent(r.image) +
          '" onclick="showPhoto(this.src)"></td>'
        : '<td></td>';
      return '<tr><td>' + r.ts + '</td><td>' + r.node + '</td><td>' + r.event_type +
             '</td><td>' + (r.detail || '') + '</td><td>' + (r.person_count ?? '') +
             '</td>' + thumb + '</tr>';
    }).join('');
  }).catch(() => {});
}

// ---------------- 사진 크게 보기 ----------------
function showPhoto(src) {
  document.getElementById('viewerimg').src = src;
  document.getElementById('viewer').style.display = 'flex';
}
document.getElementById('viewer').addEventListener('click', () => {
  document.getElementById('viewer').style.display = 'none';
});

// ---------------- 모드 / 화재 시연 ----------------
function post(url, payload) {
  return fetch(url, {method: 'POST', headers: {'Content-Type': 'application/json'},
                     body: JSON.stringify(payload)}).catch(() => {});
}
document.querySelectorAll('.modebtn[data-mode]').forEach(b => {
  b.addEventListener('click', () => {
    post('/api/mode', {mode: b.dataset.mode}).then(() => setTimeout(refreshStatus, 300));
  });
});
document.getElementById('f-on').addEventListener('click', () => {
  post('/api/fire_test', {on: true}).then(() => setTimeout(refreshStatus, 300));
});
document.getElementById('f-off').addEventListener('click', () => {
  post('/api/fire_test', {on: false}).then(() => setTimeout(refreshStatus, 300));
});

// ---------------- 키보드/버튼 주행 조작 ----------------
// 키를 누르고 있는 동안 150ms 마다 명령을 보낸다. mux 쪽 timeout(0.4초)보다
// 훨씬 짧은 간격이라, 신호가 하나 씹혀도 자동정지로 이어지지 않는다.
let teleopTimer = null;
let curLinear = 0, curAngular = 0;

function sendTeleop(linear, angular) {
  post('/api/teleop', {linear, angular});
}
function startMove(linear, angular, btnId) {
  curLinear = linear; curAngular = angular;
  if (btnId) document.getElementById(btnId).classList.add('active');
  if (teleopTimer) clearInterval(teleopTimer);
  sendTeleop(curLinear, curAngular);
  teleopTimer = setInterval(() => sendTeleop(curLinear, curAngular), 150);
}
function stopMove() {
  if (teleopTimer) { clearInterval(teleopTimer); teleopTimer = null; }
  document.querySelectorAll('.keys button').forEach(b => b.classList.remove('active'));
  sendTeleop(0, 0);
}

// 표준 ROS 관례(+x = 전진). Nav2 자율주행에서 실측 검증된 방향과 같다.
const KEYMAP = {
  'ArrowUp': [0.10, 0, 'kf'], 'w': [0.10, 0, 'kf'], 'W': [0.10, 0, 'kf'],
  'ArrowDown': [-0.10, 0, 'ks'], 's': [-0.10, 0, 'ks'], 'S': [-0.10, 0, 'ks'],
  'ArrowLeft': [0, 0.6, 'kl'], 'a': [0, 0.6, 'kl'], 'A': [0, 0.6, 'kl'],
  'ArrowRight': [0, -0.6, 'kr'], 'd': [0, -0.6, 'kr'], 'D': [0, -0.6, 'kr'],
};
const heldKeys = new Set();
document.addEventListener('keydown', e => {
  if (e.key === ' ') { stopMove(); return; }
  const m = KEYMAP[e.key];
  if (!m || heldKeys.has(e.key)) return;
  heldKeys.add(e.key);
  startMove(m[0], m[1], m[2]);
});
document.addEventListener('keyup', e => {
  heldKeys.delete(e.key);
  if (heldKeys.size === 0) stopMove();
  else {
    const last = [...heldKeys][heldKeys.size - 1];
    const m = KEYMAP[last];
    if (m) startMove(m[0], m[1], m[2]);
  }
});
window.addEventListener('blur', stopMove);

document.getElementById('kf').addEventListener('mousedown', () => startMove(0.10, 0, 'kf'));
document.getElementById('ks').addEventListener('mousedown', () => startMove(-0.10, 0, 'ks'));
document.getElementById('kl').addEventListener('mousedown', () => startMove(0, 0.6, 'kl'));
document.getElementById('kr').addEventListener('mousedown', () => startMove(0, -0.6, 'kr'));
document.querySelectorAll('.keys button').forEach(b => {
  b.addEventListener('mouseup', stopMove);
  b.addEventListener('mouseleave', stopMove);
});
document.getElementById('stopbtn').addEventListener('click', stopMove);

// ---------------- 주기 갱신 ----------------
setInterval(refreshCam, 1000);
setInterval(refreshStatus, 1500);
setInterval(refreshPhotos, 8000);
setInterval(refreshEvents, 5000);
refreshCam(); refreshStatus(); refreshPhotos(); refreshEvents();
</script>
</body>
</html>
"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass  # 콘솔 도배 방지 — 조용히 서빙한다

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index"):
            self._send(200, "text/html; charset=utf-8", PAGE_TMPL.encode("utf-8"))
        elif self.path.startswith("/snapshot.jpg"):
            with state_lock:
                frame = state["frame"]
            if frame is None:
                self._send(503, "text/plain", b"no frame yet")
            else:
                self._send(200, "image/jpeg", frame)
        elif self.path.startswith("/api/status"):
            body = json.dumps(build_status()).encode("utf-8")
            self._send(200, "application/json", body)
        elif self.path.startswith("/api/events"):
            body = json.dumps(fetch_events()).encode("utf-8")
            self._send(200, "application/json", body)
        elif self.path.startswith("/api/photos"):
            body = json.dumps(fetch_photos()).encode("utf-8")
            self._send(200, "application/json", body)
        elif self.path.startswith("/photo/"):
            self._send_photo(self.path[len("/photo/"):])
        else:
            self._send(404, "text/plain", b"not found")

    def _send_photo(self, name):
        """증거 사진 한 장을 보낸다.

        ⚠️ 경로 검사를 반드시 한다 — 파일명을 그대로 이어붙이면 "../../.ssh/id_rsa"
        같은 요청으로 이 노트북의 아무 파일이나 읽힐 수 있다(경로 순회 취약점).
        파일명만 뽑고(basename), 실제 경로가 사진 폴더 안인지 다시 확인한다.
        """
        name = unquote(name.split("?")[0])
        safe = os.path.basename(name)
        if not safe or not safe.lower().endswith((".jpg", ".jpeg", ".png")):
            self._send(404, "text/plain", b"not found")
            return
        base = os.path.realpath(event_log.SHOT_DIR)
        path = os.path.realpath(os.path.join(base, safe))
        if not path.startswith(base + os.sep) or not os.path.isfile(path):
            self._send(404, "text/plain", b"not found")
            return
        ctype = "image/png" if path.lower().endswith(".png") else "image/jpeg"
        with open(path, "rb") as f:
            self._send(200, ctype, f.read())

    def do_POST(self):
        if self.path.startswith("/api/teleop"):
            length = int(self.headers.get("Content-Length", 0))
            try:
                data = json.loads(self.rfile.read(length) or b"{}")
                linear = float(data.get("linear", 0.0))
                angular = float(data.get("angular", 0.0))
            except (ValueError, json.JSONDecodeError):
                self._send(400, "text/plain", b"bad request")
                return
            _dashboard_node.send_teleop(linear, angular)
            self._send(200, "application/json", b'{"ok": true}')
        elif self.path.startswith("/api/mode"):
            length = int(self.headers.get("Content-Length", 0))
            try:
                data = json.loads(self.rfile.read(length) or b"{}")
                mode = str(data.get("mode", "")).lower()
            except (ValueError, json.JSONDecodeError):
                self._send(400, "text/plain", b"bad request")
                return
            if mode not in ("auto", "on", "off"):
                self._send(400, "application/json", b'{"error": "mode must be auto|on|off"}')
                return
            _dashboard_node.send_mode(mode)
            self._send(200, "application/json", b'{"ok": true}')
        elif self.path.startswith("/api/fire_test"):
            length = int(self.headers.get("Content-Length", 0))
            try:
                data = json.loads(self.rfile.read(length) or b"{}")
                on = bool(data.get("on", False))
            except (ValueError, json.JSONDecodeError):
                self._send(400, "text/plain", b"bad request")
                return
            _dashboard_node.send_fire_trigger(on)
            self._send(200, "application/json", b'{"ok": true}')
        else:
            self._send(404, "text/plain", b"not found")

    def _send(self, code, ctype, body):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


_dashboard_node = None


def main():
    global _dashboard_node
    rclpy.init()
    node = DashboardNode()
    _dashboard_node = node
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    server = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    print(f"대시보드: http://localhost:{PORT}  (Ctrl-C 로 종료)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
