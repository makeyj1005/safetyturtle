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

sys.path.insert(0, os.path.join(os.path.expanduser("~"), "vibe", "ex1",
                                "ros2_ws", "src", "patrol_core"))

import rclpy                                    # noqa: E402
from geometry_msgs.msg import Twist             # noqa: E402
from rclpy.node import Node                     # noqa: E402
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import CompressedImage     # noqa: E402
from std_msgs.msg import String                 # noqa: E402

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
}


class DashboardNode(Node):
    def __init__(self):
        super().__init__("dashboard_server")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(CompressedImage, "/webcam/image_raw/compressed",
                                 self.on_frame, qos_profile_sensor_data)
        self.create_subscription(String, "/restricted/status", self.on_status, qos)
        self.pub_teleop = self.create_publisher(Twist, TELEOP_TOPIC, qos)

    def on_frame(self, msg):
        with state_lock:
            state["frame"] = bytes(msg.data)
            state["frame_at"] = time.time()

    def on_status(self, msg):
        with state_lock:
            state["restricted_status"] = msg.data
            state["restricted_at"] = time.time()

    def send_teleop(self, linear, angular):
        t = Twist()
        t.linear.x = max(-MAX_LINEAR, min(float(linear), MAX_LINEAR))
        t.angular.z = max(-MAX_ANGULAR, min(float(angular), MAX_ANGULAR))
        self.pub_teleop.publish(t)


def fetch_events(limit=30):
    db_path = event_log.DEFAULT_DB
    if not os.path.exists(db_path):
        return []
    con = sqlite3.connect(db_path, timeout=3.0)
    try:
        con.row_factory = sqlite3.Row
        rows = con.execute(
            "SELECT ts, node, event_type, detail, person_count, zone "
            "FROM events ORDER BY ts_epoch DESC LIMIT ?", (limit,)
        ).fetchall()
        return [dict(r) for r in rows]
    finally:
        con.close()


PAGE_TMPL = """<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8">
<title>순찰 로봇 대시보드</title>
<style>
  body { font-family: system-ui, sans-serif; margin: 0; background: #f2f2f2; }
  header { background: #222; color: #fff; padding: 12px 20px; }
  .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; padding: 16px; }
  .card { background: #fff; border-radius: 8px; padding: 16px;
          box-shadow: 0 1px 3px rgba(0,0,0,0.15); }
  .card h2 { margin-top: 0; font-size: 16px; color: #444; }
  #cam img { width: 100%; border-radius: 4px; background: #ddd; }
  table { width: 100%; border-collapse: collapse; font-size: 13px; }
  th, td { text-align: left; padding: 6px 8px; border-bottom: 1px solid #eee; }
  th { color: #666; font-weight: 600; }
  .wide { grid-column: 1 / -1; }

  /* 경고 알림 배너 */
  #alertbar { padding: 14px 20px; font-weight: bold; font-size: 15px;
              background: #e9f7ee; color: #1a7f37; transition: background 0.2s; }
  #alertbar.on { background: #ffe1e1; color: #d1242f;
                 animation: pulse 1s infinite; }
  @keyframes pulse { 50% { opacity: 0.55; } }

  /* 키보드 조작 */
  .keys { display: grid; grid-template-columns: repeat(3, 56px); grid-template-rows: repeat(2, 48px);
          gap: 6px; justify-content: center; margin: 12px 0; }
  .keys button { font-size: 18px; border: 1px solid #ccc; border-radius: 6px;
                 background: #f7f7f7; cursor: pointer; }
  .keys button:active, .keys button.active { background: #333; color: #fff; }
  #kf { grid-column: 2; grid-row: 1; }
  #kl { grid-column: 1; grid-row: 2; }
  #ks { grid-column: 2; grid-row: 2; }
  #kr { grid-column: 3; grid-row: 2; }
  #stopbtn { display: block; margin: 8px auto 0; padding: 8px 24px; background: #d1242f;
             color: #fff; border: none; border-radius: 6px; font-weight: bold; cursor: pointer; }
</style>
</head>
<body>
<header><h1 style="margin:0;font-size:18px;">순찰 로봇 대시보드</h1></header>
<div id="alertbar">상태 확인 중...</div>
<div class="grid">
  <div class="card" id="cam">
    <h2>실시간 카메라</h2>
    <img id="camimg" src="/snapshot.jpg" alt="카메라 로딩 중...">
  </div>
  <div class="card">
    <h2>주행 조작 (키보드: ↑↓←→ 또는 WASD)</h2>
    <div class="keys">
      <button id="kf">▲</button>
      <button id="kl">◀</button>
      <button id="ks">▼</button>
      <button id="kr">▶</button>
    </div>
    <button id="stopbtn">정지 (Space)</button>
    <p style="font-size:12px;color:#888;text-align:center;">
      키를 떼거나 0.4초 안에 신호가 안 오면 자동으로 멈춥니다(안전장치 내장)
    </p>
  </div>
  <div class="card wide">
    <h2>최근 이벤트 (SQLite 기록)</h2>
    <table>
      <thead><tr><th>시각</th><th>노드</th><th>종류</th><th>내용</th><th>인원</th><th>구역</th></tr></thead>
      <tbody id="eventsbody"><tr><td colspan="6">불러오는 중...</td></tr></tbody>
    </table>
  </div>
</div>
<script>
function refreshCam() {
  document.getElementById('camimg').src = '/snapshot.jpg?t=' + Date.now();
}
function refreshStatus() {
  fetch('/api/status').then(r => r.json()).then(d => {
    const bar = document.getElementById('alertbar');
    const alerting = d.status.includes('ALERT');
    bar.className = alerting ? 'on' : '';
    bar.textContent = d.status + '  (' + d.age_sec.toFixed(0) + '초 전 갱신)';
  });
}
function refreshEvents() {
  fetch('/api/events').then(r => r.json()).then(rows => {
    const body = document.getElementById('eventsbody');
    if (rows.length === 0) { body.innerHTML = '<tr><td colspan="6">기록 없음</td></tr>'; return; }
    body.innerHTML = rows.map(r =>
      '<tr><td>' + r.ts + '</td><td>' + r.node + '</td><td>' + r.event_type + '</td>' +
      '<td>' + (r.detail || '') + '</td><td>' + (r.person_count ?? '') + '</td>' +
      '<td>' + (r.zone || '') + '</td></tr>'
    ).join('');
  });
}
setInterval(refreshCam, 1000);
setInterval(refreshStatus, 2000);
setInterval(refreshEvents, 5000);
refreshCam(); refreshStatus(); refreshEvents();

// ---------------- 키보드/버튼 주행 조작 ----------------
// 키를 누르고 있는 동안 150ms 마다 명령을 보낸다. mux 쪽 timeout(0.4초)보다
// 훨씬 짧은 간격이라, 신호가 하나 씹혀도 자동정지로 이어지지 않는다.
// 키를 떼거나 창이 포커스를 잃으면 즉시 정지 명령을 보낸다.
let teleopTimer = null;
let curLinear = 0, curAngular = 0;

function sendTeleop(linear, angular) {
  fetch('/api/teleop', {
    method: 'POST', headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({linear, angular})
  }).catch(() => {});
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

const KEYMAP = {
  // 표준 ROS 관례(+x = 전진). Nav2 자율주행에서 실측 검증된 방향과 같다.
  // 2026-09-02 실측: 이 로봇은 +x 명령이 물리적으로는 후진이다(odom 자체는 일관돼서
  // Nav2 자율주행엔 영향 없음 — 사람이 보는 버튼 이름표만 실제 방향에 맞춘다).
  'ArrowUp': [-0.10, 0, 'kf'], 'w': [-0.10, 0, 'kf'], 'W': [-0.10, 0, 'kf'],
  'ArrowDown': [0.10, 0, 'ks'], 's': [0.10, 0, 'ks'], 'S': [0.10, 0, 'ks'],
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

document.getElementById('kf').addEventListener('mousedown', () => startMove(-0.10, 0, 'kf'));
document.getElementById('ks').addEventListener('mousedown', () => startMove(0.10, 0, 'ks'));
document.getElementById('kl').addEventListener('mousedown', () => startMove(0, 0.6, 'kl'));
document.getElementById('kr').addEventListener('mousedown', () => startMove(0, -0.6, 'kr'));
document.querySelectorAll('.keys button').forEach(b => {
  b.addEventListener('mouseup', stopMove);
  b.addEventListener('mouseleave', stopMove);
});
document.getElementById('stopbtn').addEventListener('click', stopMove);
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
            with state_lock:
                status = state["restricted_status"]
                age = time.time() - state["restricted_at"] if state["restricted_at"] else -1
            body = json.dumps({"status": status, "age_sec": age}).encode("utf-8")
            self._send(200, "application/json", body)
        elif self.path.startswith("/api/events"):
            body = json.dumps(fetch_events()).encode("utf-8")
            self._send(200, "application/json", body)
        else:
            self._send(404, "text/plain", b"not found")

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
