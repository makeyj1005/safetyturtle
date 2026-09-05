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
    "frame": None,           # 최신 jpeg 바이트 (전면 웹캠)
    "frame_at": 0.0,
    "frame_rear": None,      # 최신 jpeg 바이트 (후면 CSI)
    "frame_rear_at": 0.0,
    "restricted_status": "(아직 없음)",
    "restricted_at": 0.0,
    "fire_status": "(아직 없음)",
    "fire_at": 0.0,
    "helmet_status": "(아직 없음)",
    "helmet_at": 0.0,
    "extinguisher_status": "(아직 없음)",
    "extinguisher_at": 0.0,
    "speaker_status": "(아직 없음)",
    "speaker_at": 0.0,
    "inspect_status": "(아직 없음)",
    "inspect_at": 0.0,
    "safety_status": "(아직 없음)",
    "safety_at": 0.0,
    "helmet_rear_status": "(아직 없음)",
    "helmet_rear_at": 0.0,
}

# 상태가 이 시간(초) 넘게 갱신되지 않으면 "그 경고는 끝났다"고 본다.
# 각 노드가 상태를 바뀔 때만 내는 경우가 있어서, 화면이 옛 경고에 붙어있지 않게
# 하는 안전장치다(lcd_node 의 STALE_SEC 과 같은 이유).
STALE_SEC = 15.0

# 웹에서 누른 '전체 시동/정지'를 호스트의 supervisor.sh 에 전달하는 통로.
# 대시보드는 컨테이너 안이라 docker·ssh 를 쓸 수 없다. 대신 공유 폴더에
# 한 줄 써 두면 호스트에서 도는 감시 프로세스가 대신 실행해 준다.
CTL_DIR = os.path.join(os.path.expanduser('~'), 'vibe', 'ex1', 'logs', 'control')
CTL_REQUEST = os.path.join(CTL_DIR, 'request')
CTL_STATUS = os.path.join(CTL_DIR, 'status')


def ctl_write(action):
    """재시작/정지 요청을 남긴다. 성공하면 (True, '')."""
    try:
        os.makedirs(CTL_DIR, exist_ok=True)
        # 임시 파일에 쓰고 옮긴다 — 감시 쪽이 반쯤 써진 파일을 읽는 것을 막는다.
        tmp = CTL_REQUEST + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as fh:
            fh.write(action + '\n')
        os.replace(tmp, CTL_REQUEST)
        return True, ''
    except OSError as e:
        return False, str(e)


def ctl_read():
    """감시 프로세스의 상태를 (상태, 문구) 로 준다.

    파일이 없으면 감시 프로세스가 안 떠 있다는 뜻이다 — 그때 버튼을
    눌러도 아무 일도 안 일어나므로, 화면에서 미리 알려줘야 한다.
    """
    try:
        with open(CTL_STATUS, encoding='utf-8') as fh:
            line = fh.read().strip()
    except OSError:
        return 'absent', '감시 프로세스 미실행 (tools/supervisor.sh --daemon)'
    kind, _, text = line.partition('|')
    return (kind or 'idle'), (text or '대기 중')


class DashboardNode(Node):
    def __init__(self):
        super().__init__("dashboard_server")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(CompressedImage, "/webcam/image_raw/compressed",
                                 self.on_frame, qos_profile_sensor_data)
        # 후면 CSI. 카메라가 안 떠 있으면 그냥 프레임이 안 올 뿐,
        # 페이지는 "영상 없음" 으로 뜨고 나머지 기능은 그대로 돈다.
        self.create_subscription(CompressedImage, "/csi/image_raw/compressed",
                                 self.on_frame_rear, qos_profile_sensor_data)
        self.create_subscription(String, "/restricted/status", self.on_status, qos)
        self.create_subscription(String, "/fire/status", self.on_fire, qos)
        self.create_subscription(String, "/helmet/status", self.on_helmet, qos)
        self.create_subscription(String, "/extinguisher/status",
                                 self.on_extinguisher, qos)
        self.pub_teleop = self.create_publisher(Twist, TELEOP_TOPIC, qos)
        # 작업자가 웹에서 작업금지 감시 모드를 바꾼다(auto | on | off)
        self.pub_mode = self.create_publisher(String, "/restricted/mode", qos)
        # 재시작 감시용. 감시 프로세스의 상태가 바뀌는 순간을 잡는다.
        self._ctl_prev = None
        self.create_timer(2.0, self.watch_restart)
        # 화재 시연/수동 해제용
        self.pub_fire_trigger = self.create_publisher(Bool, "/fire/trigger", qos)
        # 감지 기능 켜고 끄기 (작업자가 웹에서 직접)
        self.pub_fire_enable = self.create_publisher(Bool, "/fire/enable", qos)
        self.pub_helmet_enable = self.create_publisher(Bool, "/helmet/enable", qos)
        self.pub_speaker_enable = self.create_publisher(Bool, "/speaker/enable", qos)
        self.create_subscription(String, "/speaker/status", self.on_speaker, qos)
        self.create_subscription(String, "/extinguisher/inspect_status",
                                 self.on_inspect, qos)
        # 웹 "지금 점검" 버튼 -> extinguisher_inspect_node
        self.pub_inspect = self.create_publisher(Bool, "/extinguisher/inspect", qos)
        # 라이다 충돌 방지 상태 (cmd_vel_mux 가 낸다)
        self.create_subscription(String, "/mux/safety", self.on_safety, qos)
        # 후면(CSI) 안전모 감시 — 앞에서만 쓰고 지나면 벗는 경우를 잡는 두 번째 인스턴스
        self.create_subscription(String, "/helmet_rear/status",
                                 self.on_helmet_rear, qos)

    def on_frame(self, msg):
        with state_lock:
            state["frame"] = bytes(msg.data)
            state["frame_at"] = time.time()

    def on_frame_rear(self, msg):
        with state_lock:
            state["frame_rear"] = bytes(msg.data)
            state["frame_rear_at"] = time.time()

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

    def on_speaker(self, msg):
        with state_lock:
            state["speaker_status"] = msg.data
            state["speaker_at"] = time.time()

    def on_inspect(self, msg):
        with state_lock:
            state["inspect_status"] = msg.data
            state["inspect_at"] = time.time()

    def on_helmet_rear(self, msg):
        with state_lock:
            state["helmet_rear_status"] = msg.data
            state["helmet_rear_at"] = time.time()

    def on_safety(self, msg):
        with state_lock:
            state["safety_status"] = msg.data
            state["safety_at"] = time.time()

    def send_inspect_now(self):
        m = Bool()
        m.data = True
        self.pub_inspect.publish(m)

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

    def send_enable(self, what, on):
        m = Bool()
        m.data = bool(on)
        if what == "fire":
            self.pub_fire_enable.publish(m)
        elif what == "helmet":
            self.pub_helmet_enable.publish(m)
        elif what == "speaker":
            self.pub_speaker_enable.publish(m)

    def watch_restart(self):
        """재시작이 끝나면 감지 기능을 켠다.

        갓 뜬 노드에 켜라고 미리 보내두면 구독자가 아직 없어서 그냥 버려진다.
        그래서 감시 프로세스가 running 에서 done 으로 넘어가는 순간에 보낸다.
        """
        kind, _ = ctl_read()
        prev = self._ctl_prev
        self._ctl_prev = kind
        if prev == 'running' and kind == 'done':
            self.get_logger().info('재시작 완료 — 감지 기능을 다시 켠다')
            self.send_start_all(True)

    def send_start_all(self, on):
        """감지 기능을 한꺼번에 켜거나 끈다(시동 / 전체 정지).

        노드(프로세스)를 띄우는 게 아니라 이미 떠 있는 노드의 감지를 켜고 끄는 것이다.
        프로세스까지 한 번에 띄우려면 tools/start_all.sh 를 쓴다 — 대시보드는
        컨테이너 안에서 돌아 도커·ssh 를 건드릴 수 없기 때문이다.
        """
        self.send_enable("fire", on)
        self.send_enable("helmet", on)
        self.send_enable("speaker", on)      # 시동하면 스피커도 함께 켠다
        # 작업금지구역은 켤 때 시간표(auto)로 돌려놓는다 — 강제 감시(on)로 켜면
        # 낮에도 계속 경고가 나서 작업자가 놀란다.
        self.send_mode("auto" if on else "off")


# CPU 사용률은 누적값의 차이로 구해야 한다 — /proc/stat 은 부팅 후 누적 시간이라
# 한 번만 읽으면 "부팅 후 평균"이 나온다. 이전 값을 들고 있다가 차이를 본다.
_cpu_prev = {"total": 0, "idle": 0}

# GPU 사용률 파일. 카드 번호는 기계마다 다를 수 있어 처음 한 번 찾아 둔다
# (이 노트북은 card1 = Radeon iGPU).
def _find_gpu_file():
    import glob
    for path in sorted(glob.glob("/sys/class/drm/card*/device/gpu_busy_percent")):
        try:
            with open(path) as f:
                int(f.read().strip())
            return path
        except (OSError, ValueError):
            continue
    return None


_GPU_FILE = _find_gpu_file()


def read_resources():
    """호스트의 CPU·GPU·메모리 사용률. 컨테이너 안에서도 /proc·/sys 로 호스트 값이 보인다."""
    out = {"cpu": None, "gpu": None, "mem": None, "cores": None, "load": None}

    # CPU — /proc/stat 첫 줄의 누적 지터를 이전 호출과 비교한다
    try:
        with open("/proc/stat") as f:
            parts = f.readline().split()
        vals = [int(v) for v in parts[1:]]
        total = sum(vals)
        idle = vals[3] + (vals[4] if len(vals) > 4 else 0)   # idle + iowait
        dt = total - _cpu_prev["total"]
        di = idle - _cpu_prev["idle"]
        if _cpu_prev["total"] and dt > 0:
            out["cpu"] = max(0.0, min(100.0, 100.0 * (dt - di) / dt))
        _cpu_prev["total"], _cpu_prev["idle"] = total, idle
    except (OSError, ValueError, IndexError):
        pass

    if _GPU_FILE:
        try:
            with open(_GPU_FILE) as f:
                out["gpu"] = float(f.read().strip())
        except (OSError, ValueError):
            pass

    try:
        info = {}
        with open("/proc/meminfo") as f:
            for line in f:
                k, _, v = line.partition(":")
                info[k] = int(v.split()[0])
        tot = info.get("MemTotal", 0)
        avail = info.get("MemAvailable", 0)
        if tot:
            out["mem"] = 100.0 * (tot - avail) / tot
            out["mem_total_gb"] = tot / 1024.0 / 1024.0
    except (OSError, ValueError, IndexError):
        pass

    try:
        out["cores"] = os.cpu_count()
        with open("/proc/loadavg") as f:
            out["load"] = float(f.read().split()[0])
    except (OSError, ValueError, IndexError):
        pass
    return out


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
    helmet_alert = fresh("helmet") and (
        helmet_txt.startswith("hold") or "미착용" in helmet_txt)

    def enabled_of(txt, alive):
        """노드가 내는 "idle enabled=yes" 에서 켜짐 여부를 뽑는다.
        노드 자체가 안 떠 있으면(신호 없음) None — 화면에서 "노드 없음"으로 구분한다."""
        if not alive:
            return None
        if "enabled=yes" in txt:
            return True
        if "enabled=no" in txt or txt == "disabled":
            return False
        return True      # 경보 중이면 enabled 문구가 없다 — 켜져 있는 것이다

    fire_enabled = enabled_of(fire_txt, fresh("fire"))
    helmet_enabled = enabled_of(helmet_txt, fresh("helmet"))
    speaker_enabled = enabled_of(snap["speaker_status"], fresh("speaker"))

    rear_txt = snap["helmet_rear_status"]
    helmet_rear_alert = fresh("helmet_rear") and (
        rear_txt.startswith("hold") or "미착용" in rear_txt)

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
        "fire": {"alert": fire_alert, "text": fire_txt, "age_sec": age("fire"),
                 "enabled": fire_enabled},
        "intrusion": {"alert": intrusion_alert, "text": restricted_txt,
                      "age_sec": age("restricted"), "mode": mode,
                      "watching": watching, "window": window},
        "helmet": {"alert": helmet_alert, "text": helmet_txt, "age_sec": age("helmet"),
                   "enabled": helmet_enabled},
        "helmet_rear": {"alert": helmet_rear_alert, "text": rear_txt,
                        "age_sec": age("helmet_rear")},
        "extinguisher": {"text": snap["extinguisher_status"],
                         "age_sec": age("extinguisher")},
        "speaker": {"text": snap["speaker_status"], "age_sec": age("speaker"),
                    "enabled": speaker_enabled},
        "inspect": {"text": snap["inspect_status"], "age_sec": age("inspect")},
        "safety": {"text": snap["safety_status"], "age_sec": age("safety"),
                   "blocked": fresh("safety")
                              and snap["safety_status"].startswith("blocked")},
        "camera_age_sec": age("frame"),
        "ctl": dict(zip(("kind", "text"), ctl_read())),
        "camera_rear_age_sec": age("frame_rear"),
        "res": read_resources(),
    }


def fetch_photos(node=None, limit=24):
    """저장된 증거 사진 목록(최신순).

    node 를 주면 그 노드가 남긴 것만 준다 — 작업금지구역(restricted_node)과
    안전모(helmet_node) 사진을 화면에서 다른 영역에 나눠 보여주기 위함이다.
    """
    db_path = event_log.DEFAULT_DB
    rows = []
    if os.path.exists(db_path):
        con = sqlite3.connect(db_path, timeout=3.0)
        try:
            con.row_factory = sqlite3.Row
            sql = ("SELECT ts, node, detail, person_count, image FROM events "
                   "WHERE image IS NOT NULL")
            args = []
            if node:
                sql += " AND node = ?"
                args.append(node)
            sql += " ORDER BY ts_epoch DESC LIMIT ?"
            args.append(limit)
            rows = [dict(r) for r in con.execute(sql, args).fetchall()]
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


def fetch_inspections(limit=30):
    """소화기 점검 기록(최신순)."""
    db_path = event_log.DEFAULT_DB
    if not os.path.exists(db_path):
        return []
    con = sqlite3.connect(db_path, timeout=3.0)
    try:
        con.row_factory = sqlite3.Row
        rows = con.execute(
            "SELECT ts, name, qr_id, verdict, detail, mfg_date, expiry_date, "
            "manager, days_left, image FROM inspections "
            "ORDER BY ts_epoch DESC LIMIT ?", (limit,)
        ).fetchall()
        return [dict(r) for r in rows]
    except sqlite3.Error:
        # inspections 표가 아직 없는 옛 DB 일 수 있다 — 빈 목록으로 넘긴다.
        return []
    finally:
        con.close()


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
  .hdrright { display: flex; align-items: center; gap: 10px; }
  /* 자원 사용량 게이지 — 숫자만 있으면 눈에 안 들어와서 막대를 같이 둔다 */
  .res { display: flex; gap: 12px; margin-right: 6px; }
  .resitem { display: flex; align-items: center; gap: 5px; font-size: 11px; }
  .reslabel { opacity: 0.6; letter-spacing: 0.5px; }
  .bar { display: inline-block; width: 52px; height: 7px; border-radius: 4px;
         background: rgba(255,255,255,0.18); overflow: hidden; }
  .bar i { display: block; height: 100%; width: 0%; background: #3fb950;
           transition: width 0.4s, background 0.4s; }
  .bar i.mid { background: #d29922; }
  .bar i.hot { background: #f85149; }
  .resval { font-variant-numeric: tabular-nums; width: 30px; text-align: right; }
  @media (max-width: 900px) { .res { display: none; } }

  .bigbtn { padding: 9px 16px; border: none; border-radius: 8px; cursor: pointer;
            font-size: 14px; font-weight: 700; }
  .bigbtn.go { background: #1a7f37; color: #fff; }
  .bigbtn.stop { background: #4a5157; color: #fff; }
  .bigbtn:active { transform: translateY(1px); }

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

  .grid { display: grid; grid-template-columns: 2.3fr 1fr; gap: 14px;
          padding: 14px 16px; align-items: start; }
  .card { background: #fff; border-radius: 10px; padding: 15px;
          box-shadow: 0 1px 3px rgba(0,0,0,0.09); }
  .card h2 { margin: 0 0 10px; font-size: 14px; color: #57606a;
             text-transform: uppercase; letter-spacing: 0.4px; }
  .wide { grid-column: 1 / -1; }
  #camimg, #camimgrear { width: 100%; border-radius: 6px; background: #ddd;
                         display: block; }
  .ctlrow { display: flex; align-items: center; gap: 8px; margin-top: 8px;
            flex-wrap: wrap; }
  .smallbtn { padding: 5px 10px; font-size: 12px; border: 1px solid #bbb;
              border-radius: 5px; background: #fff; cursor: pointer; }
  .smallbtn:hover { background: #f0f0f0; }
  .ctlstat { font-size: 12px; color: #555; }
  .ctlstat.run { color: #b45309; font-weight: 700; }
  .ctlstat.err { color: #b00020; font-weight: 700; }
  .camwrap { display: grid; grid-template-columns: 1fr; gap: 10px; }
  /* 아주 넓은 화면에서만 좌우로. 그보다 좁으면 나누는 순간
     각 화면이 절반이 되어 '크게 보자'는 목적과 반대가 된다. */
  @media (min-width: 2200px) { .camwrap { grid-template-columns: 1fr 1fr; } }
  .camlabel { font-size: 12px; font-weight: 700; margin: 0 0 4px; color: #333; }

  /* ---------- 작업금지 모드 제어 ---------- */
  .moderow { display: flex; gap: 6px; margin-bottom: 6px; }
  .modebtn { flex: 1; padding: 8px 6px; border: 1px solid #d0d7de; background: #f6f8fa;
             border-radius: 8px; cursor: pointer; font-size: 13px; font-weight: 600; }
  .modebtn.active { background: #16191d; color: #fff; border-color: #16191d; }
  .modehint { font-size: 11px; color: #57606a; line-height: 1.5; }
  /* 좁은 칸에 카드가 여러 개 겹쳐 들어가므로 제목 위 여백을 줄인다 */
  .card h2 + .moderow { margin-top: 0; }
  .compact h2 { margin: 12px 0 6px; }
  .compact h2:first-child { margin-top: 0; }
  .compact .modehint { margin-bottom: 2px; }
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
  .gallery { display: grid; gap: 10px;
             grid-template-columns: repeat(auto-fill, minmax(150px, 1fr)); }
  .shot { border: 1px solid #e2e6ea; border-radius: 8px; overflow: hidden; background: #fafbfc; }
  .shot img { width: 100%; display: block; cursor: pointer; }
  .shot .cap { font-size: 11px; padding: 6px 7px; color: #57606a; }

  /* 이벤트 필터 — 기록이 쌓이면 눈으로 훑기 어려워서 구분·검색을 둔다 */
  .filters { display: flex; flex-wrap: wrap; gap: 6px; align-items: center;
             margin-bottom: 10px; }
  .chip { padding: 5px 11px; border: 1px solid #d0d7de; background: #f6f8fa;
          border-radius: 99px; cursor: pointer; font-size: 12px; }
  .chip.active { background: #16191d; color: #fff; border-color: #16191d; }
  .search { flex: 1; min-width: 130px; padding: 6px 10px; border: 1px solid #d0d7de;
            border-radius: 7px; font-size: 12px; }
  .evcount { font-size: 12px; color: #57606a; }
  th.sortable { cursor: pointer; user-select: none; }
  th.sortable:hover { color: #16191d; }
  .arrow { font-size: 9px; opacity: 0.7; }
  .kindtag { display: inline-block; padding: 2px 7px; border-radius: 99px;
             font-size: 11px; font-weight: 600; white-space: nowrap; }

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
  <div class="hdrright">
    <div class="res" id="resbox">
      <div class="resitem"><span class="reslabel">CPU</span>
        <span class="bar"><i id="bar-cpu"></i></span><span class="resval" id="v-cpu">–</span></div>
      <div class="resitem"><span class="reslabel">GPU</span>
        <span class="bar"><i id="bar-gpu"></i></span><span class="resval" id="v-gpu">–</span></div>
      <div class="resitem"><span class="reslabel">MEM</span>
        <span class="bar"><i id="bar-mem"></i></span><span class="resval" id="v-mem">–</span></div>
    </div>
    <button id="startall" class="bigbtn go">▶ 전체 시동 (재시작)</button>
    <button id="stopall" class="bigbtn stop">■ 전체 정지</button>
    <div class="ctlrow">
      <button id="enableall" class="smallbtn">감지만 켜기</button>
      <button id="disableall" class="smallbtn">감지만 끄기</button>
      <span class="ctlstat" id="ctlstat">확인 중...</span>
    </div>
    <div class="clock" id="clock">--:--:--</div>
  </div>
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
    <div class="body"><div class="title">안전모 (전면)</div><div class="sub">확인 중...</div></div>
  </div>
  <div class="alert ok" id="a-helmet-rear">
    <div class="icon">🔄</div>
    <div class="body"><div class="title">안전모 (후면)</div><div class="sub">확인 중...</div></div>
  </div>
</div>

<div class="grid">
  <div class="card">
    <h2>실시간 카메라</h2>
    <div class="camwrap">
      <div>
        <div class="camlabel">전면 · 웹캠 (진행방향)</div>
        <img id="camimg" src="/stream.mjpg" alt="전면 카메라 로딩 중...">
        <div class="note" id="camnote"></div>
      </div>
      <div>
        <div class="camlabel">후면 · CSI (소화기 점검)</div>
        <img id="camimgrear" src="/stream_rear.mjpg" alt="후면 카메라 로딩 중...">
        <div class="note" id="camnoterear"></div>
      </div>
    </div>
  </div>

  <div class="card compact">
    <h2>작업금지구역 감시</h2>
    <div class="moderow">
      <button class="modebtn" data-mode="auto" id="m-auto">자동 (시간표)</button>
      <button class="modebtn" data-mode="on" id="m-on">강제 감시</button>
      <button class="modebtn" data-mode="off" id="m-off">감시 해제</button>
    </div>
    <div class="modehint" id="modehint">불러오는 중...</div>

    <hr style="border:none;border-top:1px solid #eee;margin:14px 0">
    <h2>화재 감지 (24시간)</h2>
    <div class="moderow">
      <button class="modebtn" data-en="fire" data-on="1" id="fe-on">감지 켜기</button>
      <button class="modebtn" data-en="fire" data-on="0" id="fe-off">감지 끄기</button>
    </div>
    <div class="modehint" id="firehint">불러오는 중...</div>

    <hr style="border:none;border-top:1px solid #eee;margin:14px 0">
    <h2>안전모 감지</h2>
    <div class="moderow">
      <button class="modebtn" data-en="helmet" data-on="1" id="he-on">감지 켜기</button>
      <button class="modebtn" data-en="helmet" data-on="0" id="he-off">감지 끄기</button>
    </div>
    <div class="modehint" id="helmethint">불러오는 중...</div>

    <hr style="border:none;border-top:1px solid #eee;margin:14px 0">
    <h2>스피커 (음성 안내)</h2>
    <div class="moderow">
      <button class="modebtn" data-en="speaker" data-on="1" id="se-on">소리 켜기</button>
      <button class="modebtn" data-en="speaker" data-on="0" id="se-off">음소거</button>
    </div>
    <div class="modehint" id="speakerhint">불러오는 중...</div>

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
    <div class="modehint" id="safetyhint" style="margin-top:10px"></div>
    <div class="note">키를 떼거나 0.4초간 신호가 없으면 자동 정지합니다</div>
  </div>

  <div class="card">
    <h2>소화기 점검</h2>
    <div class="modehint" id="extstatus">불러오는 중...</div>
    <div class="modehint" id="inspstatus" style="margin-top:8px"></div>
    <div class="moderow" style="margin-top:10px">
      <button class="modebtn" id="insp-now">지금 점검</button>
    </div>
    <div class="note">QR 이 없어도 대장 첫 소화기로 압력계를 판정합니다</div>
  </div>

  <div class="card wide">
    <h2>🚷 작업금지구역 침입 — 증거 사진</h2>
    <div class="gallery" id="photos-intrusion"><div class="modehint">불러오는 중...</div></div>
  </div>

  <div class="card wide">
    <h2>⛑️ 안전모 미착용 — 증거 사진</h2>
    <div class="gallery" id="photos-helmet"><div class="modehint">불러오는 중...</div></div>
  </div>

  <div class="card wide">
    <h2>🔄 안전모 미착용 (후면 CSI) — 증거 사진</h2>
    <div class="gallery" id="photos-helmet-rear"><div class="modehint">불러오는 중...</div></div>
  </div>

  <div class="card wide">
    <h2>🧯 소화기 점검 기록</h2>
    <table>
      <thead><tr><th>점검시각</th><th>소화기</th><th>QR</th><th>압력계</th>
                 <th>제조년월</th><th>교체년월</th><th>남은일수</th>
                 <th>책임자</th><th>사진</th></tr></thead>
      <tbody id="inspbody"><tr><td colspan="9">불러오는 중...</td></tr></tbody>
    </table>
  </div>

  <div class="card wide">
    <h2>이벤트 기록</h2>
    <div class="filters">
      <button class="chip active" data-f="all">전체</button>
      <button class="chip" data-f="fire">🔥 화재</button>
      <button class="chip" data-f="intrusion">🚷 침입</button>
      <button class="chip" data-f="helmet">⛑️ 안전모(전면)</button>
      <button class="chip" data-f="helmet_rear">🔄 안전모(후면)</button>
      <button class="chip" data-f="extinguisher">🧯 소화기</button>
      <button class="chip" data-f="photo">📷 사진 있는 것만</button>
      <input id="evsearch" class="search" placeholder="내용 검색...">
      <span class="evcount" id="evcount"></span>
    </div>
    <table>
      <thead><tr>
        <th class="sortable" data-k="ts">시각 <span class="arrow">▼</span></th>
        <th class="sortable" data-k="kind">구분 <span class="arrow"></span></th>
        <th>내용</th>
        <th class="sortable" data-k="person_count">인원 <span class="arrow"></span></th>
        <th>사진</th>
      </tr></thead>
      <tbody id="eventsbody"><tr><td colspan="5">불러오는 중...</td></tr></tbody>
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

    // 안전모 (전면 웹캠)
    paint(document.getElementById('a-helmet'),
          d.helmet.alert ? 'warn' : 'ok',
          d.helmet.alert ? '안전모 미착용 감지! (전면)' : '안전모 전면 — 이상 없음',
          d.helmet.text);

    // 안전모 (후면 CSI) — 로봇이 지나간 뒤 벗는 경우를 잡는다
    const hr = d.helmet_rear || {alert: false, text: '', age_sec: -1};
    paint(document.getElementById('a-helmet-rear'),
          hr.alert ? 'warn' : 'ok',
          hr.alert ? '안전모 미착용 감지! (후면)'
                   : (hr.age_sec < 0 ? '안전모 후면 — 노드 미실행' : '안전모 후면 — 이상 없음'),
          hr.text || 'CSI 카메라 필요');

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

    // 감지 켜짐/꺼짐 표시 + 버튼 활성
    function enHint(en, name) {
      if (en === null || en === undefined)
        return name + ' 노드가 실행되지 않았습니다';
      return '현재: <span class="badge ' + (en ? 'on' : 'off') + '">' +
             (en ? '감지 켜짐' : '감지 꺼짐') + '</span>';
    }
    document.getElementById('firehint').innerHTML = enHint(d.fire.enabled, '화재 감지');
    document.getElementById('helmethint').innerHTML = enHint(d.helmet.enabled, '안전모 감지');
    document.getElementById('speakerhint').innerHTML =
      (d.speaker.enabled === null || d.speaker.enabled === undefined)
        ? '스피커 노드가 실행되지 않았습니다'
        : ('현재: <span class="badge ' + (d.speaker.enabled ? 'on' : 'off') + '">' +
           (d.speaker.enabled ? '소리 켜짐' : '음소거') + '</span>');
    const enMap = {fire: d.fire.enabled, helmet: d.helmet.enabled,
                   speaker: d.speaker.enabled};
    document.querySelectorAll('.modebtn[data-en]').forEach(b => {
      const en = enMap[b.dataset.en];
      const wantOn = b.dataset.on === '1';
      b.classList.toggle('active', en !== null && en !== undefined && en === wantOn);
    });

    // 라이다 충돌 방지 표시 — 막혔으면 눈에 띄게
    const sh = document.getElementById('safetyhint');
    if (!d.safety || d.safety.age_sec < 0) {
      sh.innerHTML = '<span class="badge off">라이다 안전기능 대기</span>';
    } else if (d.safety.blocked) {
      sh.innerHTML = '<span class="badge on">⛔ 안전 정지 — 장애물 ' +
                     d.safety.text.replace('blocked ', '') +
                     '</span><br><small>회전은 가능합니다</small>';
    } else {
      sh.innerHTML = '<span class="badge off">✓ 진행 방향 안전</span>';
    }

    // 자원 사용량 — 60% 넘으면 주황, 85% 넘으면 빨강
    function setBar(id, vid, val) {
      const bar = document.getElementById(id), lab = document.getElementById(vid);
      if (val === null || val === undefined) {
        bar.style.width = '0%'; lab.textContent = '–'; return;
      }
      bar.style.width = Math.max(2, Math.min(100, val)) + '%';
      bar.className = val >= 85 ? 'hot' : (val >= 60 ? 'mid' : '');
      lab.textContent = val.toFixed(0) + '%';
    }
    const r = d.res || {};
    setBar('bar-cpu', 'v-cpu', r.cpu);
    setBar('bar-gpu', 'v-gpu', r.gpu);
    setBar('bar-mem', 'v-mem', r.mem);
    const rb = document.getElementById('resbox');
    if (rb) {
      rb.title = 'CPU ' + (r.cores || '?') + '코어, load ' + (r.load ?? '?') +
                 (r.mem_total_gb ? (' / 메모리 ' + r.mem_total_gb.toFixed(1) + 'GB') : '');
    }

    document.getElementById('extstatus').textContent = d.extinguisher.text;
    document.getElementById('inspstatus').textContent =
      (d.inspect && d.inspect.age_sec >= 0) ? d.inspect.text : '점검 노드 미실행';
    document.getElementById('camnote').textContent =
      d.camera_age_sec >= 0 ? ('영상 ' + d.camera_age_sec.toFixed(1) + '초 전') : '영상 없음';
    // 서버는 프레임을 받고 있는데 화면만 멈춘 경우(스트림 연결이 끊긴 것)
    // 다시 붙인다. 서버도 프레임이 없으면 카메라 문제이므로 건드리지 않는다.
    if (d.camera_age_sec >= 0 && d.camera_age_sec < 2) {
      const el = document.getElementById('camimg');
      if (el && !el.naturalWidth) reconnectStream('camimg', '/stream.mjpg');
    }
    if (d.camera_rear_age_sec >= 0 && d.camera_rear_age_sec < 3) {
      const er = document.getElementById('camimgrear');
      if (er && !er.naturalWidth) reconnectStream('camimgrear', '/stream_rear.mjpg');
    }
    if (d.ctl) {
      const el = document.getElementById('ctlstat');
      el.textContent = d.ctl.text;
      el.className = 'ctlstat'
        + (d.ctl.kind === 'running' ? ' run'
           : (d.ctl.kind === 'error' || d.ctl.kind === 'absent') ? ' err' : '');
    }
    document.getElementById('camnoterear').textContent =
      d.camera_rear_age_sec >= 0
        ? ('영상 ' + d.camera_rear_age_sec.toFixed(1) + '초 전')
        : 'CSI 카메라 미실행 (로봇에서 csi_camera.sh)';
  }).catch(() => {});
}

// 카메라는 MJPEG 스트림이라 주기 갱신이 필요 없다 — 서버가 새 프레임을
// 밀어준다. 다만 카메라가 끊겼다 돌아오면 브라우저가 스스로 다시 붙지
// 않는 경우가 있어, 영상이 오래 멈춘 게 보이면 그때만 다시 연결한다.
function reconnectStream(id, url) {
  const el = document.getElementById(id);
  if (el) el.src = url + '?t=' + Date.now();
}

function refreshGallery(kind, elId, emptyMsg) {
  fetch('/api/photos?kind=' + kind).then(r => r.json()).then(rows => {
    const box = document.getElementById(elId);
    if (!rows.length) {
      box.innerHTML = '<div class="modehint">' + emptyMsg + '</div>';
      return;
    }
    box.innerHTML = rows.map(r => {
      const who = (r.person_count !== null && r.person_count !== undefined)
                  ? (r.person_count + '명') : '';
      return '<div class="shot"><img src="/photo/' + encodeURIComponent(r.image) +
             '" onclick="showPhoto(this.src)">' +
             '<div class="cap">' + r.ts + (who ? '<br>' + who : '') + '</div></div>';
    }).join('');
  }).catch(() => {});
}

function refreshPhotos() {
  refreshGallery('intrusion', 'photos-intrusion', '아직 침입 감지 사진이 없습니다');
  refreshGallery('helmet', 'photos-helmet', '아직 안전모 미착용 사진이 없습니다');
  refreshGallery('helmet_rear', 'photos-helmet-rear',
                 '아직 후면 안전모 미착용 사진이 없습니다');
}

function refreshInspections() {
  fetch('/api/inspections').then(r => r.json()).then(rows => {
    const body = document.getElementById('inspbody');
    if (!rows.length) {
      body.innerHTML = '<tr><td colspan="9">아직 점검 기록이 없습니다</td></tr>';
      return;
    }
    body.innerHTML = rows.map(r => {
      const bad = r.verdict !== '정상';
      const vcell = '<td style="font-weight:700;color:' +
                    (bad ? '#d1242f' : '#1a7f37') + '">' + r.verdict + '</td>';
      let dcell = '';
      if (r.days_left !== null && r.days_left !== undefined) {
        const soon = r.days_left < 30;
        dcell = '<td style="color:' + (soon ? '#d1242f' : '#57606a') + '">D' +
                (r.days_left >= 0 ? '+' : '') + r.days_left + '</td>';
      } else { dcell = '<td></td>'; }
      const thumb = r.image
        ? '<td class="thumb"><img src="/photo/' + encodeURIComponent(r.image) +
          '" onclick="showPhoto(this.src)"></td>'
        : '<td></td>';
      return '<tr><td>' + r.ts + '</td><td>' + (r.name || '') + '</td><td>' +
             (r.qr_id || '-') + '</td>' + vcell +
             '<td>' + (r.mfg_date || '-') + '</td><td>' + (r.expiry_date || '-') +
             '</td>' + dcell + '<td>' + (r.manager || '-') + '</td>' + thumb + '</tr>';
    }).join('');
  }).catch(() => {});
}

// ---------------- 이벤트 기록: 구분·검색·정렬 ----------------
// 노드 이름(restricted_node 등)은 사람이 훑기 어려워서 보기 좋은 구분으로 바꾼다.
const KINDS = {
  fire_node:        {key: 'fire',        label: '🔥 화재',           bg: '#ffe3e3', fg: '#8b0f16'},
  restricted_node:  {key: 'intrusion',   label: '🚷 침입',           bg: '#fff4e0', fg: '#8a4b00'},
  helmet_node:      {key: 'helmet',      label: '⛑️ 안전모(전면)',   bg: '#fff8dc', fg: '#7a5c00'},
  helmet_node_rear: {key: 'helmet_rear', label: '🔄 안전모(후면)',   bg: '#eef4ff', fg: '#1f3f8a'},
  extinguisher_expiry_node:  {key: 'extinguisher', label: '🧯 소화기 기한', bg: '#eaf7ee', fg: '#14532d'},
  extinguisher_inspect_node: {key: 'extinguisher', label: '🧯 소화기 점검', bg: '#eaf7ee', fg: '#14532d'},
};
function kindOf(node) {
  return KINDS[node] || {key: 'etc', label: node, bg: '#eaeef2', fg: '#57606a'};
}

let evAll = [];            // 서버에서 받은 전체 기록
let evFilter = 'all';
let evSort = {key: 'ts', desc: true};

function renderEvents() {
  const body = document.getElementById('eventsbody');
  const q = (document.getElementById('evsearch').value || '').trim().toLowerCase();

  let rows = evAll.filter(r => {
    const k = kindOf(r.node);
    if (evFilter === 'photo') { if (!r.image) return false; }
    else if (evFilter !== 'all' && k.key !== evFilter) return false;
    if (q) {
      const hay = (r.ts + ' ' + k.label + ' ' + (r.detail || '') + ' ' +
                   (r.event_type || '')).toLowerCase();
      if (!hay.includes(q)) return false;
    }
    return true;
  });

  rows.sort((a, b) => {
    let x, y;
    if (evSort.key === 'kind') { x = kindOf(a.node).label; y = kindOf(b.node).label; }
    else if (evSort.key === 'person_count') {
      x = a.person_count ?? -1; y = b.person_count ?? -1;
    } else { x = a.ts; y = b.ts; }
    if (x < y) return evSort.desc ? 1 : -1;
    if (x > y) return evSort.desc ? -1 : 1;
    return 0;
  });

  document.getElementById('evcount').textContent =
    rows.length + ' / ' + evAll.length + '건';

  if (!rows.length) {
    body.innerHTML = '<tr><td colspan="5">해당하는 기록이 없습니다</td></tr>';
    return;
  }
  body.innerHTML = rows.map(r => {
    const k = kindOf(r.node);
    const tag = '<span class="kindtag" style="background:' + k.bg + ';color:' + k.fg +
                '">' + k.label + '</span>';
    const thumb = r.image
      ? '<td class="thumb"><img src="/photo/' + encodeURIComponent(r.image) +
        '" onclick="showPhoto(this.src)"></td>'
      : '<td></td>';
    return '<tr><td>' + r.ts + '</td><td>' + tag + '</td><td>' +
           (r.detail || r.event_type || '') + '</td><td>' +
           (r.person_count ?? '') + '</td>' + thumb + '</tr>';
  }).join('');
}

function refreshEvents() {
  fetch('/api/events').then(r => r.json()).then(rows => {
    evAll = rows;
    renderEvents();
  }).catch(() => {});
}

// 구분 버튼
document.querySelectorAll('.chip[data-f]').forEach(b => {
  b.addEventListener('click', () => {
    document.querySelectorAll('.chip[data-f]').forEach(x => x.classList.remove('active'));
    b.classList.add('active');
    evFilter = b.dataset.f;
    renderEvents();
  });
});
// 검색
document.getElementById('evsearch').addEventListener('input', renderEvents);
// 열 제목 클릭 정렬
document.querySelectorAll('th.sortable').forEach(th => {
  th.addEventListener('click', () => {
    const k = th.dataset.k;
    // 같은 열을 다시 누르면 오름/내림을 뒤집는다
    evSort = (evSort.key === k) ? {key: k, desc: !evSort.desc} : {key: k, desc: true};
    document.querySelectorAll('th.sortable .arrow').forEach(a => a.textContent = '');
    th.querySelector('.arrow').textContent = evSort.desc ? '▼' : '▲';
    renderEvents();
  });
});

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

// 버튼을 누르면 노드가 상태를 낼 때까지 잠깐 걸린다(2초 주기). 그 사이 화면이
// 그대로면 "안 눌렸나" 싶으므로, 누른 버튼을 바로 강조하고 몇 번 더 빨리 새로고침한다.
function markPressed(btn, group) {
  if (group) group.forEach(b => b.classList.remove('active'));
  btn.classList.add('active');
}
function pollFast() {
  [200, 600, 1200, 2200].forEach(ms => setTimeout(refreshStatus, ms));
}
const modeBtns = [...document.querySelectorAll('.modebtn[data-mode]')];
modeBtns.forEach(b => {
  b.addEventListener('click', () => {
    markPressed(b, modeBtns);
    post('/api/mode', {mode: b.dataset.mode}).then(pollFast);
  });
});
document.querySelectorAll('.modebtn[data-en]').forEach(b => {
  b.addEventListener('click', () => {
    // 같은 기능(fire/helmet/speaker)의 켜기·끄기 버튼끼리만 강조를 옮긴다
    const group = [...document.querySelectorAll(
      '.modebtn[data-en="' + b.dataset.en + '"]')];
    markPressed(b, group);
    post('/api/enable', {what: b.dataset.en, on: b.dataset.on === '1'}).then(pollFast);
  });
});
document.getElementById('insp-now').addEventListener('click', () => {
  post('/api/inspect_now', {}).then(pollFast);
});
document.getElementById('startall').addEventListener('click', () => {
  if (!confirm('모든 노드를 재시작합니다 (약 40초).\\n\\n카메라·센서·감지 노드를 전부 껐다 다시 켭니다. 계속하시겠습니까?')) return;
  post('/api/restart_all', {action: 'restart'}).then(pollFast);
});
document.getElementById('stopall').addEventListener('click', () => {
  if (!confirm('모든 노드를 정지합니다.\\n\\n웹페이지만 남고 카메라·센서·감지가 전부 꺼집니다. 계속하시겠습니까?')) return;
  post('/api/restart_all', {action: 'stop'}).then(pollFast);
});
// 프로세스는 그대로 두고 감지만 껐다 켠다 — 즉시 반응한다.
document.getElementById('enableall').addEventListener('click', () => {
  post('/api/start_all', {on: true}).then(pollFast);
});
document.getElementById('disableall').addEventListener('click', () => {
  post('/api/start_all', {on: false}).then(pollFast);
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
  // ⚠️ 이 로봇은 +x 명령이 물리적으로는 후진이다(2026-09-04 실측, 이전에도 같은
  // 결론이었다). odom 자체는 일관적이라 Nav2 자율주행엔 영향이 없고, 사람이 보는
  // 버튼 이름표만 실제 방향에 맞춘 것이다.
  // 방향이 또 반대로 느껴지면 여기 부호만 뒤집으면 된다(아래 mousedown 4줄도 함께).
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

// ---------------- 주기 갱신 ----------------
setInterval(refreshStatus, 800);
setInterval(refreshPhotos, 8000);
setInterval(refreshEvents, 5000);
setInterval(refreshInspections, 6000);
refreshCam(); refreshStatus(); refreshPhotos(); refreshEvents(); refreshInspections();
</script>
</body>
</html>
"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass  # 콘솔 도배 방지 — 조용히 서빙한다

    def _send_mjpeg(self, frame_key, at_key):
        """프레임이 새로 올 때마다 밀어보낸다(multipart/x-mixed-replace).

        [왜 폴링을 버렸나]
        전에는 브라우저가 1초마다 /snapshot.jpg 를 다시 받았다. 카메라가
        13fps 로 와도 화면은 1fps 라 뚝뚝 끊겨 보였다. CPU 나 무선 탓이
        아니라 화면 갱신 주기가 병목이었다.
        폴링 주기를 100ms 로 줄이는 방법도 있지만 초당 10번씩 새 HTTP
        요청·헤더·연결이 생긴다. 스트리밍은 연결 하나로 끝나고, 새 프레임이
        있을 때만 보내므로 낭비도 없다.
        """
        boundary = "frameboundary"
        self.send_response(200)
        self.send_header("Content-Type",
                         f"multipart/x-mixed-replace; boundary={boundary}")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.end_headers()

        last_at = 0.0
        idle_since = time.time()
        try:
            while True:
                with state_lock:
                    frame = state[frame_key]
                    at = state[at_key]
                if frame is not None and at > last_at:
                    last_at = at
                    idle_since = time.time()
                    self.wfile.write(f"--{boundary}\r\n".encode())
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(
                        f"Content-Length: {len(frame)}\r\n\r\n".encode())
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
                else:
                    # 카메라가 멎었는데 연결만 붙잡고 있으면 스레드가 샌다.
                    # 60초 동안 새 프레임이 없으면 끊는다 — 브라우저가
                    # 알아서 다시 붙는다.
                    if time.time() - idle_since > 60.0:
                        break
                # 20ms 마다 확인한다. 카메라가 15fps(66ms)라 충분히 촘촘하고,
                # 그보다 짧게 돌면 CPU 만 먹는다.
                time.sleep(0.02)
        except (BrokenPipeError, ConnectionResetError):
            pass          # 사용자가 페이지를 닫았다 — 정상 종료다

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index"):
            self._send(200, "text/html; charset=utf-8", PAGE_TMPL.encode("utf-8"))
        elif self.path.startswith("/stream.mjpg"):
            self._send_mjpeg("frame", "frame_at")
        elif self.path.startswith("/stream_rear.mjpg"):
            self._send_mjpeg("frame_rear", "frame_rear_at")
        elif self.path.startswith("/snapshot.jpg"):
            with state_lock:
                frame = state["frame"]
            if frame is None:
                self._send(503, "text/plain", b"no frame yet")
            else:
                self._send(200, "image/jpeg", frame)
        elif self.path.startswith("/snapshot_rear.jpg"):
            with state_lock:
                frame = state["frame_rear"]
            if frame is None:
                self._send(503, "text/plain", b"no rear frame yet")
            else:
                self._send(200, "image/jpeg", frame)
        elif self.path.startswith("/api/status"):
            body = json.dumps(build_status()).encode("utf-8")
            self._send(200, "application/json", body)
        elif self.path.startswith("/api/events"):
            body = json.dumps(fetch_events(limit=300)).encode("utf-8")
            self._send(200, "application/json", body)
        elif self.path.startswith("/api/inspections"):
            body = json.dumps(fetch_inspections()).encode("utf-8")
            self._send(200, "application/json", body)
        elif self.path.startswith("/api/photos"):
            # /api/photos?kind=intrusion|helmet (없으면 전체)
            kind = ""
            if "?" in self.path:
                for part in self.path.split("?", 1)[1].split("&"):
                    if part.startswith("kind="):
                        kind = part[5:]
            node = {"intrusion": "restricted_node",
                    "helmet": "helmet_node",
                    "helmet_rear": "helmet_node_rear"}.get(kind)
            body = json.dumps(fetch_photos(node=node)).encode("utf-8")
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
        elif self.path.startswith("/api/enable"):
            length = int(self.headers.get("Content-Length", 0))
            try:
                data = json.loads(self.rfile.read(length) or b"{}")
                what = str(data.get("what", "")).lower()
                on = bool(data.get("on", False))
            except (ValueError, json.JSONDecodeError):
                self._send(400, "text/plain", b"bad request")
                return
            if what not in ("fire", "helmet", "speaker"):
                self._send(400, "application/json",
                          b'{"error": "what must be fire|helmet|speaker"}')
                return
            _dashboard_node.send_enable(what, on)
            self._send(200, "application/json", b'{"ok": true}')
        elif self.path.startswith("/api/inspect_now"):
            self.rfile.read(int(self.headers.get("Content-Length", 0)) or 0)
            _dashboard_node.send_inspect_now()
            self._send(200, "application/json", b'{"ok": true}')
        elif self.path.startswith("/api/restart_all"):
            length = int(self.headers.get("Content-Length", 0))
            try:
                data = json.loads(self.rfile.read(length) or b"{}")
                action = str(data.get("action", ""))
            except (ValueError, json.JSONDecodeError):
                self._send(400, "text/plain", b"bad request")
                return
            # 받을 동작을 딱 두 개로 못 박는다. 웹에서 온 문자열이
            # 그대로 셸로 흘러가지 않게 하는 게 핵심이다.
            if action not in ("restart", "stop"):
                self._send(400, "text/plain", b"unknown action")
                return
            okw, err = ctl_write(action)
            if not okw:
                self._send(500, "application/json",
                           json.dumps({"ok": False, "error": err}).encode())
                return
            self._send(200, "application/json", b'{"ok": true}')
        elif self.path.startswith("/api/start_all"):
            length = int(self.headers.get("Content-Length", 0))
            try:
                data = json.loads(self.rfile.read(length) or b"{}")
                on = bool(data.get("on", True))
            except (ValueError, json.JSONDecodeError):
                self._send(400, "text/plain", b"bad request")
                return
            _dashboard_node.send_start_all(on)
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
