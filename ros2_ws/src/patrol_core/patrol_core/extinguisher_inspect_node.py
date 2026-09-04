#!/usr/bin/env python3
"""extinguisher_inspect_node.py — QR 로 소화기를 식별하고 압력계를 판정해 기록한다.

[VM에서 실행]  (영상 인식 연산은 로봇이 아닌 VM 에서 — 절대 규칙 3)
  ros2 run patrol_core extinguisher_inspect_node

  CSI 카메라(후미)로 볼 때:
  ros2 run patrol_core extinguisher_inspect_node --ros-args \
    -p topic:=/camera/image_raw/compressed

[입력]  <topic>  (CompressedImage)  기본값은 웹캠. CSI 가 붙으면 그쪽으로 바꾼다.
        /extinguisher/inspect (Bool)  True 면 지금 보이는 화면으로 즉시 점검한다
                                      (웹 버튼·시연용. QR 없이도 강제 점검)
[출력]  /extinguisher/inspect_status (String)  진단·웹·LCD 용
        /speaker/play (String)  "gauge_ok" — 정상일 때 한 번 안내
        logs/events.sqlite 의 inspections 표에 점검 한 건 기록 + 사진 저장

[동작]
  ① 화면에서 QR 을 찾아 읽는다(zbarimg). 읽히면 그 문자열로 대장에서 소화기를 찾는다.
  ② 그 소화기의 압력계 기준(gauge_calib.yaml + gauge_ref_<이름>.png)으로 판정한다.
  ③ 판정 결과 + 대장 정보(제조년월·교체년월·책임자)를 기록하고 화면·LCD·음성으로 알린다.
  ④ 같은 소화기를 계속 보고 있어도 recheck_sec 안에는 한 번만 기록한다.

[왜 QR 판독을 zbarimg 로 하나]
이 프로젝트의 OpenCV 는 QUIRC 가 링크되지 않아 QR '판독'을 못 한다(탐지만 된다).
그래서 판독은 zbar 로 한다 — qr_probe.py 가 같은 이유로 같은 방식을 쓴다.
zbarimg 는 도커 이미지(docker/Dockerfile.ros2)에 들어있다.

[자동 정렬 — 소화기를 보면 몸체를 돌려 게이지를 화면 가운데로 맞춘다]
gauge.judge 가 게이지가 기준 자리에서 밀린 픽셀(offset dx)을 주므로, 그걸 각도로
환산해 그만큼 제자리 회전한 뒤 다시 본다. inspect_node.py 가 Nav2 로 하는 것과
같은 되먹임이지만, 여기서는 Nav2 없이 /cmd_vel_teleop(cmd_vel_mux 의 최우선 입력)
으로 돌린다 — 발표 구성에서 Nav2 를 안 띄우기 때문이다.
⚠️ /cmd_vel 을 직접 발행하지 않는다(절대 규칙). mux 가 유일한 발행자이고,
   mux 는 0.4초 안에 신호가 없으면 스스로 멈춘다(데드맨 스위치).

**회전 방향은 스스로 배운다.** 돌렸는데 오프셋이 오히려 커지면 부호를 뒤집는다 —
카메라가 후미에 달려 있어 방향을 머리로 따지면 틀리기 쉽다(원본에서도 실제로
반대로 넣어 로봇이 대상에서 멀어진 일이 있었다). 되먹임으로 확인하면 처음 부호가
틀려도 한 시도만 손해 본다.

[압력계 기준이 없으면]
gauge_calib.yaml 에 그 소화기 항목이 없거나 기준사진이 없으면 압력계는 "판정불가"
로 남기고, QR 로 읽은 대장 정보만 표시·기록한다 — 현장에서 기준사진을 아직 안 찍었어도
QR 식별과 유효기한 확인은 되어야 하기 때문이다.
  기준사진 등록: python3 ~/vibe/ex1/tools/gauge_calib.py --grab --name 소화기1 --select
"""
import datetime
import os
import subprocess
import tempfile
import time

import cv2
import math
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String

from patrol_core import event_log
from patrol_core import gauge as G

EX1 = os.path.join(os.path.expanduser("~"), "vibe", "ex1")
DEFAULT_INFO = os.path.join(EX1, "maps", "extinguisher_info.yaml")
DEFAULT_CALIB = os.path.join(EX1, "maps", "gauge_calib.yaml")


class ExtinguisherInspectNode(Node):
    def __init__(self):
        super().__init__("extinguisher_inspect_node")

        # 기본은 웹캠 — CSI 가 아직 인식되지 않아서다(2026-09-04 확인: 커널에 센서가
        # 안 잡히고 ISP 장치만 보인다 = 리본 케이블 확인 필요). CSI 가 붙으면
        # topic 만 바꿔 주면 나머지는 그대로 동작한다.
        self.declare_parameter("topic", "/webcam/image_raw/compressed")
        self.declare_parameter("info_file", DEFAULT_INFO)
        self.declare_parameter("calib_file", DEFAULT_CALIB)
        # QR 을 몇 프레임마다 찾을지. 판독은 파일로 내보내 zbarimg 를 부르므로
        # 매 프레임 하면 낭비다(1초에 한두 번이면 충분하다).
        self.declare_parameter("qr_every", 8)
        # 같은 소화기를 이 시간(초) 안에 다시 보면 기록하지 않는다 — 카메라를
        # 대고 있는 동안 같은 점검이 수십 건 쌓이는 것을 막는다.
        self.declare_parameter("recheck_sec", 60.0)
        self.declare_parameter("voice_enabled", True)
        self.declare_parameter("save_shot", True)
        self.declare_parameter("db_path", event_log.DEFAULT_DB)
        self.declare_parameter("quiet", False)

        # --- 자동 정렬 (위 주석 참고) ---
        self.declare_parameter("align_enabled", True)
        # 이 안에 들면 맞은 것으로 본다. gauge 의 max_offset_px(120)와 같게 둔다 —
        # 그 문턱을 넘으면 어차피 판정 자체를 안 하므로 더 맞출 이유가 없다.
        self.declare_parameter("align_tolerance_px", 120.0)
        self.declare_parameter("align_max_attempts", 4)
        # 한 번에 이보다 크게 돌리지 않는다(과보정하면 반대편으로 넘어가 진동한다).
        self.declare_parameter("align_max_step_deg", 12.0)
        # 이보다 작은 보정은 하지 않는다 — 회전 오차·측정 잡음보다 작으면 진동만 한다.
        self.declare_parameter("align_min_step_deg", 1.5)
        # 회전 속도(rad/s). mux 상한(1.2)보다 훨씬 낮게 — 천천히 돌아야 오버슈트가 적다.
        self.declare_parameter("align_turn_speed", 0.35)
        # 회전 직후엔 로봇이 흔들린다. 가라앉기를 기다렸다 찍는다.
        self.declare_parameter("align_settle_sec", 1.2)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_status = self.create_publisher(
            String, "/extinguisher/inspect_status", qos)
        self.pub_speaker = self.create_publisher(String, "/speaker/play", qos)
        # 정렬용 회전 — cmd_vel_mux 의 최우선 입력으로 보낸다(절대 규칙: /cmd_vel 직접 발행 금지)
        self.pub_teleop = self.create_publisher(Twist, "/cmd_vel_teleop", qos)
        self.create_subscription(Bool, "/extinguisher/inspect", self.on_request, qos)
        self.create_subscription(
            CompressedImage, str(self.get_parameter("topic").value),
            self.on_frame, qos_profile_sensor_data)

        self.frame = None
        self.n_recv = 0
        self.last_done = {}        # 이름 -> 마지막 기록 시각
        self.last_qr = None
        self.force = False         # 웹에서 "지금 점검" 을 눌렀다
        # 회전 부호. +1 이면 "양의 각도 = 게이지가 화면에서 왼쪽으로" 라는 가정이고,
        # 돌려 보고 오프셋이 커지면 -1 로 뒤집는다(위 주석: 방향은 스스로 배운다).
        self.turn_sign = 1
        self.aligning = False      # 정렬 중에는 프레임 콜백이 끼어들지 않게

        self.get_logger().info(
            f"소화기 점검 시작 — 영상={self.get_parameter('topic').value}, "
            f"대장={os.path.basename(str(self.get_parameter('info_file').value))}"
        )
        self.create_timer(2.0, self.status_tick)

    # ---------------- 대장 ----------------
    def load_info(self):
        path = str(self.get_parameter("info_file").value)
        if not os.path.exists(path):
            self.get_logger().error(f"{path} 가 없다 — 소화기 대장을 등록할 것")
            return []
        try:
            with open(path) as f:
                data = yaml.safe_load(f) or {}
            return data.get("extinguishers", [])
        except (OSError, yaml.YAMLError) as e:                 # noqa: BLE001
            self.get_logger().error(f"대장을 읽지 못했다: {e}")
            return []

    def find_by_qr(self, qr):
        for item in self.load_info():
            if str(item.get("qr_id", "")).strip() == qr.strip():
                return item
        return None

    # ---------------- QR 판독 ----------------
    def read_qr(self, frame):
        """화면에서 QR 문자열을 읽는다. 없으면 None.

        cv2 로는 판독이 안 되므로(QUIRC 미링크) 임시 파일로 내보내 zbarimg 를 부른다.
        """
        try:
            with tempfile.NamedTemporaryFile(suffix=".png", delete=True) as tf:
                if not cv2.imwrite(tf.name, frame):
                    return None
                r = subprocess.run(["zbarimg", "--quiet", "--raw", tf.name],
                                   capture_output=True, text=True, timeout=5)
            if r.returncode == 0 and r.stdout.strip():
                return r.stdout.strip().splitlines()[0].strip()
        except FileNotFoundError:
            self.get_logger().error(
                "zbarimg 가 없다 — QR 판독 불가. patrol-ros2:humble 이미지에서 실행할 것",
                throttle_duration_sec=60.0)
        except subprocess.SubprocessError:
            pass
        return None

    # ---------------- 압력계 판정 ----------------
    def judge_gauge(self, frame, name):
        """(판정, 근거) 를 돌려준다. 기준이 없으면 판정불가."""
        calib_path = str(self.get_parameter("calib_file").value)
        if not os.path.exists(calib_path):
            return "판정불가", f"{os.path.basename(calib_path)} 가 없다"
        try:
            points = G.load_calib(calib_path)
        except (OSError, yaml.YAMLError) as e:                 # noqa: BLE001
            return "판정불가", f"기준 파일을 읽지 못했다: {e}"
        cal = points.get(name)
        if cal is None:
            return "판정불가", f"'{name}' 의 압력계 기준이 없다 (gauge_calib.py 로 등록)"
        ref = G.load_ref(calib_path, name)
        if ref is None:
            return "판정불가", f"'{name}' 의 기준사진이 없다 (gauge_calib.py --grab)"
        res = G.judge(frame, cal, ref)
        status = res.get("status") or "판정불가"
        bits = []
        for key, label in (("change", "변화량"), ("score", "정합"),
                          ("contrast", "대비"), ("reflection", "반사%")):
            v = res.get(key)
            if v is not None:
                bits.append(f"{label} {v:.2f}")
        if res.get("reason"):
            bits.append(str(res["reason"]))
        return status, ", ".join(bits)

    # ---------------- 자동 정렬 ----------------
    def measure_offset(self, name):
        """지금 보이는 화면에서 게이지가 기준 자리에서 밀린 dx(px)를 잰다.

        못 재면 None — 게이지를 아예 못 찾았거나 기준이 없는 경우다.
        """
        if self.frame is None:
            return None
        calib_path = str(self.get_parameter("calib_file").value)
        if not os.path.exists(calib_path):
            return None
        try:
            cal = G.load_calib(calib_path).get(name)
        except (OSError, yaml.YAMLError):
            return None
        if cal is None:
            return None
        ref = G.load_ref(calib_path, name)
        if ref is None:
            return None
        res = G.judge(self.frame, cal, ref)
        off = res.get("offset")
        return None if off is None else float(off[0])

    def turn_degrees(self, deg):
        """제자리로 deg 만큼 돈다. /cmd_vel_teleop 으로 보내 mux 가 최종 발행한다.

        mux 는 0.4초 안에 신호가 없으면 멈추므로, 도는 동안 20Hz 로 계속 보낸다.
        끝나면 0 을 보내 확실히 세운다(타임아웃에 맡기지 않는다 — 그 사이 조금 더 간다).
        """
        speed = abs(float(self.get_parameter("align_turn_speed").value))
        if speed <= 0.0 or abs(deg) < 1e-3:
            return
        dur = math.radians(abs(deg)) / speed
        omega = speed if deg > 0 else -speed
        t = Twist()
        t.angular.z = omega
        end = time.time() + dur
        while time.time() < end:
            self.pub_teleop.publish(t)
            time.sleep(0.05)
        self.pub_teleop.publish(Twist())     # 정지

    def align(self, name):
        """게이지가 화면 기준 자리에 오도록 몸체를 돌린다. 맞으면 True."""
        if not bool(self.get_parameter("align_enabled").value):
            return True
        tol = float(self.get_parameter("align_tolerance_px").value)
        steps = max(int(self.get_parameter("align_max_attempts").value), 1)
        step_max = float(self.get_parameter("align_max_step_deg").value)
        step_min = float(self.get_parameter("align_min_step_deg").value)
        settle = float(self.get_parameter("align_settle_sec").value)
        px_per_deg = float(G.DEF["px_per_deg"])

        prev_abs = None
        for i in range(1, steps + 1):
            dx = self.measure_offset(name)
            if dx is None:
                self.get_logger().warn(
                    f"{name}: 게이지를 못 찾아 정렬을 건너뛴다 "
                    "(기준사진이 없거나 화면에 안 보인다)")
                return False
            if abs(dx) <= tol:
                self.get_logger().info(f"{name}: 정렬 완료 (dx {dx:+.0f}px)")
                return True

            # 돌렸는데 오프셋이 오히려 커졌으면 방향을 잘못 잡은 것이다 — 부호를 뒤집는다.
            if prev_abs is not None and abs(dx) > prev_abs + 20.0:
                self.turn_sign *= -1
                self.get_logger().warn(
                    f"{name}: 오프셋이 커졌다({prev_abs:.0f}→{abs(dx):.0f}px) — "
                    f"회전 방향을 뒤집는다(sign={self.turn_sign:+d})")
            prev_abs = abs(dx)

            # 0.9 를 곱해 살짝 덜 돈다 — 과보정하면 반대편으로 넘어가 진동한다.
            d = -dx / px_per_deg * 0.9 * self.turn_sign
            d = max(-step_max, min(d, step_max))
            if abs(d) < step_min:
                self.get_logger().info(
                    f"{name}: dx {dx:+.0f}px 는 {d:+.1f}° 짜리 — 회전 정밀도 아래라 "
                    "여기서 멈춘다")
                return True

            self.get_logger().info(
                f"{name}: dx {dx:+.0f}px → {d:+.1f}° 돈다 ({i}/{steps})")
            self.turn_degrees(d)
            time.sleep(settle)

        self.get_logger().warn(f"{name}: {steps}번 돌려도 자세가 안 맞았다")
        return False

    # ---------------- 프레임 ----------------
    def on_request(self, msg: Bool):
        if bool(msg.data):
            self.force = True
            self.get_logger().info("웹 요청 — 지금 보이는 화면으로 점검한다")

    def on_frame(self, msg: CompressedImage):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is None:
            return
        self.frame = frame
        self.n_recv += 1

        # 정렬 중이면 프레임만 갱신하고 빠진다 — align() 이 self.frame 을 읽어
        # 되먹임을 돌리는 중인데, 여기서 또 점검을 시작하면 겹쳐서 로봇이 계속 돈다.
        if self.aligning:
            return

        forced = self.force
        self.force = False
        if not forced and self.n_recv % int(self.get_parameter("qr_every").value):
            return

        qr = self.read_qr(frame)
        if qr and qr != self.last_qr:
            self.get_logger().info(f"QR 판독: {qr}")
        self.last_qr = qr

        item = self.find_by_qr(qr) if qr else None
        if item is None:
            if forced:
                # 강제 점검인데 QR 이 없으면 대장의 첫 소화기로 본다 — 시연에서
                # QR 을 붙이기 전에도 압력계 판정을 보여줄 수 있어야 한다.
                info = self.load_info()
                if not info:
                    self.status("점검 실패 — 대장이 비어 있다")
                    return
                item = info[0]
                self.get_logger().warn(
                    f"QR 을 못 읽어 대장 첫 항목({item.get('name')})으로 점검한다")
            else:
                return

        self.inspect(frame, item, qr)

    # ---------------- 점검 한 건 ----------------
    def inspect(self, frame, item, qr):
        name = str(item.get("name", "?"))
        now = time.time()
        gap = float(self.get_parameter("recheck_sec").value)
        if now - self.last_done.get(name, 0.0) < gap:
            return              # 같은 소화기를 방금 점검했다
        self.last_done[name] = now

        # 몸체를 돌려 게이지를 화면 기준 자리에 맞춘다. 맞춘 뒤의 최신 프레임으로
        # 판정해야 하므로, 정렬이 끝나면 frame 을 다시 읽는다.
        self.aligning = True
        try:
            self.status(f"정렬 중 — {name}")
            self.align(name)
        finally:
            self.aligning = False
        if self.frame is not None:
            frame = self.frame

        verdict, detail = self.judge_gauge(frame, name)
        mfg = str(item.get("mfg_date", "") or "")
        exp = str(item.get("expiry_date", "") or "")
        manager = str(item.get("manager", "") or "")

        days_left = None
        if exp:
            try:
                d = datetime.date.fromisoformat(exp)
                days_left = (d - datetime.date.today()).days
            except ValueError:
                self.get_logger().warn(f"{name} 교체년월 형식 오류: {exp}")

        image = None
        if bool(self.get_parameter("save_shot").value):
            shot = frame.copy()
            lines = [f"{name}  QR={qr or '-'}",
                     f"GAUGE: {verdict}",
                     f"MFG {mfg}  EXP {exp}"
                     + (f"  D{days_left:+d}" if days_left is not None else ""),
                     f"MGR {manager}"]
            y = 22
            for ln in lines:
                # 한글은 cv2 폰트에 없어 깨진다 — 사진 위 글자는 로마자·숫자만 쓴다
                # (annotate() 가 같은 이유로 로마자를 쓴다). 한글 정보는 DB·웹에 있다.
                safe = ln.encode("ascii", "ignore").decode()
                cv2.putText(shot, safe, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                           0.55, (0, 0, 255) if verdict != "정상" else (0, 160, 0), 2)
                y += 22
            image = event_log.save_shot(shot, prefix="extinguisher")

        if not event_log.log_inspection(
                name, verdict, qr_id=qr, detail=detail, mfg_date=mfg,
                expiry_date=exp, manager=manager, days_left=days_left, image=image,
                db_path=str(self.get_parameter("db_path").value)):
            self.get_logger().warn("점검 기록 실패(sqlite)", throttle_duration_sec=30.0)

        left = f", 교체까지 D{days_left:+d}" if days_left is not None else ""
        self.get_logger().warn(
            f"[{name}] 압력계 {verdict} — {detail or '근거 없음'}{left} "
            f"(제조 {mfg or '?'} / 교체 {exp or '?'} / 책임자 {manager or '?'})")
        self.status(f"점검 {name} verdict={verdict} qr={qr or '-'} "
                    f"mfg={mfg} exp={exp} mgr={manager} "
                    f"days_left={days_left if days_left is not None else '?'}")

        # 사양서: "압력계 범위를 측정하고 나서 정상이면 한 번 안내한다"
        if verdict == "정상" and bool(self.get_parameter("voice_enabled").value):
            m = String()
            m.data = "gauge_ok"
            self.pub_speaker.publish(m)

    # ---------------- 상태 ----------------
    def status(self, text):
        m = String()
        m.data = text
        self.pub_status.publish(m)

    def status_tick(self):
        if self.frame is None:
            self.status("영상 대기 중")
        elif self.last_qr:
            self.status(f"대기 — QR={self.last_qr} 보임")
        else:
            self.status("대기 — QR 없음")


def main():
    rclpy.init()
    node = ExtinguisherInspectNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
