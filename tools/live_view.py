#!/usr/bin/env python3
"""live_view.py — 로봇 카메라를 실시간으로 보면서 소화기·게이지 조준을 돕는다.

[VM 에서, 화면이 있는 터미널에서 실행]
  python3 ~/vibe/ex1/tools/live_view.py
  python3 ~/vibe/ex1/tools/live_view.py --gauge-cm 2.0 --body-cm 11

  키      s = 현재 프레임 원본 저장,  q 또는 ESC = 종료

[화면에 겹쳐 보여주는 것]
  초록 원      게이지 후보 (지름 px 와 추정 cm 를 함께 표시)
  파란 사각형  빨강 덩어리 = 소화기 몸통 추정. 폭으로 거리를 역산한다
  노란 글자    반사 경고 — 게이지 후보 안에 포화(흰색) 화소가 많으면 바늘이 묻힌다
  격자         화면 삼분할. 게이지가 위쪽 1/3 안에 오도록 두면 여유가 생긴다

[왜 이런 표시를 하나]
소화기를 어느 각도로 돌려야 게이지가 보이는지, 로봇이 얼마나 떨어져야 바늘을 볼
만큼 커지는지는 사람이 눈으로 판단하기 어렵다. 게이지 후보의 지름(px)과 반사량을
숫자로 보여주면 "지금 이 자세가 쓸 수 있는지"를 바로 알 수 있다.

[주의]
초록 원이 뜬다고 게이지라는 보장은 없다. 라벨의 동그란 로고나 원통 윤곽도 원으로
잡힌다. 추정 지름(cm)이 실제 게이지 크기와 비슷한 것만 신뢰할 것.
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import CompressedImage

# imx219 수평 화각(도). 거리 역산과 cm 추정에 쓴다.
HFOV_DEG = 62.2


def image_qos(kind):
    """구독 QoS. 발행자와 짝이 맞아야 한다.

    reliable 발행자 + best_effort 구독자는 '호환'은 되지만 재전송을 받지 못한다.
    큰 프레임(수백 KB)은 조각 유실로 통째로 버려지므로, 발행자를 reliable 로 띄웠다면
    구독자도 reliable 로 받아야 의미가 있다. 반대로 발행자가 best_effort 인데
    구독자만 reliable 이면 QoS 불일치로 아예 데이터가 오지 않는다.
    """
    if kind == "reliable":
        return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST)
    return qos_profile_sensor_data


class LiveView(Node):
    def __init__(self, topic, gauge_cm, body_cm, save_dir, max_w, qos="sensor"):
        super().__init__("live_view")
        self.gauge_cm = gauge_cm
        self.body_cm = body_cm
        self.save_dir = save_dir
        self.max_w = max_w
        self.frame = None          # 원본 (저장용)
        self.n = 0
        self.t_fps = time.time()
        self.fps = 0.0
        self.last_log = 0.0
        os.makedirs(save_dir, exist_ok=True)
        self.create_subscription(CompressedImage, topic, self.on_img, image_qos(qos))
        self.get_logger().info(
            f"{topic} 구독 (QoS={qos}) — 창이 열리면 s 저장 / q 종료")

    def on_img(self, msg):
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is not None:
            self.frame = img
            self.n += 1
            if self.n % 5 == 0:
                now = time.time()
                self.fps = 5.0 / max(now - self.t_fps, 1e-6)
                self.t_fps = now

    # ---------------- 화면 구성 ----------------
    def analyze(self, img):
        """빨강 덩어리와 원 후보를 찾는다. 큰 원본에서 바로 돌리면 느리므로 축소해서 본다."""
        h, w = img.shape[:2]
        k = 1000.0 / w if w > 1000 else 1.0
        small = cv2.resize(img, None, fx=k, fy=k, interpolation=cv2.INTER_AREA) if k != 1.0 else img

        hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
        red = cv2.inRange(hsv, (0, 120, 60), (10, 255, 255)) | \
              cv2.inRange(hsv, (170, 120, 60), (180, 255, 255))
        red = cv2.morphologyEx(red, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
        nlab, _, stats, _ = cv2.connectedComponentsWithStats(red)
        body = None
        if nlab > 1:
            i = max(range(1, nlab), key=lambda j: stats[j, 4])
            if stats[i, 4] > 500:
                x, y, bw, bh, _ = stats[i]
                body = (x / k, y / k, bw / k, bh / k)

        # px/m 추정: 몸통 폭을 실제 지름으로 나눈다. 없으면 화각으로도 계산 못 하니 None.
        scale = (body[2] / (self.body_cm / 100.0)) if body else None
        dist_cm = None
        if scale:
            dist_cm = w / (2 * np.tan(np.radians(HFOV_DEG / 2)) * scale) * 100

        gray = cv2.GaussianBlur(cv2.cvtColor(small, cv2.COLOR_BGR2GRAY), (7, 7), 1.5)
        rmin, rmax = 8, 60
        if scale:      # 기대 게이지 크기의 절반~두 배만 본다
            exp = self.gauge_cm / 100.0 * scale * k / 2
            rmin, rmax = max(int(exp * 0.5), 5), int(exp * 2.2)
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=25,
                                   param1=120, param2=40,
                                   minRadius=rmin, maxRadius=max(rmax, rmin + 5))
        found = []
        if circles is not None:
            for cx, cy, r in circles[0][:6]:
                found.append((cx / k, cy / k, r / k))
        return body, scale, dist_cm, found

    def draw(self, img, body, scale, dist_cm, circles):
        h, w = img.shape[:2]
        out = img.copy()
        for i in (1, 2):        # 삼분할 격자
            cv2.line(out, (0, h * i // 3), (w, h * i // 3), (90, 90, 90), 1)
            cv2.line(out, (w * i // 3, 0), (w * i // 3, h), (90, 90, 90), 1)
        cv2.drawMarker(out, (w // 2, h // 2), (120, 120, 120), cv2.MARKER_CROSS, 40, 1)

        if body:
            x, y, bw, bh = [int(v) for v in body]
            cv2.rectangle(out, (x, y), (x + bw, y + bh), (255, 160, 0), 2)
            cv2.putText(out, f"body {bw}px", (x, max(y - 8, 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 160, 0), 2)

        for cx, cy, r in circles:
            cx, cy, r = int(cx), int(cy), int(r)
            cm = (2 * r / scale * 100) if scale else None
            near = cm is not None and abs(cm - self.gauge_cm) < self.gauge_cm * 0.5
            col = (0, 230, 0) if near else (0, 140, 200)
            cv2.circle(out, (cx, cy), r, col, 2)
            txt = f"d={2*r}px" + (f" ~{cm:.1f}cm" if cm else "")
            cv2.putText(out, txt, (cx - r, cy - r - 8), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, col, 2)
            if near:      # 반사(포화) 검사 — 바늘이 묻히는 원인
                m = np.zeros(out.shape[:2], np.uint8)
                cv2.circle(m, (cx, cy), max(r - 2, 1), 255, -1)
                g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                sat = float((g[m > 0] > 245).mean() * 100)
                if sat > 3:
                    cv2.putText(out, f"reflection {sat:.0f}%", (cx - r, cy + r + 24),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 230, 255), 2)

        info = f"{w}x{h}  {self.fps:.1f}fps"
        if dist_cm:
            info += f"  dist~{dist_cm:.0f}cm"
        info += f"  frames {self.n}"
        cv2.putText(out, info, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
        return out

    def best_circle(self, scale, circles):
        """기대 게이지 크기에 가장 가까운 원 후보. (지름px, 지름cm, 중심) 또는 None."""
        if not scale or not circles:
            return None
        cand = [(abs(2 * r / scale * 100 - self.gauge_cm), 2 * r, 2 * r / scale * 100, (cx, cy))
                for cx, cy, r in circles]
        _, px, cm, c = min(cand)
        return px, cm, c

    def reflection_pct(self, img, center, px):
        """게이지 후보 안에서 흰색으로 포화된 화소 비율(%). 반사 정도를 나타낸다."""
        m = np.zeros(img.shape[:2], np.uint8)
        cv2.circle(m, (int(center[0]), int(center[1])), max(int(px / 2) - 2, 1), 255, -1)
        g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return float((g[m > 0] > 245).mean() * 100)

    def save(self, img, scale, dist_cm, circles):
        """원본을 저장한다. 파일명에 거리·게이지 크기를 넣어 나중에 자세를 구분할 수 있게 한다.

        여러 거리·각도로 찍어 비교할 때 파일명이 shot_0001.png 뿐이면 어느 자세였는지
        알 수 없다. 측정값을 이름과 CSV 에 함께 남긴다.
        """
        b = self.best_circle(scale, circles)
        ts = int(time.time())
        d = f"d{dist_cm:.0f}" if dist_cm else "dNA"
        g = f"g{b[0]:.0f}px" if b else "gNA"
        path = os.path.join(self.save_dir, f"shot_{d}_{g}_{ts}.png")
        cv2.imwrite(path, img)
        refl = self.reflection_pct(img, b[2], b[0]) if b else float("nan")
        csv = os.path.join(self.save_dir, "shots.csv")
        if not os.path.exists(csv):
            open(csv, "w").write("파일,거리cm,게이지px,게이지cm,반사%,해상도\n")
        with open(csv, "a") as f:
            f.write(f"{os.path.basename(path)},{dist_cm or ''},"
                    f"{b[0] if b else ''},{f'{b[1]:.2f}' if b else ''},"
                    f"{refl:.1f},{img.shape[1]}x{img.shape[0]}\n")
        self.get_logger().warn(
            f"저장 {os.path.basename(path)}"
            + (f"  게이지 {b[0]:.0f}px(~{b[1]:.1f}cm) 반사 {refl:.0f}%" if b else "  (게이지 후보 없음)")
        )

    def log(self, scale, dist_cm, circles):
        now = time.time()
        if now - self.last_log < 2.0:
            return
        self.last_log = now
        best = ""
        if scale and circles:
            cms = [(abs(2 * r / scale * 100 - self.gauge_cm), 2 * r, 2 * r / scale * 100)
                   for _, _, r in circles]
            d, px, cm = min(cms)
            best = f", 게이지 후보 {px:.0f}px (~{cm:.1f}cm)"
        self.get_logger().info(
            f"{self.fps:.1f}fps" + (f", 거리 ~{dist_cm:.0f}cm" if dist_cm else "") + best
        )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/camera/image_raw/compressed")
    ap.add_argument("--gauge-cm", type=float, default=2.0, help="게이지 실제 지름(cm)")
    ap.add_argument("--body-cm", type=float, default=11.0, help="소화기 몸통 지름(cm)")
    ap.add_argument("--save", default=os.path.expanduser("~/vibe/ex1/logs/liveview"))
    ap.add_argument("--width", type=int, default=1100, help="창 최대 가로 픽셀")
    ap.add_argument("--qos", choices=["sensor", "reliable"], default="sensor",
                    help="카메라를 reliability:=reliable 로 띄웠으면 reliable 로 줄 것")
    args = ap.parse_args()

    rclpy.init()
    node = LiveView(args.topic, args.gauge_cm, args.body_cm,
                    os.path.expanduser(args.save), args.width, args.qos)
    win = "robot camera  (s=save, q=quit)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.frame is None:
                # 프레임이 안 와도 창과 키 입력은 살아 있어야 한다. 예전에는 여기서
                # continue 해서, 전송이 막힌 상황에 창이 뜨지도 않고 s 키도 안 먹었다.
                wait = np.zeros((220, 760, 3), np.uint8)
                cv2.putText(wait, "waiting for frames...", (20, 90),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.1, (60, 200, 255), 2)
                cv2.putText(wait, f"topic: {args.topic}", (20, 140),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
                cv2.putText(wait, "q=quit", (20, 180),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
                cv2.imshow(win, wait)
                if (cv2.waitKey(30) & 0xFF) in (ord("q"), 27):
                    break
                continue
            img = node.frame
            body, scale, dist_cm, circles = node.analyze(img)
            node.log(scale, dist_cm, circles)
            view = node.draw(img, body, scale, dist_cm, circles)
            if view.shape[1] > args.width:
                s = args.width / view.shape[1]
                view = cv2.resize(view, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
            cv2.imshow(win, view)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
            if key in (ord("s"), ord("S")):        # Shift 눌린 대문자도 받는다
                node.save(img, scale, dist_cm, circles)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
