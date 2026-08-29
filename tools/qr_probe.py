#!/usr/bin/env python3
"""qr_probe.py — 카메라 영상에서 QR 판독을 실시간으로 시험한다. 판독 한계 실측용.

[VM에서 실행]  (로봇에서 CSI 카메라가 떠 있어야 한다)
  python3 ~/vibe/ex1/tools/qr_probe.py
  python3 ~/vibe/ex1/tools/qr_probe.py --expect FE-01 --save ~/vibe/ex1/logs/qrprobe

[무엇을 위한 것인가]
소화기 지점 좌표를 어디에 찍어야 하는지는 "QR 을 몇 cm 거리, 몇 도 기울기까지
읽을 수 있는가"에 달려 있다. 그걸 추측하지 않고 재기 위한 도구다.
인쇄한 QR 을 들고 거리와 각도를 바꿔가며 아래 출력을 보면 된다.

[출력 읽는 법]
  판독 OK "FE-01"  한 변 128px      → 이 조건은 쓸 수 있다
  탐지만 됨 (한 변 46px)            → QR 은 보이는데 해상도가 부족하다.
                                       더 가까이 가거나 QR 을 크게 만들어야 한다
  아무것도 없음                      → 화각을 벗어났거나 너무 흐리다

[왜 탐지와 판독을 다른 도구로 하나]
이 시스템의 OpenCV 는 QUIRC 가 링크되지 않아 QR '판독'을 못 한다. 대신 '탐지'는
되므로, 위치와 크기는 cv2 로 얻고 판독은 zbar(zbarimg)로 한다. 크기(px)를 함께
보여주는 이유는, 판독 실패가 '안 보임' 때문인지 '너무 작음' 때문인지 구분하려는 것이다.
"""
import argparse
import os
import subprocess
import sys
import tempfile
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class QrProbe(Node):
    def __init__(self, topic, hz, expect, save_dir):
        super().__init__("qr_probe")
        self.expect = expect
        self.save_dir = save_dir
        if save_dir:
            os.makedirs(save_dir, exist_ok=True)
        self.det = cv2.QRCodeDetector()
        self.min_gap = 1.0 / hz
        self.last = 0.0
        self.n_try = self.n_ok = self.n_seen = 0
        self.tmp = os.path.join(tempfile.gettempdir(), f"_qrprobe_{os.getpid()}.png")
        self.create_subscription(CompressedImage, topic, self.on_img,
                                 qos_profile_sensor_data)
        self.get_logger().info(
            f"{topic} 구독 시작 — 초당 {hz:.1f}회 판독 시도. Ctrl+C 로 종료"
            + (f" / 기대값 '{expect}'" if expect else "")
        )

    def zbar(self, img):
        cv2.imwrite(self.tmp, img)
        try:
            r = subprocess.run(["zbarimg", "--quiet", "--raw", self.tmp],
                               capture_output=True, text=True, timeout=10)
            return r.stdout.strip()
        except Exception:
            return ""

    def on_img(self, msg: CompressedImage):
        now = time.time()
        if now - self.last < self.min_gap:
            return
        self.last = now

        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        self.n_try += 1

        # 위치·크기: cv2 로 탐지 (판독은 못 하지만 네 꼭짓점은 준다)
        side = None
        try:
            ok, pts = self.det.detect(img)
            if ok and pts is not None:
                p = pts.reshape(-1, 2)
                sides = [np.linalg.norm(p[i] - p[(i + 1) % 4]) for i in range(4)]
                side = float(np.mean(sides))
        except cv2.error:
            pass

        text = self.zbar(img)
        if text:
            self.n_ok += 1
            mark = ""
            if self.expect:
                mark = "  일치" if text == self.expect else f"  기대값 불일치({self.expect})"
            self.get_logger().info(
                f'판독 OK "{text}"'
                + (f"  한 변 {side:.0f}px" if side else "  (크기 미측정)")
                + mark
                + f"   [{self.n_ok}/{self.n_try} = {self.n_ok/self.n_try*100:.0f}%]"
            )
            if self.save_dir:
                cv2.imwrite(os.path.join(self.save_dir, f"ok_{int(now)}.png"), img)
        elif side:
            self.n_seen += 1
            self.get_logger().warn(
                f"탐지만 됨 (한 변 {side:.0f}px) — 해상도 부족이거나 흐리다"
                f"   [{self.n_ok}/{self.n_try}]"
            )
            if self.save_dir:
                cv2.imwrite(os.path.join(self.save_dir, f"seen_{int(now)}.png"), img)
        else:
            self.get_logger().info(f"아무것도 없음   [{self.n_ok}/{self.n_try}]",
                                   throttle_duration_sec=2.0)

    def summary(self):
        if not self.n_try:
            print("\n프레임을 한 장도 못 받았다. 로봇에서 카메라가 떠 있는지 확인할 것"
                  " (ros2 launch ~/launch/robot_camera.launch.py)")
            return
        print(f"\n=== 시도 {self.n_try} / 판독 성공 {self.n_ok}"
              f" ({self.n_ok/self.n_try*100:.0f}%) / 탐지만 {self.n_seen} ===")
        if self.save_dir:
            print(f"프레임 저장: {self.save_dir}")
        if os.path.exists(self.tmp):
            os.remove(self.tmp)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/camera/image_raw/compressed")
    ap.add_argument("--hz", type=float, default=3.0, help="초당 판독 시도 횟수")
    ap.add_argument("--expect", default="", help="기대하는 QR 문자열")
    ap.add_argument("--save", default="", help="프레임 저장 폴더(선택)")
    args = ap.parse_args()

    rclpy.init()
    node = QrProbe(args.topic, args.hz, args.expect, os.path.expanduser(args.save))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
