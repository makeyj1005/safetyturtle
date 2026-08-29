#!/usr/bin/env python3
"""
capture_frame.py — 로봇 카메라에서 한 프레임을 받아 파일로 저장하고 밝기 통계를 낸다.

검정 테이프 라인 추종의 임계값을 추측이 아니라 실측으로 정하기 위한 도구.

[실행 방법]
  python3 ~/vibe/ex1/tools/capture_frame.py
  python3 ~/vibe/ex1/tools/capture_frame.py --out /tmp/floor.png --frames 10

저장물:
  <out>            원본 프레임 (640x480)
  <out>.gray.png   그레이스케일 (밝기만 본 것)
"""
import argparse
import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class Grabber(Node):
    def __init__(self, topic, n_frames):
        super().__init__("capture_frame")
        self.bridge = CvBridge()
        self.n_frames = n_frames
        self.frames = []
        self.create_subscription(CompressedImage, topic, self.cb, qos_profile_sensor_data)

    def cb(self, msg):
        if len(self.frames) < self.n_frames:
            try:
                self.frames.append(
                    self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
                )
            except Exception:
                pass


def describe(gray, label):
    """밝기 분포를 사람이 읽을 수 있게 요약."""
    p = np.percentile(gray, [1, 5, 25, 50, 75, 95, 99]).astype(int)
    print(f"  {label:22s} 평균 {gray.mean():5.1f}  최소 {gray.min():3d}  최대 {gray.max():3d}")
    print(f"  {'':22s} 백분위 1%={p[0]} 5%={p[1]} 25%={p[2]} 50%={p[3]} 75%={p[4]} 95%={p[5]} 99%={p[6]}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/camera/image_raw/compressed")
    ap.add_argument("--out", default="/tmp/frame.png")
    ap.add_argument("--frames", type=int, default=10)
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = Grabber(args.topic, args.frames)
    print(f"{args.topic} 에서 {args.frames} 프레임 수집 중...")
    import time
    deadline = time.monotonic() + 15.0
    while rclpy.ok() and len(node.frames) < args.frames and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    if not node.frames:
        print("[실패] 프레임을 못 받았다. 카메라 노드와 ROS_DOMAIN_ID 확인.")
        return 1

    # 마지막 프레임 사용 (자동노출이 안정된 뒤의 것)
    bgr = node.frames[-1]
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape

    cv2.imwrite(args.out, bgr)
    cv2.imwrite(args.out + ".gray.png", gray)
    print(f"저장: {args.out}  ({w}x{h}, {len(node.frames)}프레임 중 마지막)\n")

    print("=== 밝기 분포 (0=검정, 255=흰색) ===")
    describe(gray, "전체 화면")

    # 화면을 위/중간/하단으로 나눠 본다. 라인 추종은 하단만 쓴다.
    for name, sl in [
        ("상단 1/3", slice(0, h // 3)),
        ("중단 1/3", slice(h // 3, 2 * h // 3)),
        ("하단 1/3", slice(2 * h // 3, h)),
        ("최하단 20%", slice(int(h * 0.8), h)),
    ]:
        describe(gray[sl, :], name)

    # Otsu: 밝기 분포를 두 덩어리로 가장 잘 쪼개는 임계값을 자동으로 찾아준다.
    roi = gray[int(h * 0.6):, :]          # 하단 40%를 ROI 후보로
    otsu_t, _ = cv2.threshold(roi, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    print(f"\n=== Otsu 자동 임계값 (하단 40% 기준): {otsu_t:.0f} ===")
    print("  (이 값보다 어두운 픽셀 = 테이프 후보)")

    # 여러 임계값에서 '어두운 픽셀 비율'을 본다.
    # 라인은 화면에서 좁은 띠여야 하므로 이 비율이 5~25% 정도면 건강하다.
    print("\n=== 임계값별 어두운 픽셀 비율 (하단 40%) ===")
    for t in (40, 60, 80, 100, 120, 140, int(otsu_t)):
        ratio = float((roi < t).mean()) * 100
        flag = "  <- 적정" if 3 <= ratio <= 25 else ("  <- 너무 많음" if ratio > 25 else "  <- 너무 적음")
        print(f"  임계 {t:3d}: {ratio:5.1f}%{flag}")

    # ROI 중앙 가로줄 하나를 뜯어서 어두운 구간의 폭(픽셀)을 재본다 = 테이프 두께.
    row = roi[roi.shape[0] // 2, :]
    dark = row < otsu_t
    runs, cur = [], 0
    for d in dark:
        if d:
            cur += 1
        elif cur:
            runs.append(cur)
            cur = 0
    if cur:
        runs.append(cur)
    print(f"\n=== ROI 중앙 가로줄의 어두운 구간 폭 (픽셀) ===")
    print(f"  구간 개수 {len(runs)}, 폭: {sorted(runs, reverse=True)[:8] if runs else '없음'}")
    print("  (구간이 1개이고 폭이 적당하면 라인만 잡힌 것. 여러 개면 그림자 등이 섞인 것)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
