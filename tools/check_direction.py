#!/usr/bin/env python3
"""check_direction.py — 명령 부호 / 오도메트리 / 실제 움직임이 서로 맞는지 잰다.

[무엇을 가리려는 것인가]
"w 를 누르면 실물은 앞으로 가는데 RViz 에서는 뒤로 간다" 는 증상은 세 가지가
서로 어긋날 때 생긴다.
  ① 명령 부호(cmd_vel.linear.x)
  ② 오도메트리(휠 인코더가 보고하는 위치)
  ③ 실제 움직임(라이다가 본 벽의 거리 변화 — 이것이 진실이다)

②와 ③이 어긋나면 Nav2/AMCL 이 망가진다. 스캔 매칭은 ③을 보고
모션 모델은 ②를 보는데, 둘이 반대면 위치 추정이 계속 싸운다.
지도가 실제보다 부풀거나 두 겹으로 그려지는 원인이 된다.

[안전] 아주 짧게(기본 1.5초, 0.05m/s = 약 7cm) 움직인다.
로봇 앞뒤로 30cm 이상 공간을 두고 실행할 것.

[쓰는 법]
  python3 tools/check_direction.py            # +x 명령으로 시험
  python3 tools/check_direction.py --sign -1  # -x 명령으로 시험
"""
import argparse
import math
import statistics
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def sector_median(scan, center_deg, half_deg=12.0):
    """center_deg 방향 주변의 거리 중앙값(m). 없으면 None."""
    vals = []
    for i, r in enumerate(scan.ranges):
        if r is None or math.isinf(r) or math.isnan(r):
            continue
        if r < max(scan.range_min, 0.05) or r > scan.range_max:
            continue
        a = math.degrees(scan.angle_min + i * scan.angle_increment)
        d = (a - center_deg + 180) % 360 - 180
        if abs(d) <= half_deg:
            vals.append(r)
    return statistics.median(vals) if vals else None


class Checker(Node):
    def __init__(self, sign, secs, speed):
        super().__init__("check_direction")
        self.sign, self.secs, self.speed = sign, secs, speed
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(Twist, "/cmd_vel_teleop", qos)
        self.odom = None
        self.scan = None
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan,
                                 qos_profile_sensor_data)

    def on_odom(self, m):
        self.odom = m

    def on_scan(self, m):
        self.scan = m

    def wait_data(self, timeout=10.0):
        t0 = time.time()
        while (self.odom is None or self.scan is None) and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.odom is not None and self.scan is not None

    def snapshot(self):
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
        p = self.odom.pose.pose.position
        return {
            "x": p.x, "y": p.y,
            "front": sector_median(self.scan, 0.0),      # 라이다 0도 = 차체 앞
            "back": sector_median(self.scan, 180.0),
        }

    def drive(self):
        t = Twist()
        t.linear.x = self.sign * self.speed
        t0 = time.time()
        while time.time() - t0 < self.secs:
            self.pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.05)
        stop = Twist()
        for _ in range(10):
            self.pub.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sign", type=int, default=1, choices=(1, -1))
    ap.add_argument("--secs", type=float, default=1.5)
    ap.add_argument("--speed", type=float, default=0.05)
    a = ap.parse_args()

    rclpy.init()
    n = Checker(a.sign, a.secs, a.speed)
    if not n.wait_data():
        print("오도메트리나 라이다가 안 온다 — bringup 확인")
        raise SystemExit(1)

    before = n.snapshot()
    cmd = a.sign * a.speed
    print(f"명령: linear.x = {cmd:+.3f} m/s 를 {a.secs}초")
    print(f"  전: odom x={before['x']:+.3f}  앞벽={before['front']}  뒤벽={before['back']}")
    n.drive()
    time.sleep(0.5)
    after = n.snapshot()
    print(f"  후: odom x={after['x']:+.3f}  앞벽={after['front']}  뒤벽={after['back']}")

    dx = after["x"] - before["x"]
    dy = after["y"] - before["y"]
    moved = math.hypot(dx, dy)
    print(f"\n오도메트리 이동량: {moved*100:.1f}cm  (dx={dx:+.3f}, dy={dy:+.3f})")

    # 라이다로 본 진짜 움직임: 앞벽이 가까워졌으면 물리적으로 전진한 것이다
    verdict_scan = None
    if before["front"] and after["front"]:
        d_front = after["front"] - before["front"]
        print(f"앞벽 거리 변화: {d_front*100:+.1f}cm "
              f"({'가까워짐=물리적 전진' if d_front < 0 else '멀어짐=물리적 후진'})")
        if abs(d_front) > 0.02:
            verdict_scan = "앞" if d_front < 0 else "뒤"
    if before["back"] and after["back"]:
        d_back = after["back"] - before["back"]
        print(f"뒤벽 거리 변화: {d_back*100:+.1f}cm")

    print("\n--- 판정 ---")
    if moved < 0.01:
        print("  거의 안 움직였다. 라이다 안전정지에 막혔거나 속도가 너무 작다.")
        print("  로봇 앞뒤 공간을 확보하고 --secs 를 늘려 다시 해볼 것.")
    elif verdict_scan is None:
        print("  벽이 안 보여 실제 방향을 못 쟀다. 벽을 마주보게 두고 다시 할 것.")
    else:
        odom_dir = "앞" if (dx * math.cos(0) + dy * 0) > 0 else "뒤"
        print(f"  오도메트리가 말하는 방향(부호): x {'증가' if dx > 0 else '감소'}")
        print(f"  라이다가 본 실제 방향        : 차체 {verdict_scan}쪽")
        print(f"  보낸 명령                    : linear.x {'양수' if cmd > 0 else '음수'}")
        print()
        if (cmd > 0) == (verdict_scan == "앞"):
            print("  ✅ 명령 부호와 실제 방향이 일치한다 (표준: +x = 전진)")
        else:
            print("  ⚠️ 명령 부호와 실제 방향이 **반대**다.")
            print("     Nav2 는 +x 를 전진으로 알고 명령하므로, 이대로면")
            print("     자율주행 때 로봇이 목표와 반대로 간다.")

    n.destroy_node()
    if rclpy.ok():
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
