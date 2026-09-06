#!/usr/bin/env python3
"""scan_extent.py — 라이다가 지금 보고 있는 공간의 실제 크기를 잰다.

[왜 필요한가]
저장된 지도와 실제 벽이 안 맞을 때, "지도가 틀린 건지 위치추정이 틀린 건지"
를 가려야 한다. 위치추정(AMCL)이 틀린 것이면 방 크기는 같고 위치·각도만
어긋난다. 지도 자체가 부풀었다면 크기가 다르다. 이 도구는 크기를 잰다.

한 장만 보면 사람·의자에 가려 작게 나올 수 있으므로 여러 장을 모아
방향별 최대 거리를 취한다(가려진 순간이 있어도 다른 순간에 벽이 보인다).
"""
import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class Extent(Node):
    def __init__(self, n_scans):
        super().__init__("scan_extent")
        self.n_scans = n_scans
        self.seen = 0
        self.far = {}          # 각도(도 단위 정수) -> 그 방향의 최대 거리
        self.create_subscription(LaserScan, "/scan", self.on_scan,
                                 qos_profile_sensor_data)

    def on_scan(self, msg):
        for i, r in enumerate(msg.ranges):
            if r is None or math.isinf(r) or math.isnan(r):
                continue
            if r < max(msg.range_min, 0.05) or r > msg.range_max:
                continue
            deg = int(round(math.degrees(msg.angle_min + i * msg.angle_increment)))
            deg %= 360
            if r > self.far.get(deg, 0.0):
                self.far[deg] = r
        self.seen += 1
        if self.seen >= self.n_scans:
            self.report()
            raise SystemExit(0)

    def report(self):
        if not self.far:
            print("측정값이 없다 — 라이다가 도는지 확인할 것")
            return
        xs, ys = [], []
        for deg, r in self.far.items():
            a = math.radians(deg)
            xs.append(r * math.cos(a))
            ys.append(r * math.sin(a))
        w = max(xs) - min(xs)
        h = max(ys) - min(ys)
        print(f"스캔 {self.seen}장, 측정 방향 {len(self.far)}개")
        print(f"  로봇 기준 앞뒤 폭 : {w:.2f} m  (x {min(xs):+.2f} ~ {max(xs):+.2f})")
        print(f"  로봇 기준 좌우 폭 : {h:.2f} m  (y {min(ys):+.2f} ~ {max(ys):+.2f})")
        print(f"  가장 먼 벽까지    : {max(self.far.values()):.2f} m")
        print(f"  가장 가까운 벽까지: {min(self.far.values()):.2f} m")
        print("\n  ※ 로봇이 방 한가운데 있지 않으면 위 폭이 방 크기와 같다.")
        print("     로봇이 구석에 있으면 방이 이보다 클 수 있다.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scans", type=int, default=15)
    a = ap.parse_args()
    rclpy.init()
    node = Extent(a.scans)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
