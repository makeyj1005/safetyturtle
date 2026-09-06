#!/usr/bin/env python3
"""scan_to_map.py — 제자리에서 라이다 스캔만으로 지도를 만든다.

[왜 이 방법인가 — 2026-09-06]
카토그래퍼로 방을 돌며 만든 지도가 실제보다 컸다(실측 2.1x2.2m 인데 지도는
3.75x3.05m). 작은 방에서 네 벽이 비슷해 스캔 매칭이 미끄러지고, 그 오차가
지도를 부풀린 것이다. 루프클로저를 켜도 완전히는 안 잡혔다.

방이 작아 **한 자리에서 라이다가 방 전체를 본다**(가장 먼 벽 1.95m,
라이다 사거리 3.5m). 로봇을 안 움직이면 자세 추정이 끼어들 여지가 없으므로
드리프트가 원천적으로 없다. 그래서 제자리 스캔을 그대로 지도로 굳힌다.

[한계] 로봇에서 안 보이는 곳(기둥 뒤, 다른 방)은 지도에 안 들어간다.
한 방에서 도는 시연에는 충분하지만, 여러 방을 도는 순찰에는 못 쓴다.

[좌표계] 지도는 **지금 로봇 자리를 원점(0,0), 지금 바라보는 쪽을 +x** 로 잡는다.
그래서 지도를 만든 뒤 AMCL 초기 위치를 (0, 0, 0) 으로 주면 정확히 맞는다.
  python3 tools/set_initial_pose.py --x 0 --y 0 --yaw 0

[쓰는 법]
  python3 tools/scan_to_map.py --out maps/room --scans 40
"""
import argparse
import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

FREE, UNKNOWN, WALL = 254, 205, 0


class ScanMapper(Node):
    def __init__(self, n_scans, res, margin):
        super().__init__("scan_to_map")
        self.n_scans, self.res, self.margin = n_scans, res, margin
        self.seen = 0
        self.hits = []          # 벽으로 본 점 (x, y)
        self.rays = []          # (끝점 x, y) — 그 선까지는 빈 공간
        self.create_subscription(LaserScan, "/scan", self.on_scan,
                                 qos_profile_sensor_data)
        self.get_logger().info(f"{n_scans}장 모으는 중... 로봇을 움직이지 말 것")

    def on_scan(self, msg):
        for i, r in enumerate(msg.ranges):
            if r is None or math.isinf(r) or math.isnan(r):
                continue
            a = msg.angle_min + i * msg.angle_increment
            if r < max(msg.range_min, 0.05) or r > msg.range_max:
                # 아무것도 못 맞춘 방향 — 사거리 끝까지는 비어 있다고 본다
                self.rays.append((msg.range_max * math.cos(a),
                                  msg.range_max * math.sin(a), False))
                continue
            x, y = r * math.cos(a), r * math.sin(a)
            self.hits.append((x, y))
            self.rays.append((x, y, True))
        self.seen += 1
        if self.seen % 10 == 0:
            self.get_logger().info(f"  {self.seen}/{self.n_scans}장")
        if self.seen >= self.n_scans:
            raise SystemExit(0)

    def build(self):
        if not self.hits:
            raise SystemExit("스캔이 하나도 안 들어왔다 — 라이다를 확인할 것")
        hx = [p[0] for p in self.hits]
        hy = [p[1] for p in self.hits]
        lo_x, hi_x = min(hx) - self.margin, max(hx) + self.margin
        lo_y, hi_y = min(hy) - self.margin, max(hy) + self.margin
        w = int(math.ceil((hi_x - lo_x) / self.res))
        h = int(math.ceil((hi_y - lo_y) / self.res))

        def to_px(x, y):
            # 이미지 y 축은 아래로 커지므로 뒤집는다
            return int((x - lo_x) / self.res), int(h - 1 - (y - lo_y) / self.res)

        grid = np.full((h, w), UNKNOWN, np.uint8)
        r0 = to_px(0.0, 0.0)          # 로봇 자리

        # 광선을 따라 빈 공간을 칠한다. 끝점만 찍으면 벽만 남고 안이 미탐색이 된다.
        for x, y, is_hit in self.rays:
            p1 = to_px(x, y)
            cv2.line(grid, r0, p1, FREE, 1)
        # 벽은 빈 공간 위에 덮어쓴다(순서가 중요하다 — 반대로 하면 벽이 지워진다)
        for x, y in self.hits:
            px, py = to_px(x, y)
            if 0 <= px < w and 0 <= py < h:
                grid[py, px] = WALL

        # 한두 칸씩 뚫린 벽을 메운다. 안 메우면 Nav2 가 그 틈으로 경로를 낸다.
        wall = (grid == WALL).astype(np.uint8)
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        wall = cv2.morphologyEx(wall, cv2.MORPH_CLOSE, k)
        grid[wall == 1] = WALL

        origin = (lo_x, lo_y)
        return grid, origin, r0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="maps/room", help="확장자 없이")
    ap.add_argument("--scans", type=int, default=40)
    ap.add_argument("--res", type=float, default=0.05)
    ap.add_argument("--margin", type=float, default=0.25,
                    help="벽 바깥으로 남길 여백(m)")
    a = ap.parse_args()

    rclpy.init()
    node = ScanMapper(a.scans, a.res, a.margin)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass

    grid, origin, robot_px = node.build()
    node.destroy_node()
    if rclpy.ok():
        rclpy.try_shutdown()

    pgm = a.out + ".pgm"
    cv2.imwrite(pgm, grid)
    h, w = grid.shape
    with open(a.out + ".yaml", "w", encoding="utf-8") as f:
        f.write(f"image: {pgm.split('/')[-1]}\n")
        f.write("mode: trinary\n")
        f.write(f"resolution: {a.res}\n")
        f.write(f"origin: [{origin[0]:.4f}, {origin[1]:.4f}, 0]\n")
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.25\n")

    free = float((grid == FREE).mean() * 100)
    wall = float((grid == WALL).mean() * 100)
    print(f"\n지도 {w}x{h}px = {w*a.res:.2f}m x {h*a.res:.2f}m")
    print(f"  빈공간 {free:.1f}%   벽 {wall:.1f}%   "
          f"넓이 {(grid == FREE).sum()*a.res*a.res:.2f}m^2")
    print(f"  로봇 자리: 지도 픽셀 {robot_px} = 좌표 (0.00, 0.00)")
    print(f"저장: {pgm}, {a.out}.yaml")
    print("\n다음: AMCL 초기 위치를 원점으로 준다")
    print("  python3 tools/set_initial_pose.py --x 0 --y 0 --yaw 0")


if __name__ == "__main__":
    main()
