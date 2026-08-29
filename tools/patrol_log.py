#!/usr/bin/env python3
"""patrol_log.py — 순찰 회차를 자동으로 기록한다. 시연 중에 사람이 적을 필요가 없게.

[VM에서 실행]  순찰 시작 전에 켜두고, 끝나면 Ctrl+C. 읽기만 하므로 주행에 영향이 없다.
  python3 ~/vibe/ex1/tools/patrol_log.py
  python3 ~/vibe/ex1/tools/patrol_log.py --out ~/vibe/ex1/logs/demo1

[기록하는 것]
  회차별 소요 시간 / 바퀴별 시간 / 주행 거리(odom 적분) / 전압 최저·강하폭 /
  실패·거절 발생 횟수 / 회차 사이 쉰 시간

[왜 필요한가]
순찰 노드 로그는 터미널을 닫으면 사라지고, 여러 회차를 눈으로 세면 놓친다.
시연 뒤에 "몇 분 걸렸나, 전압이 얼마나 떨어졌나"를 답할 근거를 남긴다.
전압은 부하 시 처지고 정지하면 회복하므로, 회차 중 최저값과 회차 전후 기준선을
따로 본다. 기준선이 내려가면 배터리, 회복하면 어댑터다.

[출력]
  화면      회차가 끝날 때마다 한 줄 요약
  CSV       --out 경로.csv  (회차별 한 줄)
  텍스트    --out 경로.txt  (상태 변화 전체 시각 기록)
"""
import argparse
import math
import os
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

DEFAULT_OUT = os.path.join(
    os.path.expanduser("~"), "vibe", "ex1", "logs", time.strftime("patrol_%m%d_%H%M")
)


class PatrolLogger(Node):
    def __init__(self, out):
        super().__init__("patrol_log")
        self.out = out
        os.makedirs(os.path.dirname(out), exist_ok=True)
        self.csv = open(out + ".csv", "a")
        self.txt = open(out + ".txt", "a")
        if self.csv.tell() == 0:
            self.csv.write("회차,시작,종료,소요초,바퀴수,거리m,전압시작,전압최저,전압종료,실패,거절\n")
        self.csv.flush()

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, "/patrol/status", self.on_status, qos)
        self.create_subscription(String, "/patrol/schedule", self.on_sched, qos)
        # /battery_state 와 /odom 은 로봇이 best_effort 로 낼 수 있어 완화해서 받는다.
        lossy = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(BatteryState, "/battery_state", self.on_batt, lossy)
        self.create_subscription(Odometry, "/odom", self.on_odom, lossy)

        self.volt = None
        self.xy = None
        self.runs = []
        self.cur = None            # 진행 중인 회차
        self.last_end = None       # 직전 회차 종료 시각 (쉰 시간 계산용)
        self.get_logger().info(f"기록 시작 — {out}.csv / .txt (Ctrl+C 로 종료 요약)")

    # ---------------- 입력 ----------------
    def on_batt(self, m):
        self.volt = m.voltage
        if self.cur and m.voltage > 0:
            self.cur["vmin"] = min(self.cur["vmin"], m.voltage)

    def on_odom(self, m):
        p = m.pose.pose.position
        if self.xy is not None and self.cur:
            d = math.hypot(p.x - self.xy[0], p.y - self.xy[1])
            if d < 0.5:          # 순간 도약(위치추정 점프)은 거리에 넣지 않는다
                self.cur["dist"] += d
        self.xy = (p.x, p.y)

    def on_sched(self, m):
        self.note(f"[예정] {m.data}")

    def on_status(self, m):
        s = m.data
        self.note(f"[상태] {s}")

        if s.startswith("run started"):
            self.start_run()
        elif s.startswith("lap ") and self.cur:
            now = time.time()
            prev = self.cur["laps"][-1] if self.cur["laps"] else self.cur["t0"]
            self.cur["laps"].append(now)
            self.get_logger().info(
                f"  바퀴 {len(self.cur['laps'])} 완주 — {now - prev:.0f}초"
            )
        elif s.startswith("failed") and self.cur:
            self.cur["fail"] += 1
        elif s.startswith("goal rejected") and self.cur:
            self.cur["reject"] += 1
        elif s.startswith(("done", "stopped")):
            self.end_run(s)

    # ---------------- 회차 ----------------
    def start_run(self):
        rest = f", 직전 회차 후 {time.time() - self.last_end:.0f}초 쉬었다" if self.last_end else ""
        self.cur = {
            "t0": time.time(), "laps": [], "dist": 0.0,
            "v0": self.volt or 0.0, "vmin": self.volt or 99.0,
            "fail": 0, "reject": 0,
        }
        self.get_logger().info(f"회차 {len(self.runs) + 1} 시작 (전압 {self.cur['v0']:.2f}V{rest})")

    def end_run(self, why):
        if not self.cur:
            return
        c = self.cur
        t1 = time.time()
        took = t1 - c["t0"]
        n = len(c["laps"])
        v1 = self.volt or 0.0
        self.runs.append({**c, "t1": t1, "took": took, "n": n, "v1": v1, "why": why})
        self.cur = None
        self.last_end = t1

        self.get_logger().warn(
            f"회차 {len(self.runs)} 종료 — {n}바퀴 / {took / 60:.1f}분 / {c['dist']:.1f}m / "
            f"전압 {c['v0']:.2f}->{v1:.2f}V (최저 {c['vmin']:.2f}) / "
            f"실패 {c['fail']} 거절 {c['reject']}"
        )
        self.csv.write(
            f"{len(self.runs)},{time.strftime('%H:%M:%S', time.localtime(c['t0']))},"
            f"{time.strftime('%H:%M:%S', time.localtime(t1))},{took:.0f},{n},"
            f"{c['dist']:.2f},{c['v0']:.2f},{c['vmin']:.2f},{v1:.2f},"
            f"{c['fail']},{c['reject']}\n"
        )
        self.csv.flush()

    def note(self, text):
        self.txt.write(f"{time.strftime('%H:%M:%S')} {text}\n")
        self.txt.flush()

    # ---------------- 종료 요약 ----------------
    def summary(self):
        if self.cur:
            self.end_run("종료(진행 중이던 회차)")
        if not self.runs:
            print("\n기록된 회차가 없다. /patrol/status 가 들어왔는지 확인할 것.")
            return
        print(f"\n=== 요약: {len(self.runs)}회차 ===")
        print(f"{'회차':<5}{'바퀴':>5}{'소요':>8}{'거리':>8}{'전압최저':>10}{'실패':>6}{'거절':>6}")
        for i, r in enumerate(self.runs, 1):
            print(f"{i:<5}{r['n']:>5}{r['took'] / 60:>7.1f}분{r['dist']:>7.1f}m"
                  f"{r['vmin']:>9.2f}V{r['fail']:>6}{r['reject']:>6}")
        laps = [r["took"] / r["n"] for r in self.runs if r["n"]]
        if laps:
            print(f"\n한 바퀴 평균 {sum(laps) / len(laps) / 60:.1f}분 "
                  f"(최소 {min(laps) / 60:.1f} / 최대 {max(laps) / 60:.1f})")
        vmin = min(r["vmin"] for r in self.runs)
        print(f"전체 최저 전압 {vmin:.2f}V — 11.0V 에서 turtlebot3_node 가 죽는다")
        print(f"파일: {self.out}.csv / {self.out}.txt")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=DEFAULT_OUT, help="기록 파일 경로(확장자 없이)")
    args = ap.parse_args()

    rclpy.init()
    node = PatrolLogger(os.path.expanduser(args.out))
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
