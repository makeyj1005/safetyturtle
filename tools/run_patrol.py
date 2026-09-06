#!/usr/bin/env python3
"""run_patrol.py — 등록된 웨이포인트를 순서대로 돌며 순찰한다. 시연용.

patrol_node 가 하는 일의 단순한 형태다. 시연 영상에서 "지금 몇 번 지점으로
가는 중" 을 화면에 남기려고 진행 상황을 또박또박 찍는다.

소화기 지점에서는 잠깐 멈춰 점검할 시간을 준다(extinguisher_inspect_node 가
QR 을 읽고 압력계를 판정하는 데 몇 초가 걸린다).

  python3 tools/run_patrol.py                 # 한 바퀴
  python3 tools/run_patrol.py --laps 2        # 두 바퀴
  python3 tools/run_patrol.py --inspect-at 3번 --inspect-sec 12
"""
import argparse
import math
import os
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

EX1 = os.path.join(os.path.expanduser("~"), "vibe", "ex1")
DEFAULT_WP = os.path.join(EX1, "maps", "patrol_waypoints.yaml")


class Patrol(Node):
    def __init__(self):
        super().__init__("run_patrol")
        self.cli = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def goto(self, wp, timeout):
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = "map"
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x = float(wp["x"])
        g.pose.pose.position.y = float(wp["y"])
        yaw = float(wp.get("yaw", 0.0))
        g.pose.pose.orientation.z = math.sin(yaw / 2.0)
        g.pose.pose.orientation.w = math.cos(yaw / 2.0)

        fut = self.cli.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        h = fut.result()
        if h is None or not h.accepted:
            print(f"    ❌ 거부됨 (벽에 너무 가깝거나 경로 없음)")
            return False
        res = h.get_result_async()
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.3)
            if res.done():
                break
        if not res.done():
            print(f"    ⏱ {timeout}초 초과 — 취소")
            h.cancel_goal_async()
            rclpy.spin_once(self, timeout_sec=2.0)
            return False
        if res.result().status == GoalStatus.STATUS_SUCCEEDED:
            print(f"    ✅ 도착 ({time.time()-t0:.1f}초)")
            return True
        print(f"    ❌ 실패 (status={res.result().status})")
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--file", default=DEFAULT_WP)
    ap.add_argument("--laps", type=int, default=1)
    ap.add_argument("--timeout", type=float, default=70.0)
    ap.add_argument("--inspect-at", default="",
                    help="이 이름의 지점에서 잠시 멈춘다(소화기 점검용)")
    ap.add_argument("--inspect-sec", type=float, default=10.0)
    a = ap.parse_args()

    wps = yaml.safe_load(open(a.file, encoding="utf-8"))["waypoints"]
    if not wps:
        raise SystemExit(f"웨이포인트가 없다: {a.file}")

    rclpy.init()
    n = Patrol()
    if not n.cli.wait_for_server(timeout_sec=15.0):
        print("❌ Nav2 액션 서버가 없다 — nav2 컨테이너 확인")
        raise SystemExit(1)

    print(f"순찰 시작 — 지점 {len(wps)}개 x {a.laps}바퀴\n")
    ok_cnt = fail_cnt = 0
    t_all = time.time()
    try:
        for lap in range(1, a.laps + 1):
            if a.laps > 1:
                print(f"[{lap}바퀴째]")
            for wp in wps:
                print(f"  → {wp['name']} ({wp['x']:+.2f}, {wp['y']:+.2f})")
                if n.goto(wp, a.timeout):
                    ok_cnt += 1
                else:
                    fail_cnt += 1
                if a.inspect_at and wp["name"] == a.inspect_at:
                    print(f"    ⏸ 소화기 점검 대기 {a.inspect_sec:.0f}초")
                    t0 = time.time()
                    while time.time() - t0 < a.inspect_sec:
                        rclpy.spin_once(n, timeout_sec=0.3)
    except KeyboardInterrupt:
        print("\n중단됨")
    finally:
        print(f"\n순찰 종료 — 성공 {ok_cnt} / 실패 {fail_cnt}, "
              f"총 {time.time()-t_all:.0f}초")
        n.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
