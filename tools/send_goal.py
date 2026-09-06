#!/usr/bin/env python3
"""send_goal.py — Nav2 에 목표 한 곳을 보내고 결과를 지켜본다.

RViz 의 Nav2 Goal 과 같은 일을 하지만, 진행 상황과 실패 사유를 글로 남긴다.
"경로를 못 만든다" 인지 "가다가 멈췄다" 인지 구분하려면 이게 필요하다.

  python3 tools/send_goal.py --x -0.28 --y 0.01
  python3 tools/send_goal.py --x -0.28 --y 0.01 --yaw 1.57 --timeout 60
"""
import argparse
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class Sender(Node):
    def __init__(self):
        super().__init__("send_goal")
        self.cli = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.last_dist = None

    def go(self, x, y, yaw, timeout):
        if not self.cli.wait_for_server(timeout_sec=10.0):
            print("❌ Nav2 액션 서버가 없다 (bt_navigator 확인)")
            return False

        g = NavigateToPose.Goal()
        g.pose.header.frame_id = "map"
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x = float(x)
        g.pose.pose.position.y = float(y)
        g.pose.pose.orientation.z = math.sin(yaw / 2.0)
        g.pose.pose.orientation.w = math.cos(yaw / 2.0)

        print(f"목표 전송: ({x:+.2f}, {y:+.2f}) yaw={math.degrees(yaw):.0f}도")

        def on_fb(fb):
            d = fb.feedback.distance_remaining
            # 같은 값이 계속 찍히면 지저분하므로 바뀔 때만 남긴다
            if self.last_dist is None or abs(d - self.last_dist) > 0.03:
                self.last_dist = d
                print(f"  남은 거리 {d:.2f}m")

        fut = self.cli.send_goal_async(g, feedback_callback=on_fb)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            print("❌ 목표가 거부됐다 — 그 지점이 벽/미탐색이거나 경로가 없다")
            return False
        print("  목표 접수됨, 이동 시작")

        res_fut = handle.get_result_async()
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.3)
            if res_fut.done():
                break
        if not res_fut.done():
            print(f"⏱ {timeout}초 안에 못 끝냈다 — 취소한다")
            handle.cancel_goal_async()
            rclpy.spin_once(self, timeout_sec=2.0)
            return False

        st = res_fut.result().status
        if st == GoalStatus.STATUS_SUCCEEDED:
            print(f"✅ 도착 ({time.time()-t0:.1f}초)")
            return True
        names = {GoalStatus.STATUS_ABORTED: "중단(경로 실패/장애물)",
                 GoalStatus.STATUS_CANCELED: "취소됨"}
        print(f"❌ 실패: {names.get(st, st)}")
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--x", type=float, required=True)
    ap.add_argument("--y", type=float, required=True)
    ap.add_argument("--yaw", type=float, default=0.0)
    ap.add_argument("--timeout", type=float, default=60.0)
    a = ap.parse_args()

    rclpy.init()
    n = Sender()
    try:
        n.go(a.x, a.y, a.yaw, a.timeout)
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
