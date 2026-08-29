#!/usr/bin/env python3
"""Nav2 흉내만 내는 가짜 액션 서버. 모든 목표를 즉시 성공 처리한다.

로봇·Nav2 없이 patrol_node 의 진행 흐름(시작지점 이동 -> 코너 4개 -> 바퀴 세기
-> laps_per_run 에서 정지)만 확인하기 위한 것이다. 주행 품질과는 무관하다.

  터미널 1: python3 ~/vibe/ex1/tools/fake_nav2.py
  터미널 2: ros2 launch ~/vibe/ex1/launch/patrol_auto.launch.py \
              fire_on_start:=true laps:=2 rest_min:=0.5 cycle_min:=1.5 dwell_sec:=0.3

주의: 진짜 Nav2 가 떠 있을 때 함께 켜지 말 것. 액션 서버가 둘이 되어
      "Ignoring unexpected goal response" 가 쏟아지고 결과가 섞인다.

[travel_sec — 주행 중에 끼어드는 동작을 볼 때 (2026-08-01 추가)]
기본값 0 은 목표를 즉시 성공시킨다. 바퀴 세기·재시작 로직에는 그게 편하지만,
/patrol/hold(안전모 미착용 시 정지)처럼 **가는 도중에** 끼어드는 동작은 확인할 수
없다 — 끼어들 틈이 없기 때문이다. travel_sec 을 주면 그 시간만큼 가는 척하고
그동안 취소 요청을 받아들인다. 진짜 Nav2 가 취소를 받으면 로봇을 세우는 것과 같다.

  python3 ~/vibe/ex1/tools/fake_nav2.py --ros-args -p travel_sec:=6.0
"""
import time

import rclpy
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class FakeNav2(Node):
    def __init__(self):
        super().__init__("fake_nav2")
        # 0 이면 즉시 성공(기존 동작). 양수면 그만큼 가는 척하며 취소를 받는다.
        self.declare_parameter("travel_sec", 0.0)
        self.n = 0
        # 가는 도중에 오는 취소를 처리하려면 실행 콜백과 취소 콜백이 서로를 막지
        # 않아야 한다. 단일 스레드 실행기로는 execute 가 끝나야 취소가 처리된다.
        cb = ReentrantCallbackGroup()
        ActionServer(self, NavigateToPose, "navigate_to_pose", self.on_pose,
                     cancel_callback=self.on_cancel, callback_group=cb)
        ActionServer(self, NavigateThroughPoses, "navigate_through_poses",
                     self.on_route, cancel_callback=self.on_cancel, callback_group=cb)
        self.get_logger().info("가짜 Nav2 준비됨")

    def on_cancel(self, gh):
        self.get_logger().info("취소 요청 받음")
        return CancelResponse.ACCEPT

    def travel(self, gh):
        """travel_sec 동안 가는 척한다. 취소되면 False 를 돌려준다."""
        sec = float(self.get_parameter("travel_sec").value)
        t0 = time.time()
        while time.time() - t0 < sec:
            if gh.is_cancel_requested:
                gh.canceled()
                self.get_logger().info(f"  취소됨 ({time.time() - t0:.1f}초 지점)")
                return False
            time.sleep(0.05)
        gh.succeed()
        return True

    def on_pose(self, gh):
        self.n += 1
        p = gh.request.pose.pose.position
        self.get_logger().info(f"[{self.n}] to_pose x={p.x:.3f} y={p.y:.3f}")
        self.travel(gh)
        return NavigateToPose.Result()

    def on_route(self, gh):
        self.n += 1
        self.get_logger().info(f"[{self.n}] through_poses {len(gh.request.poses)}점")
        self.travel(gh)
        return NavigateThroughPoses.Result()


def main():
    rclpy.init()
    node = FakeNav2()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
