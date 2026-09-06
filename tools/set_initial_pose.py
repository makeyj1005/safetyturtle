#!/usr/bin/env python3
"""set_initial_pose.py — AMCL 에 초기 위치를 알려준다.

[왜 ros2 topic pub 으로는 안 되나 — 2026-09-06 실측]
`ros2 topic pub` 은 header.stamp 를 채워주지 않아 0 으로 나간다. AMCL 은
그 시각의 TF 를 찾아 자세를 옮겨야 하는데, 0 은 "아주 옛날" 이라 변환을
구하지 못하고 조용히 버린다. 로그에는 계속
  "AMCL cannot publish a pose or update the transform. Please set the initial pose..."
만 찍혀서, 메시지가 안 가는 것처럼 보인다(실제로는 토픽에 잘 실려 있다 —
다른 컨테이너에서 echo 하면 보인다).

그래서 시각을 제대로 찍어 보내는 작은 노드를 따로 둔다.

[쓰는 법]
  python3 tools/set_initial_pose.py --x -0.81 --y -0.95 --yaw 0.51
  python3 tools/set_initial_pose.py --from-odom      # 지금 오도메트리 값을 그대로
"""
import argparse
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class Setter(Node):
    def __init__(self, x, y, yaw, from_odom):
        super().__init__("set_initial_pose")
        # AMCL 은 /initialpose 를 BEST_EFFORT 로 구독한다. RELIABLE 로 내도
        # 규격상 붙지만, 맞춰 두는 편이 확실하다.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(PoseWithCovarianceStamped,
                                         "/initialpose", qos)
        self.x, self.y, self.yaw = x, y, yaw
        self.got_odom = not from_odom
        self.sent = 0
        if from_odom:
            self.create_subscription(Odometry, "/odom", self.on_odom, 10)
            self.get_logger().info("오도메트리를 기다린다...")
        self.create_timer(0.5, self.tick)

    def on_odom(self, msg):
        if self.got_odom:
            return
        p = msg.pose.pose
        self.x, self.y = p.position.x, p.position.y
        # 쿼터니언 -> yaw (2D 라 z, w 만 있으면 된다)
        self.yaw = 2.0 * math.atan2(p.orientation.z, p.orientation.w)
        self.got_odom = True
        self.get_logger().info(
            f"오도메트리 기준: x={self.x:.3f} y={self.y:.3f} "
            f"yaw={math.degrees(self.yaw):.1f}도")

    def tick(self):
        if not self.got_odom:
            return
        m = PoseWithCovarianceStamped()
        # 이 한 줄이 핵심이다 — 시각이 0 이면 AMCL 이 버린다
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "map"
        m.pose.pose.position.x = float(self.x)
        m.pose.pose.position.y = float(self.y)
        m.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        m.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        # 처음 추정이 얼마나 불확실한지. 너무 작게 주면 AMCL 이 자기 확신에
        # 갇혀 잘못된 위치를 못 벗어난다. 30cm / 15도 정도로 둔다.
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.09        # x, y 분산 (0.3m)^2
        cov[35] = 0.068               # yaw 분산 (15도)^2
        m.pose.covariance = cov
        self.pub.publish(m)
        self.sent += 1
        if self.sent >= 10:
            self.get_logger().info(f"초기 위치 {self.sent}회 발행 완료")
            raise SystemExit(0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--x", type=float, default=0.0)
    ap.add_argument("--y", type=float, default=0.0)
    ap.add_argument("--yaw", type=float, default=0.0, help="라디안")
    ap.add_argument("--from-odom", action="store_true",
                    help="현재 오도메트리 값을 그대로 초기 위치로 쓴다")
    a = ap.parse_args()

    rclpy.init()
    node = Setter(a.x, a.y, a.yaw, a.from_odom)
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
