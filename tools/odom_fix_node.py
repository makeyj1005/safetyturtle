#!/usr/bin/env python3
"""odom_fix_node.py — 로봇 오도메트리가 실측(반전)된 것을 180도 회전시켜 보정한다.

[배경 — 2026-09-01 실측]
로봇을 물리적으로 전진시켰는데 /odom 의 x 가 오히려 줄었다(후진한 것처럼 기록됨).
디피렌셜드라이브 컨트롤러(펌웨어 쪽)의 오도메트리 계산 자체가 뒤집혀 있는 것으로
보인다 — 카토그래퍼는 라이다 스캔매칭이 이걸 실시간으로 보정해 줘서 지도는 깔끔하게
나왔지만, AMCL 은 오도메트리(모션 모델)에 더 크게 의존해서 파티클이 마구 흩어졌다.

[고치는 방법]
원본 /odom 을 그대로 180도 회전(x,y 부호 반전 + yaw 에 pi 더하기)하면 실제 이동
방향과 일치하게 된다. 로봇 자체의 odom->base_footprint TF 발행은 꺼두고
(ros2 param set /diff_drive_controller odometry.publish_tf false) 이 노드가 대신
보정된 TF 를 낸다. Nav2 설정(patrol_nav2.yaml)의 odom_topic 도 이 노드가 내는
/odom_fixed 로 바꿔야 한다.

[VM에서 실행]
  python3 ~/vibe/ex1/tools/odom_fix_node.py
"""
import math

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_ros import TransformBroadcaster


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_from_yaw(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class OdomFixNode(Node):
    def __init__(self):
        super().__init__("odom_fix_node")
        self.declare_parameter("child_frame_id", "base_footprint")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(Odometry, "/odom_fixed", qos)
        self.br = TransformBroadcaster(self)
        self.create_subscription(Odometry, "/odom", self.on_odom, qos)
        self.get_logger().info("odom_fix_node 시작 — /odom 을 180도 회전해 /odom_fixed 로 낸다")

    def on_odom(self, msg: Odometry):
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        fixed_yaw = yaw + math.pi
        fx = -msg.pose.pose.position.x
        fy = -msg.pose.pose.position.y
        fvx = -msg.twist.twist.linear.x
        fvy = -msg.twist.twist.linear.y

        out = Odometry()
        out.header = msg.header
        out.child_frame_id = str(self.get_parameter("child_frame_id").value)
        out.pose.pose.position.x = fx
        out.pose.pose.position.y = fy
        out.pose.pose.position.z = msg.pose.pose.position.z
        out.pose.pose.orientation = quat_from_yaw(fixed_yaw)
        out.pose.covariance = msg.pose.covariance
        out.twist.twist.linear.x = fvx
        out.twist.twist.linear.y = fvy
        out.twist.twist.angular.z = msg.twist.twist.angular.z
        out.twist.covariance = msg.twist.covariance
        self.pub.publish(out)

        t = TransformStamped()
        t.header = msg.header
        t.header.frame_id = "odom"
        t.child_frame_id = out.child_frame_id
        t.transform.translation.x = fx
        t.transform.translation.y = fy
        t.transform.translation.z = 0.0
        t.transform.rotation = out.pose.pose.orientation
        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = OdomFixNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
