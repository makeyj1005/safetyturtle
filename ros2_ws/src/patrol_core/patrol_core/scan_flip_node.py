#!/usr/bin/env python3
"""scan_flip_node.py — /scan 을 180도 돌려 /scan_flipped 로 다시 낸다.

[왜 필요한가 — 2026-09-06 실측]
이 로봇은 명령·오도메트리와 실제 움직임이 앞뒤로 뒤집혀 있다.
  linear.x = +0.06 m/s 를 2초 -> 오도메트리 x 는 +11.1cm 증가(전진했다고 보고)
  같은 동안 라이다 앞벽은 +9.9cm 멀어지고 뒷벽은 -12.3cm 가까워짐(실제로는 후진)
즉 모터와 인코더가 함께 뒤집혀 있고, 라이다만 진실을 말한다.

카토그래퍼와 AMCL 은 스캔 매칭(실제 움직임)과 모션 모델(오도메트리)을 합치는데,
둘이 반대를 가리키니 위치 추정이 계속 싸운다. 지도가 실제(2.1x2.2m)보다
부풀고(3.75x3.05m) 벽이 두 겹으로 그려진 원인이 이것이다.

라이다를 180도 돌려 놓으면 셋이 다시 한 방향으로 맞는다. 물리적으로 센서를
돌릴 필요는 없다 — 각도 배열을 절반만큼 회전시키면 같은 효과다.

[대가] 이렇게 하면 ROS 가 생각하는 "앞" 이 로봇의 물리적 뒤쪽(CSI 카메라 쪽)이 된다.
자율주행 때 로봇은 CSI 쪽을 앞세우고 간다. 시연에는 지장이 없다.

[근본 해결이 아니다] 제대로 고치려면 OpenCR 쪽 모터 방향을 바로잡아야 한다.
발표를 앞두고 펌웨어를 건드리는 위험을 피하려고 택한 우회로다.

[쓰는 법]
  ros2 run patrol_core scan_flip_node
  ros2 run patrol_core scan_flip_node --ros-args -p out_topic:=/scan_flipped
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanFlip(Node):
    def __init__(self):
        super().__init__("scan_flip_node")
        self.declare_parameter("in_topic", "/scan_raw")
        self.declare_parameter("out_topic", "/scan")
        # 180도가 기본. 혹시 다른 각도로 어긋나 있으면 이 값만 바꾸면 된다.
        self.declare_parameter("shift_deg", 180.0)

        self.in_topic = str(self.get_parameter("in_topic").value)
        self.out_topic = str(self.get_parameter("out_topic").value)

        self.pub = self.create_publisher(LaserScan, self.out_topic,
                                         qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.in_topic, self.on_scan,
                                 qos_profile_sensor_data)
        self.n = 0
        self.create_timer(10.0, self.beat)
        self.get_logger().info(
            f"스캔 회전 {self.get_parameter('shift_deg').value}도: "
            f"{self.in_topic} -> {self.out_topic}")

    def beat(self):
        self.get_logger().info(f"  {self.n}장 처리")

    def on_scan(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0:
            return
        # 각도 하나가 몇 도인지로 나눠 몇 칸 밀지 구한다.
        # 라이다마다 분해능이 다르므로 고정값(180칸)을 쓰면 안 된다.
        import math
        step_deg = math.degrees(msg.angle_increment)
        if step_deg <= 0:
            return
        shift = int(round(float(self.get_parameter("shift_deg").value) / step_deg))
        shift %= n

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        # 배열을 shift 만큼 돌린다. 0번 칸이 원래 shift 번 방향을 보게 된다.
        out.ranges = list(msg.ranges[shift:]) + list(msg.ranges[:shift])
        if msg.intensities and len(msg.intensities) == n:
            out.intensities = (list(msg.intensities[shift:])
                               + list(msg.intensities[:shift]))
        self.pub.publish(out)
        self.n += 1


def main():
    rclpy.init()
    node = ScanFlip()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
