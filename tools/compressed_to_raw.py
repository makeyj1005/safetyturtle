#!/usr/bin/env python3
"""
compressed_to_raw.py — 압축 영상을 RViz가 볼 수 있는 raw 영상으로 되돌리는 중계 노드.

[왜 필요한가]
로봇(Pi 3)은 Wi-Fi 대역폭을 아끼려고 /camera/image_raw/compressed (JPEG)만 실질적으로
쓸 수 있고, 끊김을 막으려고 QoS를 BEST_EFFORT로 발행한다.
그런데 문제가 두 개 겹친다:
  1. RViz2 의 Image 디스플레이는 sensor_msgs/Image 만 받는다 (compressed 직접 구독 불가)
  2. ros2 run image_transport republish 는 RELIABLE 로만 구독해서
     BEST_EFFORT 발행자와 QoS가 안 맞아 한 장도 못 받는다
     (로그: "offering incompatible QoS ... RELIABILITY_QOS_POLICY")

그래서 이 노드가 그 사이를 메운다.
  입력: /camera/image_raw/compressed   (BEST_EFFORT 로 구독  <- 핵심)
  출력: /vm/image_raw                  (RELIABLE 로 발행, VM 안에서만 도니 대역폭 걱정 없음)

[실행 방법]
  python3 ~/vibe/ex1/tools/compressed_to_raw.py

  토픽 이름을 바꾸려면:
  python3 compressed_to_raw.py --in /다른/compressed 토픽 --out /다른/출력토픽

빌드 불필요(순수 파이썬 스크립트). ROS 2 환경만 source 되어 있으면 된다.
"""
import argparse
import sys

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


class CompressedToRaw(Node):
    def __init__(self, in_topic: str, out_topic: str):
        super().__init__("compressed_to_raw")
        self.bridge = CvBridge()
        self.in_topic = in_topic
        self.out_topic = out_topic

        # 구독은 BEST_EFFORT. qos_profile_sensor_data 가 곧 (best_effort + depth 5) 이며
        # 센서 스트림의 표준 프로파일이다. 이게 이 노드의 존재 이유.
        self.sub = self.create_subscription(
            CompressedImage, in_topic, self.on_compressed, qos_profile_sensor_data
        )

        # 발행은 RELIABLE. VM 내부(loopback)라 재전송 비용이 사실상 없고,
        # RViz 기본 설정(RELIABLE)으로 바로 붙을 수 있어 편하다.
        self.pub = self.create_publisher(
            Image, out_topic, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.n_in = 0
        self.n_err = 0
        # 5초마다 처리량을 알려준다. 조용히 죽어있는지 바로 알 수 있게.
        self.create_timer(5.0, self.report)
        self.get_logger().info(f"중계 시작: {in_topic} (best_effort) -> {out_topic} (reliable)")

    def on_compressed(self, msg: CompressedImage):
        try:
            # JPEG 디코딩. desired_encoding="bgr8" 은 OpenCV 기본 채널 순서.
            cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            out = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
            # 타임스탬프와 frame_id 는 원본 것을 그대로 유지해야
            # RViz 의 TF 연동과 시간 정렬이 깨지지 않는다.
            out.header = msg.header
            self.pub.publish(out)
            self.n_in += 1
        except Exception as e:  # 한 프레임 깨져도 노드가 죽지 않게 한다
            self.n_err += 1
            if self.n_err <= 3:
                self.get_logger().warn(f"디코딩 실패: {e}")

    def report(self):
        if self.n_in == 0:
            self.get_logger().warn(
                f"5초간 수신 0장. '{self.in_topic}' 발행 중인지, "
                f"ROS_DOMAIN_ID 가 로봇과 같은지 확인."
            )
        else:
            self.get_logger().info(f"중계 {self.n_in}장 (최근 5초) / 실패 {self.n_err}장")
        self.n_in = 0


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--in", dest="in_topic", default="/camera/image_raw/compressed")
    p.add_argument("--out", dest="out_topic", default="/vm/image_raw")
    # ros2 run 으로 넘어오는 --ros-args 등을 무시하기 위해 known 만 파싱
    args, _ = p.parse_known_args()

    rclpy.init()
    node = CompressedToRaw(args.in_topic, args.out_topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
