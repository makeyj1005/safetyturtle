#!/usr/bin/env python3
"""grab_webcam_once.py — /webcam/image_raw/compressed 에서 한 장만 받아 파일로 저장.

로봇 webcam_node 가 이미 떠 있어야 한다(3fps, 640x480 로 스트리밍 중이므로 DDS로
바로 받아도 무리 없다 — grab_shot.py 가 다루는 CSI 고해상도 점검용 프레임과 다르다).
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "/tmp/webcam_shot.jpg"
    rclpy.init()
    node = Node("grab_webcam_once")
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST)
    got = {}

    def on_img(msg):
        got["data"] = bytes(msg.data)

    node.create_subscription(CompressedImage, "/webcam/image_raw/compressed", on_img, qos)
    deadline = node.get_clock().now().nanoseconds + 15_000_000_000
    while rclpy.ok() and "data" not in got and node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)

    if "data" not in got:
        print("15초 안에 프레임을 못 받았다 — 웹캠 노드가 떠 있는지 확인할 것", file=sys.stderr)
        sys.exit(1)

    with open(out, "wb") as f:
        f.write(got["data"])
    print(f"저장됨: {out} ({len(got['data'])} bytes)")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
