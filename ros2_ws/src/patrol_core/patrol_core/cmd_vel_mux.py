#!/usr/bin/env python3
"""
cmd_vel_mux.py — 여러 주행 명령 중 하나를 골라 /cmd_vel 로 최종 발행하는 중재 노드.

[프로젝트 절대 규칙] 어떤 노드도 /cmd_vel 에 직접 발행하지 않는다.
                    각자 /cmd_vel_line, /cmd_vel_nav 등으로 내고
                    이 노드 하나만 /cmd_vel 로 발행한다.

[입력] 우선순위가 높은 순서
  /cmd_vel_teleop  0  사람이 직접 조작 — 항상 최우선(비상 개입 수단)
  /cmd_vel_nav     1  Nav2 자율주행

주의: 기본 turtlebot3_teleop 은 /cmd_vel 에 직접 발행하므로 이 노드와 충돌한다.
      반드시 출력을 바꿔서 실행한다:
        ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r /cmd_vel:=/cmd_vel_teleop

[출력] /cmd_vel        (Twist)  최종 주행 명령
       /mux/active     (String) 지금 어느 입력이 채택됐는지 (진단용)

[동작 방식]
  - 각 입력의 마지막 수신 시각을 기록하고, timeout 안에 들어온 것만 "살아있다"고 본다
  - 살아있는 것 중 우선순위가 가장 높은 하나를 고른다
  - 아무것도 살아있지 않으면 0 을 발행한다 (데드맨 스위치)
  - 통과시키는 게 아니라 고정 주기로 발행한다. 입력이 끊기면 자동으로 0 이 나가야 하기 때문이다.
  - /mux/enable 로 False 를 받으면 무조건 0 을 발행한다 (소프트 비상정지)
"""
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

# (토픽, 우선순위) — 숫자가 작을수록 우선.
# 새 주행 소스를 추가할 때는 여기에 한 줄 넣으면 된다.
SOURCES = [
    ("/cmd_vel_teleop", 0),
    ("/cmd_vel_nav", 1),
]


class CmdVelMux(Node):
    def __init__(self):
        super().__init__("cmd_vel_mux")

        # 입력이 이 시간(초) 안에 안 오면 그 입력은 죽은 것으로 본다.
        self.declare_parameter("timeout", 0.4)
        # 최종 발행 주기(Hz). 로봇 제어 주기와 맞춘다.
        self.declare_parameter("rate", 20.0)
        # 안전 상한. 어떤 입력이 들어와도 이 값을 넘겨 발행하지 않는다.
        # 절대 규칙은 아니지만, 잘못된 노드 하나가 로봇을 폭주시키는 것을 막는다.
        self.declare_parameter("max_linear", 0.15)
        self.declare_parameter("max_angular", 1.2)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(Twist, "/cmd_vel", qos)
        self.pub_active = self.create_publisher(String, "/mux/active", qos)

        self._last = {t: 0.0 for t, _ in SOURCES}
        self._msg = {t: Twist() for t, _ in SOURCES}
        self._enabled = True
        self._prev_active = None

        for topic, _ in SOURCES:
            # 기본 인자로 topic 을 묶어야 클로저가 마지막 값만 잡는 문제를 피한다
            self.create_subscription(
                Twist, topic, lambda m, t=topic: self.on_cmd(t, m), qos
            )
        self.create_subscription(Bool, "/mux/enable", self.on_enable, qos)

        self.create_timer(1.0 / float(self.get_parameter("rate").value), self.tick)

        srcs = ", ".join(f"{t}({p})" for t, p in SOURCES)
        self.get_logger().info(f"중재 노드 시작. 입력 우선순위: {srcs}")
        self.get_logger().info("/cmd_vel 은 이 노드만 발행한다")

    def on_cmd(self, topic: str, msg: Twist):
        self._msg[topic] = msg
        self._last[topic] = time.monotonic()

    def on_enable(self, msg: Bool):
        self._enabled = bool(msg.data)
        self.get_logger().warn(f"/mux/enable = {self._enabled}" + ("" if self._enabled else " (정지)"))

    def tick(self):
        now = time.monotonic()
        timeout = float(self.get_parameter("timeout").value)

        chosen, out = None, Twist()
        if self._enabled:
            for topic, _ in SOURCES:      # SOURCES 가 이미 우선순위 순서
                if self._last[topic] > 0.0 and now - self._last[topic] <= timeout:
                    chosen, out = topic, self._msg[topic]
                    break

        # 안전 클램프
        ml = float(self.get_parameter("max_linear").value)
        ma = float(self.get_parameter("max_angular").value)
        t = Twist()
        t.linear.x = max(-ml, min(out.linear.x, ml))
        t.linear.y = 0.0
        t.linear.z = 0.0
        t.angular.x = 0.0
        t.angular.y = 0.0
        t.angular.z = max(-ma, min(out.angular.z, ma))
        self.pub.publish(t)

        label = chosen if chosen else ("STOP (no input)" if self._enabled else "STOP (disabled)")
        if label != self._prev_active:
            self.get_logger().info(f"채택: {label}")
            self._prev_active = label
        s = String()
        s.data = label
        self.pub_active.publish(s)


def main():
    rclpy.init()
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub.publish(Twist())   # 종료 시 정지
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
