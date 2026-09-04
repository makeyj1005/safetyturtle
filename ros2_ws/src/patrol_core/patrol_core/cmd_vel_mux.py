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
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
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

        # --- 라이다 충돌 방지 (2026-09-04 추가) ---
        # 진행 방향에 이 거리(m) 안쪽으로 뭔가 있으면 그 방향 직진을 막는다.
        # 회전은 막지 않는다 — 막으면 벽에 붙었을 때 빠져나올 방법이 없어진다.
        # 여기(mux)에 넣는 이유: /cmd_vel 을 내는 유일한 지점이라 웹 조작·Nav2·
        # 정렬 회전까지 한 곳에서 전부 보호된다.
        self.declare_parameter("safety_enabled", True)
        self.declare_parameter("stop_distance", 0.25)
        # 진행 방향 기준 좌우 이 각도(도)만큼을 "앞"으로 본다. 너무 넓게 잡으면
        # 옆으로 지나가는 벽에도 걸려 못 움직인다.
        self.declare_parameter("sector_deg", 50.0)
        # [⚠️ 방향 매핑 — 2026-09-04 실측]
        # 이 로봇은 +x 명령이 물리적으로 **차체 뒤쪽**으로 간다(대시보드가 ▲ 에
        # -x 를 보내는 이유). 라이다 0° 는 차체 앞을 본다. 그래서
        #   x < 0 (물리적 전진) -> 라이다 0°   구역을 본다
        #   x > 0 (물리적 후진) -> 라이다 180° 구역을 본다
        # 만약 실측에서 반대로 걸리면 이 값을 false 로 주면 매핑이 뒤집힌다.
        self.declare_parameter("negative_x_is_chassis_front", True)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(Twist, "/cmd_vel", qos)
        self.pub_active = self.create_publisher(String, "/mux/active", qos)
        self.pub_safety = self.create_publisher(String, "/mux/safety", qos)

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
        # 라이다는 센서 QoS(best_effort)로 온다 — RELIABLE 로 구독하면 안 받힌다.
        self.create_subscription(LaserScan, "/scan", self.on_scan,
                                 qos_profile_sensor_data)

        self._scan = None
        self._scan_at = 0.0
        self._prev_block = None

        self.create_timer(1.0 / float(self.get_parameter("rate").value), self.tick)

        srcs = ", ".join(f"{t}({p})" for t, p in SOURCES)
        self.get_logger().info(f"중재 노드 시작. 입력 우선순위: {srcs}")
        self.get_logger().info("/cmd_vel 은 이 노드만 발행한다")

    def on_cmd(self, topic: str, msg: Twist):
        self._msg[topic] = msg
        self._last[topic] = time.monotonic()

    def on_scan(self, msg: LaserScan):
        self._scan = msg
        self._scan_at = time.monotonic()

    def sector_min(self, center_deg):
        """center_deg 를 중심으로 sector_deg 범위 안의 최단 거리(m). 없으면 None.

        라이다는 측정 못 한 방향에 0.0 이나 inf 를 넣는다 — 그걸 "가깝다"로
        읽으면 아무것도 없는데 멈춘다. range_min 아래와 무한값은 버린다.
        """
        scan = self._scan
        if scan is None or not scan.ranges:
            return None
        half = math.radians(float(self.get_parameter("sector_deg").value)) / 2.0
        center = math.radians(center_deg)
        best = None
        for i, r in enumerate(scan.ranges):
            if r is None or math.isinf(r) or math.isnan(r):
                continue
            if r < max(scan.range_min, 0.05) or r > scan.range_max:
                continue
            ang = scan.angle_min + i * scan.angle_increment
            # 각도 차이를 -pi~pi 로 정규화해서 비교한다(0°/360° 경계 문제를 피한다)
            d = math.atan2(math.sin(ang - center), math.cos(ang - center))
            if abs(d) <= half and (best is None or r < best):
                best = r
        return best

    def safety_block(self, linear_x):
        """이 직진 명령을 막아야 하면 (True, 사유)를 준다."""
        if not bool(self.get_parameter("safety_enabled").value):
            return False, ""
        if abs(linear_x) < 1e-3:
            return False, ""          # 안 움직이는 명령은 막을 것도 없다

        # 라이다가 끊겼으면 막지 않는다. 안전 기능이 센서 고장으로 로봇을
        # 아예 못 움직이게 만들면, 사람이 손으로 빼내야 해서 오히려 위험하다.
        # 대신 상태로 알린다.
        if self._scan is None or (time.monotonic() - self._scan_at) > 1.5:
            return False, "라이다 신호 없음(안전기능 비활성)"

        neg_is_front = bool(self.get_parameter(
            "negative_x_is_chassis_front").value)
        going_chassis_front = (linear_x < 0) if neg_is_front else (linear_x > 0)
        center = 0.0 if going_chassis_front else 180.0

        dist = self.sector_min(center)
        if dist is None:
            return False, ""
        stop = float(self.get_parameter("stop_distance").value)
        if dist < stop:
            where = "앞" if going_chassis_front else "뒤"
            return True, f"{where} {dist:.2f}m (기준 {stop:.2f}m)"
        return False, ""

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

        # 라이다 충돌 방지 — 직진만 막고 회전은 남긴다(벽에서 빠져나올 수 있어야 한다)
        blocked, why = self.safety_block(t.linear.x)
        if blocked:
            t.linear.x = 0.0
        if why != self._prev_block:
            self._prev_block = why
            if blocked:
                self.get_logger().warn(f"안전 정지 — 장애물 {why} / 회전은 가능하다")
            elif why:
                self.get_logger().warn(why)
        s = String()
        s.data = (f"blocked {why}" if blocked
                  else (why if why else "clear"))
        self.pub_safety.publish(s)

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
