#!/usr/bin/env python3
"""extinguisher_expiry_node.py — 소화기 유효기한(내용연한)이 다가오면 알린다.

[VM에서 실행]
  ros2 run patrol_core extinguisher_expiry_node

  바로 확인해보고 싶을 때(주기를 짧게):
  ros2 run patrol_core extinguisher_expiry_node --ros-args -p check_interval_sec:=5.0

[입력]  maps/extinguisher_info.yaml   소화기별 제조일·유효기한 (이름은
        fire_extinguisher_points.yaml / gauge_calib.yaml 과 같은 이름을 쓴다 —
        압력계 판정과 같은 소화기를 가리키게 하기 위함)
[출력]  /extinguisher/status  (String)  진단·웹 대시보드용 — 전체 요약
        logs/events.sqlite 에 상태가 바뀔 때만 기록한다(정상 유지 중엔 반복 기록 안 함 —
        restricted_node/fire_node 와 같은 "경고만 남긴다" 정책, 2026-08-29 결정)

[왜 카메라·QR 이 아니라 등록된 날짜로만 판단하나]
QR 은 지금 소화기 식별자 문자열만 담고(make_qr.py 참고) 날짜 정보는 없다. 라벨의
제조일·유효기한을 광학판독으로 읽는 건 범위 밖이라, 사람이 한 번 등록해 두고
(이 파일을 손으로 채운다) 그 날짜를 매일 점검하는 방식으로 간단히 간다.

[경보 등급]
  정상    : 유효기한까지 warn_days 보다 많이 남음
  경고    : 유효기한까지 warn_days 이하로 남음 (교체 준비)
  만료    : 유효기한이 지남 (즉시 교체 필요)
등급이 바뀔 때만(정상->경고, 경고->만료) 기록한다 — 매 점검마다 같은 경고를
반복 기록하면 SQLite 만 커진다.
"""
import datetime
import os

import rclpy
import yaml
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from patrol_core import event_log

EX1 = os.path.join(os.path.expanduser("~"), "vibe", "ex1")
DEFAULT_INFO_FILE = os.path.join(EX1, "maps", "extinguisher_info.yaml")

LEVELS = ("정상", "경고", "만료")


def level_for(days_left, warn_days):
    if days_left < 0:
        return "만료"
    if days_left <= warn_days:
        return "경고"
    return "정상"


class ExtinguisherExpiryNode(Node):
    def __init__(self):
        super().__init__("extinguisher_expiry_node")

        self.declare_parameter("info_file", DEFAULT_INFO_FILE)
        self.declare_parameter("warn_days", 30)
        # 하루 한 번이면 충분하다(날짜 단위 판단이라 더 자주 볼 이유가 없다).
        # 노드를 새로 켤 때는 시작 직후 1회를 별도로 한다. 시험할 땐 짧게 줄 것.
        self.declare_parameter("check_interval_sec", 86400.0)
        self.declare_parameter("db_path", event_log.DEFAULT_DB)
        self.declare_parameter("quiet", False)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_status = self.create_publisher(String, "/extinguisher/status", qos)

        self.last_level = {}     # name -> 마지막으로 기록한 등급
        self.last_text = ""      # 마지막으로 낸 요약 문구

        interval = float(self.get_parameter("check_interval_sec").value)
        self.create_timer(interval, self.check_all)
        self.check_all()         # 시작 직후 1회

        # 판정은 하루 한 번이면 되지만, **알리는 것**은 자주 해야 한다.
        # 대시보드처럼 나중에 켜지는 구독자는 그 사이에 발행이 없으면
        # 다음 판정(최대 24시간 뒤)까지 옛 값을 그대로 보여준다.
        # 실제로 소화기 정보를 고치고 이 노드만 재시작했더니 대시보드가
        # 11분 넘게 옛 날짜를 띄우고 있었다(2026-09-05).
        # 날짜 계산을 다시 하지 않고 이미 만든 문구만 다시 낸다.
        self.declare_parameter("status_period_sec", 5.0)
        self.create_timer(
            float(self.get_parameter("status_period_sec").value),
            self.republish_status)

        self.get_logger().info(
            f"소화기 유효기한 감시 시작 — {interval / 3600:.0f}시간마다 점검, "
            f"경고 기준 {self.get_parameter('warn_days').value}일 전"
        )

    def load_info(self):
        path = str(self.get_parameter("info_file").value)
        if not os.path.exists(path):
            self.get_logger().error(f"{path} 가 없다 — 소화기 정보를 등록할 것")
            return []
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        return data.get("extinguishers", [])

    def check_all(self):
        items = self.load_info()
        if not items:
            return
        today = datetime.date.today()
        warn_days = int(self.get_parameter("warn_days").value)
        summaries = []

        for item in items:
            name = item.get("name", "?")
            try:
                expiry = datetime.date.fromisoformat(str(item["expiry_date"]))
            except (KeyError, ValueError) as e:                  # noqa: BLE001
                self.get_logger().error(f"{name} expiry_date 형식 오류: {e}")
                continue
            days_left = (expiry - today).days
            level = level_for(days_left, warn_days)
            summaries.append(f"{name}: {level}(D{days_left:+d})")

            prev = self.last_level.get(name)
            if level != prev and level != "정상":
                self.get_logger().warn(
                    f"소화기 [{name}] {level} — 유효기한 {expiry} (D{days_left:+d})"
                )
                event_log.log_event(
                    "extinguisher_expiry_node", "alert",
                    f"[{name}] {level} — 유효기한 {expiry} (D{days_left:+d})",
                    db_path=str(self.get_parameter("db_path").value),
                )
            if level != prev:
                self.last_level[name] = level

        text = " / ".join(summaries) if summaries else "등록된 소화기 없음"
        self.last_text = text
        m = String()
        m.data = text
        self.pub_status.publish(m)
        if not bool(self.get_parameter("quiet").value):
            self.get_logger().info(f"점검 결과: {text}")


    def republish_status(self):
        """이미 판정해 둔 문구를 다시 낸다(계산도 로그도 하지 않는다).

        로그까지 같이 내면 5초마다 같은 줄이 쌓여서 정작 중요한 경고가
        묻힌다. 그래서 발행만 한다.
        """
        if not self.last_text:
            return
        m = String()
        m.data = self.last_text
        self.pub_status.publish(m)


def main():
    rclpy.init()
    node = ExtinguisherExpiryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
