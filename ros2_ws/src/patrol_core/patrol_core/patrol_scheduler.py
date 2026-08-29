#!/usr/bin/env python3
"""
patrol_scheduler.py — 순찰이 끝나면 잠시 쉬고, 랜덤한 시각에 다시 시작시키는 노드.

[VM에서 실행]  (로봇 없이도 동작 확인 가능 — /patrol/enable 만 발행한다)
  기본 (끝난 뒤 1분 쉬고, 5분 안의 랜덤 시점에 다시):
  ros2 run patrol_core patrol_scheduler

  더 짧게 보려면 (30초 쉬고 90초 안에):
  ros2 run patrol_core patrol_scheduler --ros-args \
    -p rest_min:=0.5 -p cycle_min:=1.5 -p fire_on_start:=true

[입력]  /patrol/status   (String)  순찰이 끝났는지 안다. 이게 시각 계산의 기준점이다.
        /inspect/status  (String)  소화기 점검이 끝났는지 안다.
        /inspect/request (Bool)    "다음 순번에 점검을 해라" 예약. 즉시 실행이 아니다.
[출력]  /patrol/enable  (Bool)     True 만 발행한다. False 는 보내지 않는다.
        /inspect/start  (Bool)     점검 차례일 때 True.
        /patrol/schedule (String)  다음 발행 예정 시각과 작업 종류 (진단용)

[한 순번에 한 작업만]
순찰과 점검은 둘 다 Nav2 에 목표를 보내므로 동시에 돌면 서로 목표를 뺏는다.
그래서 스케줄러가 순번마다 **하나만** 배정한다. 점검 예약이 있으면 그 순번은
점검이 가져가고, 그 순번에 순찰은 하지 않는다. 점검이 끝나면 다시 순찰로 돌아간다.
이 구조 덕분에 주행 중 끼어들기(cmd_vel 차단 등)가 필요 없다.

  순찰 중 예약 접수 → 순찰은 끝까지 → 쉼+랜덤 → [점검] → 쉼+랜덤 → [순찰] → ...

예약은 여러 번 와도 하나로 합친다(중복 무시). 점검 중에 다시 와도 하나로 유지한다.

[언제 다음 순찰을 시작하는가]
  enable=True ── 순찰 (laps_per_run 바퀴) ── done ─┤ 쉼 ├── 랜덤 ──→ 다음 enable=True
                                                   └──── cycle_min ────┘

  다음 시작 = 순찰이 끝난 시각 + 균등난수(rest_min ~ cycle_min)
  기본값으로는 끝난 뒤 1~5분 사이의 랜덤한 시점이다.

[왜 시계(정각)를 기준으로 하지 않는가 — 2026-07-30 변경]
처음엔 "매 시간 0~50분 사이 랜덤"처럼 시계 경계를 기준으로 뽑았다. 주기를 5분으로
줄이자 그 방식이 깨진다: 순찰 자체가 2~3분이라, 시계 기준으로 뽑으면 순찰이 끝난
직후에 다음 시각이 걸리거나(쉬는 시간 0) 이미 지나가 버려 그 주기를 건너뛴다.
"무조건 돌아오고 1분은 쉰다"를 지키려면 기준점이 시계가 아니라
**직전 순찰이 끝난 시각**이어야 한다. 그래서 최소 간격(min_gap) 파라미터도 없앴다 —
시작 간격은 자동으로 (순찰 시간 + rest_min) 이상이 되므로 규칙이 중복이다.

[순찰이 끝난 걸 어떻게 아는가]
patrol_node 가 바퀴 수를 채우고 스스로 멈출 때 /patrol/status 로 "done (...)" 을
발행한다. 스케줄러는 그걸 받은 시점부터 쉼+랜덤을 센다. 순찰 중에는 아무것도
발행하지 않는다(겹쳐 보내지 않는다). 순찰이 끝나지 않으면 다음 순찰도 잡히지
않는데, 이건 의도한 동작이다. 대신 너무 오래 걸리면 경고를 낸다.

[발행 전에 구독자를 확인한다]
QoS 가 VOLATILE 이라 구독자가 없을 때 발행한 메시지는 그냥 사라진다. launch 로
patrol_node 와 함께 띄우면 스케줄러가 먼저 올라오는 일이 잦아 첫 트리거를 잃는다
(실측: 0.08초 차이로 놓쳤다). 그래서 예정 시각이 되면 바로 쏘지 않고,
구독자가 생길 때까지 subscriber_wait_sec 동안 기다렸다가 발행한다.
반대로 TRANSIENT_LOCAL 로 바꾸는 방법도 있으나, 그러면 나중에 patrol_node 를
재시작할 때 지난 True 가 곧바로 배달돼 로봇이 예고 없이 움직인다. 그래서 택하지 않았다.
"""
import random
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

# patrol_node 가 /patrol/status 로 내는 값 중 "회차가 끝났다"는 뜻인 것들.
# done (...)               바퀴 수를 채워 스스로 멈췄다 (정상 종료)
# stopped by enable=false  누가 밖에서 껐다
FINISHED_PREFIXES = ("done", "stopped")


def hhmmss(t):
    return time.strftime("%H:%M:%S", time.localtime(t))


class PatrolScheduler(Node):
    def __init__(self):
        super().__init__("patrol_scheduler")

        # 순찰이 끝난 뒤 다음 순찰이 시작될 수 있는 최대 시간(분).
        # 2026-08-01 사용자 결정: 확인하기 좋게 2분으로 줄였다(원래 5분).
        # 되돌리려면 cycle_min:=5.0 을 주면 된다.
        self.declare_parameter("cycle_min", 2.0)
        # 끝난 직후 반드시 쉬는 시간(분). 이 시간 안에는 절대 시작하지 않는다.
        self.declare_parameter("rest_min", 1.0)
        # 노드를 켠 직후 한 번 발행하고 시작할지. 기다리지 않고 확인할 때 쓴다.
        self.declare_parameter("fire_on_start", False)
        # 난수 씨앗. 0 이면 매번 다르게(실제 운용), 그 외 값이면 재현 가능(테스트).
        self.declare_parameter("seed", 0)
        # 구독자(patrol_node)가 없을 때 이 시간(초)까지 기다리며 발행을 재시도한다.
        self.declare_parameter("subscriber_wait_sec", 30.0)
        # fire_on_start 일 때 첫 회차까지 기다리는 시간(초).
        # 켜자마자 순찰을 시작하면 DDS 디스커버리(원격 서비스 발견에 실측 11.5~30초),
        # 부저 확인, Nav2 목표 전송이 한꺼번에 몰린다. 무선이 느릴 때 그 셋이 겹치면
        # 첫 목표가 거절되거나 로그가 뒤엉켜 원인을 찾기 어렵다. 조금 늦춰 순서를 만든다.
        self.declare_parameter("start_delay_sec", 10.0)

        self.cycle = float(self.get_parameter("cycle_min").value) * 60.0
        self.rest = float(self.get_parameter("rest_min").value) * 60.0

        if self.rest < 0.0:
            self.rest = 0.0
        if self.cycle <= self.rest:
            # 쉬는 시간이 창보다 크면 뽑을 구간이 없다. 창을 쉼의 2배로 벌린다.
            new = max(self.rest * 2.0, self.rest + 60.0)
            self.get_logger().warn(
                f"cycle_min({self.cycle / 60:.1f}) 이 rest_min({self.rest / 60:.1f}) "
                f"보다 커야 한다 -> {new / 60:.1f} 로 조정"
            )
            self.cycle = new

        seed = int(self.get_parameter("seed").value)
        self.rng = random.Random(seed if seed != 0 else None)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_enable = self.create_publisher(Bool, "/patrol/enable", qos)
        self.pub_inspect = self.create_publisher(Bool, "/inspect/start", qos)
        self.pub_schedule = self.create_publisher(String, "/patrol/schedule", qos)
        self.create_subscription(String, "/patrol/status", self.on_status, qos)
        self.create_subscription(String, "/inspect/status", self.on_status, qos)
        self.create_subscription(Bool, "/inspect/request", self.on_request, qos)

        self.n_fire = 0
        self.next_fire = None      # 예정 시각. 작업 중에는 None (아직 안 잡힌 상태)
        self.pending = None        # (사유, 처음 시도한 시각) — 구독자 대기 중인 발행
        self.running_since = None  # 발행 후 done 을 기다리는 중이면 그 시각
        self.running_task = None   # 지금 돌고 있는 작업 ("patrol" | "inspect")
        self.inspect_queued = False  # 점검 예약 (중복은 하나로 합친다)

        self.get_logger().info(
            f"스케줄러 시작. 순찰이 끝나면 {self.rest / 60:.1f}~{self.cycle / 60:.1f}분 "
            f"뒤의 랜덤 시점에 다시 시작한다" + (f" (seed={seed})" if seed != 0 else "")
        )

        if bool(self.get_parameter("fire_on_start").value):
            delay = max(float(self.get_parameter("start_delay_sec").value), 0.0)
            if delay > 0:
                self.next_fire = time.time() + delay
                self.get_logger().info(
                    f"첫 회차는 {delay:.0f}초 뒤에 시작한다 "
                    "(노드 발견·부저 확인이 먼저 끝나도록)"
                )
            else:
                self.pending = ("시작 직후 발행(fire_on_start)", time.time())
        else:
            # 노드를 켠 시각을 "직전 순찰이 끝난 시각"으로 보고 첫 시각을 뽑는다.
            self.schedule_after(time.time(), "노드 시작")

        self.create_timer(1.0, self.tick)

    # ---------------- 시각 선택 ----------------
    def next_task(self):
        """다음 순번에 할 작업. 점검 예약이 있으면 점검이 그 순번을 가져간다."""
        return "inspect" if self.inspect_queued else "patrol"

    def schedule_after(self, base, why):
        """base(직전 작업이 끝난 시각) 기준으로 다음 발행 시각을 뽑는다."""
        self.next_fire = base + self.rng.uniform(self.rest, self.cycle)
        wait = self.next_fire - time.time()
        task = "점검" if self.next_task() == "inspect" else "순찰"
        text = (f"next {hhmmss(self.next_fire)} (약 {wait / 60:.1f}분 후, "
                f"{why} 기준, 작업={task})")
        self.get_logger().info(f"다음 순찰 시작 예정: {text}")
        m = String()
        m.data = text
        self.pub_schedule.publish(m)

    # ---------------- 상태 수신 ----------------
    def on_request(self, msg: Bool):
        if not msg.data:
            return
        if self.inspect_queued:
            self.get_logger().info("점검 예약이 이미 있다 — 중복 요청은 무시한다")
            return
        self.inspect_queued = True
        when = ("현재 작업이 끝난 뒤" if self.running_task
                else f"{hhmmss(self.next_fire)}" if self.next_fire else "다음 순번")
        self.get_logger().warn(f"소화기 점검 예약 접수 — {when} 순번에 수행한다")
        if self.next_fire is not None:
            # 이미 잡혀 있던 순번의 작업 종류만 바꿔 다시 알린다(시각은 그대로).
            self.announce_current()

    def announce_current(self):
        wait = self.next_fire - time.time()
        task = "점검" if self.next_task() == "inspect" else "순찰"
        text = f"next {hhmmss(self.next_fire)} (약 {wait / 60:.1f}분 후, 작업={task})"
        self.get_logger().info(f"다음 순번: {text}")
        m = String()
        m.data = text
        self.pub_schedule.publish(m)

    def on_status(self, msg: String):
        if not msg.data.startswith(FINISHED_PREFIXES):
            return
        label = {"inspect": "점검", "patrol": "순찰"}.get(self.running_task, "작업")
        if self.running_since is None:
            # 우리가 시작시킨 회차가 아니다(수동 실행 등). 기준점만 갱신한다.
            self.get_logger().info(f"{label} 종료 감지 (status={msg.data})")
        else:
            took = time.time() - self.running_since
            self.get_logger().info(
                f"{label} 종료 (status={msg.data}, {took / 60:.1f}분 걸렸다)"
            )
            self.running_since = None
        self.running_task = None
        self.schedule_after(time.time(), f"{label} 종료")

    # ---------------- 주기 판단 ----------------
    def tick(self):
        if self.pending is None and self.next_fire is not None:
            if time.time() >= self.next_fire:
                self.pending = (f"예정 시각 {hhmmss(self.next_fire)}", time.time())

        if self.pending is not None:
            self.service_pending()
        elif self.next_fire is None and self.running_since is not None:
            # 순찰이 끝나야 다음 시각이 잡힌다. 너무 오래 걸리면 알린다.
            late = time.time() - self.running_since
            if late > self.cycle * 2.0:
                self.get_logger().warn(
                    f"순찰이 {late / 60:.1f}분째 끝나지 않아 다음 순찰을 잡지 못한다 "
                    "— patrol_node 로그를 볼 것 (목표 거절 / 로봇 정지 / 노드 사망)",
                    throttle_duration_sec=30.0,
                )

    def service_pending(self):
        """발행 예정이 잡혀 있다. 구독자가 생기면 보내고, 없으면 기다린다."""
        why, since = self.pending
        task = self.next_task()
        pub = self.pub_inspect if task == "inspect" else self.pub_enable
        topic = "/inspect/start" if task == "inspect" else "/patrol/enable"
        node = "inspect_node" if task == "inspect" else "patrol_node"

        if pub.get_subscription_count() > 0:
            self.pending = None
            self.fire(why, task)
            return

        waited = time.time() - since
        limit = float(self.get_parameter("subscriber_wait_sec").value)
        if waited >= limit:
            self.pending = None
            self.get_logger().error(
                f"{topic} 구독자가 {limit:.0f}초 동안 없어 이번 순번을 건너뛴다 "
                f"— {node} 가 떠 있는지 확인 (`ros2 node list`)"
            )
            # 점검 예약은 유지한다. 노드를 띄우면 다음 순번에 다시 시도된다.
            self.schedule_after(time.time(), "구독자 없어 건너뜀")
        else:
            self.get_logger().warn(
                f"{node} 를 기다린다 ({waited:.0f}/{limit:.0f}초) — "
                "구독자가 없을 때 발행하면 메시지가 사라진다",
                throttle_duration_sec=5.0,
            )

    def fire(self, why, task):
        now = time.time()
        m = Bool()
        m.data = True
        if task == "inspect":
            self.pub_inspect.publish(m)
            self.inspect_queued = False      # 예약 소모
            topic = "/inspect/start"
        else:
            self.pub_enable.publish(m)
            topic = "/patrol/enable"
        self.n_fire += 1
        self.next_fire = None          # 작업이 끝날 때까지 다음 시각을 잡지 않는다
        self.running_since = now
        self.running_task = task
        self.get_logger().warn(
            f"[{self.n_fire}] {topic} = True  ({hhmmss(now)}, {why})"
        )


def main():
    rclpy.init()
    try:
        node = PatrolScheduler()
    except SystemExit:
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # launch 파일 종료 시 SIGTERM -> ExternalShutdownException. Ctrl+C 와 동일 취급.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
