#!/usr/bin/env python3
"""
patrol_node.py — 등록된 웨이포인트를 따라 순찰하는 노드.

[VM에서 실행]  (Nav2 가 켜져 있고 RViz 에서 2D Pose Estimate 를 한 상태여야 한다)
  ros2 run patrol_core patrol_node

  한 바퀴 순환 + 코너에서 멈춰 회전:
  ros2 run patrol_core patrol_node --ros-args -p mode:=loop -p stop_at_corners:=true

  스케줄러와 함께 (트리거가 올 때까지 가만히 있는다):
  ros2 run patrol_core patrol_node --ros-args -p mode:=loop -p stop_at_corners:=true \
    -p auto_start:=false -p laps_per_run:=2

[입력]  maps/patrol_waypoints.yaml   save_waypoint.py 로 등록한 좌표 목록
        /patrol/enable  (Bool)       True 일 때만 순찰. patrol_scheduler 가 보낸다.
        /patrol/hold    (Bool)       True 면 그 자리에 선다. helmet_node 가 보낸다.
[출력]  /navigate_to_pose        (action)  코너 정지 모드에서 지점별 목표
        /navigate_through_poses  (action)  통과 모드에서 경로 전체
        /patrol/status  (String)           현재 상태 (진단용)

[동작 흐름]
  ① goto_start_first=true 면 먼저 첫 웨이포인트로 이동 (로봇이 어디 있든 시작지점으로)
  ② 그 다음 mode 에 따라 순찰 반복
  ③ laps_per_run 바퀴를 채우면 스스로 멈춘다 (enable 이 다시 True 로 오면 재시작)

[laps_per_run — 언제 멈추는가]
순찰을 얼마나 하고 멈출지는 시간이 아니라 **바퀴 수**로 정한다. 사각형이
1.15m x 0.95m 로 작아 한 바퀴가 짧고, 매번 같은 양을 돌아 결과가 일정하다.
시간 기준으로 하면 코너 중간에 끊겨 로봇이 애매한 자세로 선다.
멈추는 판단을 노드 자신이 하는 이유: 바퀴 수를 아는 건 여기뿐이고,
스케줄러는 시각만 안다. 스케줄러는 True 만 보내고 False 는 보내지 않는다.

[mode]
  roundtrip : wp0 -> wp1 -> ... -> wpN -> ... -> wp0 -> ...   (끝에서 되돌아옴)
  loop      : wp0 -> wp1 -> ... -> wpN -> wp0 -> ...          (같은 방향으로 순환 = 한 바퀴)
  어느 쪽이든 후진하지 않는다. 방향을 바꿀 때 Nav2 가 회전한 뒤 정면 주행한다.

[stop_at_corners]
  false : 구간 전체를 navigate_through_poses 로 한 번에 보낸다. 중간 지점을 멈추지 않고
          부드럽게 통과하지만, Nav2 가 코너를 둥글게 깎는다.
  true  : 지점마다 navigate_to_pose 로 따로 보낸다. 각 지점에서 정지·회전하므로
          코너를 정확히 돈다. 좁은 방(벽에서 30cm)에서는 이쪽이 안전하다.
          Nav2 의 코너 반경이 여유보다 크면 벽에 붙기 때문이다.

[실측 배경 — 2026-07-29]
지점마다 navigate_to_pose 를 보내던 초기 버전은 중간중간 멈추고 지그재그가 됐다.
원인은 코너 처리가 아니라 파라미터였다: inflation_radius 0.5 라서 벽 주변 50cm 가
전부 고비용 구역이고, 30cm 경로가 그 안쪽이라 Nav2 가 계속 벗어나려 했다.
config/patrol_nav2.yaml 에서 0.25 로 낮추고 xy_goal_tolerance 도 0.25->0.10 으로 조였다.

[/patrol/hold — 주행 중에 끼어들어 세우기 (2026-08-01)]
안전모 미착용자를 보면 그 자리에 서야 한다. 점검(inspect_node)은 스케줄러가 순번을
배정하므로 끼어들 필요가 없었지만, 안전모는 **순찰이 도는 도중에** 세워야 한다.
세우는 방법은 진행 중인 Nav2 목표를 **취소**하는 것이다 — Nav2 가 취소를 받으면
컨트롤러가 스스로 로봇을 멈춘다. 우리가 /cmd_vel 을 직접 내지 않으므로 절대 규칙 1을
건드리지 않고, cmd_vel_mux 를 순찰 경로에 넣지 않아도 된다.
풀리면 취소된 그 목표를 다시 보낸다(leg_pos 를 건드리지 않으므로 이어서 간다).
⚠️ 취소는 "외부 취소"(다른 노드가 Nav2 를 가져간 경우)와 구분해야 한다. 그쪽은
회차를 끝내지만 hold 는 끝내면 안 된다 — on_result 에서 held 를 먼저 본다.
"""
import math
import os
import sys
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

DEFAULT_WAYPOINT_FILE = os.path.join(
    os.path.expanduser("~"), "vibe", "ex1", "maps", "patrol_waypoints.yaml"
)


class PatrolNode(Node):
    def __init__(self):
        super().__init__("patrol_node")

        self.declare_parameter("waypoint_file", DEFAULT_WAYPOINT_FILE)
        self.declare_parameter("auto_start", True)
        self.declare_parameter("goto_start_first", True)
        self.declare_parameter("mode", "roundtrip")        # roundtrip | loop
        self.declare_parameter("stop_at_corners", False)
        self.declare_parameter("use_saved_yaw_at_end", False)
        # 한 번 시작하면 몇 바퀴 돌고 멈출지. 0 이면 무제한(수동 테스트용).
        # roundtrip 모드에서는 한쪽 방향 주행 1회를 1로 센다.
        self.declare_parameter("laps_per_run", 2)
        # 첫 회차만 다르게 돌 바퀴 수. 0 이면 첫 회차도 laps_per_run 을 따른다.
        # 켜자마자 한 바퀴 돌려 동작을 눈으로 확인하고, 그 다음부터 정상 회차로
        # 넘어가기 위한 것이다(스케줄러의 fire_on_start 와 짝을 이룬다).
        self.declare_parameter("first_laps", 0)
        # 구간이 끝난 뒤(끝 지점 도달 / 한 바퀴 완주) 다음 구간 전 대기 시간(초).
        # 급하게 뒤돌면 관성으로 위치추정이 흔들린다. 나중에 QR 확인을 넣을 자리이기도 하다.
        self.declare_parameter("dwell_sec", 1.0)
        # 시작지점에 도착한 뒤 안전모 영상이 들어올 때까지 기다리는 최대 시간(초).
        # /helmet/ready 가 오면 그 전에 출발한다. 0 이면 기다리지 않는다.
        self.declare_parameter("vision_wait_sec", 40.0)
        # 보고서·시연 영상용. 정지/재개 한 번에 네 줄씩 나오는 것을 한 줄로 줄인다.
        self.declare_parameter("quiet", False)

        self.waypoints = self.load_waypoints()
        if len(self.waypoints) < 2:
            self.get_logger().error(
                f"웨이포인트가 2개 이상 필요하다 (현재 {len(self.waypoints)}개): "
                f"{self.get_parameter('waypoint_file').value}"
            )
            raise SystemExit(1)

        self.mode = str(self.get_parameter("mode").value)
        if self.mode not in ("roundtrip", "loop"):
            self.get_logger().warn(f"알 수 없는 mode '{self.mode}' -> roundtrip 으로 처리")
            self.mode = "roundtrip"

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_status = self.create_publisher(String, "/patrol/status", qos)
        self.create_subscription(Bool, "/patrol/enable", self.on_enable, qos)
        self.create_subscription(Bool, "/patrol/hold", self.on_hold, qos)
        # 안전모 판정이 영상을 받고 있는지. 시작지점에서 이걸 기다린다.
        self.create_subscription(Bool, "/helmet/ready", self.on_vision, qos)

        self.cli_pose = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.cli_route = ActionClient(self, NavigateThroughPoses, "navigate_through_poses")

        self.enabled = bool(self.get_parameter("auto_start").value)
        self.held = False         # /patrol/hold 로 세워 둔 상태인가
        # 시작지점에서 영상이 준비되기를 기다리는 상태
        self.vision_ready = False
        self.waiting_vision = False
        self.vision_deadline = 0.0
        self.goal_handle = None
        self.busy = False
        self.dwell_timer = None
        self.n_lap = 0            # 노드가 켜진 뒤 누적 완주 수 (진단용)
        self.lap_in_run = 0       # 이번 회차(enable=True 이후) 완주 수 — 정지 판단용
        self.runs_done = 0        # 시작한 회차 수 (첫 회차 판단용)
        self.run_limit = self.limit_for_run()   # 이번 회차에 돌 바퀴 수
        # 이번 회차 시작 시각 (소요 시간 로그용). auto_start 면 지금이 시작이다.
        self.run_started = time.time() if self.enabled else None
        if self.enabled:
            self.runs_done = 1        # auto_start 는 첫 회차를 이미 시작한 것이다

        # 첫 단계: 시작지점으로 이동할지 판단
        self.going_to_start = bool(self.get_parameter("goto_start_first").value)
        # 현재 구간(leg)의 목표 인덱스 목록과 그 안에서의 진행 위치
        self.leg = []
        self.leg_pos = 0
        self.forward = True    # roundtrip 에서 방향

        laps = self.run_limit
        self.get_logger().info(
            f"순찰 노드 시작. 웨이포인트 {len(self.waypoints)}개, "
            f"mode={self.mode}, 코너정지={self.get_parameter('stop_at_corners').value}, "
            f"시작지점자동이동={self.going_to_start}, "
            f"1회={'무제한' if laps <= 0 else f'{laps}바퀴'}"
        )
        for i, w in enumerate(self.waypoints):
            self.get_logger().info(
                f"  [{i}] {w.get('name','')}  x={w['x']:.3f} y={w['y']:.3f}"
            )
        if not self.enabled:
            self.get_logger().info("/patrol/enable 에 true 가 오면 시작한다")

        self.create_timer(0.5, self.tick)

    # ---------------- 준비 ----------------
    def load_waypoints(self):
        path = self.get_parameter("waypoint_file").value
        if not os.path.exists(path):
            return []
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        return data.get("waypoints", [])

    def on_enable(self, msg: Bool):
        want = bool(msg.data)
        if want == self.enabled:
            return
        self.enabled = want
        self.get_logger().warn(f"/patrol/enable = {self.enabled}")
        if self.enabled:
            self.start_run()
        else:
            self.cancel_current()
            self.status("stopped by enable=false")

    def on_vision(self, msg: Bool):
        """안전모 판정이 영상을 받기 시작했다."""
        if bool(msg.data) == self.vision_ready:
            return
        self.vision_ready = bool(msg.data)
        if self.vision_ready and self.waiting_vision:
            self.get_logger().info("카메라 영상 확인 — 순찰을 시작한다")
            self.waiting_vision = False
            self.finish_step(dwell=True)

    def on_hold(self, msg: Bool):
        """안전모 미착용 등으로 그 자리에 세운다. 회차는 끝내지 않는다."""
        want = bool(msg.data)
        if want == self.held:
            return
        quiet = bool(self.get_parameter("quiet").value)
        self.held = want
        if self.held:
            if not quiet:
                self.get_logger().warn("/patrol/hold = True — 진행 중인 목표를 취소하고 선다")
            self.cancel_current(quiet=quiet)
            self.status("held")
        else:
            if not quiet:
                self.get_logger().warn("/patrol/hold = False — 순찰을 이어간다")
            # busy 를 풀어 두면 다음 tick 이 취소된 그 목표를 다시 보낸다.
            self.busy = False
            self.status("resumed")

    def limit_for_run(self):
        """이번 회차에 돌 바퀴 수. 첫 회차만 first_laps 를 쓴다(0 이면 무시)."""
        first = int(self.get_parameter("first_laps").value)
        if self.runs_done == 0 and first > 0:
            return first
        return int(self.get_parameter("laps_per_run").value)

    def start_run(self):
        """새 회차 시작. 바퀴 수를 0 으로 되돌리고 처음부터 다시 구성한다."""
        self.lap_in_run = 0
        self.run_limit = self.limit_for_run()
        self.runs_done += 1
        # 회차가 실제로 몇 분 걸리는지가 스케줄러 주기(cycle_min)를 정하는 근거다.
        self.run_started = time.time()
        self.going_to_start = bool(self.get_parameter("goto_start_first").value)
        self.leg = []
        self.leg_pos = 0
        self.forward = True
        laps = self.run_limit
        self.get_logger().info(
            f"순찰 회차 시작 ({'무제한' if laps <= 0 else f'{laps}바퀴'} 예정"
            + (", 첫 회차" if self.runs_done == 1 else "") + ")"
        )
        self.status("run started")

    def stop_run(self, reason):
        """이번 회차를 끝낸다. 스스로 enable 을 내려 다음 트리거를 기다린다."""
        self.enabled = False
        self.cancel_current()
        took = ""
        if self.run_started is not None:
            took = f", {(time.time() - self.run_started) / 60:.1f}분 걸렸다"
            self.run_started = None
        self.get_logger().warn(
            f"순찰 회차 종료 — {reason}{took}. "
            "/patrol/enable 에 true 가 다시 오면 재시작한다"
        )
        # 스케줄러가 이 "done" 을 받은 시점부터 쉼+랜덤을 센다.
        self.status(f"done ({reason})")

    def cancel_current(self, quiet=False):
        if self.goal_handle is not None:
            if not quiet:
                self.get_logger().info("진행 중인 목표 취소")
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        if self.dwell_timer is not None:
            self.dwell_timer.cancel()
            self.dwell_timer = None
        self.busy = False

    def lap_label(self):
        """로그용 진행 표시. 이번 회차 진행이 먼저 보여야 한다(누적은 참고값)."""
        limit = self.run_limit
        cur = self.lap_in_run + 1        # 지금 도는 중인 바퀴
        if limit > 0:
            return f"{cur}/{limit}바퀴, 누적 {self.n_lap}"
        return f"{cur}바퀴째, 누적 {self.n_lap}"

    def status(self, text):
        m = String()
        m.data = text
        self.pub_status.publish(m)

    # ---------------- 경로 구성 ----------------
    def build_leg(self):
        """다음에 이동할 웨이포인트 인덱스 목록을 만든다."""
        n = len(self.waypoints)
        if self.mode == "loop":
            # 한 바퀴: 1, 2, ..., n-1, 0  (돌아와서 다시 같은 방향)
            return list(range(1, n)) + [0]
        # 왕복: 정방향이면 1..n-1, 역방향이면 n-2..0
        if self.forward:
            return list(range(1, n))
        return list(range(n - 2, -1, -1))

    def make_pose(self, w, yaw):
        p = PoseStamped()
        p.header.frame_id = "map"
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(w["x"])
        p.pose.position.y = float(w["y"])
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        return p

    def yaw_for(self, idx, next_idx):
        """지점 idx 에서 취할 방향. next_idx 가 있으면 그쪽을 향한다."""
        w = self.waypoints[idx]
        if next_idx is not None:
            t = self.waypoints[next_idx]
            d = math.hypot(t["x"] - w["x"], t["y"] - w["y"])
            if d > 1e-3:
                return math.atan2(t["y"] - w["y"], t["x"] - w["x"])
        if bool(self.get_parameter("use_saved_yaw_at_end").value):
            return float(w.get("yaw", 0.0))
        return float(w.get("yaw", 0.0))

    # ---------------- 주기 판단 ----------------
    def tick(self):
        # 영상을 기다리는 중이면, 준비되거나 시간이 다 되면 출발한다.
        if self.waiting_vision:
            if self.vision_ready or time.time() >= self.vision_deadline:
                if not self.vision_ready:
                    self.get_logger().warn(
                        "영상이 안 들어왔지만 시간이 다 돼 순찰을 시작한다 "
                        "(안전모 감지 없이 돌게 된다 — helmet_node 로그 확인)")
                self.waiting_vision = False
                self.finish_step(dwell=True)
            return
        if not self.enabled or self.busy or self.held:
            return

        if self.going_to_start:
            if not self.cli_pose.server_is_ready():
                self.warn_not_ready()
                return
            self.send_single(0, label="시작지점으로 이동")
            return

        if not self.leg:
            self.leg = self.build_leg()
            self.leg_pos = 0

        if bool(self.get_parameter("stop_at_corners").value):
            if not self.cli_pose.server_is_ready():
                self.warn_not_ready()
                return
            idx = self.leg[self.leg_pos]
            nxt = self.leg[self.leg_pos + 1] if self.leg_pos + 1 < len(self.leg) else None
            self.send_single(idx, next_idx=nxt)
        else:
            if not self.cli_route.server_is_ready():
                self.warn_not_ready()
                return
            self.send_route(self.leg)

    def warn_not_ready(self):
        self.get_logger().warn(
            "Nav2 액션 서버 대기 중 — Nav2 가 켜져 있고 RViz 에서 "
            "2D Pose Estimate 를 했는지 확인 (안 하면 영원히 대기한다)",
            throttle_duration_sec=5.0,
        )

    # ---------------- 목표 전송 ----------------
    def send_single(self, idx, next_idx=None, label=None):
        w = self.waypoints[idx]
        goal = NavigateToPose.Goal()
        goal.pose = self.make_pose(w, self.yaw_for(idx, next_idx))
        self.busy = True
        name = w.get("name", f"wp{idx}")
        self.get_logger().info(f"{label or f'-> [{idx}] {name}'} ({self.lap_label()})")
        self.status(f"moving to {name}")
        fut = self.cli_pose.send_goal_async(goal)
        fut.add_done_callback(self.on_goal_response)

    def send_route(self, indices):
        poses = []
        for i, idx in enumerate(indices):
            nxt = indices[i + 1] if i + 1 < len(indices) else None
            poses.append(self.make_pose(self.waypoints[idx], self.yaw_for(idx, nxt)))
        goal = NavigateThroughPoses.Goal()
        goal.poses = poses
        self.busy = True
        names = " -> ".join(self.waypoints[i].get("name", "?") for i in indices)
        self.get_logger().info(f"경로 전송 ({self.lap_label()}): {names}")
        self.status("following route")
        fut = self.cli_route.send_goal_async(goal)
        fut.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            # 거절은 Nav2 잘못이 아닐 때가 많다. 실측한 원인은 전부 아래 두 가지였다.
            # 0.5초마다 같은 줄이 쏟아지면 원인이 묻히므로 5초로 조인다.
            self.get_logger().error(
                "Nav2 가 목표를 거절했다 — 확인할 것: "
                "① turtlebot3_node 가 살아있는지(`ros2 topic hz /odom`). 죽으면 odom "
                "프레임이 사라져 Nav2 가 모든 목표를 거절한다(전원 문제가 원인이었다) "
                "② RViz 에서 2D Pose Estimate 를 했는지 / Localization 이 active 인지",
                throttle_duration_sec=5.0,
            )
            self.status("goal rejected")
            self.busy = False
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self.on_result)

    # ---------------- 결과 처리 ----------------
    def on_result(self, future):
        status = future.result().status
        self.goal_handle = None

        if status == GoalStatus.STATUS_CANCELED:
            self.busy = False
            if self.held:
                # 우리가 세운 것이다. 회차를 끝내지 않고 풀릴 때까지 기다린다.
                if not bool(self.get_parameter("quiet").value):
                    self.get_logger().info("정지 완료 — /patrol/hold 가 풀리면 이어서 간다")
                return
            if not self.enabled:
                # 우리가 껐거나(stop_run/enable=false) 정상 종료 경로다.
                self.get_logger().info("목표가 취소됨")
                self.status("canceled")
                return
            # 순찰 중인데 밖에서 취소됐다 = 다른 노드(점검 등)가 Nav2 목표를 가져갔다.
            # 여기서 다음 목표를 다시 쏘면 둘이 Nav2 를 두고 싸운다(실측 2026-08-01:
            # 점검이 도는 동안 순찰이 계속 목표를 다시 보내 서로 취소시켰다).
            # 이 회차는 여기서 끝내고 쉰다 — 취소됐다고 바퀴를 더 돌 이유가 없다.
            self.get_logger().warn(
                "이동이 밖에서 취소됐다 — 다른 노드가 Nav2 목표를 가져간 것으로 보고 "
                "이번 회차를 끝낸다 (소화기 점검 중이면 정상이다)"
            )
            self.stop_run("외부 취소")
            return

        ok = status == GoalStatus.STATUS_SUCCEEDED
        if not ok:
            # 실패해도 순찰을 멈추지 않는다. 한 구간이 일시적으로 막혔다고
            # 순찰 전체가 정지하면 안 된다. 다음 지점으로 넘어가 복구를 시도한다.
            self.get_logger().warn(f"이동 실패(status={status}). 다음으로 넘어간다")
            self.status("failed, continuing")

        # 시작지점 이동 완료 -> 영상이 준비되면 순찰 시작
        if self.going_to_start:
            self.going_to_start = False
            self.leg = self.build_leg()
            self.leg_pos = 0
            wait = float(self.get_parameter("vision_wait_sec").value)
            if wait > 0 and not self.vision_ready:
                # 여기서 기다리는 이유: 무선 DDS 디스커버리 때문에 안전모 판정이 첫
                # 프레임을 받기까지 16~40초가 걸린다(실측). 그 전에 순찰을 시작하면
                # 첫 바퀴를 눈 감고 돈다. **시작지점에 도착한 뒤** 기다리는 것은
                # 사용자 요구다 — 중간 지점에 멈춰 있으면 고장난 것처럼 보인다.
                self.waiting_vision = True
                self.vision_deadline = time.time() + wait
                self.get_logger().info(
                    f"시작지점 도착 — 카메라 영상이 들어올 때까지 최대 {wait:.0f}초 기다린다")
                self.status("waiting for camera")
                return
            self.get_logger().info("시작지점 도착 — 순찰을 시작한다")
            self.finish_step(dwell=True)
            return

        if bool(self.get_parameter("stop_at_corners").value):
            self.leg_pos += 1
            if self.leg_pos < len(self.leg):
                # 같은 구간 안의 다음 지점 — 대기 없이 바로 진행
                self.busy = False
                return
            self.end_of_leg()
        else:
            self.end_of_leg()

    def end_of_leg(self):
        """한 구간(왕복 한쪽 / 한 바퀴)을 마쳤을 때."""
        self.n_lap += 1
        self.lap_in_run += 1
        limit = self.run_limit
        progress = f"{self.lap_in_run}/{limit}" if limit > 0 else f"{self.lap_in_run}"

        if self.mode == "roundtrip":
            self.forward = not self.forward
            self.get_logger().info(
                f"{'끝' if not self.forward else '시작'} 지점 도달 -> 방향 전환 "
                f"(이번 회차 {progress}, 누적 {self.n_lap}회)"
            )
        else:
            self.get_logger().info(
                f"한 바퀴 완주 (이번 회차 {progress}, 누적 {self.n_lap}회)"
            )
        self.leg = []
        self.leg_pos = 0

        if limit > 0 and self.lap_in_run >= limit:
            # 마지막 바퀴의 끝은 곧 시작지점이다. 여기서 멈추면 다음 회차가
            # 시작지점 이동 없이 바로 순찰을 시작할 수 있다.
            self.stop_run(f"{self.lap_in_run}바퀴 완주")
            return

        self.status(f"lap {progress} done")
        self.finish_step(dwell=True)

    def finish_step(self, dwell=False):
        d = float(self.get_parameter("dwell_sec").value)
        if dwell and d > 0:
            self.dwell_timer = self.create_timer(d, self.release_dwell)
        else:
            self.busy = False

    def release_dwell(self):
        if self.dwell_timer is not None:
            self.dwell_timer.cancel()
            self.dwell_timer = None
        self.busy = False


def main():
    rclpy.init()
    try:
        node = PatrolNode()
    except SystemExit:
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # launch 파일로 띄우면 종료 시 SIGTERM 이 오고 rclpy 가
        # ExternalShutdownException 을 던진다. Ctrl+C 와 똑같이 취급한다.
        pass
    finally:
        node.cancel_current()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
