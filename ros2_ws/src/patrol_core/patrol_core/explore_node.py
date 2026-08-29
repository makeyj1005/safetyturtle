#!/usr/bin/env python3
"""
explore_node.py — 미탐사 영역을 자동으로 찾아가며 지도를 채우는 탐사 노드.

[VM에서 실행]  (explore.launch.py 로 cartographer + Nav2 주행부가 떠 있어야 한다)
  ros2 run patrol_core explore_node

  다 채우면 자동으로 멈춘다. 중간에 멈추려면 Ctrl+C.

[원리 — frontier(경계) 탐사]
지도의 각 칸은 세 가지 상태다: 빈공간(흰) / 장애물(검) / 미탐사(회).
"빈공간이면서 미탐사와 맞닿은 칸"을 frontier 라 부른다. 그 지점에 가면
라이다가 미탐사 쪽을 볼 수 있으므로 지도가 채워진다.
  ① /map 을 받아 frontier 를 모두 찾는다
  ② 인접한 것끼리 묶어 덩어리로 만들고, 너무 작은 덩어리는 버린다(노이즈)
  ③ 로봇이 실제로 갈 수 있는(벽에서 충분히 떨어진) 지점을 골라 Nav2 에 보낸다
  ④ 도착하면 지도가 갱신되고 frontier 가 줄어든다. 반복.
  ⑤ 남은 frontier 가 없으면 탐사 완료.

[왜 직접 만들었나]
explore_lite 같은 기성 패키지가 Humble apt 에 없다. 원리가 단순하고
우리 상황(좁은 방, 벽 30cm 여유)에 맞춰 조정할 수 있어 직접 구현했다.

[안전]
로봇이 실제로 움직인다. 라이다 기반 장애물 회피는 Nav2 의 costmap 이 담당하지만,
사람이 지켜보는 상태에서만 돌릴 것.
"""
import math
import sys

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


class ExploreNode(Node):
    def __init__(self):
        super().__init__("explore_node")

        # frontier 덩어리가 이 칸 수보다 작으면 노이즈로 보고 버린다.
        # 지도 해상도 5cm 기준 8칸 = 약 40cm 폭. 그보다 작은 틈은 로봇이 못 들어간다.
        self.declare_parameter("min_frontier_size", 8)
        # 목표 지점이 벽에서 최소 이만큼 떨어져 있어야 한다(m).
        # robot_radius 0.1 + 여유. 너무 크게 잡으면 좁은 곳을 못 채운다.
        self.declare_parameter("min_clearance", 0.25)
        # 이 거리(m) 안의 frontier 는 이미 본 것으로 보고 무시한다. 제자리 맴돌기 방지.
        self.declare_parameter("skip_radius", 0.35)
        # 목표 하나에 허용하는 시간(초). 넘으면 포기하고 다른 frontier 로 간다.
        self.declare_parameter("goal_timeout", 45.0)
        # 연속 실패가 이 횟수를 넘으면 탐사를 종료한다.
        self.declare_parameter("max_failures", 5)

        qos_map = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/map", self.on_map, qos_map)
        self.pub_status = self.create_publisher(
            String, "/explore/status",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
        )

        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map = None
        self.busy = False
        self.goal_handle = None
        self.failures = 0
        self.n_goals = 0
        self.done = False
        self.blacklist = []      # 도달 실패한 지점들 (다시 시도하지 않음)
        self.goal_sent_at = None

        self.create_timer(2.0, self.tick)
        self.get_logger().info("자동 탐사 시작. /map 을 기다린다...")
        self.get_logger().info("미탐사 영역이 없어지면 자동으로 종료한다 (중단은 Ctrl+C)")

    # ---------------- 입력 ----------------
    def on_map(self, msg: OccupancyGrid):
        self.map = msg

    def robot_xy(self):
        """map 좌표계에서 로봇 위치. TF 가 아직 없으면 None."""
        try:
            t = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:
            return None
        return (t.transform.translation.x, t.transform.translation.y)

    def status(self, text):
        m = String()
        m.data = text
        self.pub_status.publish(m)

    # ---------------- frontier 찾기 ----------------
    def find_frontiers(self):
        """(x, y, 크기) 목록을 반환. x,y 는 map 좌표계(m)."""
        m = self.map
        w, h = m.info.width, m.info.height
        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y
        grid = np.array(m.data, dtype=np.int8).reshape(h, w)

        free = grid == 0            # 빈공간
        unknown = grid < 0          # 미탐사 (-1)
        occupied = grid > 50        # 장애물

        # 미탐사와 4방향으로 맞닿은 빈공간 칸 = frontier
        un_pad = np.zeros((h + 2, w + 2), bool)
        un_pad[1:-1, 1:-1] = unknown
        touches_unknown = (
            un_pad[:-2, 1:-1] | un_pad[2:, 1:-1] | un_pad[1:-1, :-2] | un_pad[1:-1, 2:]
        )
        frontier = free & touches_unknown
        if not frontier.any():
            return []

        # 벽에서 충분히 떨어진 곳만 목표로 삼는다.
        # 거리변환: 장애물/미탐사가 아닌 칸에서 가장 가까운 장애물까지의 거리
        try:
            import cv2
            blocked = (occupied | unknown).astype(np.uint8)
            dist = cv2.distanceTransform(1 - blocked, cv2.DIST_L2, 5) * res
            # 인접한 frontier 끼리 묶어 덩어리로
            n, labels, stats, cents = cv2.connectedComponentsWithStats(
                frontier.astype(np.uint8), 8
            )
        except ImportError:
            self.get_logger().error("opencv(cv2) 가 필요하다")
            return []

        min_size = int(self.get_parameter("min_frontier_size").value)
        min_clear = float(self.get_parameter("min_clearance").value)

        out = []
        for i in range(1, n):
            size = int(stats[i, cv2.CC_STAT_AREA])
            if size < min_size:
                continue
            # 덩어리 안에서 벽에서 가장 여유 있는 칸을 대표 지점으로 고른다.
            ys, xs = np.nonzero(labels == i)
            d = dist[ys, xs]
            k = int(np.argmax(d))
            if d[k] < min_clear:
                continue                      # 로봇이 갈 수 없는 좁은 곳
            mx = xs[k] * res + ox
            my = ys[k] * res + oy
            out.append((mx, my, size, float(d[k])))
        return out

    # ---------------- 판단 ----------------
    def tick(self):
        if self.done or self.busy:
            self.check_timeout()
            return
        if self.map is None:
            self.get_logger().warn("/map 대기 중 (cartographer 확인)", throttle_duration_sec=5.0)
            return
        rp = self.robot_xy()
        if rp is None:
            self.get_logger().warn(
                "map->base_link TF 대기 중 (cartographer 가 위치를 잡는 중)",
                throttle_duration_sec=5.0,
            )
            return
        if not self.client.server_is_ready():
            self.get_logger().warn(
                "navigate_to_pose 액션 서버 대기 중 (Nav2 주행부 확인)",
                throttle_duration_sec=5.0,
            )
            return

        fs = self.find_frontiers()
        skip_r = float(self.get_parameter("skip_radius").value)

        # 블랙리스트(도달 실패)와 너무 가까운 것은 제외
        cand = []
        for mx, my, size, clear in fs:
            if any(math.dist((mx, my), b) < skip_r for b in self.blacklist):
                continue
            cand.append((mx, my, size, clear))

        if not cand:
            self.finish("미탐사 영역을 모두 채웠다" if not fs
                        else "남은 미탐사 영역에 도달할 수 없다")
            return

        # 가까운 것부터 간다. 이동 시간을 줄이고 지도가 자연스럽게 이어진다.
        cand.sort(key=lambda c: math.dist((c[0], c[1]), rp))
        mx, my, size, clear = cand[0]
        d = math.dist((mx, my), rp)
        self.get_logger().info(
            f"목표 #{self.n_goals+1}: ({mx:+.2f}, {my:+.2f})  "
            f"거리 {d:.2f}m  크기 {size}칸  벽여유 {clear*100:.0f}cm  "
            f"(남은 후보 {len(cand)}개)"
        )
        self.status(f"exploring {len(cand)} frontiers left")
        self.send_goal(mx, my, rp)

    def check_timeout(self):
        if self.goal_sent_at is None or not self.busy:
            return
        el = (self.get_clock().now() - self.goal_sent_at).nanoseconds / 1e9
        if el > float(self.get_parameter("goal_timeout").value):
            self.get_logger().warn(f"목표 시간 초과({el:.0f}초) — 취소하고 다른 곳으로")
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()

    # ---------------- 이동 ----------------
    def send_goal(self, mx, my, rp):
        goal = NavigateToPose.Goal()
        p = PoseStamped()
        p.header.frame_id = "map"
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(mx)
        p.pose.position.y = float(my)
        # 목표를 향하는 방향으로 두면 도착 시 그쪽(미탐사 방향)을 보게 되어
        # 라이다가 새 영역을 바로 스캔한다.
        yaw = math.atan2(my - rp[1], mx - rp[0])
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        goal.pose = p

        self.busy = True
        self.n_goals += 1
        self.goal_sent_at = self.get_clock().now()
        self.last_goal = (mx, my)
        fut = self.client.send_goal_async(goal)
        fut.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("Nav2 가 목표를 거절 — 블랙리스트에 넣는다")
            self.blacklist.append(self.last_goal)
            self.busy = False
            self.goal_sent_at = None
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future):
        status = future.result().status
        self.goal_handle = None
        self.busy = False
        self.goal_sent_at = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.failures = 0
            self.get_logger().info("도착 — 지도 갱신 대기")
        else:
            # 도달 못 한 지점은 블랙리스트에 넣어 무한 재시도를 막는다.
            self.failures += 1
            self.blacklist.append(self.last_goal)
            self.get_logger().warn(
                f"도달 실패(status={status}) — 건너뛴다 "
                f"(연속 실패 {self.failures}회)"
            )
            if self.failures >= int(self.get_parameter("max_failures").value):
                self.finish(f"연속 {self.failures}회 실패 — 탐사를 중단한다")

    def finish(self, why):
        self.done = True
        self.get_logger().info("=" * 50)
        self.get_logger().info(f"탐사 종료: {why}")
        self.get_logger().info(f"방문한 목표 {self.n_goals}개")
        self.get_logger().info("지도를 저장할 것:")
        self.get_logger().info(
            "  ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "
            "\"{filename: '/home/ohinseop/vibe/ex1/maps/state/explored.pbstream', "
            "include_unfinished_submaps: true}\""
        )
        self.get_logger().info(
            "  ros2 run nav2_map_server map_saver_cli -f "
            "~/vibe/ex1/maps/patrol_map_explored"
        )
        self.get_logger().info("=" * 50)
        self.status("done")


def main():
    rclpy.init()
    node = ExploreNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.goal_handle is not None:
            node.goal_handle.cancel_goal_async()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
