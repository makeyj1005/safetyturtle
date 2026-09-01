#!/usr/bin/env python3
"""fire_node.py — 화재 감지 시 비상대피구로 이동하며 알림음을 낸다.

[VM에서 실행]
  ros2 run patrol_core fire_node

  하드웨어(아두이노 불꽃센서) 없이 테스트 — 수동으로 화재를 발생/해제시킨다:
  ros2 topic pub --once /fire/trigger std_msgs/msg/Bool "{data: true}"   # 발생
  ros2 topic pub --once /fire/trigger std_msgs/msg/Bool "{data: false}"  # 해제(수동)

[입력]  /fire/trigger  (Bool)  True=화재 감지, False=수동 해제
        (아두이노 불꽃센서 시리얼 입력은 TODO — 하드웨어 연결 전. 연결되면 시리얼
        읽기 결과를 이 토픽에 발행하는 브리지만 추가하면 이 노드는 그대로 재사용된다)
[출력]  /fire/status   (String)  진단·웹 대시보드용
        /patrol/hold   (Bool)    True — 순찰을 세우고 이 노드가 Nav2 를 넘겨받는다
        /sound         (service) 반복 경고음
        음성("화재입니다 대피하세요")은 로봇 스피커로 ssh+espeak-ng 재생(미검증 —
        restricted_node 와 같은 방식, 스피커 연결 전)
        logs/events.sqlite 에 화재 발생 기록(해제는 안 남긴다 — restricted_node 와
        같은 정책, 2026-08-29 사용자 결정)

[왜 /patrol/hold 로 순찰을 세우고 직접 Nav2 를 쓰는가]
patrol_node 는 held=True 인 동안 tick() 에서 바로 return 하고 새 목표를 안 보낸다
(patrol_node.py 참고) — 그래서 fire_node 가 같은 시간에 또 다른 NavigateToPose 목표를
보내도 서로 충돌하지 않는다(HANDOFF2 절대규칙 7 — 같은 목표를 두 노드가 다투지 않게).
helmet_node 의 hold 는 "그 자리에 서기"만 하지만, 화재는 **어디론가 이동**해야 하므로
같은 신호를 쓰되 fire_node 가 직접 목표를 보내는 점이 다르다.

[비상대피구 좌표]
maps/evacuation_points.yaml 의 첫 번째 지점으로 간다(가장 가까운 지점을 고르는 것은
TODO — 대피구가 여러 개 생기면 추가할 것). 아직 등록된 지점이 없으면 이동은 포기하고
그 자리에서 알림음만 낸다(경고 없이 조용히 넘어가지 않는다 — 반드시 로그로 남긴다).
    python3 ~/vibe/ex1/tools/save_waypoint.py --file ~/vibe/ex1/maps/evacuation_points.yaml --name 비상구1

[알림음을 왜 계속 반복하나]
연기로 시야가 막혀도 소리로는 로봇을 따라올 수 있어야 한다. 그래서 도착 여부와
무관하게 화재가 해제될 때까지 alarm_interval_sec 마다 부저+음성을 반복한다 —
이동 중에도 울려야 사람이 로봇을 "따라갈" 방향을 소리로 짐작할 수 있다.

TODO: 아두이노 불꽃센서 시리얼 프로토콜 확정 후 pyserial 로 읽어 /fire/trigger 에
      발행하는 브리지 추가(tools/fire_arduino/ 스케치와 맞출 것). 지금은 이 토픽을
      직접 publish 해서 수동/시험용으로 쓴다.
"""
import math
import os
import subprocess
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from turtlebot3_msgs.srv import Sound

from patrol_core import event_log

EX1 = os.path.join(os.path.expanduser("~"), "vibe", "ex1")
DEFAULT_WAYPOINT_FILE = os.path.join(EX1, "maps", "evacuation_points.yaml")

# turtlebot3_msgs/srv/Sound: 0 OFF/1 ON/2 LOW_BATTERY/3 ERROR/4,5 BUTTON
# helmet_node=1,3 / restricted_node=4 와 안 겹치게 5(BUTTON2)를 쓴다.
SOUND_FIRE = 5

SSH_OPTS = ["-o", "ConnectTimeout", "8", "-o", "BatchMode=yes",
           "-o", "StrictHostKeyChecking=accept-new"]


class FireNode(Node):
    def __init__(self):
        super().__init__("fire_node")

        # 아두이노 시리얼 — TODO, 하드웨어 연결 전. 지금은 안 쓴다(/fire/trigger 로 대신 시험).
        self.declare_parameter("serial_port", "/dev/ttyACM1")
        self.declare_parameter("baud_rate", 9600)

        self.declare_parameter("waypoint_file", DEFAULT_WAYPOINT_FILE)
        # 매핑·Nav2 준비가 안 된 시연(2026-09-07 발표처럼 현장 재매핑할 시간이 없는
        # 경우)에서는 false 로 두면 Nav2 시도 자체를 건너뛰고 바로 알람만 낸다.
        # 그동안 대피구까지 이동은 대시보드 키보드 조작으로 대신한다는 시나리오.
        self.declare_parameter("use_nav", True)

        self.declare_parameter("alarm_interval_sec", 3.0)
        self.declare_parameter("sound", True)
        self.declare_parameter("sound_value", SOUND_FIRE)
        self.declare_parameter("sound_repeat", 3)
        self.declare_parameter("sound_wait_sec", 15.0)

        self.declare_parameter("voice_enabled", True)
        self.declare_parameter("voice_text", "화재입니다 대피하세요")
        self.declare_parameter("voice_lang", "ko")
        self.declare_parameter("robot_host", "rpi@192.168.0.73")

        self.declare_parameter("db_path", event_log.DEFAULT_DB)
        self.declare_parameter("quiet", False)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_status = self.create_publisher(String, "/fire/status", qos)
        self.pub_hold = self.create_publisher(Bool, "/patrol/hold", qos)
        self.cli_sound = self.create_client(Sound, "/sound")
        self.cli_pose = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.create_subscription(Bool, "/fire/trigger", self.on_trigger, qos)

        self.active = False
        self.goal_handle = None
        self.alarm_timer = None

        self.get_logger().warn(
            "fire_node 시작 — 아두이노 불꽃센서 미연결(TODO). "
            "/fire/trigger 로 수동 시험할 것"
        )

    # ---------------- 트리거 ----------------
    def on_trigger(self, msg: Bool):
        if bool(msg.data):
            self.start_fire()
        else:
            self.clear_fire()

    def start_fire(self):
        if self.active:
            return
        self.active = True
        self.get_logger().error("화재 감지 — 비상대피구로 이동, 순찰을 넘겨받는다")
        self.status("FIRE — 대피구로 이동 중")
        event_log.log_event("fire_node", "alert", "화재 감지 — 대피구 이동 시작",
                           db_path=str(self.get_parameter("db_path").value))

        hold = Bool()
        hold.data = True
        self.pub_hold.publish(hold)

        if bool(self.get_parameter("use_nav").value):
            self.go_to_evacuation_point()
        else:
            self.get_logger().warn(
                "use_nav:=false — Nav2 자율이동 생략, 대피구까지는 수동(대시보드 키보드"
                " 조작)으로 이동한다는 시나리오. 알람만 계속 낸다"
            )
            self.status("FIRE — 수동 대피 유도 중 (알람)")

        interval = float(self.get_parameter("alarm_interval_sec").value)
        self.alarm_timer = self.create_timer(interval, self.on_alarm_tick)
        self.on_alarm_tick()      # 첫 알림은 바로

    def clear_fire(self):
        if not self.active:
            return
        self.active = False
        self.get_logger().warn("화재 해제(수동) — 순찰로 돌아간다")
        self.status("clear (수동 해제)")

        if self.alarm_timer is not None:
            self.alarm_timer.cancel()
            self.alarm_timer = None
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

        hold = Bool()
        hold.data = False
        self.pub_hold.publish(hold)

    # ---------------- 이동 ----------------
    def load_evacuation_point(self):
        path = str(self.get_parameter("waypoint_file").value)
        if not os.path.exists(path):
            return None
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        wps = data.get("waypoints", [])
        return wps[0] if wps else None

    def go_to_evacuation_point(self):
        w = self.load_evacuation_point()
        if w is None:
            self.get_logger().error(
                "비상대피구 좌표가 없다 — maps/evacuation_points.yaml 에 등록할 것 "
                "(tools/save_waypoint.py --file ...). 이동은 못 하고 제자리에서 알림음만 낸다"
            )
            return
        if not self.cli_pose.server_is_ready():
            self.get_logger().warn("Nav2 액션 서버 대기 중...", throttle_duration_sec=5.0)

        goal = NavigateToPose.Goal()
        p = PoseStamped()
        p.header.frame_id = "map"
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(w["x"])
        p.pose.position.y = float(w["y"])
        yaw = float(w.get("yaw", 0.0))
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        goal.pose = p

        name = w.get("name", "대피구")
        self.get_logger().warn(f"-> [{name}] 로 이동 시작")
        fut = self.cli_pose.send_goal_async(goal)
        fut.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("대피구 이동 목표가 거절됐다 — Nav2 상태 확인할 것")
            return
        self.goal_handle = handle
        result_fut = handle.get_result_async()
        result_fut.add_done_callback(self.on_arrived)

    def on_arrived(self, future):
        if not self.active:
            return         # 이동 중에 해제됐다
        self.get_logger().warn("대피구 도착 — 알림음 계속 (해제 전까지)")
        self.status("도착 — 알림음 유지")

    # ---------------- 알림 ----------------
    def on_alarm_tick(self):
        if not self.active:
            return
        self.beep()
        self.speak()

    def beep(self):
        if not bool(self.get_parameter("sound").value):
            return
        wait = float(self.get_parameter("sound_wait_sec").value)
        if not self.cli_sound.wait_for_service(timeout_sec=wait):
            self.get_logger().warn(f"/sound 가 {wait:.0f}초 안에 안 보인다 (bringup 확인)")
            return
        value = int(self.get_parameter("sound_value").value)
        reps = max(int(self.get_parameter("sound_repeat").value), 1)
        for i in range(reps):
            req = Sound.Request()
            req.value = value
            self.cli_sound.call_async(req)
            if i < reps - 1:
                time.sleep(0.2)

    def speak(self):
        if not bool(self.get_parameter("voice_enabled").value):
            return
        text = str(self.get_parameter("voice_text").value)
        host = str(self.get_parameter("robot_host").value)
        cmd = f'espeak-ng -v {self.get_parameter("voice_lang").value} "{text}" 2>&1'
        try:
            r = subprocess.run(["ssh", *SSH_OPTS, host, cmd],
                               capture_output=True, text=True, timeout=10)
            if r.returncode != 0:
                self.get_logger().warn(
                    f"음성 재생 실패(스피커 미연결일 수 있음): {r.stderr.strip()[:200]}",
                    throttle_duration_sec=30.0)
        except subprocess.SubprocessError as e:                 # noqa: BLE001
            self.get_logger().warn(f"음성 재생 ssh 실패: {e}", throttle_duration_sec=30.0)

    def status(self, text):
        m = String()
        m.data = text
        self.pub_status.publish(m)


def main():
    rclpy.init()
    node = FireNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
