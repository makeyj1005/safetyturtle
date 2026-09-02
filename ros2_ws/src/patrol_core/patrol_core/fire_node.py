#!/usr/bin/env python3
"""fire_node.py — 화재 감지 시 비상대피구로 이동하며 알림음을 낸다.

[VM에서 실행]
  ros2 run patrol_core fire_node

  수동으로도 화재를 발생/해제시킬 수 있다(하드웨어 없이 시험할 때도 이걸로):
  ros2 topic pub --once /fire/trigger std_msgs/msg/Bool "{data: true}"   # 발생
  ros2 topic pub --once /fire/trigger std_msgs/msg/Bool "{data: false}"  # 해제(수동)

[입력]  /fire/trigger    (Bool)  True=화재 감지, False=수동 해제 (수동/시험용)
        /flame/detected  (Bool)  로봇 GPIO23 불꽃센서 실측값(gpio_io_node.py 가 낸다,
                                 2026-09-02 연동 — 아두이노 아니라 Pi GPIO 직결이었다).
                                 두 입력 모두 같은 on_trigger 로 들어간다 — 센서든
                                 사람이든 True 를 보내면 화재 대응을 시작한다.
                                 ⚠️ 해제(False)도 센서가 자동으로 보낼 수 있다는 뜻이다
                                 — 실측 오탐이 걱정되면 gpio_io_node 의
                                 flame_debounce_n 을 늘릴 것(기본 3=0.3초 연속).
[출력]  /fire/status   (String)  진단·웹 대시보드용
        /patrol/hold   (Bool)    True — 순찰을 세우고 이 노드가 Nav2 를 넘겨받는다
        /sound         (service) 반복 경고음
        음성은 로봇 스피커(I2S, MAX98357A)로 ssh+mpg123 재생 — sounds/fire_alarm.mp3
        (edge-tts 로 미리 생성, tools/make_voice_lines.py 참고)
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

SSH_OPTS = ["-o", "ConnectTimeout=8", "-o", "BatchMode=yes",
           "-o", "StrictHostKeyChecking=accept-new"]


class FireNode(Node):
    def __init__(self):
        super().__init__("fire_node")

        self.declare_parameter("waypoint_file", DEFAULT_WAYPOINT_FILE)
        # 매핑·Nav2 준비가 안 된 시연(2026-09-07 발표처럼 현장 재매핑할 시간이 없는
        # 경우)에서는 false 로 두면 Nav2 시도 자체를 건너뛰고 바로 알람만 낸다.
        # 그동안 대피구까지 이동은 대시보드 키보드 조작으로 대신한다는 시나리오.
        self.declare_parameter("use_nav", True)

        self.declare_parameter("alarm_interval_sec", 3.0)
        # 한 번 화재로 판정하면 최소 이 시간(초)은 경보를 유지한다.
        # [왜 필요한가 — 2026-09-02 실측] 불꽃센서는 불을 조금만 흔들거나 각도가
        # 바뀌면 감지/미감지를 1초 안에 오간다. 그대로 두면 음성 안내(14초)가 한 문장도
        # 못 끝내고 끊기고, 부저·LED·LCD 가 깜빡거려 오히려 상황 파악을 방해했다.
        # 실제 화재경보기도 한 번 울리면 바로 멈추지 않는다(래치) — 같은 이유다.
        self.declare_parameter("min_alarm_sec", 20.0)
        # 부저를 이 간격(초)마다 뒤집어 삐-삐- 경보음을 만든다. 능동부저는 톤을
        # 바꿀 수 없어서(3.3V 고정) 켜고 끄는 리듬으로만 "경보" 느낌을 낸다.
        self.declare_parameter("buzzer_beep_sec", 0.25)
        self.declare_parameter("buzzer_enabled", True)
        self.declare_parameter("sound", True)
        self.declare_parameter("sound_value", SOUND_FIRE)
        self.declare_parameter("sound_repeat", 3)
        self.declare_parameter("sound_wait_sec", 15.0)

        # 음성 안내 — 로봇 speaker_node 에 /speaker/play 로 이름만 보낸다.
        # 실제 파일(sounds/fire_alarm.mp3)과 볼륨은 speaker_node 가 관리한다.
        # 문구를 바꾸려면 tools/make_voice_lines.py 를 고쳐 mp3 만 다시 만들면 된다.
        self.declare_parameter("voice_enabled", True)
        self.declare_parameter("voice_sound", "fire_alarm")

        self.declare_parameter("db_path", event_log.DEFAULT_DB)
        self.declare_parameter("quiet", False)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_status = self.create_publisher(String, "/fire/status", qos)
        self.pub_hold = self.create_publisher(Bool, "/patrol/hold", qos)
        # 로봇 GPIO 부저(24)·LED(25) — gpio_io_node.py 가 받아서 실제 핀을 켠다.
        # 사양서: "능동부저를 경보음처럼 울리고 LED를 켠다" (2026-09-02 연동)
        self.pub_buzzer = self.create_publisher(Bool, "/buzzer/set", qos)
        self.pub_led = self.create_publisher(Bool, "/led/set", qos)
        self.pub_speaker = self.create_publisher(String, "/speaker/play", qos)
        self.cli_sound = self.create_client(Sound, "/sound")
        self.cli_pose = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.create_subscription(Bool, "/fire/trigger", self.on_trigger, qos)
        # 로봇 GPIO23 불꽃센서 실측값 — gpio_io_node.py 가 낸다. 수동 트리거와 같은
        # 콜백으로 합친다(둘 다 True 를 보내면 화재 대응 시작).
        self.create_subscription(Bool, "/flame/detected", self.on_trigger, qos)

        self.active = False
        self.goal_handle = None
        self.alarm_timer = None
        self.buzzer_timer = None
        self.buzzer_on = False
        self.fire_started_at = 0.0
        self.clear_timer = None
        # 센서가 마지막으로 보고한 값. 최소 경보 시간이 끝났을 때 "아직도 불이
        # 있는지"를 이걸로 판단한다.
        self.flame_now = False

        self.get_logger().info(
            "fire_node 시작 — /flame/detected(GPIO 불꽃센서) 와 /fire/trigger(수동) 둘 다 듣는다"
        )

    # ---------------- 트리거 ----------------
    def on_trigger(self, msg: Bool):
        self.flame_now = bool(msg.data)
        if self.flame_now:
            self.start_fire()
        else:
            self.clear_fire()

    def start_fire(self):
        if self.active:
            return
        self.active = True
        self.fire_started_at = time.time()
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

        # LED 는 화재 동안 계속 켜 둔다(사양서: "LED를 켠다").
        self.set_led(True)

        interval = float(self.get_parameter("alarm_interval_sec").value)
        self.alarm_timer = self.create_timer(interval, self.on_alarm_tick)
        self.on_alarm_tick()      # 첫 알림은 바로

        # 부저는 짧게 껐다 켜서 경보음처럼 만든다(능동부저라 톤 조절이 안 되므로
        # 켜고 끄는 리듬으로 "경보" 느낌을 낸다 — 2026-09-02 실측으로 확인).
        if bool(self.get_parameter("buzzer_enabled").value):
            bz = float(self.get_parameter("buzzer_beep_sec").value)
            self.buzzer_timer = self.create_timer(bz, self.on_buzzer_tick)

    def clear_fire(self, force=False):
        if not self.active:
            return

        # 최소 경보 시간이 안 지났으면 아직 끄지 않는다 — 남은 시간 뒤에 다시 본다.
        min_sec = float(self.get_parameter("min_alarm_sec").value)
        elapsed = time.time() - self.fire_started_at
        if not force and elapsed < min_sec:
            if self.clear_timer is None:
                remain = max(min_sec - elapsed, 0.1)
                self.clear_timer = self.create_timer(remain, self.on_min_alarm_done)
                self.get_logger().info(
                    f"불꽃이 사라졌지만 최소 경보시간({min_sec:.0f}초)까지 "
                    f"{remain:.0f}초 더 유지한다"
                )
            return

        self.active = False
        if self.clear_timer is not None:
            self.clear_timer.cancel()
            self.clear_timer = None
        self.get_logger().warn("화재 해제 — 순찰로 돌아간다")
        self.status("clear (해제)")

        if self.alarm_timer is not None:
            self.alarm_timer.cancel()
            self.alarm_timer = None
        if self.buzzer_timer is not None:
            self.buzzer_timer.cancel()
            self.buzzer_timer = None
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

        # 부저·LED 를 반드시 끈다 — 여기서 빠뜨리면 화재가 끝나도 계속 울린다.
        self.set_buzzer(False)
        self.set_led(False)

        hold = Bool()
        hold.data = False
        self.pub_hold.publish(hold)

    def on_min_alarm_done(self):
        """최소 경보 시간이 끝났다 — 불이 아직 있으면 계속, 없으면 진짜로 해제한다."""
        if self.clear_timer is not None:
            self.clear_timer.cancel()
            self.clear_timer = None
        if self.flame_now:
            self.get_logger().warn("최소 경보시간 지났지만 불꽃이 여전히 감지된다 — 경보 유지")
            return
        self.clear_fire(force=True)

    # ---------------- 부저·LED ----------------
    def set_buzzer(self, on):
        m = Bool()
        m.data = bool(on)
        self.pub_buzzer.publish(m)
        self.buzzer_on = bool(on)

    def set_led(self, on):
        m = Bool()
        m.data = bool(on)
        self.pub_led.publish(m)

    def on_buzzer_tick(self):
        """buzzer_beep_sec 마다 부저를 뒤집어 삐-삐- 경보음을 만든다."""
        if not self.active:
            return
        self.set_buzzer(not self.buzzer_on)

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
        """로봇 speaker_node 에 재생 요청만 보낸다(ssh 안 씀 — speaker_node.py 주석 참고)."""
        if not bool(self.get_parameter("voice_enabled").value):
            return
        m = String()
        m.data = str(self.get_parameter("voice_sound").value)
        self.pub_speaker.publish(m)

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
