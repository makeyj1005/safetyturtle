#!/usr/bin/env python3
"""inspect_node.py — 소화기 지점을 돌며 압력계를 점검하는 노드.

[VM에서 실행]  (Nav2 가 켜져 있고 RViz 에서 2D Pose Estimate 를 한 상태여야 한다)
  스케줄러와 함께 (기다렸다가 /inspect/start 로 시작):
    ros2 run patrol_core inspect_node --ros-args -p robot_host:=192.168.0.67

  지금 바로 한 번 점검:
    ros2 run patrol_core inspect_node --ros-args -p auto_start:=true -p robot_host:=192.168.0.67

  로봇 없이 흐름만 확인 (사진은 저장된 것을 쓴다. tools/fake_nav2.py 를 함께 띄운다):
    ros2 run patrol_core inspect_node --ros-args -p auto_start:=true \
      -p photo_dir:=/home/ohinseop/vibe/ex1/logs/shots_final -p sound:=false

[입력]  maps/fire_extinguisher_points.yaml  점검할 지점 좌표 (save_waypoint.py --file 로 등록)
        maps/gauge_calib.yaml + gauge_ref_<이름>.png   정상 기준
        /inspect/start (Bool)  True 면 한 회차 시작. patrol_scheduler 가 보낸다.
[출력]  /navigate_to_pose (action)  지점별 이동
        /inspect/status (String)    진행 상황. 끝나면 "done (...)" — 스케줄러가 이걸 본다
        /sound (service)            판정을 소리로 알린다
                                    정상   1번 음을 여러 번 이어 울려 **길게**
                                    그 외  2번 음 한 번 (짧게)
        logs/inspect_<시각>.csv     지점별 판정 결과 한 줄씩
        logs/inspect_<시각>/        증빙 사진 (판정 표시를 그려서 저장)

[한 지점의 처리]
  Nav2 로 이동 → settle_sec 정지 대기 → 로봇에서 사진 shots 장 → gauge.judge() 로 판정
  → 이상/부재면 부저 + 기록 → 판정불가면 자세를 조금 틀어 다시 시도(max_attempts)

[마지막에 순찰 시작지점으로 돌아간다 — return_to_start]
지점을 다 본 뒤 순찰 웨이포인트의 첫 지점으로 이동하고 나서 done 을 발행한다.
스케줄러는 done 시각부터 쉼을 세므로, 쉬는 시간을 시작지점에서 보내게 되고 다음
순찰이 이동 없이 바로 시작된다. 안 그러면 소화기 앞에서 쉬다가 순찰 시작 후에야
시작지점으로 이동해서, 그 이동 시간이 순찰 회차에 붙는다.

[왜 사진을 스트리밍하지 않는가]
1640x1232 프레임은 177KB 라 무선 DDS 로는 사실상 오지 않는다(20초에 0장).
로봇에서 로컬 저장 후 tar 스트림으로 가져온다 — patrol_core/shot_grab.py 참고.

[주행 중에는 카메라를 끈다]
카메라 스트림이 켜진 채로 주행하면 무선이 포화돼 /scan·/odom 이 밀리고 Nav2 가
모든 목표를 거절한다(실측 ping 1000ms, HANDOFF "함정 12"). 그래서 manage_camera=true
(기본)면 사진을 찍는 그 순간에만 로봇에서 카메라를 띄우고 바로 내린다.
카메라를 손으로 미리 띄워두고 쓰려면 manage_camera:=false 로 준다.

[판정불가일 때 — 사진으로 각도를 계산해 다시 간다 (2026-08-01)]
자세가 틀어지면 게이지가 화면에서 옆으로 밀린다. gauge.judge 가 그 어긋난 픽셀을
각도로 환산해 주므로(`yaw_fix_deg`), **같은 좌표에 그만큼 돌린 yaw 로 목표를 다시
보낸다.** 지도·AMCL 이 아니라 카메라로 닫는 되먹임이라 위치추정 오차와 무관하다.
게이지를 아예 못 찾아 계산이 안 되면 `retry_yaw_deg`(기본 8°)로 좌우를 훑는다.

**회전 방향은 스스로 배운다.** 돌렸는데 오프셋이 오히려 커지면(20px 넘게) 부호를
뒤집고 그 뒤로는 그 방향을 쓴다. 부호를 한 번 반대로 넣어 로봇이 대상에서 멀어지는
쪽으로 돌았던 일이 있어서다 — 카메라가 후면에 달려 있어 방향을 머리로 따지면 틀리기
쉽다. 되먹임으로 확인하면 처음 부호가 틀려도 한 시도만 손해 본다.

제자리 회전을 직접 시키지 않는 이유: /cmd_vel 을 발행해야 하는데 그건 절대 규칙
위반이다(/cmd_vel 은 cmd_vel_mux 만 발행한다). 회전은 Nav2 에 맡긴다.

[여러 장을 어떻게 합치는가 — 나쁜 쪽을 택한다]
한 장이라도 이상/부재/판정불가면 지점 판정을 그쪽으로 한다. 반사나 흔들림으로
한 장이 흐렸을 뿐이어도 사람이 한 번 더 보는 편이 낫다. 놓치는 것이 더 나쁘다.
CSV 에는 장별 판정을 모두 남겨 어느 쪽이 몇 장이었는지 볼 수 있게 한다.

[동작 흐름을 작업 스레드에서 도는 이유]
사진 취득이 ssh 왕복 10~20초로 오래 걸린다. 콜백 안에서 하면 그 동안 액션 결과
콜백이 처리되지 않아 이동이 끝난 걸 알 수 없다. 그래서 회차 전체를 작업 스레드에서
돌리고 메인 스레드는 계속 spin 한다. 작업 스레드는 spin 하지 않고 future.done() 만
확인하므로(대기 중에는 sleep) executor 가 하나여도 콜백이 밀리지 않는다.

MultiThreadedExecutor 를 쓰지 않는 이유(실측 2026-07-31): 그 executor 의 스레드풀이
종료를 붙잡아 **SIGINT 를 줘도 프로세스가 죽지 않았다.** launch 로 띄우면 종료가
멈춰버린다. 단일 executor 로 충분하므로 patrol_node 와 같은 rclpy.spin 을 쓴다.
"""
import csv
import math
import os
import sys
import threading
import time

import cv2
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, Spin
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from turtlebot3_msgs.srv import Sound

from patrol_core import gauge, shot_grab

EX1 = os.path.join(os.path.expanduser("~"), "vibe", "ex1")
DEFAULT_POINT_FILE = os.path.join(EX1, "maps", "fire_extinguisher_points.yaml")
DEFAULT_CALIB_FILE = os.path.join(EX1, "maps", "gauge_calib.yaml")
DEFAULT_LOG_DIR = os.path.join(EX1, "logs")
DEFAULT_PATROL_WP = os.path.join(EX1, "maps", "patrol_waypoints.yaml")

# 나쁜 쪽이 앞이다. 여러 장 / 여러 시도를 합칠 때 이 순서로 고른다.
SEVERITY = ["부재", "이상", "판정불가", "정상"]
# 사람을 불러야 하는 판정(이상 쪽). 정상과 소리를 달리해 귀로 구분한다.
ALARM = ("부재", "이상", "판정불가")
# turtlebot3_msgs/srv/Sound 의 값: 0 OFF / 1 ON / 2 LOW_BATTERY / 3 ERROR / 4,5 BUTTON
# 2026-08-01 사용자 결정: 정상 1, 이상 2. 파라미터로 바꿀 수 있다.
# ⚠️ 값 2 는 OpenCR 이 저전압일 때 스스로 내는 음과 같다 — 배터리가 11V 아래로
#    떨어지면 같은 소리가 나므로, 경보가 잦으면 전압부터 확인할 것.
SOUND_OK = 1
SOUND_BAD = 2

# 보정 루프 상수
K_MIN, K_MAX = 8.0, 150.0        # 환산값이 이 범위 밖이면 잘못 잰 것으로 본다
K_MIN_MOVE_DEG = 3.0             # 이만큼은 돌려야 재는 의미가 있다(정지 오차 ±2.9°)
PROBE_MAX = 12.0                 # 아직 못 쟀을 때 한 번에 돌릴 최대 각도
STEP_MAX = 30.0                  # 재고 나서도 한 번에 이 이상은 안 돈다

CSV_COLS = ["time", "point", "status", "attempt", "shots", "per_shot",
            "change", "angle", "score", "offset_px", "yaw_fix_deg",
            "contrast", "reflection", "red", "nav", "image", "reason"]


def home_pose(wps):
    """복귀할 자세를 정한다. 반환 (웨이포인트, yaw) 또는 (None, 0.0).

    방향은 다음 웨이포인트 쪽으로 잡는다 — patrol_node 가 순찰을 시작할 때 향하는
    방향과 같아서, 다음 순찰이 제자리 회전 없이 바로 출발한다.
    """
    if not wps:
        return None, 0.0
    home = wps[0]
    yaw = float(home.get("yaw", 0.0))
    if len(wps) > 1:
        dx = float(wps[1]["x"]) - float(home["x"])
        dy = float(wps[1]["y"]) - float(home["y"])
        if math.hypot(dx, dy) > 1e-3:
            yaw = math.atan2(dy, dx)
    return home, yaw


def worst(statuses):
    for s in SEVERITY:
        if s in statuses:
            return s
    return "판정불가"


class InspectNode(Node):
    def __init__(self):
        super().__init__("inspect_node")

        self.declare_parameter("point_file", DEFAULT_POINT_FILE)
        self.declare_parameter("calib_file", DEFAULT_CALIB_FILE)
        self.declare_parameter("log_dir", DEFAULT_LOG_DIR)
        self.declare_parameter("auto_start", False)
        # 사진을 가져올 로봇. IP 는 재부팅마다 바뀐다(HANDOFF 참고).
        self.declare_parameter("robot_host", "rpi@192.168.0.67")
        self.declare_parameter("topic", "/camera/image_raw/compressed")
        self.declare_parameter("domain", "3")
        self.declare_parameter("shots", 3)
        # 도착 직후엔 아직 흔들린다. 멈춘 뒤 이만큼 기다렸다 찍는다.
        self.declare_parameter("settle_sec", 2.0)
        # 사진 찍는 동안만 로봇에서 카메라를 띄운다(주행 중 무선 포화 방지).
        self.declare_parameter("manage_camera", True)
        # CSI 카메라의 libcamera 인덱스. USB 웹캠을 빼둔 상태에서는 0 이다.
        # (웹캠을 꽂으면 1 로 밀린다 — HANDOFF 함정 9. 그때는 camera_index:=1 로 준다.
        #  자동 전환은 넣지 않는다: 사진 한 번이 30초 이상이라 헛시도가 비싸다.)
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("camera_fps", 3.0)
        self.declare_parameter("jpeg_quality", 85)
        self.declare_parameter("grab_wait_sec", 15.0)
        # ssh 한 번의 상한(초). 0 이면 자동(대기시간 + 여유). 무선이 느리면 키운다.
        self.declare_parameter("grab_timeout_sec", 0.0)
        # 판정불가일 때 자세를 틀어 다시 시도하는 횟수(첫 시도 포함).
        self.declare_parameter("max_attempts", 4)
        self.declare_parameter("retry_yaw_deg", 8.0)
        # 사진으로 계산한 보정각에 곱할 값. 1.0 은 과보정 위험이 있다(위 설명).
        self.declare_parameter("fix_gain", 0.5)
        self.declare_parameter("sound", True)
        # 정상일 때 소리를 몇 번 이어 울릴지. /sound 서비스는 길이를 정할 수 없어서
        # (값 0~5 의 고정 음만 낸다) 여러 번 붙여 울려 "길게" 들리도록 한다.
        # 시연에서 **로봇이 압력계를 정상으로 인식했다**는 걸 소리만 듣고 알 수 있게
        # 하려는 것이다 — 이상음(짧은 경보)과 길이로 구분된다.
        self.declare_parameter("sound_ok_repeat", 4)
        self.declare_parameter("sound_gap_sec", 0.25)
        # 어떤 음을 쓸지. 0 OFF / 1 ON / 2 LOW_BATTERY / 3 ERROR / 4,5 BUTTON
        self.declare_parameter("sound_ok_value", SOUND_OK)
        self.declare_parameter("sound_bad_value", SOUND_BAD)
        self.declare_parameter("sound_bad_repeat", 1)
        # /sound 를 기다리는 시간(초). 무선 DDS 는 원격 서비스 발견에 실측 11.5초가
        # 걸렸다 — 짧게 잡으면 부저가 있는데도 "없다"고 넘어간다.
        self.declare_parameter("sound_wait_sec", 15.0)
        self.declare_parameter("nav_timeout_sec", 120.0)
        # 로봇 없이 시험할 때: 이 폴더의 사진을 판정한다(ssh 를 쓰지 않는다).
        self.declare_parameter("photo_dir", "")
        # 도착한 뒤 **카메라를 보며** 자세를 맞출지. 지도 각도(2D Pose Estimate 오차)에
        # 의존하지 않는 유일한 방법이다 — 아래 align_by_camera 설명 참고.
        self.declare_parameter("visual_align", True)
        # 이 안에 들면 맞은 것으로 본다. **판정 문턱(max_offset_px 120)과 같게 둔다** —
        # 그보다 더 정밀하게 맞추려 하면 측정 잡음(±100~200px)을 쫓아 진동한다
        # (2026-08-01 실측: 60px 로 뒀더니 dx 가 +60/-112/+138/-62 로 부호만 뒤집혔다).
        self.declare_parameter("align_tol_px", 120.0)
        self.declare_parameter("align_max_steps", 6)     # 정렬 시도 횟수 상한
        self.declare_parameter("align_scan_deg", 15.0)   # 게이지를 못 찾을 때 훑는 각도
        # 이보다 작은 보정은 하지 않는다(회전 오차·측정 잡음보다 작으면 진동만 한다).
        # ⚠️ 허용오차보다 작아야 한다. 실측 1도당 101px 이라 120px = 1.2° 인데
        # 여기를 2° 로 뒀더니 "1.7° 돌려야 하는데 최소치 미만이라 안 돈다"는 교착에
        # 빠졌다(2026-08-01). 1.0° = 약 100px 로 잡는다.
        self.declare_parameter("align_min_step_deg", 1.0)
        # 회전 뒤 흔들림이 가라앉기를 기다리는 시간. 바로 찍으면 위치가 튄다.
        self.declare_parameter("align_settle_sec", 1.5)
        # 점검이 끝나면 순찰 시작지점으로 돌아간 뒤에 done 을 발행한다.
        # 그래야 쉬는 시간을 시작지점에서 보내고, 다음 순찰이 이동 없이 바로 시작된다.
        # (돌아가지 않으면 patrol_node 의 goto_start_first 가 순찰 시작 후에 그 이동을
        #  하게 되어, 이동 시간이 순찰 회차 시간에 붙는다)
        self.declare_parameter("return_to_start", True)
        self.declare_parameter("patrol_waypoint_file", DEFAULT_PATROL_WP)

        self.points = self.load_points()
        self.calib = {}
        cal_path = str(self.get_parameter("calib_file").value)
        if os.path.exists(cal_path):
            self.calib = gauge.load_calib(cal_path)
        else:
            self.get_logger().error(
                f"캘리브레이션 파일이 없다: {cal_path} — tools/gauge_calib.py 로 먼저 등록할 것"
            )

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_status = self.create_publisher(String, "/inspect/status", qos)
        self.create_subscription(Bool, "/inspect/start", self.on_start, qos)
        self.cli_pose = ActionClient(self, NavigateToPose, "navigate_to_pose")
        # 상대 회전. 지도(AMCL) 각도와 무관하게 "지금 자세에서 N도" 돌린다 —
        # 카메라로 자세를 맞출 때 이게 핵심이다(아래 align_by_camera 설명).
        self.cli_spin = ActionClient(self, Spin, "spin")
        self.cli_sound = self.create_client(Sound, "/sound")

        self.worker = None
        self.stopping = False
        # 1도 돌리면 화면이 몇 px 움직이는지(부호 포함). 한 번 재면 계속 쓴다 —
        # 다음 지점·다음 회차는 첫 시도부터 정확한 보정을 준다.
        self.k_est = None
        self.goal_handle = None
        self.n_run = 0

        self.get_logger().info(
            f"점검 노드 시작. 지점 {len(self.points)}개, 기준 {len(self.calib)}개, "
            f"사진 {int(self.get_parameter('shots').value)}장/지점, "
            f"로봇={self.get_parameter('robot_host').value}"
        )
        for p in self.points:
            name = p.get("name", "?")
            mark = "" if name in self.calib else "  ⚠ 기준 없음(판정불가로 기록된다)"
            self.get_logger().info(
                f"  {name}  x={p['x']:.3f} y={p['y']:.3f} "
                f"yaw={math.degrees(float(p.get('yaw', 0.0))):.1f}°{mark}"
            )
        if str(self.get_parameter("photo_dir").value):
            self.get_logger().warn(
                f"photo_dir 모드 — 사진을 로봇에서 가져오지 않고 "
                f"{self.get_parameter('photo_dir').value} 의 것을 판정한다(시험용)"
            )
        if not self.points:
            self.get_logger().error(
                f"점검 지점이 없다: {self.get_parameter('point_file').value} — "
                "tools/save_waypoint.py --file 로 등록할 것"
            )

        # 부저가 보이는지 미리 알아둔다. 이상을 찾은 순간에야 "부저가 없다"를
        # 알게 되면 늦다(경보가 필요한 바로 그때 못 울린다).
        # 같은 노드가 두 개 떠 있으면 둘 다 /inspect/start 를 받아 각자 Nav2 목표를
        # 보내고 같은 CSV 에 겹쳐 쓴다(2026-08-01 실측: 시험용으로 띄워둔 노드가
        # 1시간 넘게 살아있어 회차 하나를 통째로 버렸다. CSV 가 깨져서 알아챘다).
        # 발견에 시간이 걸리므로 조금 뒤에 확인한다.
        self.timer_dup = self.create_timer(15.0, self.check_duplicate)

        # 무선 DDS 는 원격 서비스 발견이 느리고 편차가 크다(실측 0.4초 ~ 30초 이상).
        # 한 번만 보고 "없다"고 하면 있는 부저를 없다고 하게 된다. 여러 번 확인한다.
        # 첫 확인을 8초로 앞당긴 이유: 순찰 첫 회차(기본 10초 뒤)보다 **먼저** 부저
        # 상태를 알려주기 위해서다. 못 찾아도 조용히 다시 보므로 손해가 없다.
        self.probe_left = 5
        self.timer_probe = self.create_timer(8.0, self.probe_sound)

        if bool(self.get_parameter("auto_start").value):
            self.begin("auto_start")
        else:
            self.get_logger().info("/inspect/start 에 true 가 오면 시작한다")

    def check_duplicate(self):
        """같은 이름의 노드가 또 있는지 한 번 확인하고 알린다."""
        self.timer_dup.cancel()
        try:
            names = [n for n, _ in self.get_node_names_and_namespaces()]
        except Exception:                                      # noqa: BLE001
            return
        n = names.count("inspect_node")
        if n > 1:
            self.get_logger().error(
                f"⚠️ inspect_node 가 {n} 개 떠 있다 — 둘 다 목표를 보내고 기록을 "
                "겹쳐 쓴다. 하나만 남길 것: "
                "`ps -eo pid,etime,cmd | grep [i]nspect_node` 로 찾아 오래된 것을 kill -9"
            )

    def probe_sound(self):
        """부저가 보이는지 확인해 알려준다. 보일 때까지 몇 번 더 본다."""
        if not bool(self.get_parameter("sound").value):
            self.timer_probe.cancel()
            return
        if self.cli_sound.service_is_ready():
            self.timer_probe.cancel()
            self.get_logger().info("부저(/sound) 확인됨")
            return
        self.probe_left -= 1
        if self.probe_left > 0:
            if self.timer_probe.timer_period_ns < 12_000_000_000:
                self.timer_probe.timer_period_ns = 12_000_000_000   # 이후 12초 간격
            return          # 아직 발견 중일 수 있다. 조용히 다시 본다
        self.timer_probe.cancel()
        self.get_logger().warn(
            "부저(/sound)가 한참 보이지 않는다 — 로봇 bringup 이 떠 있는지 확인. "
            "이상을 찾아도 소리가 나지 않는다"
        )

    # ---------------- 준비 ----------------
    def load_points(self):
        path = str(self.get_parameter("point_file").value)
        if not os.path.exists(path):
            return []
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        # save_waypoint.py 가 쓰면 waypoints, 손으로 쓰면 points 일 수 있다.
        return data.get("waypoints") or data.get("points") or []

    def status(self, text):
        m = String()
        m.data = text
        try:
            self.pub_status.publish(m)
        except Exception as e:                                 # noqa: BLE001
            # 종료 중이면 노드가 이미 정리돼 발행이 실패할 수 있다. 죽을 이유는 아니다.
            self.get_logger().debug(f"상태 발행 실패({text}): {e}")

    # ---------------- 시작·종료 ----------------
    def on_start(self, msg: Bool):
        if not msg.data:
            return
        self.begin("/inspect/start")

    def begin(self, why):
        if self.worker is not None and self.worker.is_alive():
            self.get_logger().warn(f"이미 점검 중이다 — {why} 무시")
            return
        self.n_run += 1
        self.get_logger().warn(f"[{self.n_run}] 소화기 점검 시작 ({why})")
        self.worker = threading.Thread(target=self.run_guard, daemon=True)
        self.worker.start()

    def run_guard(self):
        """회차를 돈다. 어떻게 끝나든 done 을 발행한다.

        done 을 못 내면 스케줄러가 다음 순번을 잡지 못해 순찰까지 멈춘다
        (HANDOFF "함정 8"). 그래서 예외가 나도 반드시 발행한다.
        """
        summary = "실패"
        try:
            summary = self.run_once()
        except Exception as e:                                  # noqa: BLE001
            self.get_logger().error(f"점검 중 예외: {type(e).__name__}: {e}")
            summary = f"예외 {type(e).__name__}"
        finally:
            self.get_logger().warn(f"소화기 점검 종료 — {summary}")
            self.status(f"done ({summary})")

    def stop(self):
        """종료·중단. 작업 스레드가 스스로 빠져나오게 하고 잠깐 기다린다."""
        self.stopping = True
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        w = self.worker
        if w is not None and w.is_alive():
            # ssh 왕복 중이면 그것까지는 못 끊는다. 그건 데몬 스레드로 두고 나간다.
            w.join(timeout=5.0)

    # ---------------- 회차 ----------------
    def run_once(self):
        if not self.points:
            return "지점 없음"

        stamp = time.strftime("%m%d_%H%M%S")
        log_dir = str(self.get_parameter("log_dir").value)
        shot_dir = os.path.join(log_dir, f"inspect_{stamp}")
        csv_path = os.path.join(log_dir, f"inspect_{stamp}.csv")
        os.makedirs(shot_dir, exist_ok=True)
        self.status(f"start ({len(self.points)}지점)")

        counts = {}
        with open(csv_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=CSV_COLS)
            w.writeheader()
            for pt in self.points:
                if self.stopping:
                    break
                row = self.do_point(pt, shot_dir)
                w.writerow(row)
                f.flush()          # 중간에 죽어도 여기까지는 남는다
                counts[row["status"]] = counts.get(row["status"], 0) + 1

        self.get_logger().info(f"기록: {csv_path}")
        parts = [f"{k} {v}" for k, v in counts.items()]
        done = sum(counts.values())
        summary = f"{done}/{len(self.points)}지점" + (f", {', '.join(parts)}" if parts else "")

        back = self.return_home()
        if back is not None:
            summary += ", 복귀 " + ("완료" if back else "실패")
        return summary

    def return_home(self):
        """순찰 시작지점으로 돌아간다. 반환 True/False, 안 할 상황이면 None.

        여기서 돌아가 두면 쉬는 시간(rest_min)을 시작지점에서 보내게 되어
        다음 순찰이 이동 없이 바로 시작된다. done 은 복귀까지 마친 뒤에 나간다 —
        스케줄러가 done 시각부터 쉼을 세기 때문이다.
        """
        if not bool(self.get_parameter("return_to_start").value) or self.stopping:
            return None
        wps = self.load_patrol_waypoints()
        if not wps:
            self.get_logger().warn(
                f"순찰 웨이포인트가 없어 복귀를 건너뛴다: "
                f"{self.get_parameter('patrol_waypoint_file').value}"
            )
            return None

        home, yaw = home_pose(wps)
        self.status("returning to start")
        name = home.get("name", "시작지점")
        ok = self.goto(name, home, yaw, label=f"순찰 시작지점({name})으로 복귀")
        if ok:
            self.get_logger().info("순찰 시작지점 복귀 완료 — 여기서 다음 순찰을 기다린다")
        else:
            self.get_logger().warn(
                "시작지점 복귀에 실패했다 — 다음 순찰이 시작될 때 patrol_node 가 "
                "시작지점으로 먼저 이동한다(goto_start_first)"
            )
        return ok

    def load_patrol_waypoints(self):
        path = str(self.get_parameter("patrol_waypoint_file").value)
        if not os.path.exists(path):
            return []
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        return data.get("waypoints") or []

    def do_point(self, pt, shot_dir):
        """지점 하나를 점검한다. CSV 한 줄을 돌려준다."""
        name = pt.get("name", "?")
        base_yaw = float(pt.get("yaw", 0.0))
        cal = self.calib.get(name)
        row = {c: "" for c in CSV_COLS}
        row.update(time=time.strftime("%H:%M:%S"), point=name, status="판정불가",
                   nav="-", attempt=0, shots=0)

        if cal is None:
            row["reason"] = (f"'{name}' 의 캘리브레이션이 없다 — "
                             "tools/gauge_calib.py 로 등록할 것")
            self.get_logger().error(f"{name}: {row['reason']}")
            self.alarm(name, row["status"])
            return row

        ref = gauge.load_ref(str(self.get_parameter("calib_file").value), name)
        max_attempts = max(int(self.get_parameter("max_attempts").value), 1)
        fallback = math.radians(float(self.get_parameter("retry_yaw_deg").value))
        off = 0.0          # 이번 시도에 base_yaw 에 더할 각도
        prev_dx = None     # 직전 시도의 화면 오프셋
        prev_off = 0.0     # 그때 준 각도 (둘로 1도당 몇 px 인지 계산한다)

        for i in range(1, max_attempts + 1):
            if self.stopping:
                row["reason"] = "중단됨"
                return row
            nav_ok = self.goto(name, pt, base_yaw + off,
                               label=f"{name} 시도 {i}/{max_attempts}"
                               + (f" (yaw {math.degrees(off):+.1f}°)" if off else ""))
            row["nav"] = "ok" if nav_ok else "fail"
            # 이동이 실패해도 사진은 찍어본다 — 오차가 작아 판정이 되는 경우가 있다.
            time.sleep(float(self.get_parameter("settle_sec").value))

            self.status(f"judging {name}")
            # 카메라로 자세를 맞춘 뒤 찍는다. 정렬하는 동안 카메라를 켜둬야
            # 한 장이 7~9초로 빨라진다(껐다 켜면 30~40초).
            aligned = None
            cam_on = False
            manage = bool(self.get_parameter("manage_camera").value)
            use_align = (bool(self.get_parameter("visual_align").value)
                         and not str(self.get_parameter("photo_dir").value))
            if use_align and manage:
                w_px, h_px = cal.get("resolution", [1640, 1232])
                cam_on = shot_grab.start_camera(
                    str(self.get_parameter("robot_host").value), w_px, h_px,
                    fps=float(self.get_parameter("camera_fps").value),
                    jpeg_quality=int(self.get_parameter("jpeg_quality").value),
                    index=int(self.get_parameter("camera_index").value),
                )
                if cam_on:
                    time.sleep(9.0)          # 기동 대기
                else:
                    self.get_logger().warn(f"{name}: 카메라를 켜지 못해 정렬을 건너뛴다")
            try:
                if cam_on:
                    aligned = self.align_by_camera(name, cal, ref, shot_dir)
                shots, log = self.take_shots(name, cal, shot_dir, f"{name}_a{i}",
                                             manage=False if cam_on else None)
            finally:
                if cam_on:
                    if not shot_grab.stop_camera(
                            str(self.get_parameter("robot_host").value)):
                        self.get_logger().error(
                            "로봇에 카메라가 남았다 — 주행 전에 끌 것: "
                            "ssh <로봇> \"pkill -9 -f camera_node\""
                        )
            if aligned is False:
                self.get_logger().warn(f"{name}: 자세 정렬 실패 — 그래도 판정해본다")
            row["attempt"], row["shots"] = i, len(shots)
            if not shots:
                row["status"], row["reason"] = "판정불가", f"사진을 얻지 못했다 — {log}"
                self.get_logger().error(f"{name}: {row['reason']}")
                continue

            res, per, img_path = self.judge_shots(shots, cal, ref, shot_dir, name, i)
            row.update(
                status=res["status"], per_shot="/".join(per), image=img_path,
                reason=res.get("reason", ""),
                change="" if res.get("change") is None else f"{res['change']:.2f}",
                angle="" if res.get("angle") is None else f"{res['angle']:.0f}",
                score="" if res.get("score") is None else f"{res['score']:.2f}",
                offset_px="" if res.get("offset") is None
                          else f"{res['offset'][0]:+.0f},{res['offset'][1]:+.0f}",
                yaw_fix_deg="" if res.get("yaw_fix_deg") is None
                            else f"{res['yaw_fix_deg']:+.1f}",
                contrast="" if res.get("contrast") is None else f"{res['contrast']:.0f}",
                reflection="" if res.get("reflection") is None else f"{res['reflection']:.1f}",
                red="" if res.get("red") is None else f"{res['red']:.1f}",
            )
            lg = self.get_logger()
            (lg.info if res["status"] == "정상" else lg.error)(
                f"{name}: {res['status']} — {res.get('reason','')} "
                f"[{'/'.join(per)}]"
            )
            # 판정불가만 다시 시도한다. 이상·부재는 다시 봐도 같고, 알려야 한다.
            if res["status"] != "판정불가":
                break
            if i >= max_attempts:
                break

            # ---- 다음 시도의 각도를 정한다 ----
            # 화면에서 벗어난 픽셀(dx)을 각도로 바꿔 같은 좌표에 yaw 만 다른 목표를
            # 다시 보낸다. 지도·AMCL 이 아니라 카메라로 닫는 되먹임이다.
            #
            # **환산값(1도에 몇 px)과 그 부호를 추측하지 않고 직접 잰다.** 한 번
            # 돌려보면 (돌린 각도, 화면이 움직인 픽셀)이 나오므로 그 자리에서
            # k = Δdx / Δ각도 를 계산할 수 있다. 부호도 여기에 같이 들어 있다 —
            # 카메라가 후면에 달려 방향을 머리로 따지면 틀리기 쉽고, 실제로 두 번
            # 틀렸다. 재보면 틀릴 일이 없다.
            dx = None if res.get("offset") is None else res["offset"][0]
            if dx is None:
                # 게이지를 아예 못 찾아 계산할 수 없다. 정해진 각도로 훑는다.
                off = fallback if i % 2 else -fallback
                self.get_logger().warn(
                    f"{name}: 판정불가(게이지를 못 찾음) — "
                    f"yaw {math.degrees(off):+.0f}° 로 훑어본다"
                )
                continue

            d_off = math.degrees(off - prev_off)
            if prev_dx is not None and abs(d_off) >= K_MIN_MOVE_DEG:
                cand = (dx - prev_dx) / d_off
                if K_MIN <= abs(cand) <= K_MAX:
                    self.k_est = cand
                    self.get_logger().info(
                        f"{name}: 화면 이동을 재보니 1도당 {cand:+.0f}px "
                        f"({d_off:+.1f}° 돌렸더니 {dx - prev_dx:+.0f}px 움직였다)"
                    )
            prev_dx, prev_off = dx, off

            if self.k_est is not None:
                step = math.radians(max(-STEP_MAX, min(STEP_MAX, -dx / self.k_est * 0.9)))
                why = f"실측 {self.k_est:+.0f}px/도"
            else:
                # 아직 못 쟀다. 기본 환산값으로 절반만 — 이 시도가 곧 측정용 탐침이다.
                blind = -dx / float(gauge.DEF["px_per_deg"]) * 0.5
                step = math.radians(max(-PROBE_MAX, min(PROBE_MAX, blind)))
                why = "환산값 추정치(탐침)"
            off += step
            self.get_logger().warn(
                f"{name}: 자세가 어긋났다(dx {dx:+.0f}px) — {math.degrees(step):+.1f}° "
                f"돌려 다시 본다 [{why}, 누적 {math.degrees(off):+.1f}°]"
            )

        self.status(f"{name} 압력계 {row['status']}")
        self.alarm(name, row["status"])
        self.report(name, row)
        return row

    def report(self, name, row):
        """사람이 읽을 결과 한 줄. 로그가 길어 판정이 묻히지 않게 따로 찍는다."""
        st = row["status"]
        head = {
            "정상": f"{name} 압력계 정상 — 압력이 기준과 같다",
            "이상": f"{name} 압력계 이상 — 사람이 확인해야 한다",
            "부재": f"{name} 소화기가 보이지 않는다 — 사람이 확인해야 한다",
            "판정불가": f"{name} 판정 불가 — 사람이 확인해야 한다",
        }.get(st, f"{name} {st}")

        detail = []
        if row.get("change") not in (None, ""):
            detail.append(f"변화량 {row['change']} (문턱 0.90)")
        if row.get("angle"):
            detail.append(f"바늘 {row['angle']}°")
        if row.get("score") not in (None, ""):
            detail.append(f"정합 {row['score']}")
        if row.get("per_shot"):
            detail.append(f"사진 {row['per_shot']}")

        lg = self.get_logger()
        line = "─" * 46
        out = (f"\n{line}\n  [{st}]  {head}\n"
               + (f"  근거: {' / '.join(detail)}\n" if detail else "")
               + (f"  증빙: {row['image']}\n" if row.get("image") else "")
               + line)
        (lg.info if st == "정상" else lg.error)(out)

    # ---------------- 이동 ----------------
    def goto(self, name, pt, yaw, label=""):
        if not self.wait_server():
            return False
        p = PoseStamped()
        p.header.frame_id = "map"
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(pt["x"])
        p.pose.position.y = float(pt["y"])
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        goal = NavigateToPose.Goal()
        goal.pose = p

        self.get_logger().info(f"-> {label or name} (x={pt['x']:.3f} y={pt['y']:.3f})")
        self.status(f"moving to {name}")
        fut = self.cli_pose.send_goal_async(goal)
        if not self.wait(fut, 10.0):
            self.get_logger().error("목표 전송 응답이 없다 — Nav2 상태를 확인할 것")
            return False
        handle = fut.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(
                "Nav2 가 목표를 거절했다 — 확인할 것: "
                "① turtlebot3_node 가 살아있는지(`ros2 topic hz /odom`) "
                "② RViz 에서 2D Pose Estimate 를 했는지 / Localization 이 active 인지"
            )
            self.status("goal rejected")
            return False

        self.goal_handle = handle
        rf = handle.get_result_async()
        ok = self.wait(rf, float(self.get_parameter("nav_timeout_sec").value))
        self.goal_handle = None
        if not ok:
            self.get_logger().warn("이동이 시간 안에 끝나지 않았다 — 목표를 취소한다")
            handle.cancel_goal_async()
            return False
        st = rf.result().status
        if st != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f"이동 실패(status={st}) — 그래도 찍어본다")
            return False
        return True

    def wait_server(self):
        if self.cli_pose.server_is_ready():
            return True
        self.get_logger().warn(
            "Nav2 액션 서버 대기 중 — Nav2 가 켜져 있고 RViz 에서 "
            "2D Pose Estimate 를 했는지 확인 (로봇 없이 시험하려면 tools/fake_nav2.py)"
        )
        t0 = time.time()
        while not self.cli_pose.server_is_ready():
            if self.stopping or time.time() - t0 > 30.0:
                self.get_logger().error("Nav2 액션 서버가 없다 — 이 지점을 건너뛴다")
                return False
            time.sleep(0.2)
        return True

    def wait(self, fut, timeout):
        """future 를 기다린다. spin 은 executor 가 하므로 여기서는 확인만 한다."""
        t0 = time.time()
        while not fut.done():
            if self.stopping or time.time() - t0 > timeout:
                return False
            time.sleep(0.05)
        return True

    # ---------------- 카메라로 자세 맞추기 ----------------
    def spin(self, deg, timeout=40.0):
        """제자리에서 deg 만큼 돌린다(+는 왼쪽/CCW). 지도 각도와 무관한 상대 회전.

        왜 Nav2 목표(yaw)가 아니라 /spin 인가: 목표 yaw 는 지도 좌표라
        2D Pose Estimate 가 몇 도 틀어지면 실제 자세도 그만큼 틀어진다. 오늘 겪은
        각도 문제의 대부분이 그것이었다. /spin 은 "지금부터 N도"라서 지도가 틀려도
        정확히 그만큼 돈다. 회전은 Nav2 가 하므로 /cmd_vel 규칙도 지킨다.
        """
        if not self.cli_spin.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("/spin 액션이 없다 — 카메라 정렬을 건너뛴다")
            return False
        goal = Spin.Goal()
        goal.target_yaw = float(math.radians(deg))
        goal.time_allowance = Duration(sec=int(timeout))
        fut = self.cli_spin.send_goal_async(goal)
        if not self.wait(fut, 10.0) or fut.result() is None or not fut.result().accepted:
            self.get_logger().warn(f"회전 {deg:+.1f}° 요청이 거절됐다")
            return False
        self.goal_handle = fut.result()
        rf = self.goal_handle.get_result_async()
        ok = self.wait(rf, timeout)
        self.goal_handle = None
        return ok

    def align_by_camera(self, name, cal, ref, shot_dir):
        """게이지가 화면 기준 자리에 오도록 제자리에서 돌린다.

        [왜 필요한가 — 2026-08-01]
        저장된 yaw 는 지도 좌표라 2D Pose Estimate 가 틀어진 만큼 실제 자세가 틀어진다.
        실측에서 그것 때문에 소화기를 1m 밖에서 찍거나 화면 밖으로 놓쳤다. 지도를
        믿는 대신 **카메라로 직접 맞추면** 위치추정 오차와 무관해진다.

        한 걸음:
          사진 1장 → 화면 전체에서 게이지를 찾는다 → 기준 자리에서 몇 px 벗어났나
          → 그만큼을 각도로 바꿔 /spin 으로 돌린다 → 다시 찍는다
        1도에 몇 px 인지는 **돌려본 결과로 직접 잰다**(부호까지). 카메라가 후면에
        달려 있어 방향을 머리로 따지면 틀리기 때문이다.
        게이지를 아예 못 찾으면 align_scan_deg 씩 좌우로 훑는다.

        카메라는 이 함수를 부르기 전에 켜져 있어야 한다(한 장에 7~9초로 빨라진다).
        """
        tol = float(self.get_parameter("align_tol_px").value)
        steps = int(self.get_parameter("align_max_steps").value)
        scan = float(self.get_parameter("align_scan_deg").value)
        ex, ey = gauge.roi_center(cal)
        prev = None           # (dx, 그때까지 돈 각도)
        turned = 0.0
        scan_seq = [scan, -2 * scan, 3 * scan, -4 * scan]   # 좌우로 넓혀가며 훑는다

        for i in range(1, steps + 1):
            if self.stopping:
                return False
            files, log = self.take_shots(name, cal, shot_dir, f"{name}_align{i}",
                                         n=1, manage=False)
            if not files:
                self.get_logger().warn(f"{name}: 정렬용 사진 실패 — {log[:60]}")
                return False
            img = cv2.imread(files[0])
            os.remove(files[0])
            found = gauge.find_anywhere(img, ref) if img is not None else None
            if found is None or found[0] < 0.45:
                if not scan_seq:
                    self.get_logger().warn(f"{name}: 훑어봐도 게이지를 못 찾았다")
                    return False
                d = scan_seq.pop(0)
                self.get_logger().warn(
                    f"{name}: 게이지가 화면에 없다(정합 {0 if not found else found[0]:.2f}) "
                    f"— {d:+.0f}° 돌려 훑는다"
                )
                if not self.spin(d):
                    return False
                turned += d
                time.sleep(float(self.get_parameter("align_settle_sec").value))
                prev = None
                continue

            sc, cx, cy, scale = found
            dx = cx - ex
            if abs(dx) <= tol:
                self.get_logger().info(
                    f"{name}: 자세 맞음 (dx {dx:+.0f}px, 정합 {sc:.2f}, "
                    f"총 {turned:+.1f}° 돌렸다)"
                )
                return True

            # 직전 회전의 반응으로 1도당 몇 px 인지 잰다(부호 포함).
            if prev is not None and abs(turned - prev[1]) >= K_MIN_MOVE_DEG:
                cand = (dx - prev[0]) / (turned - prev[1])
                if K_MIN <= abs(cand) <= K_MAX:
                    self.k_est = cand
                    self.get_logger().info(
                        f"{name}: 회전 반응 실측 — 1도당 {cand:+.0f}px"
                    )
            prev = (dx, turned)

            k = self.k_est
            if k is not None:
                d = max(-STEP_MAX, min(STEP_MAX, -dx / k * 0.9))
                why = f"실측 {k:+.0f}px/도"
            else:
                d = max(-PROBE_MAX, min(PROBE_MAX,
                                        -dx / float(gauge.DEF["px_per_deg"]) * 0.5))
                why = "환산 추정치(탐침)"

            min_step = float(self.get_parameter("align_min_step_deg").value)
            if abs(d) < min_step:
                # 회전 오차·측정 잡음보다 작은 보정이다. 더 해봐야 진동만 한다.
                self.get_logger().info(
                    f"{name}: dx {dx:+.0f}px 는 {d:+.1f}° 짜리 — 회전 정밀도 아래라 "
                    f"여기서 멈춘다 (총 {turned:+.1f}° 돌렸다)"
                )
                return True

            self.get_logger().info(
                f"{name}: dx {dx:+.0f}px → {d:+.1f}° 돌린다 [{why}] ({i}/{steps})"
            )
            if not self.spin(d):
                return False
            turned += d
            # 회전 직후엔 로봇이 흔들린다. 가라앉기를 기다렸다 찍는다.
            time.sleep(float(self.get_parameter("align_settle_sec").value))
        self.get_logger().warn(f"{name}: {steps}번 돌려도 자세가 안 맞았다")
        return False

    # ---------------- 사진·판정 ----------------
    def take_shots(self, name, cal, shot_dir, prefix, n=None, manage=None, quiet=False):
        """사진을 받는다. manage=False 면 이미 켜져 있는 카메라를 그대로 쓴다."""
        n = int(self.get_parameter("shots").value) if n is None else n
        photo_dir = str(self.get_parameter("photo_dir").value)
        if photo_dir:
            return shot_grab.local_grab(photo_dir, n=n)

        if manage is None:
            manage = bool(self.get_parameter("manage_camera").value)
        cam = None
        if manage:
            w, h = cal.get("resolution", [1640, 1232])
            cam = shot_grab.camera_args(
                w, h,
                fps=float(self.get_parameter("camera_fps").value),
                jpeg_quality=int(self.get_parameter("jpeg_quality").value),
                index=int(self.get_parameter("camera_index").value),
            )
        t0 = time.time()
        files, log = shot_grab.grab(
            str(self.get_parameter("robot_host").value),
            os.path.join(shot_dir, "raw"), n=n,
            topic=str(self.get_parameter("topic").value),
            domain=str(self.get_parameter("domain").value),
            camera=cam, wait_sec=float(self.get_parameter("grab_wait_sec").value),
            prefix=prefix,
            timeout=(float(self.get_parameter("grab_timeout_sec").value) or None),
        )
        if not quiet or not files:
            self.get_logger().info(
                f"{name}: 사진 {len(files)}장 ({time.time() - t0:.1f}초)"
                + ("" if files else f" — {log}")
            )
        if shot_grab.CAM_LEFT in log:
            # 남은 카메라는 다음 순찰의 Nav2 를 무너뜨린다. 묻히면 안 되는 경고다.
            self.get_logger().error(
                "로봇에 카메라가 남아있다 — 주행 전에 끌 것: "
                "ssh <로봇> \"pkill -f camera_node\""
            )
        return files, log

    def judge_shots(self, files, cal, ref, shot_dir, name, attempt):
        """장마다 판정하고 나쁜 쪽으로 합친다. 대표 사진 한 장을 그려 저장한다."""
        results, per = [], []
        for p in files:
            img = cv2.imread(p)
            if img is None:
                per.append("읽기실패")
                continue
            r = gauge.judge(img, cal, ref)
            r["_img"], r["_path"] = img, p
            results.append(r)
            per.append(r["status"])
        if not results:
            return ({"status": "판정불가", "reason": "사진을 읽을 수 없다"}, per, "")

        pick = worst([r["status"] for r in results])
        # 같은 판정이 여러 장이면 변화량이 가장 큰 것을 증빙으로 남긴다(가장 뚜렷한 장).
        cand = [r for r in results if r["status"] == pick]
        rep = max(cand, key=lambda r: (r.get("change") or 0.0))
        out_path = os.path.join(shot_dir, f"{name}_a{attempt}_{pick}.png")
        try:
            cv2.imwrite(out_path, gauge.annotate(rep["_img"], rep, cal.get("roi")))
        except cv2.error as e:
            self.get_logger().warn(f"증빙 사진 저장 실패: {e}")
            out_path = rep["_path"]
        rep = dict(rep)
        rep.pop("_img", None)
        rep.pop("_path", None)
        rep["status"] = pick
        return rep, per, out_path

    # ---------------- 부저 ----------------
    def alarm(self, name, status):
        """판정을 소리로 알린다. 정상과 이상의 소리를 달리한다.

        정상에도 울리는 이유: 로봇을 보고 있지 않아도 **점검이 끝났고 결과가 무엇인지**
        를 그 자리에서 알 수 있다. 소리가 아예 안 나면 "판정이 정상이었는지, 노드가
        죽었는지" 구분이 안 된다.

        정상은 **길게**(1번 음을 sound_ok_repeat 번 이어서), 이상은 **짧게**(2번 음 한 번)
        울린다. 시연에서 로봇이 압력계를 정상으로 인식했다는 걸 소리 길이만으로
        알 수 있게 한 것이다. /sound 서비스에는 길이 인자가 없어 반복으로 만든다.
        """
        bad = status in ALARM
        value = int(self.get_parameter(
            "sound_bad_value" if bad else "sound_ok_value").value)
        label = f"이상음 {value}번" if bad else f"정상음 {value}번"
        if not bool(self.get_parameter("sound").value):
            self.get_logger().warn(f"{name}: {status} (부저 꺼짐)")
            return
        wait = float(self.get_parameter("sound_wait_sec").value)
        if not self.cli_sound.wait_for_service(timeout_sec=wait):
            self.get_logger().warn(
                f"{name}: {status} — /sound 서비스가 {wait:.0f}초 안에 안 보여 "
                "부저를 못 울린다 (로봇 bringup 확인)"
            )
            return
        reps = max(int(self.get_parameter(
            "sound_bad_repeat" if bad else "sound_ok_repeat").value), 1)
        gap = float(self.get_parameter("sound_gap_sec").value)
        lg = self.get_logger()
        (lg.warn if bad else lg.info)(
            f"{name}: {status} — 부저({label}"
            + (f" {reps}회 연속" if reps > 1 else "") + ")"
        )
        fut = None
        for i in range(reps):
            req = Sound.Request()
            req.value = value
            fut = self.cli_sound.call_async(req)
            if i < reps - 1:
                time.sleep(gap)

        # 서비스가 있어도 OpenCR 이 못 울릴 수 있다(모터 전원이 없을 때 등).
        # 응답을 확인해 두지 않으면 "울린 줄 알았는데 안 울린" 걸 모른다.
        def on_done(f):
            try:
                r = f.result()
            except Exception as e:                             # noqa: BLE001
                self.get_logger().warn(f"부저 호출 실패: {e}")
                return
            if r is not None and not r.success:
                self.get_logger().warn(f"부저가 울리지 않았다: {r.message}")

        fut.add_done_callback(on_done)


def main():
    rclpy.init()
    node = InspectNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # launch 종료 시 SIGTERM -> ExternalShutdownException. Ctrl+C 와 동일 취급.
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
