"""
patrol_auto.launch.py — 랜덤 시각 자동 순찰. patrol_node + patrol_scheduler 를 함께 띄운다.

[VM에서 실행]  (먼저 로봇 bringup + nav2_patrol.launch.py + RViz 2D Pose Estimate)
  기본 (켜자마자 1바퀴 -> 1분 쉬고 -> 1~2분 사이 랜덤 시점에 다시 1바퀴):
  ros2 launch ~/vibe/ex1/launch/patrol_auto.launch.py

  운용 주기로 되돌리려면 (1~5분):
  ros2 launch ~/vibe/ex1/launch/patrol_auto.launch.py cycle_min:=5.0

  더 빨리 확인 (30초 쉬고 90초 안에 다시):
  ros2 launch ~/vibe/ex1/launch/patrol_auto.launch.py rest_min:=0.5 cycle_min:=1.5

[역할 분담]
  patrol_scheduler  시각만 안다. 순찰이 끝나면 쉼+랜덤 뒤에 /patrol/enable=True 를 보낸다.
  patrol_node       좌표를 안다. laps 바퀴를 돌면 스스로 멈추고 다음 트리거를 기다린다.
  inspect_node      소화기 지점을 안다. /inspect/start 가 오면 가서 압력계를 보고 온다.
  helmet_node       사람을 본다. 안전모 미착용이면 /patrol/hold 로 순찰을 세운다.
스케줄러는 False 를 보내지 않는다. 멈추는 기준이 바퀴 수이고 그건 patrol_node 만 안다.

[안전모 감지 — 카메라는 항상 한쪽만 켠다]
일반 순찰은 USB 웹캠(사람·안전모), 소화기 점검은 CSI(압력계)를 쓴다. 무선이 병목이라
둘을 동시에 켜면 /scan 이 밀려 Nav2 가 경로를 못 만든다. helmet_node 가 /inspect/start
를 보면 /webcam/enable=False 를 내 로봇의 웹캠 스트림을 내리고, 점검이 끝나면 올린다.
  로봇에서 먼저: python3 ~/launch/webcam_node.py
  기준 등록:     python3 ~/vibe/ex1/tools/helmet_calib.py --grab --name <이름> --select
  세우지 않고 보기만: helmet:=true hold:=false

[소화기 점검을 시키려면]
  ros2 topic pub --once /inspect/request std_msgs/msg/Bool "{data: true}"
즉시 가지 않는다. 돌던 순찰을 끝까지 마치고, 쉼+랜덤 뒤 **다음 순번**을 점검이
가져간다(그 순번에 순찰은 하지 않는다). 순찰과 점검이 둘 다 Nav2 에 목표를 보내므로
한 순번에 하나만 돌려야 서로 목표를 뺏지 않는다. 점검이 끝나면 다시 순찰로 돌아간다.

  ros2 topic echo /patrol/schedule    다음 순번의 시각과 작업 종류
  ros2 topic echo /inspect/status     점검 진행 상황

[켠 뒤 순서 — 한꺼번에 몰지 않는다]
  0초   세 노드 시작, DDS 디스커버리
  8초   inspect_node 가 부저(/sound) 확인 결과를 알린다
  10초  첫 순찰 회차 시작 (start_delay)
켜자마자 전부 동시에 시작하면 무선이 느릴 때 첫 목표가 거절되거나 로그가 뒤엉킨다.
`start_delay:=0` 이면 예전처럼 즉시 시작한다.

[첫 회차는 오래 기다리지 않는다]
켜자마자(start_delay 뒤) 한 회차를 돌고 그 다음부터 랜덤 스케줄러로 넘어간다.
회차당 바퀴 수는 `laps`(기본 1) 로 모든 회차가 같다. 첫 회차만 다르게 하고 싶을 때만
`first_laps` 를 준다(0 이면 안 쓴다). 켜자마자 도는 게 싫으면 `fire_on_start:=false`.

[점검이 끼어들어도 바퀴가 늘지 않는다]
점검이 Nav2 목표를 가져가 순찰이 취소되면 `patrol_node` 는 그 회차를 거기서 끝낸다.
취소된 만큼 나중에 더 돌지 않는다. 점검이 끝나면 로봇은 순찰 시작지점으로 돌아와
쉬고, 다음 순번에 평소대로 `laps` 바퀴를 돈다.

[시각 기준]
다음 시작은 시계가 아니라 **직전 순찰이 끝난 시각**에서 센다:
  순찰 종료 -> rest_min(1분) 은 무조건 쉼 -> cycle_min(5분) 안의 랜덤 시점에 시작
그래서 순찰이 길어져도 쉬는 시간이 사라지지 않는다.

[주의]
- 이 launch 는 Nav2 를 띄우지 않는다. nav2_patrol.launch.py 가 먼저 떠 있어야 하고
  RViz 에서 2D Pose Estimate 를 해야 한다. 안 하면 patrol_node 가 액션 서버를
  기다리며 경고만 반복한다.
- cmd_vel_mux 를 함께 띄우지 말 것. turtlebot3_navigation2 가 /cmd_vel 로 직접 발행한다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARGS = {
    # patrol_node
    "laps": "1",                 # 한 회차에 돌 바퀴 수 (2026-08-01: 2 -> 1)
    "first_laps": "0",           # 첫 회차만 다르게 (0 = 첫 회차도 laps 와 같게)
    "mode": "loop",              # loop | roundtrip
    "stop_at_corners": "true",   # 좁은 방에서는 true (코너를 정확히 돈다)
    "dwell_sec": "1.0",
    # 시작지점에 도착한 뒤 카메라 영상을 기다리는 최대 시간(초).
    # /helmet/ready 가 오면 그 전에 출발한다. start_delay 로 미리 기다릴 필요가 없어졌다.
    "vision_wait": "40.0",
    # patrol_scheduler
    "cycle_min": "2.0",          # 순찰이 끝난 뒤 다음 순찰까지의 최대 시간 (원래 5.0)
    "rest_min": "1.0",           # 끝난 직후 반드시 쉬는 시간
    "fire_on_start": "true",     # 켠 직후 첫 회차를 시작한다 (start_delay 뒤에)
    "start_delay": "10.0",       # 켠 뒤 첫 회차까지 기다리는 시간(초)
    "seed": "0",                 # 0 이면 매번 다름
    # inspect_node — 스케줄러가 점검 순번을 주면 소화기로 간다
    "inspect": "true",           # false 면 순찰만 (점검 예약이 와도 순번을 건너뛴다)
    "robot_host": "rpi@192.168.0.67",   # 사진을 가져올 로봇. IP 는 재부팅마다 바뀐다
    "shots": "3",                # 지점당 사진 장수
    "grab_timeout": "0.0",       # ssh 한 번의 상한(초). 0=자동, 무선 느리면 키운다
    "photo_dir": "",             # 로봇 없이 시험할 때만: 이 폴더의 사진으로 판정한다
    # 점검용 CSI 카메라 인덱스. **웹캠을 꽂으면 CSI 가 1 로 밀린다**(HANDOFF 함정 9).
    # 안전모 감지 때문에 웹캠이 상시 꽂혀 있으므로 기본값이 1 이다. 0 을 주면
    # 웹캠을 잡아 `unsupported pixel format RGB888` 로 카메라 노드가 죽는다.
    # 웹캠을 뺀 상태로 점검만 돌릴 때는 camera_index:=0.
    "camera_index": "1",
    # helmet_node — 순찰 중 안전모 미착용자를 보면 세운다
    "helmet": "true",            # false 면 안전모 감지를 아예 띄우지 않는다
    "hold": "true",              # false 면 판정·기록만 하고 순찰을 세우지는 않는다
    # 판정 방식. hair = 머리카락이 보이면 미착용 / color = 등록한 색이 보이면 착용.
    # 흰 안전모는 hair 도 color 도 실패했다(흰 벽·천장과 같은 색, 2026-08-02 실측 8가지).
    # 안전모에 색을 넣으면(다른 색 안전모 또는 앞면에 색 테이프) color 가 확실하다.
    # 2026-08-03: 안전모 앞면 위쪽에 **초록 테이프**를 붙여 color 로 확정했다.
    # 같은 자리·같은 조명에서 착용/미착용 8장씩 실측한 결과가 완전히 갈렸다:
    #   착용   초록 비율 0.0199~0.0289 (8/8)
    #   미착용 초록 비율 0.0000        (8/8)
    # 테이프는 안전모 위쪽(돔 앞면)에 붙여야 한다 — 아래쪽에 붙이면 돔 그늘에 들어가
    # 채도가 반으로 떨어지고 색상이 파란 쪽으로 밀려(H 80~120, S 35~60) 벽·옷과 섞인다.
    "method": "color",
    # 0.010 — 위 실측의 가운데. 기본 0.25 는 안전모 전체가 색일 때의 값이라
    # 테이프처럼 작은 표식에는 너무 높다(테이프는 머리 영역의 2~3%).
    "helmet_ratio": "0.010",
    # color 방식에서 머리로 볼 영역의 폭(상자 폭 대비). 테이프처럼 작은 표식은
    # 영역을 좁혀야 비율이 살아난다(넓으면 벽 면적에 희석된다).
    "head_width": "0.35",
    # 통과시킨 사람이 안전모를 벗는 경우를 잡는 검사. 최근 6장 중 이만큼이면 즉시 정지.
    "alert_frames": "5",
    # 5초 판단 창의 미착용 확정 기준 — 개수와 "사람 프레임 대비 비율"을 둘 다 넘어야 한다
    "judge_bad_min": "3",
    "judge_bad_ratio": "0.5",
    "judge_sec": "5.0",
    # 부저: 미착용 = 3번(ERROR) 짧게 3연타를 realert 마다 반복 / 착용 = 1번(ON) 1회.
    # 2번(LOW_BATTERY)은 저전압 경고음과 같아 쓰지 않는다.
    "sound_value": "3",
    "sound_repeat": "3",
    "sound_ok_value": "1",
    "realert_sec": "5.0",
    # 사람이 보이면 안전모 여부와 무관하게 먼저 세운다(멈춘 상태에서 판단이 정확하다)
    "hold_on_person": "true",
    "person_frames": "2",        # 사람이 이만큼 연속 보이면 정지
    "ok_frames": "8",            # 착용이 이만큼 연속 확인되면 출발
    "detector": "auto",          # auto | dnn | hog (모델이 있으면 dnn)
    # 부하 조절 (VM 이 2코어라 Nav2 와 CPU 를 다툰다)
    # 판정 빈도. 기본 1(전속) — 3 으로 낮추니 감지가 늦어 제때 서지 않았다.
    "detect_every": "1",         # 사람이 없을 때. 2~3 은 부하가 정말 문제일 때만
    "detect_every_active": "1",  # 사람이 보일 때
    "camera_fps": "0.0",         # 로봇 발행 fps. 0=기본(3.0). 1.5 로 낮추면 무선 절반
    # true 면 판정한 프레임을 전부 사진으로 남긴다(기준을 맞추거나 오판을 볼 때).
    # 기본 false 는 미착용만 남긴다 — 착용 오판은 그때 안 보여서 원인 파악이 막혔다.
    "save_all": "false",
    # 보고서·시연 영상용. 판정 사건만 남기고 주기 보고·정지 세부 로그를 접는다.
    "quiet": "false",
    # true 면 카메라 영상과 판정을 창으로 보여준다(화면이 있는 터미널에서만).
    # 창 자체는 가볍다(추론은 어차피 돈다) — 무거운 건 화면 녹화 프로그램이다.
    "view": "false",
}


def typed(name, kind):
    """launch 인자를 노드 파라미터 타입에 맞춰 넘긴다.

    타입을 명시하지 않으면 문자열에서 추론하므로, dwell_sec:=1 처럼 쓰면
    float 파라미터에 int 가 들어가 노드가 타입 오류로 죽는다.
    """
    return ParameterValue(LaunchConfiguration(name), value_type=kind)


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()]
        + [
            Node(
                package="patrol_core",
                executable="patrol_node",
                name="patrol_node",
                output="screen",
                parameters=[{
                    # 스케줄러가 시작시킨다. 켜자마자 돌면 랜덤 순찰의 의미가 없다.
                    "auto_start": False,
                    "laps_per_run": typed("laps", int),
                    "first_laps": typed("first_laps", int),
                    "mode": typed("mode", str),
                    "stop_at_corners": typed("stop_at_corners", bool),
                    "dwell_sec": typed("dwell_sec", float),
                    "vision_wait_sec": typed("vision_wait", float),
                    "quiet": typed("quiet", bool),
                }],
            ),
            Node(
                package="patrol_core",
                executable="patrol_scheduler",
                name="patrol_scheduler",
                output="screen",
                parameters=[{
                    "cycle_min": typed("cycle_min", float),
                    "rest_min": typed("rest_min", float),
                    "fire_on_start": typed("fire_on_start", bool),
                    "start_delay_sec": typed("start_delay", float),
                    "seed": typed("seed", int),
                }],
            ),
            Node(
                package="patrol_core",
                executable="inspect_node",
                name="inspect_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("inspect")),
                parameters=[{
                    # 스케줄러가 /inspect/start 로 시작시킨다.
                    "auto_start": False,
                    "robot_host": typed("robot_host", str),
                    "shots": typed("shots", int),
                    "grab_timeout_sec": typed("grab_timeout", float),
                    "photo_dir": typed("photo_dir", str),
                    "camera_index": typed("camera_index", int),
                }],
            ),
            Node(
                package="patrol_core",
                executable="helmet_node",
                name="helmet_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("helmet")),
                parameters=[{
                    "hold": typed("hold", bool),
                    "method": typed("method", str),
                    "helmet_ratio": typed("helmet_ratio", float),
                    "head_width": typed("head_width", float),
                    "alert_frames": typed("alert_frames", int),
                    "detector": typed("detector", str),
                    # 로봇의 웹캠 노드를 ssh 로 직접 띄운다(inspect_node 와 같은 host).
                    "robot_host": typed("robot_host", str),
                    "detect_every": typed("detect_every", int),
                    "detect_every_active": typed("detect_every_active", int),
                    "camera_fps": typed("camera_fps", float),
                    "save_all": typed("save_all", bool),
                    "quiet": typed("quiet", bool),
                    "hold_on_person": typed("hold_on_person", bool),
                    "person_frames": typed("person_frames", int),
                    "ok_frames": typed("ok_frames", int),
                    "alert_frames": typed("alert_frames", int),
                    "judge_bad_min": typed("judge_bad_min", int),
                    "judge_bad_ratio": typed("judge_bad_ratio", float),
                    "judge_sec": typed("judge_sec", float),
                    "sound_value": typed("sound_value", int),
                    "sound_repeat": typed("sound_repeat", int),
                    "sound_ok_value": typed("sound_ok_value", int),
                    "realert_sec": typed("realert_sec", float),
                }],
            ),
        ]
    )
