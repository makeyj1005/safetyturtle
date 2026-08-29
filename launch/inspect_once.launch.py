"""
inspect_once.launch.py — 순찰 없이 소화기 점검만 한 번 시킨다. 시험용.

[VM에서 실행]  (먼저 로봇 bringup + nav2_patrol.launch.py + RViz 2D Pose Estimate)
  기본 (지금 바로 소화기로 가서 판정하고, 그 자리에 선다):
  ros2 launch ~/vibe/ex1/launch/inspect_once.launch.py robot_host:=rpi@192.168.0.67

  판정 후 순찰 시작지점까지 돌아오게 하려면(실제 운용과 같은 동작):
  ros2 launch ~/vibe/ex1/launch/inspect_once.launch.py robot_host:=rpi@192.168.0.67 return_home:=true

  자세 보정 없이 한 번만 찍고 끝내려면(순수 측정용):
  ros2 launch ~/vibe/ex1/launch/inspect_once.launch.py robot_host:=rpi@192.168.0.67 max_attempts:=1

[왜 따로 두나]
patrol_auto 로 시험하면 스케줄러가 순번을 뽑을 때까지 1~2분 기다려야 하고, 순찰이
먼저 도는 회차도 있어 좌표 하나를 확인하는 데 시간이 오래 걸린다. 이 launch 는
`auto_start:=true` 로 켜자마자 점검을 시작한다.

[⚠️ patrol_auto 와 같이 띄우지 말 것]
inspect_node 가 두 개가 되어 **둘 다 /inspect/start 를 받고 각자 Nav2 목표를 보낸다.**
같은 CSV 에도 겹쳐 쓴다(실측 2026-08-01: 회차 하나를 통째로 버렸다). 이 launch 를
쓸 때는 patrol_auto 창을 Ctrl+C 로 끄고, 15초 뒤 "inspect_node 가 N 개 떠 있다"
경고가 없는지 확인할 것.

[시험이 끝나면]
결과는 logs/inspect_<시각>.csv 에 남는다. 보는 열:
  score      기준 패치와의 정합. **0.9 이상이면 자세가 제대로 맞은 것**
  offset_px  기준 자리에서 몇 px 벗어났나 (부호가 곧 방향이다)
  attempt    몇 번째 시도에서 판정했나 (1이면 첫 시도에 맞춘 것)
  change     기준 대비 바늘 변화량 (0.9 미만이면 정상)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARGS = {
    "robot_host": "rpi@192.168.0.67",   # 사진을 가져올 로봇. IP 는 재부팅마다 바뀐다
    "shots": "3",                       # 지점당 사진 장수
    "max_attempts": "4",                # 자세가 어긋나면 보정해 다시 시도하는 횟수
    "settle_sec": "2.0",                # 도착 후 흔들림이 가라앉기를 기다리는 시간
    "grab_timeout": "0.0",              # ssh 한 번의 상한(초). 0=자동, 무선 느리면 키운다
    "sound": "true",                    # 이상·부재·판정불가에 부저
    "return_home": "false",             # 시험용이라 기본은 그 자리에 선다
    "photo_dir": "",                    # 로봇 없이 시험할 때만: 이 폴더 사진으로 판정
}


def typed(name, kind):
    return ParameterValue(LaunchConfiguration(name), value_type=kind)


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()]
        + [
            Node(
                package="patrol_core",
                executable="inspect_node",
                name="inspect_node",
                output="screen",
                parameters=[{
                    # 켜자마자 시작한다. 이게 patrol_auto 와 다른 유일한 점이다.
                    "auto_start": True,
                    "robot_host": typed("robot_host", str),
                    "shots": typed("shots", int),
                    "max_attempts": typed("max_attempts", int),
                    "settle_sec": typed("settle_sec", float),
                    "grab_timeout_sec": typed("grab_timeout", float),
                    "sound": typed("sound", bool),
                    "return_to_start": typed("return_home", bool),
                    "photo_dir": typed("photo_dir", str),
                }],
            ),
        ]
    )
