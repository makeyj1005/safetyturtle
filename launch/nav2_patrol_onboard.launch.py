"""
nav2_patrol_onboard.launch.py — Nav2 를 **로봇(Pi 4) 안에서** 띄운다.

[로봇에서 실행]  (VM 에서는 RViz 만 띄운다)
  ros2 launch ~/vibe/ex1/launch/nav2_patrol_onboard.launch.py

왜 nav2_patrol.launch.py 를 그냥 못 쓰는가 (2026-08-07):
  기존 nav2_patrol.launch.py 는 turtlebot3_navigation2 의 navigation2.launch.py 를
  포함한다. 그 파일은 **RViz2 를 조건 없이 함께 실행**한다(rviz 노드가 무조건 들어있고
  끄는 인자가 없다). 로봇은 화면이 없어 rviz2 를 깔지 않았으므로 그대로 쓰면 실패한다.
  그래서 navigation2.launch.py 가 하는 두 가지 일 중 **RViz 를 뺀 나머지**,
  즉 nav2_bringup/bringup_launch.py 포함만 여기에 그대로 옮겼다.
  (원본이 하는 일은 정확히 이 두 개뿐이라, 빼도 주행 기능은 하나도 줄지 않는다.)

지도·파라미터는 기존 nav2_patrol.launch.py 와 **같은 파일을 쓴다.** 로봇과 VM 의
nav2 버전이 동일함(nav2 1.1.20 / turtlebot3_navigation2 2.3.6)을 확인했으므로
patrol_nav2.yaml 을 그대로 재사용할 수 있다. 버전이 어긋나면 Navigation inactive
가 되는 함정이 있으니 파라미터를 옮길 때는 항상 양쪽 버전을 먼저 비교할 것.

[초기 위치 지정은 여전히 필요하다]
AMCL 은 출발점 힌트가 필요하다. **VM 의 RViz** 에서 "2D Pose Estimate" 로 찍는다.
Nav2 가 로봇 안에 있어도 RViz 는 VM 에서 띄우면 되고(ROS_DOMAIN_ID=3 이라 그대로 보인다),
찍은 초기 위치는 /initialpose 토픽으로 로봇의 AMCL 에 전달된다.

[이 구성의 의미]
/scan -> Nav2 -> /cmd_vel 이 전부 로봇 내부에서 돌기 때문에, 무선이 느려도(측정값
유선 7ms vs 무선 107ms) 주행이 영향을 받지 않는다. 무선으로 넘어가는 것은 RViz 화면과
로그뿐이다. 이것이 Pi 4 로 바꾼 목적이다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

NAV2_LAUNCH_DIR = os.path.join(get_package_share_directory("nav2_bringup"), "launch")
EX1 = os.path.join(os.path.expanduser("~"), "vibe", "ex1")

# nav2_patrol.launch.py 와 동일한 기본값 — 지도·파라미터를 한 곳으로 유지한다.
DEFAULT_MAP = os.path.join(EX1, "maps", "patrol_map_v5.yaml")
DEFAULT_PARAMS = os.path.join(EX1, "config", "patrol_nav2.yaml")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=DEFAULT_MAP),
        DeclareLaunchArgument("params_file", default_value=DEFAULT_PARAMS),
        DeclareLaunchArgument("use_sim_time", default_value="false"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(NAV2_LAUNCH_DIR, "bringup_launch.py")
            ),
            launch_arguments={
                "map": LaunchConfiguration("map"),
                "params_file": LaunchConfiguration("params_file"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items(),
        ),
    ])
