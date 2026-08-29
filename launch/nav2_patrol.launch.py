"""
nav2_patrol.launch.py — 우리가 만든 지도로 Nav2(자율주행)를 띄운다.

[VM에서 실행]  (로봇에서는 bringup 만 돌린다)
  ros2 launch ~/vibe/ex1/launch/nav2_patrol.launch.py

다른 지도를 쓰려면:
  ros2 launch ~/vibe/ex1/launch/nav2_patrol.launch.py map:=/경로/다른지도.yaml

[처음 실행 후 반드시 해야 하는 것 — 초기 위치 지정]
Nav2 는 로봇이 지도상 어디에 있는지 모르는 상태로 시작한다(AMCL 이 위치추정을 하지만
출발점 힌트가 필요하다). RViz 상단 "2D Pose Estimate" 버튼을 누르고,
지도에서 로봇의 실제 위치를 클릭한 뒤 로봇이 바라보는 방향으로 드래그한다.
이걸 안 하면 목표를 줘도 엉뚱한 곳으로 가거나 아예 움직이지 않는다.

[목표 지점 주기]
RViz 상단 "Nav2 Goal" 버튼 -> 지도에서 가고 싶은 곳 클릭 -> 방향 드래그.

주의: turtlebot3_navigation2 는 /cmd_vel 로 직접 발행한다. 우리 cmd_vel_mux 를
      함께 켜면 서로 충돌하므로, Nav2 테스트 단계에서는 mux 를 띄우지 않는다.
      나중에 순찰 자동화에서 mux 를 쓸 때는 Nav2 출력을 /cmd_vel_nav 로 리맵해야 한다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

TB3_NAV2 = get_package_share_directory("turtlebot3_navigation2")
EX1 = os.path.join(os.path.expanduser("~"), "vibe", "ex1")
# 순찰로 여러 번 돌며 다듬은 지도(2026-07-30). 원점(-1.59, -5.12)·해상도가
# 예전 patrol_map.yaml 과 같아 웨이포인트 좌표는 그대로 유효하다.
# 벽선의 구멍이 메워져 AMCL 스캔 매칭이 더 안정적이고, 미탐사가 0.5m² 줄어
# 코스트맵상 통행 가능 면적이 늘었다. 예전 지도로 돌리려면 map:= 로 넘기면 된다.
DEFAULT_MAP = os.path.join(EX1, "maps", "patrol_map_v5.yaml")

# 원본 burger.yaml 을 복사해 수정한 전용 파라미터.
# 좁은 방에서 벽 30cm 로 순찰하기 위해 inflation_radius 등을 조정했다.
# 각 값의 변경 근거는 그 파일 안 주석 참고.
DEFAULT_PARAMS = os.path.join(EX1, "config", "patrol_nav2.yaml")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=DEFAULT_MAP),
        DeclareLaunchArgument("params_file", default_value=DEFAULT_PARAMS),
        DeclareLaunchArgument("use_sim_time", default_value="false"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(TB3_NAV2, "launch", "navigation2.launch.py")
            ),
            launch_arguments={
                "map": LaunchConfiguration("map"),
                "params_file": LaunchConfiguration("params_file"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items(),
        ),
    ])
