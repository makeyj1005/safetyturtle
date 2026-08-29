"""
explore.launch.py — 자동 탐사 매핑용. cartographer(SLAM) + Nav2 주행부를 함께 띄운다.

[VM에서 실행]  (로봇에서는 bringup 만 돌린다)
  새 지도로 탐사:
    ros2 launch ~/vibe/ex1/launch/explore.launch.py

  기존 지도에 이어서 탐사 (원점 유지 -> 기존 웨이포인트 좌표 계속 사용 가능):
    ros2 launch ~/vibe/ex1/launch/explore.launch.py \
      load_state_filename:=/home/ohinseop/vibe/ex1/maps/state/patrol_state5.pbstream

  그 다음 별도 터미널에서 탐사 노드 실행:
    ros2 run patrol_core explore_node

[왜 nav2_patrol.launch.py 를 쓰지 않는가]
그쪽은 map_server + AMCL 을 함께 띄운다. 자동 탐사에서는 cartographer 가 지도를 만들면서
동시에 map->odom TF 를 발행하는데, AMCL 도 같은 TF 를 발행하려 해서 TF 트리가 깨진다.
그래서 여기서는 nav2_bringup 의 navigation_launch.py 를 쓴다 — 주행에 필요한
planner/controller/bt_navigator 만 띄우고 지도와 위치추정은 cartographer 에 맡긴다.

[주의]
탐사가 끝나면 지도를 저장하고, 순찰할 때는 다시 nav2_patrol.launch.py 로 전환한다.
cartographer 는 계속 지도를 갱신하므로 순찰 중에 켜두면 불필요한 연산과 왜곡 위험이 있다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

EX1 = os.path.join(os.path.expanduser("~"), "vibe", "ex1")
NAV2_BRINGUP = get_package_share_directory("nav2_bringup")
DEFAULT_PARAMS = os.path.join(EX1, "config", "patrol_nav2.yaml")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=DEFAULT_PARAMS),
        DeclareLaunchArgument("load_state_filename", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),

        # SLAM: 지도 생성 + map->odom TF 발행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(EX1, "launch", "cartographer_resumable.launch.py")
            ),
            launch_arguments={
                "load_state_filename": LaunchConfiguration("load_state_filename"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items(),
        ),

        # Nav2 주행부만 (map_server / AMCL 제외)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(NAV2_BRINGUP, "launch", "navigation_launch.py")
            ),
            launch_arguments={
                "params_file": LaunchConfiguration("params_file"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "autostart": "true",
            }.items(),
        ),
    ])
