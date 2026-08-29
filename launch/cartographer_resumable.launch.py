"""
cartographer_resumable.launch.py — turtlebot3_cartographer.launch.py 를 그대로 쓰되
cartographer_node 에 -load_state_filename 을 넘겨 이전 상태에서 이어서 매핑할 수 있게 한 버전.

[문제] turtlebot3_cartographer 패키지의 원본 cartographer.launch.py 는
cartographer_node 에 -configuration_directory / -configuration_basename 만 넘기고
-load_state_filename 을 지원하지 않는다. 그래서 launch 인자로 그 값을 줘도
"선언되지 않은 인자"로 조용히 무시되고 그냥 새 지도로 시작된다.
(2026-07-29 실측: /write_state 로 저장한 .pbstream 을 불러오려 했으나 실제로는 초기화됨)

[VM에서 실행]
  이어서 매핑 (저장된 상태 불러오기):
    ros2 launch ~/vibe/ex1/launch/cartographer_resumable.launch.py \
      load_state_filename:=/home/ohinseop/vibe/ex1/maps/state/patrol_state1.pbstream

  백지에서 새로 시작 (원본과 동일하게):
    ros2 launch ~/vibe/ex1/launch/cartographer_resumable.launch.py

[진행 중 상태 저장 — 지도를 잃지 않으려면 주기적으로]
  ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
    "{filename: '/home/ohinseop/vibe/ex1/maps/state/<이름>.pbstream', include_unfinished_submaps: true}"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TB3_CARTOGRAPHER = get_package_share_directory("turtlebot3_cartographer")


def make_cartographer_node(context, *args, **kwargs):
    load_state_filename = LaunchConfiguration("load_state_filename").perform(context)
    node_args = [
        "-configuration_directory", LaunchConfiguration("cartographer_config_dir").perform(context),
        "-configuration_basename", LaunchConfiguration("configuration_basename").perform(context),
    ]
    if load_state_filename:
        # load_frozen_state=false 로 명시해야 "그 상태에서 이어서 계속 매핑"이 된다.
        # true 로 두면 불러온 지도를 고정된 배경으로만 쓰고(순수 위치추정) 새로 확장하지 않는다.
        node_args += [
            "-load_state_filename", load_state_filename,
            "-load_frozen_state", "false",
        ]
    return [
        Node(
            package="cartographer_ros",
            executable="cartographer_node",
            name="cartographer_node",
            output="screen",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            arguments=node_args,
        )
    ]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    use_rviz = LaunchConfiguration("use_rviz", default="true")
    resolution = LaunchConfiguration("resolution", default="0.05")
    publish_period_sec = LaunchConfiguration("publish_period_sec", default="1.0")
    rviz_config_dir = os.path.join(TB3_CARTOGRAPHER, "rviz", "tb3_cartographer.rviz")

    return LaunchDescription([
        DeclareLaunchArgument(
            "cartographer_config_dir",
            default_value=os.path.join(TB3_CARTOGRAPHER, "config"),
        ),
        DeclareLaunchArgument("configuration_basename", default_value="turtlebot3_lds_2d.lua"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("resolution", default_value="0.05"),
        DeclareLaunchArgument("publish_period_sec", default_value="1.0"),
        # 비워두면 원본과 동일하게 백지에서 새로 시작한다.
        DeclareLaunchArgument("load_state_filename", default_value=""),

        OpaqueFunction(function=make_cartographer_node),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(TB3_CARTOGRAPHER, "launch", "occupancy_grid.launch.py")
            ),
            launch_arguments={
                "use_sim_time": use_sim_time, "resolution": resolution,
                "publish_period_sec": publish_period_sec,
            }.items(),
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_dir],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(use_rviz),
            output="screen",
        ),
    ])
