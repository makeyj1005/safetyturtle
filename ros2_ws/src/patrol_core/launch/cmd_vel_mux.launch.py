"""
cmd_vel_mux.launch.py — 주행 명령 중재 노드를 띄운다.

[로봇에서 실행]
  ros2 launch patrol_core cmd_vel_mux.launch.py

이 노드만 /cmd_vel 로 발행한다(프로젝트 절대 규칙).
Nav2 는 /cmd_vel_nav 로, 텔레옵은 /cmd_vel_teleop 으로 내야 한다.

즉시 정지(소프트 비상정지):
  ros2 topic pub --once /mux/enable std_msgs/msg/Bool "{data: false}"
해제:
  ros2 topic pub --once /mux/enable std_msgs/msg/Bool "{data: true}"
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARGS = [
    ("timeout", "0.4", float),      # 입력이 이 시간 안에 안 오면 죽은 것으로 본다
    ("rate", "20.0", float),        # 최종 발행 주기
    ("max_linear", "0.15", float),  # 어떤 입력이 와도 이 값을 넘겨 발행하지 않는다
    ("max_angular", "1.2", float),
]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d) for n, d, _ in ARGS]
        + [
            Node(
                package="patrol_core",
                executable="cmd_vel_mux",
                name="cmd_vel_mux",
                output="screen",
                parameters=[{
                    n: ParameterValue(LaunchConfiguration(n), value_type=t)
                    for n, _, t in ARGS
                }],
            )
        ]
    )
