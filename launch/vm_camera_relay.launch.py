"""
vm_camera_relay.launch.py — VM에서 compressed 영상을 RViz용 raw로 되돌리는 중계를 띄운다.

[이 파일은 VM에서 실행한다]
  ros2 launch ~/vibe/ex1/launch/vm_camera_relay.launch.py

왜 필요한지는 tools/compressed_to_raw.py 상단 주석 참고.
요약: RViz2의 Image 디스플레이는 compressed를 못 읽고,
      ros2 run image_transport republish 는 RELIABLE로만 구독해서
      로봇의 BEST_EFFORT 발행자와 QoS가 안 맞는다.

Node 대신 ExecuteProcess를 쓰는 이유: compressed_to_raw.py 는 아직
ament 패키지로 설치된 실행파일이 아니라 그냥 파이썬 스크립트이기 때문.
(2단계에서 패키지로 만들면 Node로 바꿀 수 있다)
"""
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    script = os.path.join(
        os.path.expanduser("~"), "vibe", "ex1", "tools", "compressed_to_raw.py"
    )
    return LaunchDescription([
        ExecuteProcess(
            cmd=["python3", script],
            name="compressed_to_raw",
            output="screen",
        ),
    ])
