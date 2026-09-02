#!/usr/bin/env python3
"""speaker_node.py — 로봇 I2S 스피커(MAX98357A)로 음성 파일을 재생한다.

[로봇에서 실행]  (스피커가 물리적으로 로봇에 붙어있으므로 여기서만 돈다)
  python3 ~/launch/speaker_node.py

[왜 이 노드가 필요한가 — 2026-09-02]
처음에는 VM 쪽 노드(fire_node 등)가 ssh 로 로봇에 붙어 mpg123 을 직접 실행했다.
그런데 그 노드들은 Docker 컨테이너 안에서 돌아서 ssh 키가 없고(Permission denied),
호스트의 ~/.ssh 를 그대로 마운트하면 소유자·권한 검사에 걸린다(Bad owner or
permissions). 게다가 경보마다 ssh 핸드셰이크가 붙어 반응이 느려진다.
그래서 재생은 로봇에서 하고, VM 은 "이 소리 틀어라"만 토픽으로 보내게 바꿨다.

[입력]  /speaker/play  (String)  재생할 소리. 두 가지를 받는다:
          - 논리 이름: "fire_alarm", "helmet_bad", "intrusion", "gauge_ok"
            (sounds/<이름>.mp3 로 해석한다 — 파일 경로를 VM 쪽에 하드코딩하지 않으려고)
          - 절대/~ 경로: "~/vibe/ex1/sounds/fire_alarm.mp3"
[출력]  /speaker/status (String)  진단용 — 재생 시작/스킵/실패

[동시 재생은 하지 않는다]
plughw 는 한 프로그램이 장치를 독점하므로, 재생 중에 또 요청이 오면 그 요청은
버린다(겹쳐 틀려고 하면 "device busy" 로 둘 다 깨진다). 화재처럼 반복해서 알려야
하는 경우는 부르는 쪽이 alarm_interval_sec 만큼 띄워서 다시 요청하면 된다.
"""
import os
import subprocess

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

SOUND_DIR = os.path.join(os.path.expanduser("~"), "vibe", "ex1", "sounds")


class SpeakerNode(Node):
    def __init__(self):
        super().__init__("speaker_node")

        # I2S 스피커는 card 1 (aplay -l 로 확인). card 0 은 헤드폰잭이라 소리가 안 난다.
        self.declare_parameter("device", "plughw:1,0")
        # mpg123 -f 값. 32768=1배, 65536=2배. 이 앰프는 하드웨어 볼륨조절이 없어서
        # (amixer -c 1 에 컨트롤이 없다) 여기서 소프트 증폭한다.
        # 2026-09-02 실측: 2배가 음질 유지하면서 충분히 크다.
        self.declare_parameter("gain", 65536)
        self.declare_parameter("sound_dir", SOUND_DIR)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_status = self.create_publisher(String, "/speaker/status", qos)
        self.create_subscription(String, "/speaker/play", self.on_play, qos)

        self.proc = None
        self.get_logger().info(
            f"speaker_node 시작 — 장치={self.get_parameter('device').value}, "
            f"소리폴더={self.get_parameter('sound_dir').value}"
        )

    def resolve(self, name):
        """논리 이름이면 sounds/<이름>.mp3 로, 경로면 그대로 쓴다."""
        name = name.strip()
        if not name:
            return None
        if "/" in name or name.startswith("~"):
            return os.path.expanduser(name)
        base = str(self.get_parameter("sound_dir").value)
        return os.path.join(os.path.expanduser(base), f"{name}.mp3")

    def busy(self):
        return self.proc is not None and self.proc.poll() is None

    def on_play(self, msg: String):
        path = self.resolve(msg.data)
        if path is None:
            return
        if not os.path.exists(path):
            self.get_logger().error(f"소리 파일이 없다: {path}")
            self.status(f"error: no file {path}")
            return
        if self.busy():
            # 이전 재생이 아직 안 끝났다 — 겹치면 둘 다 깨지므로 버린다.
            self.get_logger().info(f"재생 중이라 건너뜀: {os.path.basename(path)}")
            self.status("skipped (busy)")
            return

        dev = str(self.get_parameter("device").value)
        gain = int(self.get_parameter("gain").value)
        cmd = ["mpg123", "-q", "-a", dev, "-f", str(gain), path]
        try:
            self.proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                                         stderr=subprocess.DEVNULL)
            self.get_logger().info(f"재생: {os.path.basename(path)}")
            self.status(f"playing {os.path.basename(path)}")
        except OSError as e:                                    # noqa: BLE001
            self.get_logger().error(f"재생 실패: {e}")
            self.status(f"error: {e}")

    def status(self, text):
        m = String()
        m.data = text
        self.pub_status.publish(m)

    def destroy_node(self):
        if self.busy():
            self.proc.terminate()
        super().destroy_node()


def main():
    rclpy.init()
    node = SpeakerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
