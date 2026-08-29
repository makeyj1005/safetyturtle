#!/usr/bin/env python3
"""webcam_node.py — USB 웹캠(DICOTA D31841)을 CompressedImage 로 발행한다.

[이 파일은 로봇(ros18)에서 실행한다]  위치: 로봇의 ~/launch/webcam_node.py
  기본 (안전모 감시용 640x480 3fps):
    python3 ~/launch/webcam_node.py

  파라미터를 바꿔서:
    python3 ~/launch/webcam_node.py --ros-args -p fps:=5.0 -p jpeg_quality:=60

[출력]  /webcam/image_raw/compressed  (sensor_msgs/CompressedImage, best_effort)
[입력]  /webcam/enable  (Bool)  False 면 카메라를 놓고 발행을 멈춘다

[⚠️ 카메라는 항상 한쪽만 켠다 — 2026-08-01 사용자 결정]
일반 순찰은 이 웹캠만(사람·안전모), 소화기 점검은 CSI 만(압력계) 쓴다. 무선이
병목이라 스트림을 둘 얹으면 /scan 이 밀려 Nav2 가 경로를 못 만든다. 그래서 VM 의
helmet_node 가 점검 순번에 /webcam/enable=False 를 보내고, 이 노드는 그때
**카메라 장치를 놓는다**(발행만 멈추는 게 아니다 — USB 대역·전류도 같이 줄인다).

[왜 camera_ros 를 안 쓰고 이걸 새로 만들었나 — 2026-08-01]
안전모용 카메라는 USB 웹캠인데 세 가지가 다 막혀 있었다:
  camera_ros    웹캠이 RGB888 을 지원하지 않아 `unsupported pixel format` 로 exit -6
  v4l2_camera   로봇에 설치되어 있지 않다
  usb_cam       마찬가지
apt 설치는 sudo 가 필요해 사용자가 직접 해야 하는데, 로봇에 python3-opencv 4.5.4 가
이미 있어서 **설치 없이** 여기서 끝낼 수 있다. CSI 카메라는 그대로 camera_ros 를 쓴다
(소화기 점검용 고해상도는 camera_ros 쪽이 낫다) — 두 카메라가 서로 다른 경로를 탄다.

[⚠️ 장치 번호를 고정하지 않는 이유]
USB 를 꽂고 빼면 번호가 밀린다(CSI 가 0↔1 로 오간 것과 같은 문제). 그래서 번호 대신
**장치 이름**으로 찾는다: /sys/class/video4linux/video*/name 에 device_name 이 들어간
장치를 번호 순으로 훑어 실제로 프레임이 읽히는 첫 번째를 쓴다. DICOTA 는 video2/video3
두 개를 만드는데 실제 캡처가 되는 건 하나뿐이라, "열리는지"가 아니라 "읽히는지"로
판단해야 한다(video3 도 open 은 성공한다).

[⚠️ 왜 grab() 을 돌리고 retrieve() 만 가끔 하나]
웹캠은 30fps 로 계속 프레임을 밀어 넣는데 우리는 3fps 만 쓴다. 그냥 3fps 로 read()
하면 드라이버 큐에 쌓인 **옛 프레임**이 나와 영상이 몇 초씩 밀린다(주행 중 사람을
보는 용도라 치명적이다). grab() 은 디코딩 없이 큐만 비우므로 싸다 — 계속 grab 으로
큐를 비우다가 발행할 때만 retrieve() 로 디코딩한다.

[해상도·fps 근거 — HANDOFF "제약 5"]
무선이 이 프로젝트의 병목이다. 640x480 jpeg50 은 프레임 20KB 라 3fps 에서 60KB/s 다.
사람 판정에는 이 정도로 충분하고, 그 이상은 /scan 을 밀어내 순찰이 불안정해진다.
2560x1440 까지 지원하지만 절대 쓰지 말 것.
"""
import os
import signal
import sys
import time

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

V4L_SYS = "/sys/class/video4linux"


def find_device(name_hint):
    """이름에 name_hint 가 들어간 v4l2 장치 경로를 번호 순으로 돌려준다."""
    found = []
    if not os.path.isdir(V4L_SYS):
        return found
    # 이 폴더에는 videoN 말고 v4l-subdev0 같은 것도 있다 — 번호로 정렬하기 전에
    # videoN 만 걸러야 한다(안 그러면 int() 가 터진다. 실제로 로봇에서 터졌다).
    names = [e for e in os.listdir(V4L_SYS) if e.startswith("video")
             and e[len("video"):].isdigit()]
    for entry in sorted(names, key=lambda s: int(s[len("video"):])):
        try:
            with open(os.path.join(V4L_SYS, entry, "name")) as f:
                dev_name = f.read().strip()
        except OSError:
            continue
        if name_hint.lower() in dev_name.lower():
            found.append((f"/dev/{entry}", dev_name))
    return found


class WebcamNode(Node):
    def __init__(self):
        super().__init__("webcam_node")

        # device 를 비워두면 device_name 으로 찾는다(위 함정 설명).
        self.declare_parameter("device", "")
        self.declare_parameter("device_name", "DICOTA")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        # 발행 fps. 카메라는 30fps 로 돌지만 무선에 얹는 건 이만큼만.
        self.declare_parameter("fps", 3.0)
        self.declare_parameter("jpeg_quality", 50)
        self.declare_parameter("topic", "/webcam/image_raw/compressed")
        self.declare_parameter("frame_id", "webcam")
        # best_effort — 유실 프레임은 버리고 최신 것을 받는다. reliable 로 두면
        # 재전송을 기다리다 스트림 전체가 멈춘다(CSI 에서 977ms 실측).
        self.declare_parameter("reliability", "best_effort")
        # 카메라가 잠깐 끊겼을 때 몇 초까지 다시 열어볼지. 0 이면 바로 종료.
        self.declare_parameter("reopen_sec", 5.0)
        self.declare_parameter("enable_topic", "/webcam/enable")
        # /webcam/enable=False 일 때 카메라 장치를 놓을지.
        # **기본은 놓지 않는다(False).** 2026-08-02 실측: 이 웹캠은 스트림을 여러 번
        # 열고 닫으면 망가진다 — 그 뒤로는 v4l2-ctl 단발 캡처만 되고 OpenCV 스트림은
        # select() timeout 이 되며, 케이블 재연결·authorized 토글·uvcvideo 재적재로도
        # 안 풀려 로봇을 재부팅해야 했다. 그래서 장치는 부팅당 한 번만 열고 유지하고,
        # 끌 때는 **발행만** 멈춘다(무선 부하는 그것으로 사라진다).
        # USB 전류를 꼭 줄여야 할 때만 True 로 준다.
        self.declare_parameter("release_on_disable", False)

        self.cap = None
        self.dev_path = None
        # 켜진 상태로 시작한다 — helmet_node 가 안 떠 있어도 혼자 쓸 수 있어야 한다.
        self.enabled = True
        if not self.open_camera():
            raise SystemExit(1)

        rel = (ReliabilityPolicy.RELIABLE
               if str(self.get_parameter("reliability").value) == "reliable"
               else ReliabilityPolicy.BEST_EFFORT)
        # depth=1 — 밀린 옛 프레임을 쌓지 않고 항상 최신 것만 내보낸다.
        self.pub = self.create_publisher(
            CompressedImage, str(self.get_parameter("topic").value),
            QoSProfile(depth=1, reliability=rel),
        )
        # 켜고 끄기 신호는 놓치면 안 되므로 RELIABLE 이다(영상과 QoS 가 다르다).
        self.create_subscription(
            Bool, str(self.get_parameter("enable_topic").value), self.on_enable,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
        )

        self.period = 1.0 / max(float(self.get_parameter("fps").value), 0.1)
        self.last_pub = 0.0
        self.n_sent = 0
        self.n_fail = 0
        self.last_report = time.time()
        self.fail_since = None

        self.get_logger().info(
            f"웹캠 발행 시작: {self.dev_path} -> "
            f"{self.get_parameter('topic').value} "
            f"({self.get_parameter('width').value}x{self.get_parameter('height').value}, "
            f"{self.get_parameter('fps').value}fps, "
            f"jpeg{self.get_parameter('jpeg_quality').value}, "
            f"{self.get_parameter('reliability').value})"
        )

        # grab 은 싸므로 카메라의 원래 fps 에 가깝게 자주 돈다(큐 비우기).
        self.create_timer(0.02, self.tick)

    # ---------------- 카메라 ----------------
    def open_camera(self):
        want = str(self.get_parameter("device").value).strip()
        hint = str(self.get_parameter("device_name").value)
        cands = [(want, "(직접 지정)")] if want else find_device(hint)
        if not cands:
            self.get_logger().error(
                f"'{hint}' 이름의 카메라를 찾지 못했다. "
                "웹캠이 꽂혀 있는지 확인할 것 (`v4l2-ctl --list-devices`)"
            )
            return False

        w = int(self.get_parameter("width").value)
        h = int(self.get_parameter("height").value)
        for path, name in cands:
            cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
            if not cap.isOpened():
                cap.release()
                continue
            # MJPG 로 받는다. YUYV 는 640x480 한 장이 600KB 라 USB 대역이 모자라
            # 프레임이 뚝뚝 끊긴다. MJPG 는 카메라가 압축해서 준다.
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            # 한 번 실패했다고 장치를 버리지 않는다 — 스트림이 시작되는 첫 프레임은
            # 놓치는 일이 있다(재연결 직후에 실측). 짧게 여러 번 시도한다.
            ok, frame = False, None
            for _ in range(8):
                ok, frame = cap.read()
                if ok and frame is not None:
                    break
                time.sleep(0.3)
            if not ok or frame is None:
                # DICOTA 는 video2/video3 를 만드는데 한쪽은 캡처가 안 된다.
                # open 은 성공하므로 실제로 읽어봐야 구분된다.
                self.get_logger().info(f"{path} 은(는) 프레임이 안 나온다 — 건너뛴다")
                cap.release()
                continue
            self.cap = cap
            self.dev_path = path
            got = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                   int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
            if got != (w, h):
                self.get_logger().warn(
                    f"요청 {w}x{h} 가 아니라 {got[0]}x{got[1]} 로 열렸다 "
                    "(카메라가 지원하는 크기로 대체된 것)"
                )
            self.get_logger().info(f"카메라 열림: {path} [{name}] {got[0]}x{got[1]}")
            return True

        self.get_logger().error(
            f"'{hint}' 장치를 찾았지만 프레임을 읽지 못했다: "
            + ", ".join(p for p, _ in cands)
            + " — 다른 프로세스가 카메라를 잡고 있는지 확인할 것"
        )
        return False

    def on_enable(self, msg: Bool):
        want = bool(msg.data)
        if want == self.enabled:
            return                      # 같은 값이 주기적으로 온다 — 조용히 넘긴다
        self.enabled = want
        release = bool(self.get_parameter("release_on_disable").value)
        if not self.enabled:
            if release:
                self.get_logger().info("발행 중지 — 카메라를 놓는다 (점검 차례)")
                if self.cap is not None:
                    self.cap.release()
                    self.cap = None
            else:
                # 장치는 열어둔 채 발행만 멈춘다(위 release_on_disable 설명 참고).
                self.get_logger().info("발행 중지 — 장치는 열어둔다 (점검 차례)")
            return
        if self.cap is None:
            self.get_logger().info("발행 재개 — 카메라를 다시 연다")
            self.fail_since = None
            if not self.open_camera():
                self.get_logger().error("카메라를 다시 열지 못했다")
        else:
            self.get_logger().info("발행 재개")

    def reopen(self):
        """카메라가 끊겼을 때 다시 연다. USB 전류 문제로 장치가 사라진 적이 있다."""
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        return self.open_camera()

    # ---------------- 주기 ----------------
    def tick(self):
        if self.cap is None:
            return
        # 디코딩 없이 큐만 비운다(위 grab/retrieve 설명).
        if not self.cap.grab():
            self.on_fail("grab 실패")
            return
        if not self.enabled:
            # 발행만 멈춘 상태. grab 은 계속 돌려 큐를 비운다 — 그러지 않으면 재개할 때
            # 몇 초 밀린 옛 프레임이 나오고, 스트림을 멈춰 두는 것 자체가 이 장치에서
            # 위험하다(release_on_disable 설명).
            return

        now = time.time()
        if now - self.last_pub < self.period:
            return
        ok, frame = self.cap.retrieve()
        if not ok or frame is None:
            self.on_fail("retrieve 실패")
            return

        q = int(self.get_parameter("jpeg_quality").value)
        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), q])
        if not ok:
            self.on_fail("jpeg 인코딩 실패")
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        msg.format = "jpeg"
        msg.data = buf.tobytes()
        self.pub.publish(msg)

        self.last_pub = now
        self.n_sent += 1
        self.fail_since = None
        # 무선 상태를 눈으로 보려면 실제 발행량이 필요하다. 10초마다 한 줄.
        if now - self.last_report >= 10.0:
            fps = self.n_sent / (now - self.last_report)
            self.get_logger().info(
                f"발행 {self.n_sent}장 ({fps:.1f}fps, {len(msg.data) / 1024:.0f}KB/장)"
                + (f", 실패 {self.n_fail}" if self.n_fail else "")
            )
            self.n_sent = 0
            self.n_fail = 0
            self.last_report = now

    def on_fail(self, why):
        self.n_fail += 1
        now = time.time()
        if self.fail_since is None:
            self.fail_since = now
            self.get_logger().warn(f"프레임을 못 받았다: {why}")
            return
        limit = float(self.get_parameter("reopen_sec").value)
        if limit > 0 and now - self.fail_since > limit:
            self.get_logger().warn(f"{limit:.0f}초째 못 받는다 — 카메라를 다시 연다")
            self.fail_since = None
            if not self.reopen():
                self.get_logger().error("카메라를 다시 열지 못했다")

    def stop(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None


def main():
    # ⚠️ SIGTERM 을 반드시 잡아야 한다. 파이썬은 기본적으로 SIGTERM 에 finally 를
    # 실행하지 않고 죽는다 — 그러면 cap.release() 가 실행되지 않고 **카메라가 열린
    # 채로 남는다.** 2026-08-02 에 실제로 이것 때문에 웹캠이 멈췄다: 장치는 열거되고
    # v4l2-ctl 로는 캡처되는데 OpenCV 로는 스트림이 시작되지 않았고, 케이블 재연결과
    # authorized 토글로도 안 풀려 로봇을 재부팅해야 했다.
    # 이 노드는 helmet_node 가 ssh 로 죽이고 launch 가 SIGTERM 까지 올리므로 잡아야 한다.
    def on_term(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, on_term)

    rclpy.init()
    try:
        node = WebcamNode()
    except SystemExit:
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
