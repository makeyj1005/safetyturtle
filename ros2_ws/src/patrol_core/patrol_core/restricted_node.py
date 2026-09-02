#!/usr/bin/env python3
"""restricted_node.py — 지정 시간대(예: 새벽)에 사람이 보이면 경고한다.

[VM에서 실행]  (절대 규칙 3: 영상 인식 연산은 로봇이 아닌 VM 에서 한다)
  ros2 run patrol_core restricted_node

  테스트용 (시간 무시하고 항상 제한시간으로 취급):
  ros2 run patrol_core restricted_node --ros-args -p always_restricted:=true

[입력]  /webcam/image_raw/compressed   helmet_node 와 같은 웹캠 영상(별도로 켤 필요 없음)
[출력]  /restricted/status  (String)   진단·웹 대시보드용
        /sound   (service)             부저 경고 (helmet_node 와 겹치지 않는 값 사용)
        음성 안내는 로봇에 물린 스피커로 ssh + espeak-ng 를 통해 재생한다(2026-08-29 추가,
        스피커는 아직 로봇에 연결 전이라 미검증 — 연결되면 voice_enabled:=true 로 확인할 것)

[시간 판정 — 자정을 넘어가는 구간도 처리]
start_time <= end_time 이면 보통 구간(예 09:00~18:00), start_time > end_time 이면
자정을 넘는 구간(예 22:00~06:00, "새벽")으로 본다.

[구역 제한은 아직 미구현]
"위험구역"처럼 장소 기반 제한은 로봇이 지금 어느 웨이포인트/구간에 있는지 알아야 하는데
그 정보를 아직 어디서도 발행하지 않는다(patrol_node 가 leg_pos 를 갖고 있지만 토픽으로
안 낸다). 지금은 시간 기준만 구현하고, 구역은 patrol_node 가 현재 위치를 발행하게 되면
추가한다.
"""
import os
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from turtlebot3_msgs.srv import Sound

from patrol_core import event_log

EX1 = os.path.join(os.path.expanduser("~"), "vibe", "ex1")
DEFAULT_MODEL_DIR = os.path.join(EX1, "models")

DNN_CLASSES = [
    "background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus",
    "car", "cat", "chair", "cow", "diningtable", "dog", "horse", "motorbike",
    "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor",
]
DNN_PERSON = DNN_CLASSES.index("person")
DNN_PROTO = "MobileNetSSD_deploy.prototxt"
DNN_MODEL = "MobileNetSSD_deploy.caffemodel"

# turtlebot3_msgs/srv/Sound: 0 OFF/1 ON/2 LOW_BATTERY/3 ERROR/4,5 BUTTON
# helmet_node 가 1(ON)·3(ERROR) 를 쓰므로 겹치지 않게 4(BUTTON1) 를 쓴다.
SOUND_RESTRICTED = 4

SSH_OPTS = ["-o", "ConnectTimeout", "8", "-o", "BatchMode=yes",
           "-o", "StrictHostKeyChecking=accept-new"]


def parse_hhmm(s):
    h, m = s.split(":")
    return int(h) * 60 + int(m)


class RestrictedNode(Node):
    def __init__(self):
        super().__init__("restricted_node")

        self.declare_parameter("topic", "/webcam/image_raw/compressed")
        # auto/dnn/hog = MobileNet-SSD 또는 HOG (CPU). yolo = 우리가 학습시킨
        # models/helmet_yolov8n.pt 로 person 클래스만 본다(GPU 가능 — 2026-08-29 추가,
        # docker/Dockerfile.gpu 로 만든 이미지에서만 동작한다).
        self.declare_parameter("detector", "auto")
        self.declare_parameter("model_dir", DEFAULT_MODEL_DIR)
        self.declare_parameter("person_conf", 0.5)
        self.declare_parameter("min_person_ratio", 0.30)
        # 안전모 전용 모델(helmet_yolov8n.pt)은 얼굴이 크게 나오는 화면에서
        # person 클래스를 잘 못 잡는다(학습 데이터가 전신 사진 위주 — 2026-08-29 실측).
        # 사람 유무만 보는 이 노드는 일반 COCO 학습 모델(person 인식이 강함)을 쓴다.
        self.declare_parameter("yolo_model_path",
                               os.path.join(DEFAULT_MODEL_DIR, "yolov8n_coco.pt"))
        self.declare_parameter("yolo_device", "0")

        # 제한시간 — HH:MM. start > end 면 자정을 넘는 구간(새벽)으로 본다.
        self.declare_parameter("start_time", "22:00")
        self.declare_parameter("end_time", "06:00")
        # 테스트용 — 켜면 시간 무관하게 항상 제한시간으로 취급한다.
        self.declare_parameter("always_restricted", False)

        # 판정 히스테리시스 — helmet_node 와 같은 방식(연속 프레임)
        self.declare_parameter("alert_frames", 3)
        self.declare_parameter("clear_frames", 10)
        self.declare_parameter("realert_sec", 8.0)

        self.declare_parameter("sound", True)
        self.declare_parameter("sound_value", SOUND_RESTRICTED)
        self.declare_parameter("sound_repeat", 2)
        self.declare_parameter("sound_wait_sec", 15.0)

        # 음성 안내 — 로봇에 물린 스피커로 ssh + espeak-ng 재생 (2026-08-29 추가,
        # 스피커 연결 전이라 미검증. 실패해도 노드는 계속 돈다).
        self.declare_parameter("voice_enabled", True)
        self.declare_parameter("voice_text", "작업금지 시간입니다")
        self.declare_parameter("voice_lang", "ko")
        self.declare_parameter("robot_host", "rpi@192.168.0.73")

        self.declare_parameter("quiet", False)
        self.declare_parameter("view", False)
        self.declare_parameter("db_path", event_log.DEFAULT_DB)

        self.net = None
        self.hog = None
        self.yolo_model = None
        self.detector = self.setup_detector()

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_status = self.create_publisher(String, "/restricted/status", qos)
        self.cli_sound = self.create_client(Sound, "/sound")

        self.create_subscription(CompressedImage, str(self.get_parameter("topic").value),
                                 self.on_frame, qos_profile_sensor_data)

        self.bad_streak = 0
        self.good_streak = 0
        self.alerting = False
        self.last_alarm = 0.0
        self.n_frame = 0

        self.get_logger().info(
            f"작업금지 시간대 감시 시작 — {self.get_parameter('start_time').value}~"
            f"{self.get_parameter('end_time').value}, 검출={self.detector}"
            + (" (테스트: 항상 제한시간으로 취급)"
               if bool(self.get_parameter("always_restricted").value) else "")
        )
        self.create_timer(10.0, self.report)

    # ---------------- 준비 (helmet_node.setup_detector 와 동일한 방식) ----------------
    def setup_detector(self):
        want = str(self.get_parameter("detector").value)

        if want == "yolo":
            path = str(self.get_parameter("yolo_model_path").value)
            if not os.path.exists(path):
                self.get_logger().error(f"detector:=yolo 인데 모델이 없다 — {path}")
                raise SystemExit(1)
            try:
                from ultralytics import YOLO
            except ImportError:
                self.get_logger().error(
                    "detector:=yolo 인데 ultralytics 가 없다 — patrol-ros2:gpu 이미지에서 실행할 것")
                raise SystemExit(1)
            self.yolo_model = YOLO(path)
            self.get_logger().info(f"YOLO 모델 로드: {path} (클래스 {self.yolo_model.names})")
            return "yolo"

        mdir = str(self.get_parameter("model_dir").value)
        proto, model = os.path.join(mdir, DNN_PROTO), os.path.join(mdir, DNN_MODEL)
        have = os.path.exists(proto) and os.path.exists(model)

        if want in ("auto", "dnn") and have:
            self.net = cv2.dnn.readNetFromCaffe(proto, model)
            self.get_logger().info(f"MobileNet-SSD 로드: {mdir}")
            return "dnn"
        if want == "dnn":
            self.get_logger().error(f"detector:=dnn 인데 모델이 없다 — {proto}, {model}")
            raise SystemExit(1)

        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        if want == "auto" and not have:
            self.get_logger().warn(f"모델 파일이 없어 HOG 로 시작한다 ({mdir})")
        return "hog"

    def find_persons(self, frame):
        h, w = frame.shape[:2]
        min_h = float(self.get_parameter("min_person_ratio").value) * h
        out = []
        if self.detector == "yolo":
            conf_min = float(self.get_parameter("person_conf").value)
            results = self.yolo_model.predict(
                frame, conf=conf_min, device=str(self.get_parameter("yolo_device").value),
                verbose=False)
            names = self.yolo_model.names
            for b in results[0].boxes:
                if names[int(b.cls)] != "person":
                    continue
                x1, y1, x2, y2 = [int(v) for v in b.xyxy[0]]
                if y2 - y1 >= min_h:
                    out.append(((max(x1, 0), max(y1, 0), min(x2, w), min(y2, h)),
                               float(b.conf)))
            return out
        if self.detector == "dnn":
            blob = cv2.dnn.blobFromImage(
                cv2.resize(frame, (300, 300)), 0.007843, (300, 300), 127.5)
            self.net.setInput(blob)
            det = self.net.forward()
            conf_min = float(self.get_parameter("person_conf").value)
            for i in range(det.shape[2]):
                conf = float(det[0, 0, i, 2])
                if conf < conf_min or int(det[0, 0, i, 1]) != DNN_PERSON:
                    continue
                box = det[0, 0, i, 3:7] * np.array([w, h, w, h])
                x1, y1, x2, y2 = [int(v) for v in box]
                if y2 - y1 >= min_h:
                    out.append(((max(x1, 0), max(y1, 0), min(x2, w), min(y2, h)), conf))
            return out

        scale = 320.0 / w
        small = cv2.resize(frame, (320, int(h * scale)))
        rects, weights = self.hog.detectMultiScale(
            small, winStride=(8, 8), padding=(8, 8), scale=1.05)
        conf_min = float(self.get_parameter("person_conf").value)
        for (x, y, rw, rh), weight in zip(rects, weights):
            conf = float(weight) / 2.0
            if conf < conf_min:
                continue
            x1, y1 = int(x / scale), int(y / scale)
            x2, y2 = int((x + rw) / scale), int((y + rh) / scale)
            if y2 - y1 >= min_h:
                out.append(((max(x1, 0), max(y1, 0), min(x2, w), min(y2, h)), conf))
        return out

    # ---------------- 시간 판정 ----------------
    def in_restricted_window(self):
        if bool(self.get_parameter("always_restricted").value):
            return True
        start = parse_hhmm(str(self.get_parameter("start_time").value))
        end = parse_hhmm(str(self.get_parameter("end_time").value))
        now_min = time.localtime().tm_hour * 60 + time.localtime().tm_min
        if start <= end:
            return start <= now_min < end
        return now_min >= start or now_min < end          # 자정을 넘는 구간

    # ---------------- 프레임 처리 ----------------
    def on_frame(self, msg: CompressedImage):
        self.n_frame += 1
        if not self.in_restricted_window():
            if self.alerting:
                self.clear_alert("제한시간이 끝났다")
            self.bad_streak = self.good_streak = 0
            return

        buf = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is None:
            return
        persons = self.find_persons(frame)
        self.show(frame, persons)

        if persons:
            self.bad_streak += 1
            self.good_streak = 0
        else:
            self.good_streak += 1
            self.bad_streak = 0

        need_bad = int(self.get_parameter("alert_frames").value)
        need_good = int(self.get_parameter("clear_frames").value)

        if self.bad_streak >= need_bad and not self.alerting:
            self.raise_alert(len(persons))
        elif self.alerting and self.good_streak >= need_good:
            self.clear_alert(f"{need_good}장 연속 사람 없음")
        elif self.alerting and self.bad_streak >= need_bad:
            # 계속 사람이 있으면 realert_sec 마다 다시 알린다.
            self.raise_alert(len(persons), rerung=True)

    # ---------------- 화면 보기 ----------------
    def show(self, frame, persons):
        """helmet_node.show() 와 같은 방식 — view:=true 일 때만 창을 띄운다."""
        if not bool(self.get_parameter("view").value):
            return
        img = frame.copy()
        color = (0, 0, 255) if self.alerting else (0, 200, 0)
        for (x1, y1, x2, y2), conf in persons:
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
            cv2.putText(img, f"person {conf:.2f}", (x1, max(y1 - 6, 12)),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        restricted = self.in_restricted_window()
        cv2.putText(img, f"restricted={'YES' if restricted else 'no'}  "
                         f"{'ALERTING' if self.alerting else ''}",
                    (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 0, 255) if self.alerting else (180, 180, 180), 2)
        try:
            cv2.imshow("restricted check  (q = quit)", img)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                raise KeyboardInterrupt
        except cv2.error as e:                                  # noqa: BLE001
            self.get_logger().warn(
                f"창을 띄울 수 없다({e}) — DISPLAY 확인. view:=false 로 끄면 판정은 계속된다")
            self.set_parameters([Parameter("view", value=False)])

    # ---------------- 경고 ----------------
    def raise_alert(self, n_person, rerung=False):
        now = time.time()
        if rerung and now - self.last_alarm < float(
                self.get_parameter("realert_sec").value):
            return
        self.alerting = True
        self.last_alarm = now
        self.status(f"ALERT — 작업금지 시간에 사람 {n_person}명 발견")
        self.get_logger().warn(f"작업금지 시간대 — 사람 {n_person}명 발견")
        if not event_log.log_event(
                "restricted_node", "alert", "작업금지 시간대 사람 발견",
                person_count=n_person, db_path=str(self.get_parameter("db_path").value)):
            self.get_logger().warn("이벤트 기록 실패(sqlite)", throttle_duration_sec=30.0)
        self.beep()
        self.speak()

    def clear_alert(self, why):
        # "평상시(사람 없음/해제)"는 기본 상태라 SQLite에는 안 남긴다 — alert 만 기록해서
        # 용량을 아낀다(2026-08-29 사용자 결정). 실시간 화면 표시는 /restricted/status 로 충분하다.
        self.alerting = False
        self.status(f"clear ({why})")
        self.info(f"경고 해제 — {why}")

    def beep(self):
        if not bool(self.get_parameter("sound").value):
            return
        wait = float(self.get_parameter("sound_wait_sec").value)
        if not self.cli_sound.wait_for_service(timeout_sec=wait):
            self.get_logger().warn(f"/sound 가 {wait:.0f}초 안에 안 보인다 (bringup 확인)")
            return
        value = int(self.get_parameter("sound_value").value)
        reps = max(int(self.get_parameter("sound_repeat").value), 1)
        for i in range(reps):
            req = Sound.Request()
            req.value = value
            self.cli_sound.call_async(req)
            if i < reps - 1:
                time.sleep(0.2)

    def speak(self):
        """로봇에 물린 스피커로 espeak-ng 재생. 스피커가 없으면 실패해도 무시한다."""
        if not bool(self.get_parameter("voice_enabled").value):
            return
        text = str(self.get_parameter("voice_text").value)
        host = str(self.get_parameter("robot_host").value)
        # 2026-09-02 실측: I2S 스피커(MAX98357A)가 card 1 — 기본 출력(card 0, 헤드폰잭)
        # 이 아니라 이쪽으로 명시해서 보내야 소리가 난다.
        cmd = (f'espeak-ng -v {self.get_parameter("voice_lang").value} '
              f'--stdout "{text}" | aplay -D plughw:1,0 2>&1')
        try:
            r = subprocess.run(["ssh", *SSH_OPTS, host, cmd],
                               capture_output=True, text=True, timeout=10)
            if r.returncode != 0:
                self.get_logger().warn(
                    f"음성 재생 실패(스피커 미연결일 수 있음): {r.stderr.strip()[:200]}",
                    throttle_duration_sec=30.0)
        except subprocess.SubprocessError as e:                 # noqa: BLE001
            self.get_logger().warn(f"음성 재생 ssh 실패: {e}", throttle_duration_sec=30.0)

    # ---------------- 기록 ----------------
    def quiet(self):
        return bool(self.get_parameter("quiet").value)

    def info(self, msg, **kw):
        if not self.quiet():
            self.get_logger().info(msg, **kw)

    def status(self, text):
        m = String()
        m.data = text
        self.pub_status.publish(m)

    def report(self):
        self.info(
            f"수신 {self.n_frame}장, 제한시간={'예' if self.in_restricted_window() else '아니오'}, "
            f"경고중={'예' if self.alerting else '아니오'}"
        )


def main():
    rclpy.init()
    node = RestrictedNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
