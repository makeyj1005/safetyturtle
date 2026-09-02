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

SSH_OPTS = ["-o", "ConnectTimeout=8", "-o", "BatchMode=yes",
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
        # 2026-09-02 사용자 결정: 야간 작업금지는 자정~06:00.
        # (화재 감지는 시간 제한 없이 24시간 — fire_node 는 이 창을 아예 안 본다)
        self.declare_parameter("start_time", "00:00")
        self.declare_parameter("end_time", "06:00")
        # 테스트용 — 켜면 시간 무관하게 항상 제한시간으로 취급한다.
        # 운용 중에는 /restricted/mode 토픽(웹 대시보드)으로 바꾸는 걸 권한다.
        self.declare_parameter("always_restricted", False)
        # 작업자가 웹에서 바꾸는 운전 모드. auto=시간표대로, on=강제 감시,
        # off=감시 중지. 파라미터가 아니라 토픽으로 받는 이유: 대시보드가 파라미터
        # 서비스를 부르려면 노드 이름·타입을 알아야 하는데, 토픽이면 그냥 쏘면 된다.
        self.declare_parameter("mode", "auto")

        # 판정 히스테리시스 — helmet_node 와 같은 방식(연속 프레임)
        self.declare_parameter("alert_frames", 3)
        self.declare_parameter("clear_frames", 10)
        self.declare_parameter("realert_sec", 8.0)

        self.declare_parameter("sound", True)
        self.declare_parameter("sound_value", SOUND_RESTRICTED)
        self.declare_parameter("sound_repeat", 2)
        self.declare_parameter("sound_wait_sec", 15.0)

        # 음성 안내 — 로봇 speaker_node 에 /speaker/play 로 이름만 보낸다(fire_node 와 동일).
        self.declare_parameter("voice_enabled", True)
        self.declare_parameter("voice_sound", "intrusion")

        self.declare_parameter("quiet", False)
        self.declare_parameter("view", False)
        self.declare_parameter("db_path", event_log.DEFAULT_DB)
        # 침입 감지 시 증거 사진을 logs/shots_web/ 에 저장한다(대시보드에서 볼 수 있다).
        self.declare_parameter("save_shot", True)

        self.net = None
        self.hog = None
        self.yolo_model = None
        self.detector = self.setup_detector()

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_status = self.create_publisher(String, "/restricted/status", qos)
        self.pub_speaker = self.create_publisher(String, "/speaker/play", qos)
        self.create_subscription(String, "/restricted/mode", self.on_mode, qos)
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
    def in_time_window(self):
        """지금이 시간표상 작업금지 시간인가 (모드와 무관하게 순수 시간만 본다)."""
        start = parse_hhmm(str(self.get_parameter("start_time").value))
        end = parse_hhmm(str(self.get_parameter("end_time").value))
        now_min = time.localtime().tm_hour * 60 + time.localtime().tm_min
        if start <= end:
            return start <= now_min < end
        return now_min >= start or now_min < end          # 자정을 넘는 구간

    def in_restricted_window(self):
        """실제로 감시할 것인가 = 모드 + 시간표를 합친 결론."""
        if bool(self.get_parameter("always_restricted").value):
            return True
        mode = str(self.get_parameter("mode").value).lower()
        if mode == "on":
            return True
        if mode == "off":
            return False
        return self.in_time_window()                      # auto

    def on_mode(self, msg: String):
        """웹 대시보드가 보내는 운전 모드 변경 (auto | on | off)."""
        want = msg.data.strip().lower()
        if want not in ("auto", "on", "off"):
            self.get_logger().warn(f"알 수 없는 모드 '{msg.data}' — 무시한다")
            return
        cur = str(self.get_parameter("mode").value).lower()
        if want == cur:
            return
        self.set_parameters([Parameter("mode", value=want)])
        self.get_logger().warn(f"작업금지 감시 모드 변경: {cur} -> {want}")
        # 감시를 끄면 켜져 있던 경고도 함께 내린다(화면에 남아 오해하지 않게).
        if want == "off" and self.alerting:
            self.clear_alert("감시 모드 off")

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
            self.raise_alert(len(persons), frame=frame, persons=persons)
        elif self.alerting and self.good_streak >= need_good:
            self.clear_alert(f"{need_good}장 연속 사람 없음")
        elif self.alerting and self.bad_streak >= need_bad:
            # 계속 사람이 있으면 realert_sec 마다 다시 알린다.
            self.raise_alert(len(persons), rerung=True, frame=frame, persons=persons)

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
    def raise_alert(self, n_person, rerung=False, frame=None, persons=None):
        now = time.time()
        if rerung and now - self.last_alarm < float(
                self.get_parameter("realert_sec").value):
            return
        self.alerting = True
        self.last_alarm = now
        self.status(f"ALERT — 작업금지 시간에 사람 {n_person}명 발견")
        self.get_logger().warn(f"작업금지 시간대 — 사람 {n_person}명 발견")

        # 증거 사진을 남긴다(사양서: "사람이 감지된 시간과 카메라로 찍은 사진을
        # 웹서버에 기록"). 사람 상자를 그려서 저장한다 — 나중에 볼 때 왜 경고가
        # 났는지 바로 알 수 있어야 한다.
        image = None
        if frame is not None and bool(self.get_parameter("save_shot").value):
            shot = frame.copy()
            for (x1, y1, x2, y2), conf in (persons or []):
                cv2.rectangle(shot, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(shot, f"person {conf:.2f}", (x1, max(y1 - 6, 12)),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            stamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now))
            cv2.putText(shot, f"INTRUSION {stamp}", (10, 22),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            image = event_log.save_shot(shot, prefix="intrusion")
            if image is None:
                self.get_logger().warn("증거 사진 저장 실패", throttle_duration_sec=30.0)

        if not event_log.log_event(
                "restricted_node", "alert", "작업금지 시간대 사람 발견",
                person_count=n_person, image=image,
                db_path=str(self.get_parameter("db_path").value)):
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
        """로봇 speaker_node 에 재생 요청만 보낸다(ssh 안 씀 — speaker_node.py 주석 참고)."""
        if not bool(self.get_parameter("voice_enabled").value):
            return
        m = String()
        m.data = str(self.get_parameter("voice_sound").value)
        self.pub_speaker.publish(m)

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
            f"수신 {self.n_frame}장, 모드={self.get_parameter('mode').value}, "
            f"감시중={'예' if self.in_restricted_window() else '아니오'}, "
            f"경고중={'예' if self.alerting else '아니오'}"
        )
        # 경고가 없을 때도 대시보드가 "지금 감시 중인지 / 모드가 뭔지"를 알아야 하므로
        # 주기적으로 현재 상태를 낸다(경고 중이면 raise_alert 가 이미 ALERT 를 낸다).
        if not self.alerting:
            mode = str(self.get_parameter("mode").value).lower()
            watching = self.in_restricted_window()
            self.status(
                f"idle mode={mode} watching={'yes' if watching else 'no'} "
                f"window={self.get_parameter('start_time').value}-"
                f"{self.get_parameter('end_time').value}"
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
