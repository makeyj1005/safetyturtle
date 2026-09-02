#!/usr/bin/env python3
"""helmet_node.py — 순찰 중 사람을 찾아 안전모 착용 여부를 판정하고, 미착용이면 세운다.

[VM에서 실행]  (절대 규칙 3: 영상 인식 연산은 로봇이 아닌 VM 에서 한다)
  ros2 run patrol_core helmet_node

  판정 기준을 눈으로 맞출 때 (세우지 않고 기록만):
  ros2 run patrol_core helmet_node --ros-args -p hold:=false -p sound:=false \
    -p save_all:=true

[로봇 웹캠은 이 노드가 켜고 끈다 — 손으로 띄울 필요 없다]
뜰 때 ssh 로 로봇의 `~/launch/webcam_node.py` 를 띄우고, 내릴 때 정리한다
(inspect_node 가 점검할 때 CSI 카메라를 다루는 방식과 같다). 이미 떠 있으면 그대로
쓰고 건드리지 않는다. 손으로 관리하려면 `manage_camera:=false`.

[입력]  /webcam/image_raw/compressed  로봇 webcam_node 가 보내는 USB 웹캠 영상
        /inspect/start   (Bool)      점검이 시작됐다  -> 웹캠·탐지를 내린다
        /inspect/status  (String)    "done ..." 이면  -> 다시 올린다
[출력]  /patrol/hold     (Bool)      True 면 patrol_node 가 즉시 선다
        /webcam/enable   (Bool)      로봇 webcam_node 를 켜고 끈다
        /helmet/status   (String)    진단용
        /sound           (service)   미착용 발견 시 부저
        logs/helmet_<시각>.csv + 증빙 사진

[카메라는 항상 한쪽만 켠다 — 2026-08-01 사용자 결정]
일반 순찰은 **USB 웹캠만** 쓰고(사람·안전모), 소화기 점검은 **CSI 만** 쓴다(압력계).
둘을 동시에 켜지 않는 이유는 성능이 아니라 무선이다: 이 프로젝트의 병목은 Wi-Fi 이고
(HANDOFF "지금 남은 문제"), 스트림을 하나 더 얹으면 /scan 이 TF 캐시를 벗어나 통째로
버려져 Nav2 가 경로를 못 만든다. 그래서 점검이 시작되면 웹캠 스트림과 탐지를 함께
내리고, 점검이 끝나면 되돌린다. 순번은 이미 patrol_scheduler 가 하나씩 배정하므로
여기서는 그 신호(/inspect/start, /inspect/status)를 듣기만 하면 된다.

[탐지 방식 — 왜 이렇게 골랐나]
VM 에 pip 가 없고 설치에는 sudo 가 필요하다. 그래서 설치 없이 되는 것만 쓴다:
  dnn  cv2.dnn + MobileNet-SSD   모델 파일 2개만 있으면 된다. 정확도가 낫다
  hog  cv2 내장 HOG 보행자 검출   파일도 필요 없다. 정확도가 낮고 느리다
기본값 auto 는 모델 파일이 있으면 dnn, 없으면 hog 로 자동으로 간다 — 모델을 받기
전에도 노드가 돌아가야 나머지(정지·부저·기록)를 먼저 검증할 수 있기 때문이다.

[안전모 판정은 왜 색인가]
안전모 전용 모델은 없고 받을 수단도 마땅치 않다(pip). 대신 안전모는 **사람 머리
위쪽에 있는 채도 높은 단색**이라는 성질이 뚜렷하다. 그래서 사람 상자의 윗부분만
잘라 HSV 로 안전모 색 비율을 재고, helmet_ratio 를 넘으면 착용으로 본다.
머리카락은 어둡고 채도가 낮아 흰색·노랑·파랑·빨강 어디에도 걸리지 않는다.

[기준 색은 찍어서 등록한다 — maps/helmet_calib.yaml]
색을 코드에 박아두면 안전모를 바꾸거나 조명이 달라질 때마다 코드를 고쳐야 한다.
압력계를 gauge_calib.yaml 로 뺀 것과 같이, 실제 안전모를 웹캠으로 찍어 기준을 잡는다:
    python3 ~/vibe/ex1/tools/helmet_calib.py --grab --name 노란안전모 --select
이 파일이 있으면 여기 값을 쓰고, 없으면 내장 일반값(helmet_colors)으로 돌아간다 —
보정 전에도 노드가 돌아가야 나머지(정지·부저·기록)를 먼저 검증할 수 있기 때문이다.
찍어서 등록한 범위가 사진에서 나온 실측값이므로 **일반값보다 항상 낫다.**

[왜 한 프레임으로 판단하지 않나]
3fps 무선 영상은 흔들리고 사람이 고개를 돌리기만 해도 한 프레임이 뒤집힌다.
한 장으로 세우면 순찰이 시도 때도 없이 멈춘다. alert_frames 장 연속으로 미착용이
보여야 세우고, clear_frames 장 연속으로 안 보여야 푼다(히스테리시스).
"""
import collections
import csv
import os
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String
from turtlebot3_msgs.srv import Sound

from patrol_core import shot_grab      # ssh 옵션·헬퍼를 그대로 쓴다(ControlMaster 재사용)

EX1 = os.path.join(os.path.expanduser("~"), "vibe", "ex1")
DEFAULT_MODEL_DIR = os.path.join(EX1, "models")
DEFAULT_LOG_DIR = os.path.join(EX1, "logs")
DEFAULT_CALIB_FILE = os.path.join(EX1, "maps", "helmet_calib.yaml")
# 2026-08-28 추가: 이 VM(노트북)에는 Docker + GPU(ROCm)로 pip 설치가 가능해져서
# (원래 제약이던 "VM 에 pip 가 없다"가 더 이상 사실이 아니다) YOLO 옵션을 추가했다.
# 로봇(Pi4)에서는 여전히 못 쓴다 — 이 노드는 원래도 VM 에서만 돈다(절대 규칙 3).
DEFAULT_YOLO_MODEL = os.path.join(EX1, "models", "helmet_yolov8n.pt")
# opencv 가 함께 설치하는 얼굴 검출기. 이 빌드에는 cv2.data 가 없어 경로를 직접 쓴다.
DEFAULT_CASCADE = "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml"

# turtlebot3_msgs/srv/Sound: 0 OFF / 1 ON / 2 LOW_BATTERY / 3 ERROR / 4,5 BUTTON
# inspect_node 와 같은 규칙 — 이상은 2번 음. (⚠️ 2번은 저전압 경고음과 같다)
SOUND_OK = 1          # ON — 착용 확인(출발)
SOUND_BAD_HELMET = 3  # ERROR — 안전모 미착용. 2(LOW_BATTERY)는 저전압 경고와 겹쳐 안 쓴다

SSH_OPTS = ["-o", "ConnectTimeout", "8", "-o", "BatchMode=yes",
           "-o", "StrictHostKeyChecking=accept-new"]

# MobileNet-SSD(VOC 20종)의 클래스 순서. 우리가 쓰는 건 person 뿐이다.
DNN_CLASSES = [
    "background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus",
    "car", "cat", "chair", "cow", "diningtable", "dog", "horse", "motorbike",
    "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor",
]
DNN_PERSON = DNN_CLASSES.index("person")
DNN_PROTO = "MobileNetSSD_deploy.prototxt"
DNN_MODEL = "MobileNetSSD_deploy.caffemodel"

# 안전모 색 (OpenCV HSV: H 0~179, S·V 0~255). 여러 구간을 갖는 색은 목록으로 둔다.
# 빨강은 H 가 0 과 179 양쪽 끝에 걸쳐 있어 두 구간이 필요하다.
HELMET_COLORS = {
    "white":  [((0, 0, 175), (179, 45, 255))],
    "yellow": [((18, 90, 110), (36, 255, 255))],
    "blue":   [((95, 90, 70), (130, 255, 255))],
    "red":    [((0, 110, 80), (9, 255, 255)), ((168, 110, 80), (179, 255, 255))],
    "green":  [((40, 80, 70), (85, 255, 255))],
    "orange": [((8, 130, 120), (18, 255, 255))],
}

CSV_COLS = ["time", "verdict", "persons", "helmet_ratio", "conf",
            "box", "head", "detector", "held", "image"]


class HelmetNode(Node):
    def __init__(self):
        super().__init__("helmet_node")

        self.declare_parameter("topic", "/webcam/image_raw/compressed")
        # auto | dnn | hog  (auto = 모델 파일이 있으면 dnn)
        self.declare_parameter("detector", "auto")
        self.declare_parameter("model_dir", DEFAULT_MODEL_DIR)
        # 사람으로 인정할 최소 확신도. 낮추면 벽·의자를 사람으로 보기 시작한다.
        self.declare_parameter("person_conf", 0.5)
        # 화면 높이 대비 이보다 작은 사람은 무시한다. 멀리 있는 오탐을 걸러내고,
        # 머리 영역이 몇 픽셀밖에 안 되면 안전모 색을 재도 의미가 없다.
        self.declare_parameter("min_person_ratio", 0.30)
        # 상자 폭이 화면의 이 비율을 넘으면 판정을 보류한다.
        # 왜 필요한가 (2026-08-02 실측): 사람이 카메라에 가까워 몸이 잘리면 검출기가
        # 상자를 화면 거의 전체로 잡는다(실제로 0,14~496,479). 그러면 "머리 영역"이
        # 사람이 아닌 **천장**으로 가고, 흰 천장·형광등이 흰 안전모로 잡혀 미착용자가
        # 착용으로 통과한다(r=0.50 으로 실제 발생). 이 경우는 착용도 미착용도 아니라
        # 판정불가로 두는 게 맞다 — 착용으로 보면 위반을 놓치고, 미착용으로 보면
        # 사람이 가까이 설 때마다 순찰이 선다.
        self.declare_parameter("max_box_width_ratio", 0.85)

        # --- 판정 방식 ---
        # hair  : 사람 상자 맨 위 얇은 띠에 **머리카락(어두움)이 보이는가**.
        #         보이면 미착용, 안 보이면 뭔가 씌워진 것 = 착용.
        # color : 머리 영역에서 안전모 색 비율을 본다(예비. 색이 뚜렷한 안전모용).
        #
        # 왜 hair 가 기본인가 — 2026-08-02 같은 구도에서 착용/미착용 양쪽을 찍어 실측:
        #   안전모 색(흰색) 덩어리: 착용 0.037~0.069 / 미착용 0.030~0.048  → **안 갈린다**
        #     (덩어리 높이가 양쪽 다 144~155px 로 같았다. 잡힌 게 안전모가 아니라
        #      천장이었다. 흰 안전모와 흰·크림 천장은 색으로 구분할 수 없다)
        #   상자 top 아래 8% 띠의 어두운 비율: 착용 0.000 / 미착용 0.073~0.120 → **갈린다**
        #   상자 top 아래 12% 띠:              착용 0.000~0.031 / 미착용 0.108~0.144
        # 얼굴 검출로 머리 위치를 찍으려는 시도도 했지만 이 각도(바닥에서 올려다봄)
        # 에서는 6장 중 1장만 맞아 못 쓴다. 반면 **상자의 위쪽 변은 정확**했다
        # (상자 top y=114 / 실제 안전모 top y≈112).
        #
        # ⚠️ hair 방식의 한계: "머리카락이 보이는가"를 보므로
        #   - 어두운 색 안전모는 미착용으로 오판한다
        #   - 머리가 밝은 사람(탈색·백발·삭발)은 착용으로 오판한다
        # 안전모 색이 뚜렷하고 배경과 다르면 color 가 더 낫다.
        # yolo   : Roboflow Hard Hat Workers 데이터셋으로 학습한 YOLOv8n
        #          (models/helmet_yolov8n.pt, 2026-08-28 추가). 사람 상자 안에서
        #          head/helmet 을 직접 찾으므로 색·조명에 덜 민감하지만, 이 프로젝트
        #          장소에서 실주행 검증은 아직 안 됐다 — color/hair 가 실전 검증됨.
        self.declare_parameter("method", "color")
        # 상자 높이의 몇 %를 띠로 볼지. **0.06 이 유일하게 전부 갈렸다**(실측 10장):
        #   띠 0.06  착용 ≤0.017 / 미착용 ≥0.033   ← 갈림
        #   띠 0.08  착용 ≤0.058 / 미착용 ≥0.056   겹침
        #   띠 0.10  착용 ≤0.091 / 미착용 ≥0.077   겹침
        # 띠를 두껍게 하면 안 되는 이유: 사람 상자의 top 이 실제 머리 top 보다 20px쯤
        # 아래로 잡히는 프레임이 있고(흐린 프레임에서 32 vs 10), 그때 두꺼운 띠는
        # 챙·이마·눈썹까지 내려가 어두워진다. 얇은 띠는 그래도 안전모 안에 남는다.
        # 머리 폭만큼 창을 슬라이딩해 최대값을 보는 방법도 재봤지만 같은 프레임 때문에
        # 어느 조합도 겹쳤다(착용 0.264~0.486 / 미착용 0.254~0.431).
        self.declare_parameter("band", 0.06)
        # 이 명도 이하를 머리카락으로 본다. 실측 머리카락 V 43~100(중앙 51) + 여유.
        self.declare_parameter("hair_vmax", 115)
        # 어두운 덩어리를 머리카락으로 인정할 최소 높이(띠 높이 대비). 천장 이음선
        # 같은 얇고 긴 선을 걸러내는 장치다 — 자세한 근거는 hair_frac() 주석.
        self.declare_parameter("hair_min_blob_h", 0.65)
        # 띠에서 머리카락이 이 비율을 넘으면 미착용.
        # 실측: 착용 0.000(13장 전부) / 미착용 0.064~0.094 → 0.02 는 여유 3배 이상.
        self.declare_parameter("hair_ratio", 0.02)
        # 상자 위쪽 변이 화면 top 에서 이 픽셀 안쪽이면 **머리가 잘렸다**고 보고
        # 판정을 보류한다. 2026-08-02 실측: 사람이 가까워 머리가 프레임을 벗어난
        # 프레임에서는 띠가 얼굴·어깨에 얹혀 안전모를 쓴 사람이 미착용으로 나왔다
        # (12장 중 2장, 둘 다 머리가 화면 밖). 안 보이는 것은 판정하지 않는 게 맞다.
        self.declare_parameter("min_top_margin_px", 6)

        # --- 안전모 판정 (color 방식) ---
        # 찍어서 등록한 기준. 있으면 helmet_colors 대신 이걸 쓴다(tools/helmet_calib.py).
        self.declare_parameter("calib_file", DEFAULT_CALIB_FILE)
        self.declare_parameter("helmet_colors", ["white", "yellow", "blue", "red"])
        # 얼굴을 찾아 그 위를 보는 방식. **기본은 꺼 둔다 — 도움이 안 됐다.**
        # 2026-08-02 실측: 카메라를 얼굴 30cm 앞에서 45° 이상 올려다보게 두면
        # ① 사람 상자가 얼굴 아래만 잡혀 안전모가 상자 밖(위)에 놓이고,
        # ② Haar 얼굴 검출은 얼굴 전체가 아니라 **눈 주변 55px**만 잡는다
        #    (실제 얼굴 약 200px). 그 크기로 환산하면 영역이 챙·이마에 걸려
        #    안전모를 쓴 사람이 미착용으로 나온다(r=0.02).
        # 크기 비율로 걸러낼 수도 없다 — 전신이 보이는 정상 거리에서는 진짜 얼굴도
        # 사람 상자의 10~13% 라 같은 검사에 걸린다. 그래서 정상 거리에서는 단순한
        # "사람 상자 위쪽" 이 맞고, 위 조건은 애초에 운용 각도가 아니다(실제로는
        # 카메라를 바닥에 두고 1~3m 앞의 사람을 본다).
        self.declare_parameter("use_face", False)
        self.declare_parameter("face_cascade", DEFAULT_CASCADE)
        # 얼굴 높이의 몇 배만큼 위를 볼지 / 얼굴 안으로 얼마나 내려올지(챙 포함).
        self.declare_parameter("head_up", 1.0)
        self.declare_parameter("head_down", 0.10)
        # 얼굴 폭의 몇 배를 좌우로 볼지(안전모가 얼굴보다 넓다).
        self.declare_parameter("head_wide", 0.80)
        # 얼굴 높이가 사람 상자 높이의 이 비율보다 작으면 얼굴로 인정하지 않는다
        # (눈 주변만 잡힌 것을 얼굴로 쓰면 안전모 위치를 잘못 환산한다).
        self.declare_parameter("face_min_ratio", 0.20)
        # 얼굴을 못 찾았을 때만 쓰는 예비 방식 — 사람 상자의 위쪽 구간.
        self.declare_parameter("head_top", 0.02)
        self.declare_parameter("head_bottom", 0.26)
        self.declare_parameter("head_width", 0.60)
        # 머리 영역에서 안전모 색이 이 비율을 넘으면 착용으로 본다.
        # 0.010 — 2026-08-03 실측(초록 테이프): 착용 0.0199~0.0289 / 미착용 0.0000.
        # 안전모 전체가 색이면 0.25 가 맞지만, 테이프 같은 표식은 머리 영역의 2~3%다.
        self.declare_parameter("helmet_ratio", 0.010)

        # --- 안전모 판정 (yolo 방식, 2026-08-28 추가) ---
        self.declare_parameter("yolo_model_path", DEFAULT_YOLO_MODEL)
        # 사람 상자 안에서 head/helmet 박스를 찾을 최소 확신도.
        self.declare_parameter("yolo_conf", 0.3)
        # score = helmet 최고확신도 - head 최고확신도. 0 이상이면 착용으로 본다.
        # (color 방식의 helmet_ratio 와 같은 자리 — "낮을수록 미착용에 가깝다")
        self.declare_parameter("yolo_margin", 0.0)
        # 'cpu' 또는 GPU 인덱스 문자열('0'). Docker 컨테이너에서 ROCm GPU 를 쓰려면
        # '0' + HSA_OVERRIDE_GFX_VERSION 환경변수가 같이 필요하다(docker/run_gpu.sh 참고).
        self.declare_parameter("yolo_device", "cpu")

        # --- 언제 세우고 언제 푸나 ---
        # **사람이 보이면 안전모 여부와 무관하게 먼저 세운다** (2026-08-02 사용자 결정).
        # 근거: 사람 검출은 확신도 0.86~0.99 로 안정적인데, 안전모 판정은 로봇이
        # 움직이는 동안 흔들린다(모션블러·거리 변화·상자 top 어긋남). 실제로 순찰 중에는
        # 미착용자를 앞에 두고도 착용(0.000)으로 지나쳤다. 반면 **정지 상태**에서는
        # 오탐 0 / 검출 79% 로 잘 됐다. 그래서 믿을 수 있는 신호로 먼저 멈추고,
        # 멈춘 상태에서 안전모를 판단한다. 착용이 확인되면 다시 출발한다.
        self.declare_parameter("hold_on_person", True)
        # 사람이 이만큼 연속 보이면 세운다. 짧게 잡아야 지나치지 않는다.
        self.declare_parameter("person_frames", 2)
        # --- 판단 창: 세운 뒤 이 시간 동안 모아서 결론을 낸다 ---
        # 한 장으로 결론내지 않는 이유(2026-08-02 실측): 프레임당 판정은 미착용을 21%
        # 놓치고, 사람 상자 위쪽 변이 머리에 딱 맞지 않는 프레임에서는 착용으로 새 나간다.
        # 반면 **정지 상태에서 미착용이라고 판정한 것은 오탐이 0** 이었다(18장 전부).
        # 그래서 세운 뒤 judge_sec 동안 모아, 미착용이 judge_bad_min 장 이상이면
        # 미착용으로 확정한다. 한 장도 없으면 착용으로 보고 출발한다.
        self.declare_parameter("judge_sec", 5.0)
        # 미착용 확정 조건: 미착용이 **사람이 보인 프레임의 judge_bad_ratio 이상**이고
        # judge_bad_min 장 이상일 때. 개수만 보면 안 된다 — 2026-08-03 실주행에서
        # 안전모를 쓰고 있었는데 14장 중 4장이 미착용으로 나와(머리가 화면에서 잘리거나
        # 돌아선 프레임) 개수 기준 2장에 걸려 잘못 세웠다. 비율로 보면 갈린다:
        #   진짜 미착용  14/14, 16/16, 11/11 = 100%
        #   오판         4/14 = 29%
        #   착용         0/14, 1/13 = 0~8%
        self.declare_parameter("judge_bad_min", 3)
        self.declare_parameter("judge_bad_ratio", 0.5)
        # 세운 뒤 착용 판정이 이만큼 연속이면 출발한다(2.8fps 에서 8장 ≈ 3초).
        # **0 으로 주면 착용 판정으로는 출발하지 않는다** — 사람이 시야에서 비켜야
        # 출발한다. 흰 안전모처럼 착용 판정을 신뢰할 수 없을 때 이 쪽이 안전하다:
        # 2026-08-02 측정에서 "미착용 발견"은 오탐 0 이었지만 "착용"(어두운 덩어리
        # 없음)은 흰 안전모와 밝은 벽·천장을 구분하지 못해 미착용자도 통과시켰다.
        self.declare_parameter("ok_frames", 8)
        # 미착용 확정 기준: **최근 alert_window 장 중 alert_frames 장**.
        # ⚠️ "연속"이면 안 된다 — 판정이 프레임당 79% 라 미착용자를 앞에 두고도
        # True,True,False,True,True,False 처럼 나와 3장 연속이 거의 성립하지 않는다
        # (2026-08-02 단위 시험에서 확인. 순찰이 안 섰던 진짜 이유가 이것이다).
        # 5/6 로 거의 만장일치를 요구한다. 이 검사는 **통과시킨 사람이 안전모를 벗는
        # 경우**를 잡는 것이라 확실할 때만 세워야 한다 — 3/6 으로 두면 머리가 잠깐
        # 화면에서 잘릴 때도 세운다(2026-08-03 실주행에서 그렇게 가끔 멈췄다).
        self.declare_parameter("alert_frames", 5)
        self.declare_parameter("alert_window", 6)
        # 해제는 **느려야 한다.** 판정은 오탐 0 을 위해 놓치는 쪽을 택했고(프레임당
        # 검출 79%, 실측 27/34), 그래서 미착용자가 그대로 서 있어도 몇 프레임은 비어
        # 있는다. 5장으로 두면 세운 지 1.7초 만에 풀렸다(2026-08-02 실주행에서 실제로
        # 그랬다). 20장이면 2.8fps 에서 약 7초 — 21% 놓침이 20번 연속 겹칠 일은 없다.
        self.declare_parameter("clear_frames", 20)
        self.declare_parameter("hold", True)
        # 부저를 다시 울리기까지의 최소 간격(초). 미착용은 계속 유지되는 상태라
        # 매 프레임 울리면 소음만 된다.
        self.declare_parameter("sound", True)
        # turtlebot3_msgs/srv/Sound: 0 OFF / 1 ON / 2 LOW_BATTERY / 3 ERROR / 4,5 BUTTON.
        # 길이·음높이는 OpenCR 펌웨어가 정하므로 "짧게/길게"는 **반복 횟수와 간격**으로 만든다.
        #   미착용 = 3번(ERROR) 짧게 3연타, realert_sec 마다 계속 반복
        #   착용   = 1번(ON) 길게 1회 (출발할 때 한 번)
        # ⚠️ 미착용에 2번(LOW_BATTERY)을 쓰지 않는다 — OpenCR 이 11V 아래에서 스스로
        #    내는 소리와 같아서 안전모 경보와 배터리 경고를 구분할 수 없다.
        self.declare_parameter("sound_value", SOUND_BAD_HELMET)
        self.declare_parameter("sound_repeat", 3)
        self.declare_parameter("sound_gap_sec", 0.2)
        # 착용 확인해 출발할 때 울리는 소리(0 이면 안 울린다)
        self.declare_parameter("sound_ok_value", SOUND_OK)
        self.declare_parameter("sound_ok_repeat", 1)
        self.declare_parameter("sound_wait_sec", 15.0)
        # 미착용인 동안 다시 울리기까지의 간격(초). 판단 창(5초)마다 울리게 맞췄다.
        self.declare_parameter("realert_sec", 5.0)

        # 음성 안내 — 로봇 스피커(I2S, MAX98357A)로 ssh+espeak-ng 재생 (2026-09-02 추가).
        # restricted_node/fire_node 와 같은 방식.
        self.declare_parameter("voice_enabled", True)
        self.declare_parameter("voice_text", "안전모를 착용하십시오")
        self.declare_parameter("voice_lang", "ko")

        # --- 화면 보기 (현장 시험용) ---
        # 창을 띄워 카메라 영상과 판정을 그대로 보여준다. 순찰·Nav2 없이 그 자리에서
        # 구도와 판정을 확인할 때 쓴다. 화면이 있는 터미널에서만 켤 것.
        # 켜면 detect_every 를 무시하고 모든 프레임을 판정한다(화면이 끊기지 않게).
        self.declare_parameter("view", False)
        # 보고서·시연 영상용. 판정 사건만 남기고 주기 보고·모드 전환 같은 줄을 접는다.
        # 기록(CSV·사진)은 그대로 남으니 나중에 원인 추적에는 지장이 없다.
        self.declare_parameter("quiet", False)

        # --- 기록 ---
        self.declare_parameter("log_dir", DEFAULT_LOG_DIR)
        # 미착용만 저장하는 게 기본. 기준을 맞출 때는 true 로 두고 전부 본다.
        self.declare_parameter("save_all", False)
        self.declare_parameter("save_every_sec", 3.0)

        # --- 로봇 웹캠 노드를 직접 띄운다 ---
        # 이게 없으면 사용자가 로봇에 ssh 로 들어가 터미널 하나를 계속 붙잡고 있어야
        # 한다. inspect_node 가 점검할 때 CSI 카메라를 ssh 로 켰다 끄는 것과 같은 방식.
        self.declare_parameter("manage_camera", True)
        # 종료할 때 로봇 웹캠 프로세스를 죽일지. **기본은 죽이지 않는다(False).**
        # 2026-08-02 실측: 이 웹캠은 스트림을 여러 번 열고 닫으면 망가진다(그 뒤로는
        # OpenCV 스트림이 select() timeout 이 되고 재부팅해야 풀렸다). 그래서 장치는
        # 부팅당 한 번만 열고 유지하는 편이 안전하다. 남겨도 무선 부하는 없다 —
        # 종료 직전에 /webcam/enable=False 를 보내 **발행을 멈추고** 나가기 때문이다.
        # 로봇을 완전히 정리해야 할 때만 True 로 준다.
        self.declare_parameter("stop_camera_on_exit", False)
        self.declare_parameter("robot_host", "rpi@192.168.0.67")
        self.declare_parameter("remote_script", "~/launch/webcam_node.py")
        # 띄운 뒤 첫 프레임까지 기다리는 시간(초). 무선 DDS 디스커버리가 느리다.
        self.declare_parameter("camera_wait_sec", 20.0)
        # 로봇이 발행할 fps. 무선·VM 부하를 줄이려면 낮춘다(2.8fps 에서 34KB/s).
        # 0 이면 webcam_node 의 기본값(3.0)을 쓴다.
        self.declare_parameter("camera_fps", 0.0)
        # --- 부하 조절: 사람이 없으면 아끼고, 보이면 전속 ---
        # VM 이 2코어라 Nav2(코스트맵·컨트롤러)와 CPU 를 다툰다(추론 12~14%).
        # 순찰 시간의 대부분은 아무도 없으므로, 평소에는 detect_every 프레임마다
        # 한 번만 판정하고 **사람이 한 프레임이라도 잡히면** detect_every_active 로
        # 바꿔 전속으로 본다. 사람이 사라진 뒤 active_hold_sec 동안은 전속을 유지한다
        # (프레임마다 오가면 판정이 들쭉날쭉해진다).
        # 아껴도 사람을 놓치지 않는 이유: 2.8fps 에서 3프레임마다 = 초당 약 1회 판정이고
        # 사람은 몇 초씩 시야에 있다. 발견까지 최대 1초 늦어질 뿐이다.
        # 기본은 1(전속)이다. 3 으로 낮춰 돌려봤더니 **감지가 늦어 로봇이 제때 서지
        # 않았다**(2026-08-02 사용자 확인). 추론이 12~14% 인데 그 절약보다 반응이 중요하다.
        # 부하가 정말 문제일 때만 올린다.
        self.declare_parameter("detect_every", 1)
        self.declare_parameter("detect_every_active", 1)
        self.declare_parameter("active_hold_sec", 5.0)
        # 추론이 Nav2 의 CPU 를 빼앗지 않게 하는 두 가지. VM 이 2코어뿐이다.
        # 2026-08-02 실주행: 전속 판정으로 돌리자 순찰 한 구간이 16초 -> 107초가 됐고
        # 중간에 멈추기도 했다. 판정 빈도를 낮추면 감지가 늦어지므로, 빈도는 그대로 두고
        # **우선순위와 스레드 수**로 양보한다.
        #   nice   : 프로세스 우선순위를 낮춘다(값이 클수록 양보). Nav2 컨트롤러가 먼저 돈다
        #   cv_threads: OpenCV 가 쓸 스레드 수. 기본은 코어 전부라 2코어를 다 먹는다.
        #               1 로 두면 한 코어만 쓰고 나머지 하나는 Nav2 몫으로 남는다
        self.declare_parameter("nice", 10)
        self.declare_parameter("cv_threads", 1)

        # --- 점검과의 상호배제 ---
        # 점검 중에는 웹캠 스트림을 내린다(파일 상단 설명).
        self.declare_parameter("yield_to_inspect", True)
        # 켜라는 신호를 몇 초마다 다시 보낼지. /webcam/enable 은 VOLATILE 이라
        # 로봇 노드가 나중에 떠도 놓치지 않으려면 주기적으로 보내야 한다.
        # 2초로 둔다 — patrol_node 가 시작지점에서 /helmet/ready 를 기다리므로
        # 주기가 길면 그만큼 출발이 늦어진다. Bool 두 개라 부하는 없다.
        self.declare_parameter("enable_period_sec", 2.0)

        self.yield_cpu()
        self.net = None
        self.hog = None
        self.detector = self.setup_detector()
        self.ranges, self.range_src = self.load_ranges()
        self.yolo_model = None
        if str(self.get_parameter("method").value) == "yolo":
            self.yolo_model = self.setup_yolo()
        self.faces = self.setup_faces()

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_hold = self.create_publisher(Bool, "/patrol/hold", qos)
        self.pub_status = self.create_publisher(String, "/helmet/status", qos)
        self.pub_cam = self.create_publisher(Bool, "/webcam/enable", qos)
        # 영상을 받고 있는지 알린다. patrol_node 가 시작지점에서 이걸 기다린다 —
        # 무선 디스커버리가 16~40초 걸려서, 이 신호 없이는 첫 바퀴를 눈 감고 돈다.
        self.pub_ready = self.create_publisher(Bool, "/helmet/ready", qos)
        # 영상은 best_effort 다 — 로봇 발행자와 QoS 가 맞아야 한 장도 안 온다.
        self.create_subscription(
            CompressedImage, str(self.get_parameter("topic").value),
            self.on_frame, qos_profile_sensor_data,
        )
        self.create_subscription(Bool, "/inspect/start", self.on_inspect_start, qos)
        self.create_subscription(String, "/inspect/status", self.on_inspect_status, qos)
        self.cli_sound = self.create_client(Sound, "/sound")

        self.owns_camera = False    # 우리가 띄운 웹캠 노드인가 (내릴 때 정리할지 판단)
        self.camera_started_at = None
        self.last_verdict = None     # 콘솔 요약용 — 착용도 보이게 하려고 들고 있는다
        self.last_score = None
        self.seen_person = 0         # 이번 보고 구간에서 사람이 잡힌 프레임 수
        self.person_at = None        # 사람이 마지막으로 보인 시각 (전속 전환 판단)
        self.was_active = False      # 전속 모드였는가 (전환 로그를 한 번만 내려고)
        self.n_held = 0              # 이번 보고 구간에서 판정보류한 프레임 수
        self.frame_faces = []       # 이번 프레임에서 찾은 얼굴들
        self.active = True          # 탐지 중인가 (점검 중이면 False)
        self.holding = False        # 지금 세워 둔 상태인가
        self.bad_streak = 0
        self.good_streak = 0
        self.ok_streak = 0          # 사람이 있는데 착용으로 판정된 연속 프레임
        self.ok_cleared = False     # 이 사람은 착용 확인해 통과시켰다(사라지면 해제)
        self.absent_streak = 0      # 사람이 안 보인 연속 프레임
        self.judging = False        # 판단 창이 돌고 있는가
        self.judge_until = 0.0
        self.w_frames = self.w_bad = self.w_person = 0   # 창 안의 표
        # 최근 판정 창 — 미착용 확정을 "연속"이 아니라 "N장 중 M장"으로 본다
        self.recent = collections.deque(
            maxlen=max(int(self.get_parameter("alert_window").value), 1))
        self.person_streak = 0      # 사람이 보인 연속 프레임 (먼저 세우는 기준)
        self.last_alarm = 0.0
        self.last_save = 0.0
        self.n_frame = 0
        self.n_judged = 0           # 실제로 판정한 장수 (수신 장수와 다르다)
        self.n_recv = 0             # 받은 프레임 수 (detect_every 로 건너뛴 것 포함)
        self.last_frame_at = None
        self.last_report = time.time()

        stamp = time.strftime("%m%d_%H%M%S")
        log_dir = str(self.get_parameter("log_dir").value)
        os.makedirs(log_dir, exist_ok=True)
        self.shot_dir = os.path.join(log_dir, f"helmet_{stamp}")
        self.csv_path = os.path.join(log_dir, f"helmet_{stamp}.csv")
        self.csv_started = False

        if str(self.get_parameter("method").value) == "hair":
            how = (f"머리카락 방식 — 상자 위 {float(self.get_parameter('band').value):.0%} "
                   f"띠에 V≤{self.get_parameter('hair_vmax').value} 화소가 "
                   f"{float(self.get_parameter('hair_ratio').value):.3f} 넘으면 미착용")
        elif str(self.get_parameter("method").value) == "yolo":
            how = (f"YOLO 방식 — {self.get_parameter('yolo_model_path').value}, "
                   f"margin {self.get_parameter('yolo_margin').value}")
        else:
            how = (f"안전모색 방식 — {self.range_src}, "
                   f"기준 비율 {self.get_parameter('helmet_ratio').value}")
        self.get_logger().info(
            f"안전모 판정 시작 — 검출={self.detector}, {how}, "
            f"{self.get_parameter('alert_frames').value}장 연속이면 정지"
            + ("" if bool(self.get_parameter("hold").value) else " (정지 꺼짐)")
        )
        self.info(f"영상 대기: {self.get_parameter('topic').value}")

        self.create_timer(float(self.get_parameter("enable_period_sec").value),
                          self.push_camera_state)
        self.create_timer(10.0, self.report)
        self.push_camera_state()
        # 로봇 웹캠 노드를 띄운다. ssh 왕복이 있어 생성자에서 몇 초 걸린다.
        self.start_remote_camera()

    # ---------------- 준비 ----------------
    def quiet(self):
        return bool(self.get_parameter("quiet").value)

    def info(self, msg, **kw):
        """조용 모드에서는 접히는 안내. 판정 사건은 이걸 쓰지 않는다."""
        if not self.quiet():
            self.get_logger().info(msg, **kw)

    def yield_cpu(self):
        """Nav2 에 CPU 를 양보한다. 위 nice / cv_threads 파라미터 설명 참고."""
        n = int(self.get_parameter("nice").value)
        if n > 0:
            try:
                os.nice(n)      # 낮추는 방향은 권한이 필요 없다
            except OSError as e:                                # noqa: BLE001
                self.get_logger().warn(f"우선순위를 낮추지 못했다: {e}")
        t = int(self.get_parameter("cv_threads").value)
        if t > 0:
            cv2.setNumThreads(t)
        self.info(
            f"CPU 양보 설정: nice=+{n}, opencv 스레드={t if t > 0 else '제한없음'} "
            "(2코어 VM 에서 Nav2 주행이 먼저 CPU 를 갖게 한다)")

    def setup_detector(self):
        want = str(self.get_parameter("detector").value)
        mdir = str(self.get_parameter("model_dir").value)
        proto, model = os.path.join(mdir, DNN_PROTO), os.path.join(mdir, DNN_MODEL)
        have = os.path.exists(proto) and os.path.exists(model)

        if want in ("auto", "dnn") and have:
            self.net = cv2.dnn.readNetFromCaffe(proto, model)
            self.get_logger().info(f"MobileNet-SSD 로드: {mdir}")
            return "dnn"
        if want == "dnn":
            self.get_logger().error(
                f"detector:=dnn 인데 모델이 없다 — {proto}, {model} 를 두거나 "
                "detector:=hog 로 실행할 것"
            )
            raise SystemExit(1)

        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        if want == "auto" and not have:
            self.get_logger().warn(
                f"모델 파일이 없어 HOG 로 시작한다 ({mdir} 에 {DNN_MODEL} 를 두면 "
                "다음 실행부터 자동으로 dnn 을 쓴다). HOG 는 정면 전신에만 잘 맞는다"
            )
        return "hog"

    def setup_yolo(self):
        """method:=yolo 일 때 안전모 전용 YOLO 모델을 로드한다.

        ultralytics 는 이 프로젝트 원래 제약("VM 에 pip 없음")과 달리 Docker 이미지
        (docker/Dockerfile.gpu)에만 들어있다 — 로봇(Pi4)이나 pip 없는 VM 에서
        method:=yolo 를 쓰면 여기서 바로 실패한다. 그럴 땐 method:=color 로 돌아갈 것.
        """
        path = str(self.get_parameter("yolo_model_path").value)
        if not os.path.exists(path):
            self.get_logger().error(
                f"method:=yolo 인데 모델 파일이 없다 — {path}. "
                "tools/train_helmet_yolo.py 로 학습하거나 method:=color 로 실행할 것"
            )
            raise SystemExit(1)
        try:
            from ultralytics import YOLO
        except ImportError:
            self.get_logger().error(
                "method:=yolo 인데 ultralytics 가 설치되어 있지 않다 — "
                "docker/Dockerfile.gpu 로 만든 이미지(patrol-ros2:gpu)에서 실행할 것"
            )
            raise SystemExit(1)
        model = YOLO(path)
        self.get_logger().info(f"YOLO 안전모 모델 로드: {path} (클래스 {model.names})")
        return model

    # ---------------- 로봇 웹캠 노드 켜고 끄기 ----------------
    def start_remote_camera(self):
        """로봇에서 webcam_node.py 를 띄운다. 이미 떠 있으면 그대로 쓴다.

        ⚠️ setsid + nohup 으로 떼어 놓는다. 그러지 않으면 ssh 가 끊길 때 SIGHUP 으로
        같이 죽는다(사용자가 터미널을 닫으면 스트림이 끊기던 이유가 이것이다).
        ⚠️ pkill 패턴에 [w] 를 쓰는 이유: ssh 가 명령줄 전체를 셸에 넘기므로 패턴을
        그대로 쓰면 pkill -f 가 자기 셸까지 죽인다(shot_grab.py 에서 겪은 것).
        """
        if not bool(self.get_parameter("manage_camera").value):
            return
        host = str(self.get_parameter("robot_host").value)
        script = str(self.get_parameter("remote_script").value)
        try:
            r = shot_grab.ssh_cmd(host, "pgrep -f '[w]ebcam_node.py' >/dev/null "
                                        "&& echo RUNNING || echo NONE", timeout=30)
        except (subprocess.SubprocessError, OSError) as e:      # noqa: BLE001
            self.get_logger().warn(
                f"로봇({host})에 접속하지 못해 웹캠을 못 띄운다: {e} — "
                "로봇에서 직접 띄우려면: "
                f"ssh {host} 'export ROS_DOMAIN_ID=3 && python3 {script}'")
            return
        if "RUNNING" in r.stdout:
            self.get_logger().info(f"로봇 웹캠 노드가 이미 떠 있다 ({host})")
            self.owns_camera = False
            return

        # ⚠️ `... && setsid nohup python3 ... &` 형태로 쓰면 안 된다. `&` 가 `&&` 체인
        # 전체에 걸려 백그라운드 **서브셸**이 만들어지고, 그 서브셸이 ssh 의 stdout/stderr
        # 를 계속 물고 있어 ssh 가 EOF 를 못 본다 — 실제로 ssh 가 60초를 기다리다
        # 실패로 처리했고(카메라는 정작 떠 있었다) 로봇에는 bash 래퍼가 남았다.
        # `( ... & ) >/dev/null 2>&1` 로 서브셸의 fd 까지 끊어야 ssh 가 바로 끊긴다.
        # 시작과 확인을 한 번에 하지 않고 두 번으로 나누는 것도 같은 이유다.
        fps = float(self.get_parameter("camera_fps").value)
        args = f" --ros-args -p fps:={fps}" if fps > 0 else ""
        start = (f"export ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '3')}; "
                 "source /opt/ros/humble/setup.bash; "
                 f"( setsid nohup python3 -u {script}{args} > ~/webcam.log 2>&1 "
                 "< /dev/null & ) >/dev/null 2>&1")
        check = ("pgrep -f '[w]ebcam_node.py' >/dev/null "
                 "&& echo STARTED || (echo FAILED; tail -5 ~/webcam.log)")
        try:
            shot_grab.ssh_cmd(host, start, timeout=30)
            time.sleep(3.0)                     # 카메라 장치를 열 시간
            r = shot_grab.ssh_cmd(host, check, timeout=30)
        except (subprocess.SubprocessError, OSError) as e:      # noqa: BLE001
            self.get_logger().warn(f"웹캠 노드를 띄우지 못했다: {e}")
            return
        if "STARTED" in r.stdout:
            self.owns_camera = True
            self.camera_started_at = time.time()
            self.get_logger().info(
                f"로봇 웹캠 노드를 띄웠다 ({host}:{script}) — "
                "무선 DDS 디스커버리 때문에 첫 프레임까지 40초쯤 걸린다(실측 42초)")
        else:
            self.get_logger().error(
                f"웹캠 노드가 뜨지 않았다 — {r.stdout.strip()} {r.stderr.strip()}")

    def stop_remote_camera(self):
        """우리가 띄운 웹캠 노드를 없앤다.

        남기면 안 되는 이유: 쓰지 않는 영상 스트림이 무선을 포화시켜 /scan 이 TF 캐시를
        벗어나 버려지고 Nav2 가 경로를 못 만든다(이 프로젝트에서 실제로 겪은 문제).
        우리가 띄운 게 아니면(사용자가 미리 띄워둔 경우) 건드리지 않는다.
        """
        if not self.owns_camera:
            return
        if not bool(self.get_parameter("stop_camera_on_exit").value):
            self.get_logger().info(
                "웹캠 발행만 멈추고 프로세스는 남긴다 — 스트림을 여러 번 여닫으면 "
                "장치가 망가진다(재부팅해야 풀림). 완전히 내리려면 "
                "stop_camera_on_exit:=true 또는 로봇에서 "
                "pkill -INT -f '[w]ebcam_node.py'")
            return
        host = str(self.get_parameter("robot_host").value)
        # ⚠️ 빨라야 한다. launch 는 SIGINT 뒤 5초면 SIGTERM, 그 뒤 10초면 SIGKILL 로
        # 올린다. 여기서 오래 붙잡으면 정리 명령을 보내기 전에 죽어 로봇에 스트림이
        # 남는다(2026-08-02 실주행에서 SIGKILL 까지 갔다 — 아슬아슬하게 정리는 됐다).
        # 그래서 원격 sleep 을 1초로 줄이고 ssh 상한도 8초로 조인다.
        try:
            r = shot_grab.ssh_cmd(
                host,
                "pkill -INT -f '[w]ebcam_node.py'; sleep 1; "
                "pkill -KILL -f '[w]ebcam_node.py' 2>/dev/null; "
                "pgrep -f '[w]ebcam_node.py' >/dev/null && echo LEFT || echo GONE",
                timeout=8)
        except (subprocess.SubprocessError, OSError) as e:      # noqa: BLE001
            self.get_logger().warn(
                f"웹캠 노드를 정리하지 못했다: {e} — 로봇에서 확인할 것: "
                f"ssh {host} \"pgrep -af '[w]ebcam_node.py'\"")
            return
        if "GONE" in r.stdout:
            self.get_logger().info("로봇 웹캠 노드를 정리했다")
        else:
            self.get_logger().warn(
                "웹캠 노드가 아직 남아 있다 — 남은 스트림이 무선을 포화시킨다. "
                f"확인: ssh {host} \"pgrep -af '[w]ebcam_node.py'\"")
        self.owns_camera = False

    def setup_faces(self):
        """얼굴 검출기를 준비한다. 없으면 예비(사람 상자 위쪽) 방식으로 돈다."""
        if not bool(self.get_parameter("use_face").value):
            self.info("얼굴 검출을 끄고 사람 상자 위쪽으로만 판정한다")
            return None
        path = str(self.get_parameter("face_cascade").value)
        if not os.path.exists(path):
            self.get_logger().warn(
                f"얼굴 검출기가 없다({path}) — 사람 상자 위쪽으로 판정한다. "
                "가까이서 위로 보는 각도에서는 안전모를 놓칠 수 있다")
            return None
        clf = cv2.CascadeClassifier(path)
        if clf.empty():
            self.get_logger().warn(f"얼굴 검출기를 읽지 못했다({path})")
            return None
        return clf

    def load_ranges(self):
        """안전모 색 범위를 정한다. 찍어서 등록한 기준이 있으면 그것을 쓴다.

        반환 ([(하한, 상한), ...], 출처설명). 여러 안전모를 등록했으면 전부 합친다 —
        어느 색이든 등록된 안전모면 착용으로 본다.
        """
        path = str(self.get_parameter("calib_file").value)
        if os.path.exists(path):
            try:
                with open(path) as f:
                    data = yaml.safe_load(f) or {}
                out = []
                names = []
                for name, e in (data.get("helmets") or {}).items():
                    for r in e.get("hsv", []):
                        out.append((tuple(int(v) for v in r[:3]),
                                    tuple(int(v) for v in r[3:])))
                    names.append(str(name))
                if out:
                    return out, f"보정값 {names} ({os.path.basename(path)})"
                self.get_logger().warn(f"{path} 에 등록된 안전모가 없다 — 일반값을 쓴다")
            except (OSError, yaml.YAMLError) as e:
                self.get_logger().warn(f"{path} 를 읽지 못했다({e}) — 일반값을 쓴다")

        out = []
        for name in self.get_parameter("helmet_colors").value:
            out.extend(HELMET_COLORS.get(str(name), []))
        if not out:
            self.get_logger().error(
                "안전모 색이 하나도 없다 — helmet_colors 를 확인할 것 "
                f"(쓸 수 있는 이름: {', '.join(HELMET_COLORS)})")
            raise SystemExit(1)
        return out, (f"일반값 {list(self.get_parameter('helmet_colors').value)} "
                     "— tools/helmet_calib.py 로 실제 안전모를 찍어 등록하면 더 정확하다")

    # ---------------- 점검과의 상호배제 ----------------
    def on_inspect_start(self, msg: Bool):
        if not bool(msg.data) or not bool(self.get_parameter("yield_to_inspect").value):
            return
        if not self.active:
            return
        self.active = False
        self.get_logger().info("점검 시작 — 웹캠 스트림과 안전모 탐지를 내린다 (CSI 차례)")
        # 세워둔 채로 점검에 들어가면 점검이 못 움직인다. 반드시 먼저 푼다.
        self.release_hold("점검 시작")
        self.push_camera_state()
        self.status("paused for inspect")

    def on_inspect_status(self, msg: String):
        if self.active or not msg.data.startswith(("done", "stopped")):
            return
        self.active = True
        self.bad_streak = self.good_streak = self.ok_streak = self.person_streak = 0
        self.ok_cleared = False
        self.judging = False
        self.get_logger().info("점검 종료 — 웹캠 스트림과 안전모 탐지를 다시 올린다")
        self.push_camera_state()
        self.status("resumed")

    def push_camera_state(self):
        """로봇 webcam_node 에 켜고 끄기를 알린다. 주기적으로 다시 보낸다."""
        m = Bool()
        m.data = self.active
        self.pub_cam.publish(m)
        # 영상 준비 여부도 함께 알린다(VOLATILE 이라 주기적으로 다시 보내야 한다).
        r = Bool()
        # 최근 3초 안에 프레임이 왔으면 준비된 것으로 본다.
        r.data = (self.active and self.last_frame_at is not None
                  and (time.time() - self.last_frame_at) < 3.0)
        self.pub_ready.publish(r)

    # ---------------- 프레임 처리 ----------------
    def detect_every(self, now):
        """지금 몇 프레임마다 판정할지. 사람이 최근에 보였으면 전속으로 본다."""
        if bool(self.get_parameter("view").value):
            return 1        # 화면을 보는 중에는 건너뛰지 않는다
        hold = float(self.get_parameter("active_hold_sec").value)
        active = self.person_at is not None and (now - self.person_at) < hold
        key = "detect_every_active" if active else "detect_every"
        if active != self.was_active:
            self.was_active = active
            self.info(
                "사람이 보인다 — 전속으로 판정한다" if active
                else f"사람이 없어졌다 — {self.get_parameter('detect_every').value}"
                     "프레임마다 판정으로 돌아간다")
        return max(int(self.get_parameter(key).value), 1)

    def on_frame(self, msg: CompressedImage):
        if not self.active:
            return
        # 부하 조절: N 프레임마다 한 번만 판정한다. 디코딩 전에 걸러야 의미가 있다.
        self.n_recv += 1
        now = time.time()
        self.last_frame_at = now                # 수신 자체는 기록해 둔다
        if self.n_recv % self.detect_every(now):
            self.n_frame += 1
            return
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn("프레임 디코딩 실패", throttle_duration_sec=5.0)
            return

        self.n_frame += 1
        self.n_judged += 1
        # 얼굴은 프레임당 한 번만 찾아 사람마다 나눠 쓴다.
        self.frame_faces = self.detect_faces(frame)
        persons = self.find_persons(frame)
        limit = float(self.get_parameter("max_box_width_ratio").value) * frame.shape[1]
        method = str(self.get_parameter("method").value)
        hair = method == "hair"
        yolo = method == "yolo"
        # 미착용에 가장 가까운 사람 하나를 고른다. hair 방식은 머리카락이 **많이**
        # 보이는 쪽이, color/yolo 방식은 안전모 쪽 점수가 **적게** 나오는 쪽이 그렇다.
        worst = None
        n_wide = 0
        top_margin = int(self.get_parameter("min_top_margin_px").value)
        for box, conf in persons:
            if box[1] <= top_margin:
                # 머리가 프레임 위로 잘렸다 — 안전모가 있는지 볼 수 없다.
                n_wide += 1
                continue
            if hair:
                region = self.hair_band(frame, box)
                score = self.hair_frac(frame, region)
                worse = worst is None or score > worst[1]
            elif yolo:
                # YOLO 는 사람 상자 안에서 head/helmet 을 직접 찾으므로 color 방식의
                # head_region(위쪽 일부만 자르기) 이 필요 없다 — 상자 전체를 넘긴다.
                region = (box[0], box[1], box[2], box[3], "box")
                score = self.helmet_score_yolo(frame, box)
                worse = worst is None or score < worst[1]
            else:
                region = self.head_region(frame, box)
                # 상자가 화면을 덮으면 머리 영역이 배경일 수 있어 버린다.
                if region[4] == "box" and box[2] - box[0] >= limit:
                    n_wide += 1
                    continue
                score = self.helmet_ratio(frame, region)
                worse = worst is None or score < worst[1]
            if worse:
                worst = (box, score, conf, region)

        self.n_held += n_wide
        if n_wide:
            # 판정은 못 했어도 사람이 있는 것은 맞으니 전속으로 올린다.
            self.person_at = time.time()
        if worst is None and n_wide:
            # 사람은 있는데 전부 판정불가다. 착용·미착용 어느 쪽으로도 세지 않는다.
            self.get_logger().warn(
                f"사람 {n_wide}명의 머리가 화면 밖이라 판정을 보류한다 "
                "(너무 가깝거나 카메라 각도가 높다 — 안전모가 보이지 않는다)",
                throttle_duration_sec=10.0)
            self.show(frame, None, "판정불가")
            if bool(self.get_parameter("save_all").value):
                self.save_shot(frame, None, "판정불가", len(persons))
            return

        if hair:
            # 머리카락이 기준보다 많이 보이면 미착용
            bad = (worst is not None
                   and worst[1] >= float(self.get_parameter("hair_ratio").value))
        elif yolo:
            bad = (worst is not None
                   and worst[1] < float(self.get_parameter("yolo_margin").value))
        else:
            bad = (worst is not None
                   and worst[1] < float(self.get_parameter("helmet_ratio").value))
        self.update_streak(bad, frame, worst, len(persons))

    def find_persons(self, frame):
        """사람 상자 목록 [(x1, y1, x2, y2), 확신도] 를 돌려준다."""
        h, w = frame.shape[:2]
        min_h = float(self.get_parameter("min_person_ratio").value) * h
        out = []

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

        # HOG. 640x480 을 그대로 넣으면 Pi 급 CPU 가 아니어도 0.3초쯤 걸린다.
        # 우리는 3fps 만 받으므로 여유가 있지만, 폭을 320 으로 줄여 더 줄인다.
        scale = 320.0 / w
        small = cv2.resize(frame, (320, int(h * scale)))
        rects, weights = self.hog.detectMultiScale(
            small, winStride=(8, 8), padding=(8, 8), scale=1.05)
        conf_min = float(self.get_parameter("person_conf").value)
        for (x, y, rw, rh), weight in zip(rects, weights):
            # HOG 의 weight 는 확률이 아니라 SVM 점수(대략 0~2)다. 확신도와 비교할 수
            # 있도록 대충 맞춰 나눈다 — 정확한 환산이 아니라 임계값을 하나로 쓰기 위함이다.
            conf = float(weight) / 2.0
            if conf < conf_min:
                continue
            x1, y1 = int(x / scale), int(y / scale)
            x2, y2 = int((x + rw) / scale), int((y + rh) / scale)
            if y2 - y1 >= min_h:
                out.append(((max(x1, 0), max(y1, 0), min(x2, w), min(y2, h)), conf))
        return out

    def detect_faces(self, frame):
        """프레임 **전체**에서 얼굴을 찾는다. 프레임당 한 번만 돌고 결과를 재사용한다.

        ⚠️ 사람 상자만 잘라서 찾으면 안 된다 — equalizeHist 는 전체 히스토그램을
        쓰므로 잘라낸 조각에서는 대비가 달라져 같은 얼굴을 놓친다(2026-08-02 실측:
        전체 프레임에서는 찾고 상자 안에서는 0개였다). 원본과 평활화 둘 다 시도한다.
        """
        if self.faces is None:
            return []
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        for g in (gray, cv2.equalizeHist(gray)):
            found = self.faces.detectMultiScale(g, scaleFactor=1.1, minNeighbors=4,
                                                minSize=(35, 35))
            if len(found):
                return [tuple(int(v) for v in f) for f in found]
        return []

    def find_face(self, box):
        """사람 상자에 걸치는 얼굴 중 가장 큰 것. 없거나 너무 작으면 None.

        ⚠️ 크기 검사가 필요하다. 아래에서 올려다보는 각도에서는 검출기가 얼굴 전체가
        아니라 **눈 주변만** 49px 짜리로 잡는 일이 있었다(실제 얼굴은 200px). 그
        크기로 안전모 위치를 환산하면 챙·이마만 보게 되어 착용을 미착용으로 판정한다.
        """
        x1, y1, x2, y2 = box
        bh = y2 - y1
        best = None
        for fx, fy, fw, fh in self.frame_faces:
            cx, cy = fx + fw // 2, fy + fh // 2
            # 얼굴 중심이 사람 상자의 가로 범위 안에 있고, 상자보다 크게 위가 아닐 때
            if not (x1 <= cx <= x2 and y1 - bh * 0.5 <= cy <= y2):
                continue
            if fh < bh * float(self.get_parameter("face_min_ratio").value):
                continue
            if best is None or fw * fh > best[2] * best[3]:
                best = (fx, fy, fw, fh)
        return best

    def head_region(self, frame, box):
        """안전모를 볼 영역과 그 근거. (x1, y1, x2, y2, "face"|"box")"""
        if bool(self.get_parameter("use_face").value):
            face = self.find_face(box)
            if face is not None:
                fx, fy, fw, fh = face
                up = float(self.get_parameter("head_up").value)
                down = float(self.get_parameter("head_down").value)
                wide = float(self.get_parameter("head_wide").value)
                cx = fx + fw // 2
                half = int(fw * (0.5 + wide / 2))
                top = max(fy - int(fh * up), 0)
                bot = min(fy + int(fh * down), frame.shape[0])
                return (max(cx - half, 0), top,
                        min(cx + half, frame.shape[1]), max(bot, top + 1), "face")

        # 예비: 사람 상자의 위쪽 가운데. 전신이 멀리 보일 때는 이것도 맞는다.
        x1, y1, x2, y2 = box
        bh, bw = y2 - y1, x2 - x1
        top = y1 + int(bh * float(self.get_parameter("head_top").value))
        bot = y1 + int(bh * float(self.get_parameter("head_bottom").value))
        half = int(bw * float(self.get_parameter("head_width").value) / 2)
        cx = (x1 + x2) // 2
        return (max(cx - half, 0), max(top, 0), cx + half, max(bot, top + 1), "box")

    def hair_band(self, frame, box):
        """사람 상자 맨 위의 얇은 띠. 여기 머리카락이 보이면 미착용이다.

        상자의 **위쪽 변**을 쓰는 이유: 실측에서 상자 top 이 머리 top 과 거의 같았다
        (y=114 vs 실제 112). 반대로 상자 폭은 팔까지 포함해 머리 위치를 못 준다.
        """
        x1, y1, x2, y2 = box
        band = max(int((y2 - y1) * float(self.get_parameter("band").value)), 4)
        return (max(x1, 0), max(y1, 0), min(x2, frame.shape[1]),
                min(y1 + band, frame.shape[0]), "band")

    def hair_frac(self, frame, region):
        """띠에서 머리카락이 차지하는 비율(0~1). **띠 높이를 채우는 덩어리만** 센다.

        높이 조건이 이 판정의 핵심이다 — 2026-08-02 실측:
        띠는 사람 상자 폭 전체(약 460px)인데 머리는 그중 115px 뿐이다. 남은 75%가
        천장이라 **천장 타일 이음선** 한 줄(길고 두께 2~4px)이 머리카락과 같은 크기의
        어두운 화소를 만든다. 그래서 단순 비율로는 안전모를 쓴 사람이 미착용으로
        나갔다(실주행에서 실제로 정지가 걸렸다: 착용 0.000~0.108 / 미착용 0.056~0.115).
        머리카락 덩어리는 띠 높이를 꽉 채우고 이음선은 몇 px 이므로, 덩어리 높이가
        띠의 hair_min_blob_h 이상인 것만 세면 갈린다:
            착용 13장 전부 0.000 / 미착용 17장 중 15장 0.064~0.094 (2장은 0)
        놓치는 프레임이 12% 있지만 고립돼 있어 alert_frames 연속 조건에 걸리지 않는다.
        오탐(잘못 세우는 것)이 0 인 쪽을 택한 것이다.
        """
        hx1, hy1, hx2, hy2 = region[:4]
        patch = frame[hy1:hy2, hx1:hx2]
        if patch.size == 0:
            return 0.0
        v = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)[:, :, 2]
        mask = (v <= int(self.get_parameter("hair_vmax").value)).astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

        band_h = mask.shape[0]
        min_h = band_h * float(self.get_parameter("hair_min_blob_h").value)
        n, _, stats, _ = cv2.connectedComponentsWithStats((mask > 0).astype(np.uint8), 8)
        area = sum(stats[i, cv2.CC_STAT_AREA] for i in range(1, n)
                   if stats[i, cv2.CC_STAT_HEIGHT] >= min_h)
        return float(area) / mask.size

    def helmet_ratio(self, frame, region):
        """머리 영역에서 안전모 색이 차지하는 비율(0~1)."""
        hx1, hy1, hx2, hy2 = region[:4]
        head = frame[hy1:hy2, hx1:hx2]
        if head.size == 0:
            return 0.0
        hsv = cv2.cvtColor(head, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo, hi in self.ranges:
            mask |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        # 잡티(머리카락 사이 반사 등)를 지운다. 안전모는 덩어리로 보인다.
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        return float(np.count_nonzero(mask)) / mask.size

    def helmet_score_yolo(self, frame, box):
        """사람 상자 안에서 YOLO 로 head/helmet 을 찾아 점수를 낸다.

        score = helmet 최고확신도 - head 최고확신도. color 방식의 helmet_ratio 와
        같은 방향(낮을수록 미착용에 가깝다)으로 맞춰서 on_frame 의 worst 선택 로직을
        그대로 재사용한다. 둘 다 없으면(잘림·각도 등) 0 — 판정 유보에 가깝게 둔다.
        """
        x1, y1, x2, y2 = box
        crop = frame[max(y1, 0):y2, max(x1, 0):x2]
        if crop.size == 0:
            return 0.0
        conf_min = float(self.get_parameter("yolo_conf").value)
        results = self.yolo_model.predict(
            crop, conf=conf_min,
            device=str(self.get_parameter("yolo_device").value),
            verbose=False)
        helmet_conf, head_conf = 0.0, 0.0
        names = self.yolo_model.names
        for b in results[0].boxes:
            cls = names[int(b.cls)]
            conf = float(b.conf)
            if cls == "helmet":
                helmet_conf = max(helmet_conf, conf)
            elif cls == "head":
                head_conf = max(head_conf, conf)
        return helmet_conf - head_conf

    # ---------------- 판정 누적 ----------------
    def update_streak(self, bad, frame, worst, n_person):
        """판정을 누적해 세우고 푼다.

        흐름 (hold_on_person=True):
            사람 2장 연속       -> 세운다 (안전모 여부와 무관)
            멈춘 뒤 미착용 3장  -> 부저 (계속 정지)
            멈춘 뒤 착용 8장    -> 푼다 (출발)
            사람이 20장 없음    -> 푼다
        """
        need_bad = int(self.get_parameter("alert_frames").value)
        need_good = int(self.get_parameter("clear_frames").value)
        need_ok = int(self.get_parameter("ok_frames").value)
        need_person = int(self.get_parameter("person_frames").value)

        # 최근 판정을 창에 넣는다. 미착용 확정은 이 창의 개수로 본다(연속이 아니다).
        self.recent.append(1 if bad else 0)
        n_bad_recent = sum(self.recent)

        if bad:
            self.bad_streak += 1
            self.good_streak = 0
            self.ok_streak = 0
        else:
            self.good_streak += 1
            self.bad_streak = 0
            # 사람이 있는데 착용인 경우만 ok 로 센다. 사람이 없으면 good_streak 몫이다.
            self.ok_streak = self.ok_streak + 1 if worst is not None else 0

        if worst is not None or n_person:
            self.person_streak += 1
            self.absent_streak = 0
        else:
            self.person_streak = 0
            self.absent_streak += 1
            # 사람이 **충분히 오래** 안 보여야 다음에 다시 세운다.
            # 한 프레임만 안 보여도 풀면(검출이 프레임당 40~90% 로 끊긴다) 통과시킨
            # 직후 같은 사람에게 또 걸려 정지·출발을 반복한다 — 2026-08-02 실주행에서
            # 30초 동안 네 번 반복했다.
            if self.absent_streak >= int(self.get_parameter("clear_frames").value):
                self.ok_cleared = False

        verdict = "미착용" if bad else ("착용" if worst is not None else "사람없음")
        # 착용일 때는 아무 로그도 안 나가서 "사람이 안 잡힌 것"과 "착용으로 판정된 것"을
        # 구분할 수 없었다(2026-08-02 실주행에서 실제로 원인 파악이 막혔다).
        # report() 가 10초마다 이 요약을 함께 낸다.
        self.last_verdict = verdict
        self.last_score = worst[1] if worst else None
        if n_person:
            # 한 명이라도 잡히면(착용이든 미착용이든) 전속 판정으로 올린다.
            self.person_at = time.time()
            self.seen_person += 1
        self.show(frame, worst, verdict)
        save_all = bool(self.get_parameter("save_all").value)
        if save_all or (bad and n_bad_recent == need_bad):
            self.save_shot(frame, worst, verdict, n_person)

        on_person = bool(self.get_parameter("hold_on_person").value)
        now = time.time()

        # 판단 창이 돌고 있으면 표만 세고, 시간이 되면 결론을 낸다.
        if self.judging:
            self.w_frames += 1
            self.w_bad += 1 if bad else 0
            self.w_person += 1 if (worst is not None or n_person) else 0
            if now >= self.judge_until:
                self.decide_window()
            return

        if n_bad_recent >= need_bad:
            # 미착용이 확실하면 **언제든** 세운다. 통과시킨 사람이 안전모를 벗는 경우가
            # 있어서다 — 이 검사가 없으면 통과 잠금장치(ok_cleared) 때문에 그 사람을
            # 다시 판단하지 않는다(단위 시험에서 실제로 그냥 지나쳤다).
            self.ok_cleared = False
            self.take_hold(worst, why=None)
            if not self.judging:
                self.start_window(now)
        elif on_person and not self.holding and not self.ok_cleared \
                and self.person_streak >= need_person:
            # 사람이 보인다 — 먼저 세우고 나서 판단한다(바로 결론내지 않는다).
            # ok_cleared 검사가 없으면 통과시킨 직후 같은 사람을 보고 또 세워서
            # 정지·출발을 무한히 반복한다(단위 시험에서 실제로 그랬다).
            self.take_hold(worst, why="사람 발견 — 판단을 시작한다", alarm=False)
            self.start_window(now)
        elif self.holding and self.good_streak >= need_good:
            self.release_hold(f"{need_good}장 연속 이상 없음")

    def start_window(self, now):
        """판단 창을 시작한다."""
        sec = float(self.get_parameter("judge_sec").value)
        self.judging = True
        self.judge_until = now + sec
        self.w_frames = self.w_bad = self.w_person = 0
        self.info(f"{sec:.0f}초 동안 안전모를 판단한다")

    def decide_window(self):
        """모은 표로 결론을 낸다. 미착용이면 계속 정지하고 다시 판단한다."""
        self.judging = False
        # 개수와 비율을 **둘 다** 넘어야 미착용으로 본다(위 파라미터 설명).
        need = max(int(self.get_parameter("judge_bad_min").value),
                   int(round(self.w_person
                             * float(self.get_parameter("judge_bad_ratio").value))))
        tally = (f"{self.w_frames}장 중 미착용 {self.w_bad}장, "
                 f"사람 {self.w_person}장")
        if self.w_person == 0:
            self.get_logger().info(
                "사람이 없어졌다 — 순찰 재개" if self.quiet() else
                f"판단 결과: 사람이 없다 ({tally})")
            self.release_hold("사람이 사라졌다")
            return
        if self.w_bad >= need:
            self.get_logger().warn(
                "❗ 안전모 미착용 — 정지 유지" if self.quiet() else
                f"판단 결과: **안전모 미착용** ({tally}, 기준 {need}장) — 계속 정지한다")
            self.status(f"미착용 확정 ({tally})")
            self.alarm()
            # 안전모를 쓰거나 비킬 때까지 계속 본다.
            self.start_window(time.time())
            return
        self.get_logger().info(
            "✅ 안전모 착용 확인 — 순찰 재개" if self.quiet() else
            f"판단 결과: 안전모 착용으로 본다 ({tally}) — 출발한다")
        self.ok_cleared = True
        self.alarm(ok=True)         # 통과 신호(1번, 길게 한 번) — 미착용 소리와 구분된다
        self.release_hold(f"안전모 착용 판단 ({tally})")

    def take_hold(self, worst, why=None, alarm=True):
        """세운다. why 가 없으면 미착용 확정으로 보고 부저를 울린다."""
        ratio = worst[1] if worst else 0.0
        if why is None:
            if str(self.get_parameter("method").value) == "hair":
                why = (f"안전모 미착용 — 머리카락 {ratio:.3f} ≥ "
                       f"{float(self.get_parameter('hair_ratio').value):.3f}")
            else:
                why = (f"안전모 미착용 — 안전모 비율 {ratio:.2f} < "
                       f"{float(self.get_parameter('helmet_ratio').value):.2f}")
        if not self.holding:
            self.holding = True
            if bool(self.get_parameter("hold").value):
                m = Bool()
                m.data = True
                self.pub_hold.publish(m)
            if self.quiet():
                # 조용 모드에서는 사유를 짧게. 수치는 CSV 에 남는다.
                msg = ("사람 발견 — 순찰 정지, 안전모 확인 중"
                       if why and why.startswith("사람 발견")
                       else "❗ 안전모 미착용 — 순찰 정지")
            else:
                msg = f"순찰 정지 — {why}"
            self.get_logger().warn(
                msg + ("" if bool(self.get_parameter("hold").value)
                       else " [정지 꺼짐: 기록만]"))
            self.status(f"hold ({why})")
        if alarm:
            self.alarm()

    def release_hold(self, why):
        if not self.holding:
            return
        self.holding = False
        self.judging = False        # 세운 상태가 끝나면 판단 창도 끝난다
        m = Bool()
        m.data = False
        self.pub_hold.publish(m)
        self.info(f"해제 — 순찰을 이어간다 ({why})")
        self.status("released")

    # ---------------- 부저 ----------------
    def alarm(self, ok=False):
        """부저를 울린다.

        ok=False (미착용) — sound_value 를 sound_repeat 번 짧게. 미착용이 유지되는
                           동안 realert_sec(5초, 판단 창과 같은 주기)마다 계속 울린다.
        ok=True  (착용)   — sound_ok_value 를 sound_ok_repeat 번. 출발할 때 한 번만
                           울리므로 realert_sec 간격 제한을 받지 않는다.
        """
        if not bool(self.get_parameter("sound").value):
            return
        now = time.time()
        if not ok and now - self.last_alarm < float(
                self.get_parameter("realert_sec").value):
            return
        wait = float(self.get_parameter("sound_wait_sec").value)
        if not self.cli_sound.wait_for_service(timeout_sec=wait):
            # 무선 DDS 는 원격 서비스 발견이 느리다(실측 11.5초). 짧게 잡으면
            # 부저가 멀쩡한데 "없다"고 넘어간다 — inspect_node 와 같은 이유.
            self.get_logger().warn(
                f"/sound 가 {wait:.0f}초 안에 안 보여 부저를 못 울린다 "
                "(로봇 bringup 확인)")
            return
        if ok:
            value = int(self.get_parameter("sound_ok_value").value)
            reps = max(int(self.get_parameter("sound_ok_repeat").value), 1)
        else:
            self.last_alarm = now
            value = int(self.get_parameter("sound_value").value)
            reps = max(int(self.get_parameter("sound_repeat").value), 1)
            self.speak()
        if value <= 0:
            return
        gap = float(self.get_parameter("sound_gap_sec").value)
        fut = None
        for i in range(reps):
            req = Sound.Request()
            req.value = value
            fut = self.cli_sound.call_async(req)
            if i < reps - 1:
                time.sleep(gap)

    def speak(self):
        """로봇 스피커(I2S, card 1)로 음성 안내. 실패해도 무시한다."""
        if not bool(self.get_parameter("voice_enabled").value):
            return
        text = str(self.get_parameter("voice_text").value)
        host = str(self.get_parameter("robot_host").value)
        cmd = (f'espeak-ng -v {self.get_parameter("voice_lang").value} '
              f'--stdout "{text}" | aplay -D plughw:1,0 2>&1')
        try:
            r = subprocess.run(["ssh", *SSH_OPTS, host, cmd],
                               capture_output=True, text=True, timeout=10)
            if r.returncode != 0:
                self.get_logger().warn(
                    f"음성 재생 실패: {r.stderr.strip()[:200]}", throttle_duration_sec=30.0)
        except subprocess.SubprocessError as e:                 # noqa: BLE001
            self.get_logger().warn(f"음성 재생 ssh 실패: {e}", throttle_duration_sec=30.0)

        def on_done(f):
            try:
                r = f.result()
            except Exception as e:                             # noqa: BLE001
                self.get_logger().warn(f"부저 호출 실패: {e}")
                return
            if r is not None and not r.success:
                self.get_logger().warn(f"부저가 울리지 않았다: {r.message}")

        fut.add_done_callback(on_done)

    # ---------------- 그림 ----------------
    def annotate(self, frame, worst, verdict):
        """판정 결과를 그려 넣은 사본을 돌려준다. 저장과 화면 보기가 함께 쓴다.

        한글은 cv2.putText 로 안 나온다(폰트가 없다) — 로마자로 적는다.
        """
        img = frame.copy()
        if worst is None:
            cv2.putText(img, "NO PERSON" if verdict == "사람없음" else "UNJUDGED",
                        (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)
            return img
        box, score, conf, region = worst
        x1, y1, x2, y2 = box
        color = (0, 0, 255) if verdict == "미착용" else (0, 200, 0)
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        # region 은 판정에 쓴 영역 — hair 방식이면 상자 맨 위 띠, color 방식이면 머리 영역
        cv2.rectangle(img, region[:2], region[2:4], (255, 200, 0), 1)
        cv2.putText(img, f"{'NO HELMET' if verdict == '미착용' else 'helmet'} "
                         f"r={score:.3f} c={conf:.2f} {region[4]}",
                    (x1, max(y1 - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        return img

    def show(self, frame, worst, verdict):
        """창에 띄운다. q 또는 ESC 로 닫으면 노드도 끝난다."""
        if not bool(self.get_parameter("view").value):
            return
        img = self.annotate(frame, worst, verdict)
        thr = (float(self.get_parameter("hair_ratio").value)
               if str(self.get_parameter("method").value) == "hair"
               else float(self.get_parameter("helmet_ratio").value))
        cv2.putText(img, f"thr={thr:.3f}  {'HOLD' if self.holding else ''}",
                    (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 0, 255) if self.holding else (180, 180, 180), 2)
        try:
            cv2.imshow("helmet check  (q = quit)", img)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                raise KeyboardInterrupt
        except cv2.error as e:                                  # noqa: BLE001
            self.get_logger().warn(
                f"창을 띄울 수 없다({e}) — 화면이 있는 터미널에서 실행할 것 "
                "(DISPLAY 확인). view:=false 로 끄면 판정은 계속된다")
            self.set_parameters([Parameter("view", value=False)])

    # ---------------- 기록 ----------------
    def save_shot(self, frame, worst, verdict, n_person):
        now = time.time()
        if now - self.last_save < float(self.get_parameter("save_every_sec").value):
            return
        self.last_save = now
        os.makedirs(self.shot_dir, exist_ok=True)
        img = self.annotate(frame, worst, verdict)
        name = f"{time.strftime('%H%M%S')}_{'bad' if verdict == '미착용' else 'ok'}.jpg"
        path = os.path.join(self.shot_dir, name)
        cv2.imwrite(path, img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        self.write_row({
            "time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "verdict": verdict,
            "persons": n_person,
            "helmet_ratio": f"{worst[1]:.3f}" if worst else "",
            "conf": f"{worst[2]:.2f}" if worst else "",
            "box": "-".join(str(v) for v in worst[0]) if worst else "",
            "head": worst[3][4] if worst else "",
            "detector": self.detector,
            "held": "Y" if self.holding else "",
            "image": path,
        })

    def write_row(self, row):
        new = not self.csv_started
        with open(self.csv_path, "a", newline="") as f:
            w = csv.DictWriter(f, fieldnames=CSV_COLS)
            if new:
                w.writeheader()
                self.csv_started = True
            w.writerow(row)

    # ---------------- 진단 ----------------
    def status(self, text):
        m = String()
        m.data = text
        self.pub_status.publish(m)

    def report(self):
        now = time.time()
        if not self.active:
            return
        if self.last_frame_at is None:
            # 갓 띄운 뒤에는 아직 안 오는 게 정상이다(디스커버리 실측 42초).
            # 고장으로 오해하지 않게 문구를 나눈다.
            since = (now - self.camera_started_at) if self.camera_started_at else None
            if since is not None and since < 60.0:
                self.info(
                    f"웹캠을 띄운 뒤 {since:.0f}초 — 첫 프레임을 기다리는 중 "
                    "(무선 디스커버리에 40초쯤 걸린다)")
                return
            self.get_logger().warn(
                f"영상이 한 장도 안 온다 — 로봇에서 webcam_node 가 떠 있는지, "
                f"토픽 이름({self.get_parameter('topic').value})과 "
                "ROS_DOMAIN_ID 가 맞는지 확인",
                throttle_duration_sec=20.0)
            return
        if now - self.last_frame_at > 5.0:
            self.get_logger().warn(
                f"{now - self.last_frame_at:.0f}초째 새 프레임이 없다 (무선/카메라 확인)")
            return
        fps = self.n_frame / max(now - self.last_report, 1e-3)
        # 사람이 몇 프레임에서 잡혔는지 / 마지막 판정이 무엇인지 함께 낸다.
        # 이게 없으면 "안 잡힌 것"과 "착용으로 본 것"을 콘솔에서 구분할 수 없다.
        # 분모는 **판정한** 장수다(수신 장수와 다르다 — detect_every 로 건너뛰므로).
        seen = (f"판정 {self.n_judged}장 중 사람 {self.seen_person}장, "
                f"최근={self.last_verdict}"
                + (f" ({self.last_score:.3f})" if self.last_score is not None else ""))
        mode = "전속" if self.was_active else f"{self.get_parameter('detect_every').value}프레임마다"
        self.info(
            f"수신 {self.n_frame}장 ({fps:.1f}fps, {mode}) — {seen}"
            + (f", 보류 {self.n_held}" if self.n_held else "")
            + (" — 정지 중" if self.holding else ""))
        self.n_frame = 0
        self.n_judged = 0
        self.seen_person = 0
        self.n_held = 0
        self.last_report = now

    def stop(self):
        # 노드를 내릴 때 hold 를 걸어둔 채로 두면 순찰이 영영 멈춰 있는다.
        if self.holding:
            m = Bool()
            m.data = False
            self.pub_hold.publish(m)
        if bool(self.get_parameter("view").value):
            cv2.destroyAllWindows()
        # 프로세스를 남길 때는 발행을 멈춰 무선을 비워둔다(장치는 열어둔 채로).
        self.active = False
        self.push_camera_state()
        self.stop_remote_camera()


def main():
    rclpy.init()
    try:
        node = HelmetNode()
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
