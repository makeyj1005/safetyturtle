#!/bin/bash
# csi_camera.sh — 로봇에서 CSI 카메라(ov5647)를 camera_ros 로 띄운다.
#
# [로봇에서 실행]
#   ~/launch/csi_camera.sh              # 점검용 고해상도(1640x1232, 5fps)
#   ~/launch/csi_camera.sh 640 480 15   # 조준용 저해상도
#   ~/launch/csi_camera.sh --stop
#
# [왜 OpenCV 로 직접 못 읽는가 — 2026-09-04 실측]
# /dev/video0 은 unicam raw 장치라 cv2.VideoCapture 로 열려도 프레임이 안 나온다
# (ISP 파이프라인이 필요하다). libcamera 기반 camera_ros 를 써야 한다.
#
# [⚠️ camera 인덱스 함정]
# USB 웹캠이 꽂혀 있으면 libcamera 가 카메라를 두 대로 보고 인덱스가 밀린다.
#   camera:=0  USB 웹캠 (MJPEG)
#   camera:=1  CSI ov5647 (RGB888)   ← 우리가 쓰는 것
# 확인법: 띄울 때 로그의 "stream formats" 에 RGB888 이 있으면 CSI 다.
# USB 를 빼면 CSI 가 0 이 되므로 그때는 CSI_INDEX=0 으로 준다.
CSI_INDEX="${CSI_INDEX:-1}"

# [해상도 상한 1640x1232 — CMA 메모리]
# 이보다 크게 요청하면 센서가 전체 모드로 바뀌어 연속 메모리(CMA 64MB)를 넘긴다
# ("Cannot allocate memory"). 압력계 판정에는 1640x1232 로 충분하다
# (그 거리에서 게이지가 약 48px — 640x480 이면 19px 로 바늘을 볼 수 없다).
W="${1:-1640}"
H="${2:-1232}"
FPS="${3:-5}"

TOPIC_NS="csi"

if [ "${1:-}" = "--stop" ]; then
    pkill -f '[c]amera_ros' && echo "CSI 카메라 정지" || echo "실행 중이 아니었다"
    exit 0
fi

if pgrep -f '[c]amera_ros' >/dev/null; then
    echo "이미 실행 중 — 그대로 쓴다 (바꾸려면 --stop 후 다시)"
    exit 0
fi

export ROS_DOMAIN_ID=3
source /opt/ros/humble/setup.bash

# 큰 프레임은 best_effort 로는 사실상 전송되지 않는다(조각 하나 잃으면 프레임 폐기).
# 정지 상태 점검이라 reliable 로 재전송을 허용한다 — robot_camera.launch.py 와 같은 판단.
( setsid nohup ros2 run camera_ros camera_node --ros-args \
    -p camera:="$CSI_INDEX" \
    -p width:="$W" -p height:="$H" \
    -p format:=RGB888 \
    -r /camera/image_raw/compressed:=/"$TOPIC_NS"/image_raw/compressed \
    -r /camera/image_raw:=/"$TOPIC_NS"/image_raw \
    -r /camera/camera_info:=/"$TOPIC_NS"/camera_info \
    > ~/csi_camera.log 2>&1 < /dev/null & )

sleep 6
if grep -qi "RGB888" ~/csi_camera.log; then
    echo "CSI 카메라 시작 — ${W}x${H} @ ${FPS}fps -> /$TOPIC_NS/image_raw/compressed"
else
    echo "시작이 확인되지 않았다 — 로그를 볼 것: tail -20 ~/csi_camera.log"
    tail -8 ~/csi_camera.log
fi
