"""
robot_camera.launch.py — 로봇(ros18, Pi 3)에서 CSI 카메라를 띄우는 launch 파일.

[이 파일은 로봇에서 실행한다]  위치: 로봇의 ~/launch/robot_camera.launch.py
  기본 (감시·주행용 640x480 15fps):
    ros2 launch ~/launch/robot_camera.launch.py

  소화기 점검용 (게이지 바늘을 보려면 해상도가 필요하다):
    ros2 launch ~/launch/robot_camera.launch.py width:=1640 height:=1232 fps:=5 jpeg_quality:=80

파라미터를 매번 손으로 붙여넣지 않도록 묶어둔 것. 각 값의 근거는 아래 주석 참고.
패키지를 새로 만들 필요 없이 파일 경로를 직접 넘겨 실행할 수 있다.

[⚠️ camera 인덱스 함정 — 2026-07-31]
USB 웹캠을 달자 libcamera 가 카메라를 두 대로 보게 되어 인덱스가 밀렸다.
  camera:=0  USB 웹캠 (DICOTA, MJPEG/YUYV — RGB888 을 지원하지 않아 노드가 죽는다)
  camera:=1  CSI imx219 (RGB888, 최대 3280x2464)  ← 우리가 쓰는 것
증상: `unsupported pixel format "RGB888"` 로 exit -6. 카메라 잘못이 아니라 엉뚱한
장치를 잡은 것이다. USB 를 빼면 CSI 가 다시 0 이 되므로, 그때는 camera:=0 으로 준다.
확인 방법: `ros2 run camera_ros camera_node --ros-args -p camera:=N` 로 띄워
stream formats 에 RGB888 이 있으면 CSI 다.

[⚠️ 해상도 상한 1640x1232 — CMA 메모리 (2026-07-31 실측)]
출력이 1640x1232 를 넘으면 센서가 비닝 모드에서 3280x2464 전체 모드로 바뀌고,
raw 베이어 버퍼가 커져 CMA(연속 메모리) 64MB 를 초과한다. 증상:
  2048x1536 -> "Unable to request 4 buffers: Cannot allocate memory"
  3200x2400 -> "allocation failed: Cannot allocate memory"
1640x1232 는 raw 10MB + 출력 24MB = 34MB 로 들어간다. 더 큰 해상도가 필요하면
/boot/firmware/config.txt 에 dtoverlay=vc4-kms-v3d,cma-128 을 넣고 재부팅해야 한다
(로봇 전체 RAM 이 905MB 라 권하지 않는다). 해상도 대신 거리를 줄이는 편이 낫다.

[⚠️ 큰 프레임은 best_effort 로 전송되지 않는다 (2026-07-31 실측)]
1640x1232 jpeg85 는 프레임당 177KB 다. UDP 로 보낼 때 100 조각 넘게 쪼개지고,
best_effort 는 조각 하나만 잃어도 프레임 전체를 버린다. 실측: 로봇 로컬 2.6fps
정상 수신, VM 은 20초간 0장. 640x480 jpeg50(20KB, 14조각)에서는 문제가 없었다.
  대응 1  reliability:=reliable  — 조각을 재전송한다. 정지 상태 점검에는 이게 맞다
  대응 2  jpeg_quality 를 낮춰 프레임을 작게
  대응 3  조준용 라이브 뷰는 640x480 로, 판정용 정지 촬영만 고해상도로

[점검용 해상도의 근거]
압력계 지름은 2~2.5cm 다. 게이지를 프레임에 넣으려면 70cm 쯤 떨어져야 하는데
(카메라 높이 20cm, 수직 화각 49°), 그 거리에서 640x480 이면 게이지가 19px 로
바늘을 볼 수 없다. 1640x1232 로 올리면 약 48px 이 되어 판정이 가능해진다.
정지 상태에서 몇 장만 받으면 되므로 fps 는 낮춰 대역폭을 줄인다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGS = {
    # CSI 는 1, USB 웹캠은 0 (위 함정 설명 참고)
    "camera": "1",
    "width": "640",
    "height": "480",
    "fps": "15",
    "jpeg_quality": "50",
    # best_effort | reliable — 큰 프레임을 무선으로 보낼 때가 문제다(아래 설명)
    "reliability": "best_effort",
}


def make_node(context):
    def val(name):
        return LaunchConfiguration(name).perform(context)

    fps = float(val("fps"))
    # 프레임 간격(마이크로초). 상·하한을 같게 주면 그 fps 로 고정된다.
    dur = int(round(1e6 / fps))
    return [camera_node(int(val("camera")), int(val("width")), int(val("height")),
                        dur, int(val("jpeg_quality")), val("reliability"))]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()]
        + [OpaqueFunction(function=make_node)]
    )


def camera_node(camera, width, height, frame_duration_us, jpeg_quality, reliability):
    return (
        Node(
            package="camera_ros",
            executable="camera_node",
            name="camera",
            output="screen",
            parameters=[{
                # 카메라 인덱스. 지정하지 않으면 "no camera selected" 경고가 뜬다.
                # USB 웹캠이 꽂혀 있으면 CSI 는 1 이다(파일 상단 함정 설명).
                "camera": camera,

                # 픽셀 포맷. 지정하지 않으면 NV21(YUV)이 자동 선택되는데,
                # OpenCV로 HSV 노란색 마스크를 만들 2단계 작업에 RGB 계열이 편하다.
                "format": "RGB888",

                "width": width,
                "height": height,

                # 프레임 간격 하한/상한(마이크로초). 66667us = 15fps. fps 인자로 정한다.
                # 카메라는 30fps까지 내지만, 그러면 Wi-Fi로 2.0MB/s를 쏘게 되고
                # Pi 3의 2.4GHz 무선이 감당하지 못해 오히려 유실·지연이 커진다.
                "FrameDurationLimits": [frame_duration_us, frame_duration_us],

                # JPEG 품질. 기본값 95는 프레임당 80KB로 너무 크다.
                # 50으로 낮추면 20KB로 줄어드는데(약 4배) 주행·감시에는 충분하다.
                # 압력계 바늘처럼 미세한 것을 볼 때는 80 을 쓴다(jpeg_quality 인자).
                "jpeg_quality": jpeg_quality,

                # === QoS: 여기가 가장 중요 ===
                # RELIABLE로 두면 패킷 한 개 유실될 때마다 재전송을 기다리면서
                # 스트림 전체가 최대 1초까지 멈춘다(실측 977ms). 라인 추종 중에
                # 1초간 영상이 멈추면 로봇이 눈을 감고 달리는 셈이라 위험하다.
                # BEST_EFFORT로 바꾸면 유실 프레임은 버리고 다음 것을 받아
                # 최대 지연이 274ms로 줄고 실효 프레임레이트가 9.5 -> 15.2fps로 올라간다.
                "qos_overrides./camera/image_raw/compressed.publisher.reliability": reliability,
                # 큐 깊이 1 = 밀린 옛 프레임을 쌓아두지 않고 항상 최신 것만 내보낸다.
                "qos_overrides./camera/image_raw/compressed.publisher.depth": 1,
                "qos_overrides./camera/image_raw.publisher.reliability": reliability,
                "qos_overrides./camera/image_raw.publisher.depth": 1,
            }],
        )
    )
