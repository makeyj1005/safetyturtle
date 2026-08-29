---
name: csi-camera-stream-tuning
description: camera_ros 스트림 설정값(640x480/15fps/jpeg50/best_effort)과 그렇게 정한 실측 근거
metadata: 
  node_type: memory
  type: project
  originSessionId: 014807c6-8861-4a69-b2db-b81e5c632cb8
  modified: 2026-08-07T14:05:08.347Z
---

로봇 CSI 카메라는 `camera_ros`로 발행하며, 확정된 설정은 `~/vibe/ex1/launch/robot_camera.launch.py`에 있다: `camera=0`, `format=RGB888`, `640x480`, `FrameDurationLimits=[66667,66667]`(15fps), `jpeg_quality=50`, 그리고 image_raw / image_raw/compressed 양쪽 **`reliability=best_effort`, `depth=1`**.

## ⚠️ 정정 — 센서는 imx219 가 아니라 **ov5647** 이다 (2026-08-07 확인)

이 메모와 다른 곳에 "imx219"로 적어뒀던 건 틀렸다. 실측: 커널이 올린 모듈이 `ov5647`, i2c 주소가 `0x36`(imx219 는 `0x10`), libcamera 등록명이 `ov5647@36`. **Raspberry Pi Camera Module v1(5MP)** 이다.

## ⚠️ `camera=0` 을 쓰지 말 것 — ID 문자열로 지정한다 (2026-08-07)

카메라가 **두 개**다(CSI + 안전모용 USB 웹캠). `camera_ros` 기본값 `camera=0` 으로 띄우면 **USB 웹캠을 잡는다**(실측: `no camera selected, using default: .../1b10:2002`). 인덱스는 장치 구성이 바뀌면 밀리므로 — 이것이 옛 "웹캠을 꽂으면 CSI 가 1 로 밀린다"(HANDOFF 함정 9)의 근본 원인 — **ID 문자열로 못박는다:**

```
CSI (ov5647) : camera:=/base/soc/i2c0mux/i2c@1/ov5647@36
USB 웹캠      : camera:=/base/scb/pcie@7d500000/pci@0,0/usb@0,0-1.2:1.0-1b10:2002
```

ID 로 지정하면 정상 동작이 확인된다: `configured with 640x480-NV21 stream`, 약 21.7Hz. 웹캠 쪽은 MJPEG 640x480, 약 23.5Hz.

`Unable to open camera calibration file ... .yaml` **ERROR 는 무해하다** — `camera_info` 보정값이 없다는 뜻이고 영상은 정상 발행된다.

**새 SD 카드(2026-08-07 클린 설치)에는 `ros-humble-camera-ros` 가 없어서 따로 설치해야 했다.** ROS 2 기본 설치 목록에 안 들어있다. 카메라가 안 잡히면 리본을 다시 꽂기 전에 **이 패키지가 있는지부터 확인할 것** — 커널 로그에 `i2c 10-0036 ... fe801000.csi` 와 `/dev/v4l-subdev0` 가 있으면 하드웨어는 정상이다. `vcgencmd get_camera` 는 레거시 스택을 보므로 Ubuntu 에서는 항상 `supported=0` 이 나온다 — **이걸 "카메라 없음"으로 오판하지 말 것.**

**Why (실측 근거, 2026-07-28):** 카메라 자체는 로컬에서 30fps를 내고 CPU도 75% 유휴라 병목이 아니었다. 병목은 Pi 3의 2.4GHz Wi-Fi였다.

- `jpeg_quality=95`(기본) → 프레임당 80KB, 30fps면 2.0MB/s로 링크가 넘침. 50으로 낮추면 20KB.
- **QoS가 결정적**: RELIABLE이면 유실 1개당 재전송을 기다리며 스트림이 최대 **977ms** 멈추고 VM 수신 9.5fps. BEST_EFFORT로 바꾸니 최대 지연 **274ms**, 수신 **15.2fps**(발행량과 일치, 유실 사실상 없음), 2.21Mbps.

**How to apply:** 영상 토픽은 반드시 best_effort로 구독해야 한다(`qos_profile_sensor_data`). RELIABLE로 구독하는 도구는 **한 장도 못 받는다** — `ros2 topic hz`/`bw`(QoS 옵션 없음), `ros2 run image_transport republish`가 여기에 해당한다(로그에 `incompatible QoS ... RELIABILITY_QOS_POLICY`). 그래서 RViz용 compressed→raw 중계는 직접 만든 `~/vibe/ex1/tools/compressed_to_raw.py`를 쓴다(RViz2 Image 디스플레이는 compressed를 못 읽음). RViz에서 `/scan`도 BEST_EFFORT라 `Reliability Policy`를 Best Effort로 바꿔야 보인다.

~~남은 지터(100ms 초과 구멍 24초당 33회)는 Wi-Fi 절전 모드가 유력한데, 확인에 로봇 `iw` 설치(sudo)가 필요해서 보류 중이다.~~

→ **2026-08-07 해결됐다. 절전 모드가 맞았다.** `iw dev wlan0 set power_save off` 로 지연 평균 6.79→3.89ms, 지터 5.64→1.68ms, **100ms 초과 구멍 0회**. 영구화는 `wifi-powersave-off.service`. 상세는 [[wireless-is-the-bottleneck]].

주의: 카메라 디바이스는 **한 프로세스만 점유** 가능하다. `failed to acquire camera` / `Pipeline handler in use by another process`가 뜨면 이전 노드가 살아있는 것 — `pgrep -af camera_node`로 찾아 PID로 정리한다.

관련: [[robot-host-identity]], [[ros2-rmw-fastrtps-decision]], [[robot-undervoltage-warning]]
