---
name: ros2-rmw-fastrtps-decision
description: VM과 로봇의 RMW를 기본값 rmw_fastrtps_cpp로 통일하기로 결정 — cyclonedds 설정은 폐기함
metadata: 
  node_type: memory
  type: project
  originSessionId: 014807c6-8861-4a69-b2db-b81e5c632cb8
  modified: 2026-07-28T08:40:57.171Z
---

2026-07-28, VM과 로봇의 RMW를 **`rmw_fastrtps_cpp`(ROS 2 기본값)로 통일**했다. `ROS_DOMAIN_ID=3`.

그 전 상태는 VM만 `rmw_cyclonedds_cpp`를 강제하고 로봇은 fastrtps 기본값이라 **서로 다른 RMW로 통신**하고 있었다. `/scan`, `/odom` 같은 작은 메시지는 우연히 통했지만(둘 다 RTPS 기반), 카메라 영상처럼 조각화되는 큰 메시지에서 깨지는 조합이다.

**Why:** 로봇에는 cyclonedds가 설치조차 안 되어 있었고(fastrtps만 있음), 같은 서브넷이라 멀티캐스트 디스커버리가 되므로 cyclonedds의 정적 피어 설정이 애초에 불필요했다. 게다가 그 피어 IP는 옛 주소(`192.168.0.22`)로 잘못 박혀 있어 아무 역할도 못 하고 있었다. 로봇에 패키지를 설치하지 않는 쪽이 손댈 곳이 적다.

**How to apply:** VM `~/.bashrc`는 정리되어 `unset RMW_IMPLEMENTATION CYCLONEDDS_URI`로 끝난다(백업: `~/.bashrc.bak.20260728`). 통신이 안 될 때는 **양쪽 `ROS_DOMAIN_ID=3`인지, `RMW_IMPLEMENTATION`이 비어 있는지**를 먼저 확인. 예전 cyclonedds 설정이 남은 오래된 터미널을 쓰면 통신이 깨지므로 새 터미널을 열어야 한다. `~/cyclonedds_config.xml`은 이제 쓰이지 않는다.

관련: [[robot-host-identity]], [[csi-camera-stream-tuning]]
