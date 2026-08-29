---
name: nav2-not-line-following
description: 이동 방식은 Nav2 좌표 이동으로 확정 — 라인 추종은 실험만 하고 2026-07-28 제거함
metadata: 
  node_type: memory
  type: project
  originSessionId: 014807c6-8861-4a69-b2db-b81e5c632cb8
  modified: 2026-07-28T12:03:19.707Z
---

이 순찰 로봇의 이동 방식은 **Nav2 좌표(웨이포인트) 이동**이다. 라인 추종은 한 번 실험해본 뒤 **2026-07-28에 코드까지 완전히 제거**했다. 다시 만들자고 제안하지 말 것.

**Why — 카메라 한 대로 둘 다 못 한다:**
라인 추종은 카메라가 바닥(로봇 앞 20~40cm)을 봐야 하고, 안전모 탐지(YOLO)와 소화기 QR 점검은 정면을 봐야 한다. imx219 표준 렌즈의 수직 화각이 약 49°이고 카메라 높이가 20cm 수준이라, 아래로 30~40° 기울이면 정면이 전혀 들어오지 않는다. 반대로 수평에 맞추면 바닥은 화면 최하단 10~15%에만 눌려 잡힌다. 기하학적으로 양립이 안 된다.

게다가 최종 목표(랜덤 시각 순찰, 금지구역 침입 감지, 안전모 확인, QR 점검)가 본질적으로 **지도 + 좌표 + 자율주행** 문제다. 원래 계획의 3~6단계(SLAM → 좌표 등록 → 순찰 스케줄러 → 침입 경보)가 전부 Nav2 기반이라 라인 추종은 애초에 대체 가능한 이동 수단이었다.

**How to apply:** 카메라는 **정면 고정**으로 둔다. 다음 작업은 3단계 SLAM(cartographer)이다. `ros-humble-turtlebot3-cartographer`는 VM에 이미 설치돼 있다.

살아남은 것: `patrol_core` 패키지의 `cmd_vel_mux` 노드(로봇 `~/patrol_ws`). 절대 규칙대로 **`/cmd_vel` 은 이 노드만 발행**하며, Nav2 는 `/cmd_vel_nav` 로 내면 우선순위 1번으로 채택된다. `/cmd_vel_teleop` 이 우선순위 0번(최우선)이라 사람이 언제든 개입할 수 있다. 20Hz 고정 발행이라 입력이 끊기면 자동으로 0 이 나가는 데드맨 구조이고, `/mux/enable` 에 false 를 보내면 즉시 정지한다.

주의: 기본 `turtlebot3_teleop` 은 `/cmd_vel` 에 직접 발행해서 중재 노드와 충돌한다. 반드시 `--ros-args -r /cmd_vel:=/cmd_vel_teleop` 로 바꿔 실행한다.

관련: [[robot-host-identity]], [[csi-camera-stream-tuning]], [[motor-torque-troubleshooting]]
