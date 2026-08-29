---
name: commands-need-location-and-context
description: 명령어를 줄 때는 VM/라즈베리파이 어디서 실행하는지와 앞뒤로 필요한 명령까지 같이 줄 것
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 778ff1b0-1416-4417-b48c-f274a626fe16
  modified: 2026-08-01T00:46:32.403Z
---

터미널 명령을 안내할 때 **한 줄만 주지 않는다.** 두 가지를 항상 붙인다:
1. **실행 위치** — VM(원격 PC)인지 로봇(라즈베리파이 ros18)인지
2. **앞뒤로 같이 필요한 명령** — 지도/Nav2 띄우기, `export ROS_DOMAIN_ID=3`,
   `source ~/vibe/ex1/ros2_ws/install/setup.bash`, RViz 2D Pose Estimate 같은 선행 단계

**Why:** 2026-08-01 세션에서 점검 명령 한 줄만 줬더니 "맵 키는 명령어도, VM 에서
켜는지 라즈베리파이에서 켜는지도 알려달라"는 요청을 받았다. 노드가 두 대에 나뉘어
돌아가는 프로젝트라 위치가 빠지면 명령만으로는 실행할 수 없다.

**How to apply:** 표로 `# / 실행 위치 / 무엇 / 명령` 을 주는 게 가장 읽기 쉽다.
같은 표가 `~/vibe/ex1/HANDOFF.md` 의 "실행 순서 → 명령 요약" 에 있으니 거기서
가져다 쓰고, 바뀌면 그 표도 같이 고친다. [[patrol-progress-nav2]]
