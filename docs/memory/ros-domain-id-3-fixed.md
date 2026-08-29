---
name: ros-domain-id-3-fixed
description: ROS_DOMAIN_ID 는 3 으로 고정이다 — 시험용으로도 다른 번호를 쓰지 말 것
metadata: 
  node_type: memory
  type: project
  originSessionId: 778ff1b0-1416-4417-b48c-f274a626fe16
  modified: 2026-08-01T01:50:46.816Z
---

이 프로젝트의 `ROS_DOMAIN_ID` 는 **3 으로 지정된 값**이다. 로봇 bringup 도 3 으로 뜬다.
시험 격리 목적이라도 다른 도메인(7 등)으로 바꿔 돌리지 않는다.

**Why:** 2026-08-01 세션에서 실제 시스템과 충돌을 피하려고 도메인 7 에서 시험을
돌리려 했더니 사용자가 막았다 — "도메인은 지정해준 거여서 3 지켜야 해." 지정된 설정을
임의로 바꾸면 나중에 왜 안 되는지 추적이 어려워진다.

**How to apply:** 실제 시스템이 돌고 있을 때는 도메인을 바꾸는 대신 **시험 자체를
미룬다.** 특히 `tools/fake_nav2.py` 는 진짜 Nav2 가 떠 있을 때 같은 도메인에서
띄우면 액션 서버가 둘이 되어 결과가 섞이고, `patrol_node` 는 `auto_start` 기본값이
true 라 실행하는 순간 **실제 로봇이 움직인다.** 로직만 확인할 일이면 ROS 를 띄우지 말고
순수 파이썬으로 함수만 떼어 검사한다(실제로 `first_laps` 규칙은 그렇게 검증했다).

새 터미널에는 도메인이 안 잡혀 있다. `~/.bashrc` 에 `export ROS_DOMAIN_ID=3` 을
넣어두면 매번 붙이지 않아도 된다. 관련: [[ros2-rmw-fastrtps-decision]],
[[commands-need-location-and-context]]
