---
name: nav2-param-version-mismatch
description: turtlebot3_ws 의 burger.yaml 은 Jazzy 이후용이라 Humble 에서 Nav2 가 죽는다 — humble/burger.yaml 을 써야 함
metadata: 
  node_type: memory
  type: project
  originSessionId: e85a9eeb-96d6-4ed7-99bc-3a0459f0a0ae
  modified: 2026-07-30T01:18:32.488Z
---

Nav2 를 띄웠는데 `Navigation: inactive` 이고 `navigate_to_pose action server is not available` 이면 **파라미터 파일 버전 불일치**를 먼저 의심한다.

**증상 연쇄 (2026-07-30 실측)**
```
planner_server: [FATAL] Failed to create global planner.
  the class nav2_navfn_planner::NavfnPlanner does not exist.
  Declared types are  nav2_navfn_planner/NavfnPlanner ...
        ↓
lifecycle_manager_navigation: Failed to bring up all requested nodes. Aborting bringup.
        ↓
Navigation 그룹 노드 전부 죽음 → map 프레임 없음 → RViz 는 죽기 전 잔상만 보임
        ↓
base_scan "queue is full" 무한 반복, Startup 버튼 눌러도 무반응, 순찰 노드 영원히 대기
```

**원인:** `~/turtlebot3_ws/.../param/burger.yaml` 은 Jazzy 이후 표기(`패키지::클래스`)를 쓰고 `navigators:` 블록도 있다. Humble 은 일부 플러그인이 슬래시 표기이고 `nav2_bt_navigator` 에는 plugins.xml 자체가 없다.

**해결:** 같은 폴더의 **`param/humble/burger.yaml`** 이 진짜 Humble 용이다. 그것을 복사해 쓴다. 우리 설정은 `~/vibe/ex1/config/patrol_nav2.yaml` (그 파일 기반 + 조정값, 변경 근거 주석 포함).

**주의 — 일괄 치환하면 안 된다.** 패키지마다 표기가 다르다. 슬래시는 `nav2_navfn_planner/NavfnPlanner`, `nav2_recoveries/Spin`; 콜론은 `nav2_costmap_2d::InflationLayer`, `nav2_controller::SimpleGoalChecker`, `dwb_core::DWBLocalPlanner`, `nav2_waypoint_follower::WaitAtWaypoint`. 실제 등록 이름은 `/opt/ros/humble/share/<pkg>/*.xml` 의 `<class name=...>` 로 확인한다. 에러 메시지의 `Declared types are ...` 에도 정답이 나온다.

원본 `humble/burger.yaml` 에는 **`use_sim_time: True` 오류**가 있다(local_costmap 쪽, 235행 부근). 실제 로봇에서는 False 여야 한다.

관련: [[patrol-progress-nav2]], [[motor-torque-troubleshooting]]
