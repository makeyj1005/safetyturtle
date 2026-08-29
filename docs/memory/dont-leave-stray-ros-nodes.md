---
name: dont-leave-stray-ros-nodes
description: ros2 run 을 배경으로 띄우면 껍데기 PID만 죽어 자식 노드가 살아남는다 — 시험 노드를 띄우지 말 것
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 778ff1b0-1416-4417-b48c-f274a626fe16
  modified: 2026-08-01T09:27:50.238Z
---

`ros2 run <pkg> <node>` 를 백그라운드로 띄우고 `kill $!` 하면 **껍데기(ros2 CLI)만
죽고 실제 노드 프로세스는 살아남는다.** 남은 노드는 같은 토픽을 구독해 사용자의
실행과 충돌한다 — `inspect_node` 가 둘이 되면 각자 Nav2 목표를 보내고 같은 CSV 에
겹쳐 써서 회차가 통째로 망가진다.

**Why:** 2026-08-01 세션에서 이 실수를 **세 번** 했다. 한 번은 시험 노드가 1시간 15분
살아남아 실주행 결과를 망쳤고(원인 찾는 데 시간을 또 썼다), 카메라 프로세스에서도
같은 패턴(고아 프로세스)이 났다.

**How to apply:**
- 로봇이 실제로 도는 프로젝트에서는 **시험용 노드를 띄우지 않는다.** 코드 확인은
  ROS 없이 함수만 떼어 검사하거나(`limit_for_run`, `home_pose` 처럼), 사용자의
  실행 로그로 확인한다.
- 꼭 띄워야 하면 종료 후 `ps -eo pid,cmd | grep [i]nspect_node` 로 **자식까지**
  확인하고 남았으면 `kill -9`.
- `pkill -f <패턴>` 은 **자기 셸 명령줄까지 매치해 스스로를 죽인다.** 같은 세션에서
  두 번 당했다. PID 로 죽이거나, 패턴에 그 문자열이 명령줄에 없을 때만 `[i]` 트릭을 쓴다.

관련: [[ros-domain-id-3-fixed]] (실제 시스템이 돌 때는 시험을 미룬다)
