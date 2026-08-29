---
name: patrol-progress-nav2
description: "순찰 로봇 진행 상황 — 안전모 감지 실주행 완료, 이제 라즈베리파이4 업그레이드가 다음 단계. [[pi4-upgrade-plan]] 부터 볼 것"
metadata: 
  node_type: memory
  type: project
  originSessionId: 70e7e46d-0593-4c0a-9599-445c04a23a6c
  modified: 2026-08-07T04:39:20.771Z
---

**2026-08-05 기준. 다음 세션은 [[pi4-upgrade-plan]] 부터 읽을 것 — 지금 진행 중인 작업이 거기 있다.** 이 메모는 그 이전(순찰+소화기점검+안전모 1차 완성)까지의 기록이라 아래 본문에는 지금은 틀린 내용도 섞여 있다(각 문단에 표시해둠).

**요약 (오래된 순):**
- 순찰(Nav2 좌표 이동) + 소화기 압력계 점검 — 둘 다 실주행 검증 완료(2026-08-01)
- 안전모 감지 — 여러 방법이 실패하다가 **초록 테이프를 안전모 위쪽 돔에 부착**하는 방식으로 2026-08-03 실주행 검증 완료. 실패 이력·최종 수치는 [[helmet-detection-color-tape-solution]] 로 옮겼다 — 아래 본문의 "머리카락 방식" 설명은 그 전 단계 기록이라 최종본이 아니다
- 2026-08-05, HI-CEED 강사 피드백("VM은 화면만, 계산은 전부 로봇 안에서")으로 목표가 바뀌어 라즈베리파이4 업그레이드를 진행 중. 상세 계획은 [[pi4-upgrade-plan]]
- **아래 "이걸로 프로젝트를 끝낸다"는 옛 판단은 폐기됐다** — 안전조끼·QR·야간순찰·금지구역·웹대시보드 5개를 계속 확장하기로 함(우선순위는 [[pi4-upgrade-plan]])
- 라인 추종으로 되돌리자는 논의가 다시 나왔으나 재기각 — [[nav2-not-line-following]] 사유에 더해 장애물회피·소화기 카메라각도·(결정적으로) 금지구역용 로봇좌표 상실 문제가 추가로 확인됨

**`cmd_vel_mux` 는 순찰 정지에는 결국 안 썼다.** 세우는 방법이 Nav2 **목표 취소**여서다(취소를 받으면 컨트롤러가 스스로 멈춘다). `/cmd_vel` 을 직접 내지 않으니 절대 규칙 1도, 검증 끝난 주행 스택도 건드리지 않는다. 계획 단계에서 mux 가 필요하다고 적어뒀던 건 과한 판단이었다.

**(2026-08-02 시점 기록, 최종본 아님)** 안전모 판정은 색이 아니라 머리카락을 본다는 결론이 한때 있었다. 사람 상자 **맨 위 6% 띠**에 어두운 화소가 있으면 미착용. 흰 안전모는 흰·크림 천장과 색으로 구분이 안 됐다(착용/미착용 양쪽에서 같은 크기 덩어리가 잡히고 그건 천장이었다). 얼굴 검출로 머리를 찍으려는 시도도 실패 — 올려다보는 각도에서 Haar 는 6장 중 1장만 맞고 눈 주변 55px 만 잡는다. 반면 **사람 상자의 위쪽 변은 정확**했다(top y=114 vs 실제 112). **덩어리 높이 ≥ 띠의 65% 조건이 결정적**이었다 — 천장 타일 이음선(두께 2~4px)이 머리카락과 같은 양의 어두운 화소를 만들어 그것 없이는 착용을 미착용으로 판정했다. **이후 실제 순찰 거리에서 이 방식도 깨졌고**, 최종적으로 색 테이프 방식(`method:=color`)으로 넘어갔다. 전체 8가지 시도와 최종 수치는 [[helmet-detection-color-tape-solution]].

**색 방식(`method:=color`)의 기준은 찍어서 등록한다** — `tools/helmet_calib.py --grab --name <이름> --select` 로 `maps/helmet_calib.yaml` 에 저장하고, 없으면 내장 일반값으로 돌아간다. 게이지를 `gauge_calib.yaml` 로 뺀 것과 같은 이유다: 안전모나 조명이 바뀌어도 **코드를 안 고치고** 기준만 다시 잡는다. 사용자가 먼저 제안한 방식이다.

**작업 전 백업이 있다:** `~/vibe/ex1/backup/ex1_20260801_2044_before_helmet.tar.gz` (지도·기준패치·코드 68개 파일). 되돌리려면 `cd ~/vibe/ex1 && tar xzf backup/...tar.gz` 후 `colcon build`.
자세한 내용은 `~/vibe/ex1/HANDOFF.md` 에 있다 — 이 메모는 요약과 판단 근거만 담는다.

**2026-07-31~08-01 에 추가된 것**: 압력계 판정(`gauge.py`), 소화기 점검 노드(`inspect_node`), 사진 취득 모듈(`shot_grab.py`), 지도 v5. 순찰↔점검은 스케줄러가 **한 순번에 하나만** 배정한다(둘 다 Nav2 를 쓰므로). 점검 예약은 `/inspect/request` 로 넣고, 즉시 가지 않고 돌던 순찰을 마친 뒤 다음 순번에 간다.

## 최종 목표 (2026-08-05 갱신 — 아래는 옛 버전)
~~지금 매핑한 방에서 벽을 따라 사각형으로 한 바퀴 도는 자율 순찰. 그 위에 랜덤 시각 순찰 → 금지구역/안전모 감지를 올린다.~~
**현재 목표:** 위 순찰+점검+안전모 위에 안전조끼·QR·야간순찰·금지구역·웹대시보드를 더 얹고, 그 계산을 전부 라즈베리파이4 안에서 처리하도록 이식한다. 라인 추종은 폐기됨([[nav2-not-line-following]], 2026-08-05 재검토 후에도 재확인).

## 완료된 것

**지도** — `~/vibe/ex1/maps/patrol_map.yaml` (133x124px, 5cm/px, 원점 `[-1.59, -5.12]`).
cartographer 상태는 `maps/state/patrol_state5.pbstream` (이어서 매핑 가능한 최신본 — 2026-08-14 확인, state4 로 적혀 있던 것 정정).
방이 작다: 빈공간 6.56m² 중 **30cm 여유 확보 영역은 1.58m²** 뿐. 미탐사가 남은 게 아니라 물리적으로 좁은 것이다(자동 탐사를 돌려보니 frontier 0개 — 벽 밖 미탐사는 벽으로 완전히 차단되어 갈 수 없음). 넓히려면 사용자가 막아둔 벽면을 밖으로 옮겨야 한다.
**새 장소에서 시연할 경우** — 라인 추종 대신 그 자리에서 cartographer 로 재매핑하는 게 답이다. 실측 기준 지도+웨이포인트만 30분, 소화기·안전모 재보정까지 포함하면 1~2시간(안전모 색/조명 재보정이 가장 변수가 큼).

**순찰 웨이포인트** — `maps/patrol_waypoints.yaml` 에 사각형 4점이 등록됨. 손으로 찍은 게 아니라 **지도에서 30cm 여유 최대 사각형을 계산해 넣은 것**이다(벽여유 31~41cm):
```
좌상 (+0.160, -0.270) → 우상 (+1.310, -0.270) → 우하 (+1.310, -1.220) → 좌하 (+0.160, -1.220)
가로 1.15m x 세로 0.95m, 시계방향
```
좌표는 손으로 찍지 말고 계산해 넣는 편이 정확하다. 손으로 찍으면 위치추정 오차(covariance)가 섞여 y가 20cm씩 오르내리고 경로가 지그재그가 된다 — 실제로 그래서 두 번 지웠다.

**Nav2** — `~/vibe/ex1/launch/nav2_patrol.launch.py` (순찰용, map_server+AMCL 포함).
파라미터는 `~/vibe/ex1/config/patrol_nav2.yaml`. 반드시 이 launch 로 켜야 한다(기본 turtlebot3 launch 는 `inflation_radius: 1.0` 이라 30cm 경로가 곡선이 된다). 조정한 값: `inflation_radius 1.0→0.25`, `xy_goal_tolerance 0.25→0.10`, `max_vel_x 0.22→0.15→0.10`(안전모 판정을 위해 추가로 낮춤, [[helmet-detection-color-tape-solution]]), `sim_time 1.5→1.0`, `use_sim_time True→False`(원본 오류) 등. 버전 불일치 함정은 [[nav2-param-version-mismatch]] 참고. `transform_tolerance`(costmap 0.2→1.0, AMCL 1.0→2.0) 완화는 권장만 해두고 아직 적용 여부 미확인 — [[wireless-is-the-bottleneck]] 참고.

**patrol_core 패키지** (`~/vibe/ex1/ros2_ws`, **VM 에서 빌드·실행** — Nav2 가 VM 에 있으므로. Pi4 이식 후에는 이 배치 자체가 바뀐다, [[pi4-upgrade-plan]])
- `cmd_vel_mux` — 만들어뒀으나 순찰 정지에는 안 쓴다(위 설명). `/cmd_vel_teleop`(0) > `/cmd_vel_nav`(1) 우선순위, 20Hz 데드맨. **Nav2 테스트 중에는 띄우지 않는다.**
- `patrol_node` — 웨이포인트 순찰. `mode`(`roundtrip`|`loop`), `stop_at_corners`, `goto_start_first`, `dwell_sec`, **`vision_wait_sec`(신규, 기본 40초)** — 시작지점 도착 후 `/helmet/ready` 를 기다렸다 출발(DDS 디스커버리 16~40초 대응). `/patrol/hold` 로 안전모 미착용 시 정지(Nav2 목표 취소 방식).
- `patrol_scheduler` — 랜덤 시각에 `/patrol/enable=True` 발행.
- `helmet_node` — 사람+안전모 판정, 웹캠 원격 관리(ssh로 켜고 끔, 장치는 부팅당 한 번만 열어 유지), CPU 양보(`nice +10`, `cv2.setNumThreads(1)`). 상세는 [[helmet-detection-color-tape-solution]].
- `inspect_node` — 소화기 압력계 점검. QR 은 아직 안 붙임.
- `explore_node` — frontier 자동 탐사. 지금 지도에서는 갈 곳이 없어 즉시 종료(정상 동작).

**순찰 정지 기준은 바퀴 수다**, **다음 순찰 시각 기준점은 직전 순찰 종료 시각** — 이 두 설계는 그대로 유효, 상세 근거는 이전 버전 참고(변경 없음).

**검증된 것** — Nav2 좌표 이동, 순찰 실주행, 소화기 점검 실주행(정합 0.90, 30초), 안전모 감지 실주행(2026-08-03, 정지+부저+해제 2사이클, 오탐 0), 웹캠 상시 스트림이 `/scan` 에 영향 없음(4.98→4.98Hz).
**미검증** — QR 내용연한, 안전조끼·야간·금지구역·웹대시보드(전부 미착수), Pi4 이식 후 threading 설계.
**미해결** — 무선 지연([[wireless-is-the-bottleneck]]), VM 2코어 경쟁, 배터리 지속시간. Pi4 이식이 근본 해결책으로 논의됨.

**로봇 없이 검증하는 법** — `nav2_msgs` 액션(`navigate_to_pose`, `navigate_through_poses`) 서버(`tools/fake_nav2.py`)를 만들어 모든 목표를 즉시 succeed 시키면 순찰 흐름 전체를 초 단위로 돌려볼 수 있다. `travel_sec` 파라미터를 주면 이동 중 취소(안전모 정지)도 시험 가능.

## 실행 순서 (2026-08-03 기준, Pi4 이식 후 바뀔 예정)

① **로봇** — `ros2 launch turtlebot3_bringup robot.launch.py` (`Failed to read` 없어야 정상)
   토크 확인: `ros2 topic echo /sensor_state --once` → false 면 `ros2 service call /motor_power std_srvs/srv/SetBool "{data: true}"`

② **VM** — `ros2 launch ~/vibe/ex1/launch/nav2_patrol.launch.py` → **RViz 2D Pose Estimate 필수**

③ **VM** — `source ~/vibe/ex1/ros2_ws/install/setup.bash && ros2 launch ~/vibe/ex1/launch/patrol_auto.launch.py fire_on_start:=true laps:=1 quiet:=true`
   (patrol_node + patrol_scheduler + inspect_node + helmet_node 전부. 웹캠은 helmet_node 가 ssh로 자동으로 켠다 — 로봇에서 따로 실행할 필요 없음. `quiet:=true` 로 로그를 정리해서 보여줄 수 있다.)

**`teleop_keyboard` 를 같이 켜지 말 것.** `/cmd_vel` 에 직접 발행해 Nav2 명령을 상쇄시킨다.

## 진단 도구 (`~/vibe/ex1/tools/`)
- `save_waypoint.py` — 여유 검사 포함 웨이포인트 저장
- `helmet_calib.py` — 안전모 색 기준 등록(실물 찍어서)
- `gauge_calib.py` — 압력계 기준 등록
- `fake_nav2.py` — 로봇 없이 순찰 흐름 검증, `travel_sec` 로 이동 중 취소도 시험
- `live_view.py` — 카메라 조준 확인
- `capture_frame.py`, `compressed_to_raw.py` — 프레임 캡처/중계

## 자주 겪은 함정 (재발 시 즉시 확인)
1. **로봇에선 토픽이 보이는데 VM 에선 안 보임** → `ros2 daemon stop && rm -rf /dev/shm/fastrtps_* && ros2 daemon start`.
2. **`Nav2 가 목표를 거절했다` 반복** → `turtlebot3_node` 죽음(`/odom` 없음) 또는 무선 지연으로 `/scan` 버려짐. [[wireless-is-the-bottleneck]] 먼저 확인.
3. **배터리** — 3셀 리튬폴리머. 12.3V→11.5V 로 한 시간 만에 떨어진 사례. **11V 아래로 떨어지면 OpenCR 이 경고음을 내고 turtlebot3_node 가 죽는다.** SMPS 어댑터 권장.
4. **웹캠이 잠긴다** — 노드를 여러 번 강제종료(특히 SIGKILL)하면 장치가 `select() timeout` 상태로 잠기고 **로봇 재부팅 없이는 안 풀린다.** `SIGTERM` 핸들러로 고쳤지만, 혹시 재발하면 재부팅이 유일한 해법이었다.
5. **모터 안 돌 때** — [[motor-torque-troubleshooting]]

## 다음 할 일

**[[pi4-upgrade-plan]] 로 넘어갈 것.** 구매 확정(4GB, 200,900원), SD카드 교체 방법(복제 아닌 재설치+데이터 이전), 목표 아키텍처(VM=화면만), threading 설계 필요성이 전부 거기 있다.

Pi4 작업 착수 전 확인할 것 두 가지 (이번 세션에서 권했으나 미적용):
1. VM 코어 2→4 (VMware 설정)
2. `config/patrol_nav2.yaml` 의 `transform_tolerance` 완화

그 다음 5개 확장 기능 우선순위(안전조끼 → QR → 웹대시보드 → 야간 → 금지구역)와 각각의 걸림돌은 [[pi4-upgrade-plan]] 에 정리해뒀다.

**스케줄러 함정(재발 시 확인):**
1. 발행했는데 순찰이 시작 안 됨 — `/patrol/enable` QoS 가 VOLATILE 이라 구독자가 없으면 메시지가 사라진다.
2. 순찰이 끝났는데 다음이 안 잡힘 — `/patrol/status` 의 `done ...` 확인.

관련: [[pi4-upgrade-plan]], [[helmet-detection-color-tape-solution]], [[wireless-is-the-bottleneck]], [[robot-host-identity]], [[nav2-param-version-mismatch]], [[motor-torque-troubleshooting]], [[nav2-not-line-following]], [[ros2-rmw-fastrtps-decision]]
