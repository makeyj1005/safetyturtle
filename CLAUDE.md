# 실내 순찰 로봇 (TurtleBot3) — 프로젝트 지침

한국어로 답한다. 출품 대상: 영남이공대 HI-CEED AI 콘텐츠 경진대회.

## 무엇을 만드는 중인가

TurtleBot3 Burger 기반 실내 순찰 로봇. **Nav2 좌표 이동**으로 웨이포인트를 돌며
① 사람 발견 시 안전모 착용 여부 판정 ② 소화기 압력계·내용연한 점검 ③ 랜덤 시각 자동 순찰.

**핵심 완료 사항:** 순찰 + 소화기 점검 + 안전모 감지 실물 검증 완료. Pi 4 교체와
온보드 Nav2 (9개 노드 active) 검증 완료. 최종 보고서 작성 완료(`순찰로봇_최종보고서.pdf`).

## 세션 시작할 때 읽는 순서

1. **`HANDOFF2.md`** — **현재 기준 인수인계 (2026-08-14 전면 재작성).** 현재 상태·실행
   절차(명령어 전부)·최근 세션 기록·함정 색인이 여기 있다. 새로 맡으면 여기부터
2. **`docs/memory/MEMORY.md`** — 축적된 기억의 색인. 한 줄씩 훑고 관련된 것만 열어본다
3. **`docs/memory/nav2-tuning-for-tiny-room.md`** — 최근 진단(첫 구간 교착 원인)과 다음 검증
4. **`HANDOFF.md`** — 2026-08-02까지의 1140줄 상세 이력. 통째로 읽지 말고
   목차(`grep -nE "^#{1,3} " HANDOFF.md`)로 필요한 절만 볼 것

`docs/memory/`는 `~/.claude/projects/-home-ohinseop-vibe-ex1/memory/`의 사본이다.
어느 쪽을 고치든 **양쪽을 맞춰 둘 것**(계정·기계가 바뀌어도 살아남게 하려고 이중화했다).

## 일하는 방식 — 사용자가 요청한 것

- **작업 시작 전, 전체 흐름에서 지금 어느 부분인지 먼저 한 문단으로 설명**한다.
  바로 코드부터 고치지 않는다. (`docs/memory/explain-scope-before-starting.md`)
- **명령어를 줄 때는 "어디서 실행하는가"를 반드시 붙인다** — VM인지 로봇(ssh)인지.
  그리고 선행 명령(`source`, `export`)까지 한 덩어리로 묶어서 준다.
  (`docs/memory/commands-need-location-and-context.md`)
- 실측으로 확인한 것과 추측을 구분해서 말한다. 이 프로젝트는 추측으로 시간을 크게 낭비한 이력이 있다.

## 절대 규칙

- **`ROS_DOMAIN_ID=3` 고정.** 시험용으로도 다른 도메인을 쓰지 않는다.
  (`docs/memory/ros-domain-id-3-fixed.md`)
- **RMW는 `fastrtps`.** cyclonedds 설정은 폐기됐다. 통신 장애 시 첫 확인 지점.
- **`pkill -f` 금지** — 자기 셸까지 죽인다. `ros2 run`을 Ctrl-C 해도 자식 노드는 살아남으니
  종료 후 반드시 `pgrep -a -f` 로 확인한다. (`docs/memory/dont-leave-stray-ros-nodes.md`)
- **라인 추종을 다시 제안하지 않는다.** 세 차례 검토 후 기각됐다. 장소가 바뀌면
  그 자리에서 cartographer 재매핑이 답이다. (`docs/memory/nav2-not-line-following.md`)
- **웹캠은 부팅당 한 번만 열고 유지한다.** 여러 번 열고 닫으면 장치가 잠겨
  (`select() timeout`) 로봇을 재부팅해야 풀린다.

## 로봇에 접속하기

hostname `ros18`, Raspberry Pi 4 Model B Rev 1.5, Ubuntu 22.04, 계정 `rpi`, sudo는 NOPASSWD.
**IP는 DHCP라 매번 바뀐다.** MAC `88:a2:9e:ff:46:c2`(wlan0) / `...:c1`(eth0) 로 찾는다 —
옛 `b8:27:eb` 스윕은 Pi 3 시절 것이라 안 잡힌다. (`docs/memory/robot-host-identity.md`)

## 폴더 구조

| 경로 | 내용 |
|---|---|
| `ros2_ws/src/patrol_core/patrol_core/` | 노드 본체 — `patrol_node`, `helmet_node`, `inspect_node`, `patrol_scheduler`, `gauge`, `explore_node`, `cmd_vel_mux`, `shot_grab` |
| `launch/` | `patrol_auto.launch.py`(통합), `nav2_patrol_onboard.launch.py`(로봇용), `nav2_patrol.launch.py`(VM용), `inspect_once.launch.py`, `cartographer_resumable.launch.py` |
| `config/` | `patrol_nav2.yaml`(Nav2 파라미터), `patrol.rviz` |
| `maps/` | 지도(최종은 `patrol_map_v5.pgm/yaml` — `patrol_map.*`는 이전 버전), 웨이포인트, 안전모 색 기준(`helmet_calib.yaml`), 압력계 기준(`gauge_calib.yaml`), cartographer 상태(`state/*.pbstream`, 최신 `patrol_state5`) |
| `models/` | MobileNet-SSD (사람 검출, 23MB) |
| `tools/` | 보정·촬영·리포트 도구. `webcam_node.py`, `helmet_calib.py`, `save_waypoint.py`, `aim_gauge.py`, `make_report.py` 등 |
| `logs/` | 순찰·점검 결과 CSV + 사진 (118MB, 2026-07-29~08-03 실주행 기록) |
| `backup/` | 안전모 작업 전 스냅샷 tar.gz |

## 다음에 할 일

1. ~~로봇을 매핑한 방에 놓고 실제 온보드 주행 검증~~ — **2026-08-14 완료: 온보드 Nav2 실주행
   1바퀴 완주**(온보드 Nav2 + VM patrol_auto + 안전모 감지 동시 가동). RViz 없이 로봇에서
   `/initialpose` 반복 발행(-r 2, yaw 분산 0.3)으로 초기 위치를 잡는 방법이 실전 검증됨.
   ⚠️ 첫 구간 `Failed to make progress` 4연속(160초 교착) 후 회전 복구로 회복, 총 4.5분/바퀴.
   원인 특정됨: **초기 위치 힌트가 실제와 어긋나면 DWB 0속도 교착** — 소화기 가설은 후순위.
   해법(다음 주행 검증): 순찰 전 정렬 회전. 상세는 `docs/memory/nav2-tuning-for-tiny-room.md`
2. **ssh 전제를 로컬 모드로 고치기** — 옛 Pi 3 주소 `192.168.0.67`이 **소스 7개 파일에
   총 15곳** 하드코딩돼 있다(2026-08-14 재실측, `ros2_ws/build`·`install` 사본 제외).
   로봇 안에서 돌면 자기 자신에게 ssh 하는 셈이다. 기능에 영향 주는 기본값 6곳:
   - `patrol_core/helmet_node.py:308` — `robot_host` 기본값
   - `patrol_core/inspect_node.py:160` — `robot_host` 기본값
   - `launch/patrol_auto.launch.py:91` — `robot_host` launch 기본값
   - `launch/inspect_once.launch.py:39` — `robot_host` launch 기본값
   - `tools/grab_shot.py:42` / `tools/aim_gauge.py:239` — `--host` 기본값

   docstring 예시 9곳: `inspect_node.py:6·9`, `shot_grab.py:193`, `grab_shot.py:5·6`,
   `aim_gauge.py:5`, `inspect_once.launch.py:6·9·12`. 고친 뒤 `colcon build` 를 해야
   `install/` 사본에도 반영된다.

   **IP를 바꾸는 게 아니라 ssh를 건너뛰는 로컬 모드를 추가하는 쪽이 맞다.** 같은 기계 안이니
   프로세스를 직접 띄우면 되고, 그러면 "사진 취득이 ssh 왕복 상한을 넘겨 실패"하던 문제도 사라진다.
   ⚠️ `tools/` 쪽 파일명은 `grab_shot.py`, `patrol_core/` 쪽은 `shot_grab.py`로 **어순이 반대**다 — 혼동 주의
3. **`patrol_auto.launch.py`를 온보드용으로 점검** — VM 기준으로 쓰여 있다
4. **안전모 추론을 로봇에서 돌리고 CPU 측정** → 그 결과로 threading
   (MultiThreadedExecutor + callback group) 설계. 강사 피드백의 핵심이 이것이다
5. **확장 기능 5개** — 안전조끼 인식 → 소화기 QR → 웹 대시보드 → 야간 순찰 → 금지구역 침입
   (상세·우선순위는 `docs/memory/pi4-upgrade-plan.md`)

### 미적용으로 남은 권고

- `config/patrol_nav2.yaml`의 `transform_tolerance` 완화 (costmap 0.2→1.0, AMCL 1.0→2.0)
  — 온보드 Nav2로 옮긴 뒤에는 필요성이 줄었으니, 실주행에서 스캔 드롭이 보일 때만 적용
- 방열판 실제 구매 여부 미확인
