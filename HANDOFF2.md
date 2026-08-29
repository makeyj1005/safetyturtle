# 실내 순찰 로봇 — 인수인계 문서 (2026-08-14 전면 재작성)

**이 문서가 현재 기준이다.** 이전 인수인계(`HANDOFF.md`, 2026-08-02 기준)는 상세 이력·실측
근거 보관용이고, 시스템 구조가 그 뒤로 바뀌었다(VM 중심 → 온보드 Nav2). 처음 맡는
사람은 이 문서를 위에서부터 읽으면 된다.

**문서 지도** — 무엇을 어디서 찾나:

| 문서 | 용도 |
|---|---|
| **이 문서 (HANDOFF2.md)** | 현재 상태, 실행 절차(명령어 전부), 최근 세션 기록, 다음 할 일 |
| `CLAUDE.md` | 프로젝트 지침 요약. AI 어시스턴트 세션 시작용이지만 사람이 읽어도 좋은 요약 |
| `docs/memory/MEMORY.md` | 축적된 교훈 18개의 색인. 한 줄씩 훑고 관련된 것만 열기 |
| `HANDOFF.md` | 2026-08-02까지의 상세 이력. **통째로 읽지 말고** 목차(`grep -nE "^#{1,3} " HANDOFF.md`)로 필요한 절만 |
| `순찰로봇_최종보고서.pdf` | 대회 제출용 최종 보고서 |

---

## 1. 프로젝트가 무엇인가

TurtleBot3 Burger 기반 실내 순찰 로봇. 영남이공대 HI-CEED AI 콘텐츠 경진대회 출품작.

**세 가지 기능, 전부 실물 검증 완료:**
1. **랜덤 시각 자동 순찰** — Nav2 좌표 이동으로 사각형 웨이포인트 4점을 돈다
2. **소화기 점검** — 소화기 앞으로 이동해 압력계를 촬영·판정(기준 사진과의 차분), 부저로 알림
3. **안전모 감지** — 순찰 중 사람을 보면 정지, 안전모(초록 테이프) 착용 여부를 5초간 판정,
   미착용이면 부저 반복

**핵심 이력 세 줄:**
- 2026-07-28~08-03: VM 중심 구조로 세 기능 완성·실주행 검증 (상세는 `HANDOFF.md`)
- 2026-08-07: 라즈베리파이 3 → **4** 교체, 새 SD 클린 설치, **Nav2를 로봇 안으로 이식**(온보드)
- 2026-08-14: **온보드 첫 실주행 1바퀴 완주.** 카메라 각도 고정, 안전모 색 기준 재보정,
  "첫 구간 정체" 원인 특정 (이 문서 7절)

---

## 2. 현재 상태 요약 (2026-08-14 기준)

| 항목 | 상태 |
|---|---|
| 순찰 (Nav2 좌표 이동) | ✅ 온보드 실주행 1바퀴 완주 (4.5분, 첫 구간 정체 포함 — 원인 특정됨, 7절) |
| 소화기 압력계 점검 | ✅ 실주행 검증 완료 (2026-08-01, 정합 0.90, 30초) |
| 안전모 감지 | ✅ 실주행 검증(08-03) + **08-14 재보정** (착용 0.047~0.051 / 미착용 0.0000~0.0014) |
| 웹캠 각도 | ✅ 상향 35~40°로 고정 확정 (좁은 방 근거리 커버에 이 각도가 맞음) |
| 온보드 Nav2 | ✅ 9개 노드 active + 실주행 검증 |
| RViz 없는 초기 위치 지정 | ✅ 로봇에서 `/initialpose` 반복 발행으로 실증 (5.4절) |
| 순찰 전 정렬 회전 | ⬜ **다음 주행에서 검증할 신규 절차** (7절의 해법) |
| ssh → 로컬 모드 전환 | ⬜ 미착수 (8절) |
| 안전모 추론 온보드 이전 + threading | ⬜ 미착수 (8절) |
| QR 내용연한 / 확장 기능 5종 | ⬜ 미착수 |

**아키텍처 (현행):**

```
[로봇 ros18 (Pi 4)]                         [VM]
  bringup (모터·라이다·odom·부저)             patrol_node      ← 순찰 로직
  Nav2 전체 (map_server~behavior)      ←───  patrol_scheduler  ← 랜덤 시각
  webcam_node (USB 웹캠 640x480 3fps)  ───→  helmet_node      ← 사람·안전모 판정
  CSI 카메라 (점검 때만 ssh로 켬)       ←───  inspect_node     ← 소화기 점검
        └── /scan → Nav2 → /cmd_vel 이 전부 로봇 안 (무선 무관)
```

장기 목표는 "VM = 화면만"(판정 노드까지 전부 온보드)이지만, **지금 검증된 배치는 위와 같다.**

---

## 3. 절대 규칙 (어기면 시간을 크게 잃는다 — 전부 실제 사고에서 나온 규칙)

1. **`ROS_DOMAIN_ID=3` 고정.** 시험용으로도 다른 도메인 금지. 새 터미널마다 export 필요
2. **RMW는 fastrtps** (기본값). cyclonedds 설정은 폐기됨. 통신 안 되면 이것부터 확인
3. **`pkill -f` 금지** — 자기 셸까지 죽인다. 패턴에 `[문]자` 트릭을 쓰거나 PID로 죽일 것.
   `ros2 run`/`ros2 launch`를 죽여도 자식 노드는 살아남는다 → 종료 후 반드시 `pgrep -a -f` 확인
4. **`/cmd_vel`은 직접 발행하지 않는다.** 정지가 필요하면 Nav2 목표 취소로 한다
   (helmet_node의 `/patrol/hold`가 이 방식). teleop을 쓰려면 `-r /cmd_vel:=/cmd_vel_teleop`
5. **웹캠은 부팅당 한 번만 열고 유지한다.** 여러 번 열고 닫으면 장치가 잠겨(`select() timeout`)
   로봇 재부팅 외에는 못 푼다. 노드를 내려도 웹캠 프로세스는 남겨두는 게 기본값
6. **라인 추종을 다시 제안하지 않는다.** 세 차례 검토 후 기각. 장소가 바뀌면 재매핑이 답
7. **시험 노드를 실제 시스템과 같이 띄우지 않는다.** 같은 노드가 둘이면 Nav2 목표를 서로
   뺏고 CSV를 겹쳐 쓴다 (실측: 회차 하나를 통째로 버렸다)
8. **실측과 추측을 구분해서 기록한다.** 이 프로젝트는 추측으로 시간을 크게 잃은 이력이 많다

---

## 4. 로봇 접속

- hostname `ros18`, 계정 `rpi`, Raspberry Pi 4 Model B Rev 1.5, Ubuntu 22.04, sudo NOPASSWD
- ssh 키 인증 설정됨 (비밀번호 불필요)
- **IP는 DHCP라 재부팅마다 바뀐다.** MAC으로 찾는다:
  `88:a2:9e:ff:46:c2`(wlan0) / `88:a2:9e:ff:46:c1`(eth0)
  ⚠️ 옛 문서의 `b8:27:eb` 스윕은 Pi 3 시절 것 — 이제 안 잡힌다

```bash
# [VM] 로봇 IP 찾기
for i in $(seq 1 254); do (ping -c 1 -W 1 192.168.0.$i >/dev/null 2>&1 &); done; sleep 8
ip neigh show dev ens33 | grep -i 88:a2:9e
```

```bash
# [VM] 접속 (새 IP면 호스트키 자동 수락)
ssh -o StrictHostKeyChecking=accept-new rpi@<IP>
```

증상 구분: `No route to host` = 그 IP에 아무도 없음(꺼짐/IP 변경) /
`Connection refused` = 부팅 중(sshd 대기, 30초~1분)

---

## 5. 실행 절차 — 온보드 순찰 (2026-08-14 실주행으로 검증된 순서)

모든 VM 터미널에서 먼저:
```bash
export ROS_DOMAIN_ID=3 && source /opt/ros/humble/setup.bash
```
patrol_core 명령을 쓰는 터미널은 추가로:
```bash
source ~/vibe/ex1/ros2_ws/install/setup.bash
```

### 5.0 사전 점검

```bash
# [VM] 배터리 — 11.3V 아래면 주행 금지, 충전부터 (11.0V에서 노드가 죽는다)
ros2 topic echo /battery_state --once | grep voltage
```
실측: 순찰 1바퀴에 약 0.14V 소모. 12.3V 만충 기준 넉넉히 4~5바퀴 분량.
개발·정지 작업은 SMPS 어댑터로 할 것 (배터리는 주행 시험 전용).

### 5.1 [로봇] bringup

```bash
# [로봇 ssh]
ros2 launch turtlebot3_bringup robot.launch.py
```
`Run!`이 나오고 `Failed to read`가 없어야 정상.

```bash
# [VM] 토크 확인 — false면 아래 서비스로 켠다
ros2 topic echo /sensor_state --once | grep torque
ros2 service call /motor_power std_srvs/srv/SetBool "{data: true}"
```

### 5.2 [로봇] 웹캠 (안전모용)

이미 떠 있는지 먼저 확인하고, 떠 있으면 **그대로 둔다** (절대 규칙 5):
```bash
# [로봇 ssh]
pgrep -af '[w]ebcam_node.py'
```
없을 때만 띄운다 (ssh가 끊겨도 살아남게 setsid로 분리):
```bash
# [로봇 ssh]
export ROS_DOMAIN_ID=3; source /opt/ros/humble/setup.bash
( setsid nohup python3 -u ~/launch/webcam_node.py > ~/webcam.log 2>&1 < /dev/null & )
sleep 4 && tail -3 ~/webcam.log     # "웹캠 발행 시작" 확인
```
장치는 이름(DICOTA)으로 자동 탐지하므로 번호가 밀려도 상관없다.

### 5.3 [로봇] Nav2 온보드 기동

```bash
# [로봇 ssh]
export ROS_DOMAIN_ID=3; source /opt/ros/humble/setup.bash
( setsid nohup ros2 launch ~/vibe/ex1/launch/nav2_patrol_onboard.launch.py > ~/nav2.log 2>&1 < /dev/null & )
```
확인 (약 15초 뒤):
```bash
# [로봇 ssh]
grep -c ERROR ~/nav2.log                          # 0 이어야 함
grep -c "Managed nodes are active" ~/nav2.log     # 2 (localization + navigation)
```
⚠️ 이 시점의 `Invalid frame ID "map" ... frame does not exist`는 **ERROR가 아니라 INFO**이며
초기 위치를 기다리는 정상 대기 상태다. 오류로 오판하지 말 것.

### 5.4 [로봇] 초기 위치 지정 — RViz 없이 된다

로봇이 **어디에 어느 방향으로 놓였는지** 알아야 한다. 순찰 시작 코너(좌상)에 놓고
다음 코너(우상) 방향을 보게 두는 것이 가장 확실하다. 다른 곳에 놓으면 그 좌표·방향으로
아래 x, y, yaw를 바꾼다. 위치추정만 잡히면 시작지점까지는 로봇이 알아서 간다.

```bash
# [로봇 ssh]  (VM에서 --once로 쏘면 디스커버리 전에 사라져 안 닿는다 — 로봇에서 반복 발행)
export ROS_DOMAIN_ID=3; source /opt/ros/humble/setup.bash
timeout 8 ros2 topic pub -r 2 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {frame_id: map}, pose: {pose: {position: {x: 0.160, y: -0.270, z: 0.0},
 orientation: {z: 0.0, w: 1.0}},
 covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0,
              0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.3]}}"
```
(orientation은 yaw의 절반각: z=sin(yaw/2), w=cos(yaw/2). yaw 0 = +x = 우상 방향.
covariance 마지막 0.3은 방향 불확실성 ±30° — 놓은 방향이 살짝 틀려도 수렴할 여지)

확인:
```bash
# [로봇 ssh]
grep "Setting pose" ~/nav2.log | tail -1
timeout 10 ros2 topic echo /amcl_pose --once | head -12
```

### 5.5 [로봇] ⭐ 순찰 전 정렬 회전 — 신규 절차 (다음 주행에서 검증)

**왜:** "첫 구간 정체"의 특정된 원인(7절) 때문. 초기 위치 힌트가 실제와 어긋난 채 출발하면
DWB가 0속도 교착에 빠진다. 회전은 이동 없이 AMCL 보정(2.9°마다)을 강제하므로,
**출발 전에 어긋남을 수렴시킨다.**

```bash
# [로봇 ssh]  +90° 돌렸다가 -90° 되돌린다
ros2 action send_goal /spin nav2_msgs/action/Spin "{target_yaw: 1.57}"
ros2 action send_goal /spin nav2_msgs/action/Spin "{target_yaw: -1.57}"
```
합격 기준: 이 절차 후 순찰 첫 구간이 **45초 이내 + `Failed to make progress` 0회**.
검증되면 patrol_node에 파라미터로 넣는 코드화를 검토한다.

### 5.6 [VM] 순찰 시작

```bash
# [VM]
export ROS_DOMAIN_ID=3 && source /opt/ros/humble/setup.bash && \
source ~/vibe/ex1/ros2_ws/install/setup.bash && \
ros2 launch ~/vibe/ex1/launch/patrol_auto.launch.py robot_host:=rpi@<로봇IP>
```
- patrol_node + patrol_scheduler + inspect_node + helmet_node 네 개가 함께 뜬다
- 기본 동작: 10초 뒤 첫 회차(1바퀴) → 끝나면 1분 쉼 → 1~2분 안의 랜덤 시점에 다음 회차 반복
- `robot_host`는 반드시 오늘의 로봇 IP로 (기본값이 옛 Pi 3 주소로 박혀 있다 — 8절)
- 자주 쓰는 인자: `laps:=2`(회차당 바퀴), `cycle_min:=5.0`(운용 주기),
  `fire_on_start:=false`(켜자마자 돌지 않기), `helmet:=false`(안전모 감지 없이),
  `inspect:=false`(점검 노드 없이), `quiet:=true`(시연용 로그 정리)

### 5.7 모니터링

```bash
# [VM] 어느 터미널이든
ros2 topic echo /patrol/status      # moving to.../done(...)  — 순찰 진행
ros2 topic echo /patrol/schedule    # 다음 순번 시각과 작업 종류
ros2 topic echo /inspect/status     # 점검 진행
ros2 topic echo /helmet/status      # 안전모 판정 상태
```
```bash
# [로봇 ssh] Nav2 쪽 이상 감시
tail -f ~/nav2.log | grep -E "ERROR|Failed|Running"
```

### 5.8 정지와 정리

```bash
# [VM] 순찰만 세우기 (노드는 유지) — 진행 중 목표를 취소하고 선다
ros2 topic pub -w 1 --once /patrol/enable std_msgs/msg/Bool "{data: false}"
```
```bash
# [VM] 전체 종료: patrol_auto 터미널에서 Ctrl+C 후, 잔여 노드 확인 (절대 규칙 3)
ps -eo pid,etime,cmd | grep -E "[p]atrol_|[h]elmet_|[i]nspect_"    # 남았으면 kill -9 <PID>
```
```bash
# [로봇 ssh] Nav2 종료 (SIGINT → 라이프사이클 정리에 몇 초 걸린다)
pkill -INT -f '[n]av2_patrol_onboard'; sleep 8
pgrep -a -f '[c]omponent_container' || echo "정리 완료"
```
웹캠·bringup은 세션 안에서는 내리지 않는 게 기본 (웹캠은 절대 규칙 5, bringup은 재기동 비용).

### 5.9 소화기 점검

```bash
# [VM] 점검 예약 — 즉시 가지 않고, 돌던 순찰을 마친 뒤 다음 순번에 간다
ros2 topic pub -w 1 --once /inspect/request std_msgs/msg/Bool "{data: true}"
```
```bash
# [VM] 시험용: 순찰 없이 지금 바로 점검 한 번 (patrol_auto와 동시 실행 금지!)
ros2 launch ~/vibe/ex1/launch/inspect_once.launch.py robot_host:=rpi@<로봇IP>
```

---

## 6. 기능별 요점 — 어떻게 동작하나

### 6.1 순찰

- 웨이포인트: `maps/patrol_waypoints.yaml` — 지도에서 계산해 넣은 사각형 4점
  (좌상 +0.160,-0.270 → 우상 +1.310,-0.270 → 우하 +1.310,-1.220 → 좌하 +0.160,-1.220,
  가로 1.15m × 세로 0.95m). **손으로 찍지 말 것** — 위치추정 오차가 섞여 경로가 지그재그 된다
- 정지 기준은 시간이 아니라 **바퀴 수** (`laps`). 다음 회차 기준점은 **직전 순찰이 끝난 시각**
  (`done` 수신) + rest_min~cycle_min 균등난수. 스케줄러는 True만 보내고 False는 안 보낸다
- 순찰↔점검은 스케줄러가 **한 순번에 하나만** 배정 (둘 다 Nav2를 쓰므로)
- Nav2 파라미터: `config/patrol_nav2.yaml` (좁은 방 튜닝: inflation 0.25, xy_tol 0.10,
  yaw_tol 0.05, max_vel 0.15, progress_checker 0.10m/40s, AMCL update_min 0.05).
  조정 근거는 파일 주석과 `docs/memory/nav2-tuning-for-tiny-room.md`

### 6.2 소화기 점검

- 점검 자세: `maps/fire_extinguisher_points.yaml` — A 활성 (1.107, -0.555, yaw -88°),
  대안 B는 `_b.yaml`. 노드는 시작할 때 한 번만 읽으므로 바꾸면 재시작
- 판정 원리: 바늘 절대각도가 아니라 **기준 사진과의 차분** (고정된 어두운 것들이 상쇄됨).
  변화량 문턱 0.9 (정상 최대 0.64 / 이상 최소 1.52). "판정불가"를 반드시 따로 둔다
- 도착 후 **카메라를 보며 제자리 회전(/spin)으로 정렬** — 지도 각도를 믿지 않는다.
  회전 방향·환산값(1도당 px)은 돌려본 반응으로 스스로 잰다
- 사진은 스트리밍이 아니라 **로봇 로컬 저장 → tar 스트림 회수** (1640x1232는 무선 DDS로 안 온다)
- 부저: 정상 = 1번 음 4연타(길게), 이상 = 2번 음 1회(짧게).
  ⚠️ 2번 음은 **저전압 경고음과 같다** — 경보가 잦으면 판정보다 배터리부터 볼 것

### 6.3 안전모 감지

- 방식: `method:=color` — 흰 안전모 **위쪽 돔**에 붙인 초록 테이프를 본다
  (8가지 실패 끝의 해법. 색 있는 안전모로 바꾸면 테이프 불필요 — 기준만 재등록)
- 판정 흐름: 사람 2프레임 연속 → **일단 정지** → 5초간 모아서 미착용이
  "사람 프레임의 50% 이상 + 3장 이상"이면 확정 → 부저 3연타 반복 / 착용이면 1번 음 내고 재개
- 현재 색 기준 (**2026-08-14 재보정**): H 61~84, S 51~173, V 59~203.
  실측 분리: 착용 0.047~0.051 / 미착용 0.0000~0.0014 (문턱 0.010, 34배 여유)
- 재보정 절차 (안전모·조명·카메라 각도가 바뀌면):
  ```bash
  # [VM] 웹캠이 떠 있는 상태에서, 실제 순찰 거리·조명에서 쓴 사람을 찍어 등록
  python3 ~/vibe/ex1/tools/helmet_calib.py --grab --name 초록테이프 --select
  ```
  ⚠️ **선택 영역은 테이프(또는 안전모) 안쪽 순수 영역만.** 색 화소의 경계 사각형으로
  잡으면 모서리의 흰 헬멧 화소가 섞여 H 범위가 벌어지고 **머리카락(H 35~42)이 착용으로
  오판**된다 — 2026-08-14에 실제로 겪고 재등록으로 해결했다.
  등록 후 "사진 전체 %"가 높으면 배경 오염 신호이니 그 자리에서 다시 잡을 것
- 카메라 각도: **상향 35~40° 고정** (2026-08-14 확정). 방이 좁아 사람이 가까이(1m 안팎)
  올 수밖에 없는데, 이 각도라야 근거리 머리가 화면에 들어온다. 0.8m 이내 극근접은
  머리가 잘려 "판정 보류"가 나는 게 정상 동작

---

## 7. 2026-08-14 세션 기록 — 무엇을 어떻게 했나

### 7.1 웹캠 각도 검증·고정

방법: VM에서 웹캠 프레임을 받아 helmet_node와 동일한 검출기(MobileNet-SSD)로 사람 상자를
재고, 머리 위 여유(≥6px 필요)·상자 세로 비율(≥0.30 필요)을 수치로 확인했다.
결과: 1.5m 거리에서 여유 ~120px, 세로 비율 0.75로 합격. 각도 상향 35~40° 유지 결정
(25~30°로 낮추면 1.1m 이내 사람의 머리가 잘린다 — 좁은 방에는 가파른 쪽이 맞다).

### 7.2 안전모 색 기준 재보정

- 증상: 착용인데 초록 비율 0.003~0.006(문턱 미달) → 원인: 각도 변경으로 테이프가 어둡게
  찍혀(V 90~114) 기존 하한(V≥114)에 잘림
- 1차 재등록에서 **ROI를 경계 사각형으로 잡는 실수** → 미착용자가 착용으로 오판되는 부작용
  → 순수 테이프 영역(순도 100% 확인)으로 2차 재등록해 해결 (위 6.3의 ⚠️)
- 산출물: `maps/helmet_calib.yaml` 갱신 (백업 `.bak_20260814`,
  근거 사진 `logs/helmet_calib/on0814_recalib.jpg`)

### 7.3 온보드 첫 실주행 — 1바퀴 완주

5절의 절차 그대로 (5.5 정렬 회전만 없이) 실행했다. 결과:

| 구간 | 소요 | 비고 |
|---|---|---|
| 시작지점 이동 | 0.02초 | 이미 허용오차 안이라 즉시 성공 (실주행 아님 — 7.4 참고) |
| 좌상→우상 (첫 구간) | **197초** | `Failed to make progress` **4연속**(40초 한도마다) + 90° 회전 복구 후 22초 만에 도달 |
| 우상→우하 | 12.3초 | 정상 |
| 우하→좌하 | 13.2초 | 정상 |
| 좌하→좌상 | 10.7초 | 정상 |
| 총 | 4.5분 | 배터리 11.58 → 11.38V (1바퀴 ≈ 0.14V) |

동시 가동: 온보드 Nav2 + VM patrol_auto 4노드 + 안전모 판정(2.8fps, 오탐 정지 0회).

### 7.4 "첫 구간 정체" 원인 특정 — 이 프로젝트의 오래된 미스터리

**증거:**
- "No valid trajectories" 0건 → 장애물에 막힌 게 아님. DWB가 유효 궤적을 내면서도
  **0속도만 선택**(오류 로그 없는 완전 정지)
- 목격: 처음 ~11초는 전진, 그 뒤 완전 정지, 회전 복구 후 갑자기 정상
- 회전 복구 직후 AMCL 위치가 경로에서 남쪽으로 80cm+ — 이후 구간 소요시간들과 전부
  정합 → 이 값이 참값이고, **초기 위치 힌트가 실제 놓인 위치·방향과 어긋나 있었다**
- 정적 지도 분석: 첫 구간 통로 여유(평균 0.39m)는 다른 구간보다 오히려 넓다 → 구조 문제 아님
- costmap 비우기 복구 3회는 무효과, **회전만이 풀었다**

**메커니즘:** 초기 위치가 어긋난 채 출발 → 전진할수록 "믿는 지도"와 "실제 스캔"의 불일치
누적 → DWB가 전진 궤적을 전부 저평가해 0속도 교착 → 이동이 없으니 AMCL 보정도 정지.
**회전은 이동 없이 AMCL 갱신을 강제**하므로 교착을 깬다. 매 세션 첫 구간만 느렸던 이유 =
첫 구간이 항상 "초기 위치를 방금 찍은 직후"이기 때문. (기존의 "소화기↔local costmap"
가설은 후순위로 내림. 상세: `docs/memory/nav2-tuning-for-tiny-room.md`)

**교훈 하나 더:** "시작지점 이동 성공"은 위치추정이 정상이라는 증거가 아니다 — 믿는 위치가
허용오차(10cm) 안이면 바퀴를 굴리지 않고도 즉시 성공 처리된다.

---

## 8. 다음 할 일 (우선순위)

1. **정렬 회전 절차 검증** (5.5절) — 충전 후 주행. 합격: 첫 구간 45초 이내 + FTMP 0회.
   AMCL 위치를 초 단위로 기록하는 로거를 붙여 위치 도약 여부도 직접 확인.
   검증되면 patrol_node에 "순찰 전 정렬 회전" 파라미터로 코드화
2. **ssh 전제를 로컬 모드로** — 옛 Pi 3 주소 `192.168.0.67`이 소스 7개 파일 15곳에 하드코딩.
   기능에 영향 주는 기본값 6곳: `helmet_node.py:308`, `inspect_node.py:160`,
   `patrol_auto.launch.py:91`, `inspect_once.launch.py:39`, `grab_shot.py:42`, `aim_gauge.py:239`.
   IP만 바꾸지 말고 **같은 기계면 ssh를 건너뛰는 로컬 모드**를 추가할 것 (온보드 이전의 전제).
   2026-08-14에도 helmet_node의 ssh 시간초과 경고가 재현됐다(웹캠이 떠 있어 실해는 없었음).
   고친 뒤 `colcon build` 필수. ⚠️ `tools/grab_shot.py`와 `patrol_core/shot_grab.py`는
   이름 어순이 반대인 다른 파일이다
3. **patrol_auto.launch.py 온보드용 점검** — VM 기준으로 쓰여 있다 (2번과 한 묶음)
4. **안전모 추론을 로봇에서 돌리고 CPU 실측** → MultiThreadedExecutor + callback group
   설계 (강사 피드백의 핵심). 참고 실측: Pi 4 추론 405ms/장(2.5fps)
5. **확장 기능 5종** — 안전조끼 → 소화기 QR → 웹 대시보드 → 야간 순찰 → 금지구역
   (우선순위·걸림돌: `docs/memory/pi4-upgrade-plan.md`)

**예비 배터리 1개 구매 권장** (하나 충전/하나 주행 — 시연·대회 사실상 필수).

---

## 9. 자주 겪는 함정 — 빠른 색인

상세 재현·해법은 `HANDOFF.md`의 해당 절(번호 동일)과 `docs/memory/`를 볼 것.

| # | 증상 | 첫 확인 |
|---|---|---|
| 1 | 로봇에선 토픽이 보이는데 VM에선 안 보임 | `ros2 daemon stop && rm -rf /dev/shm/fastrtps_* && ros2 daemon start`. `ros2 topic list --no-daemon`으로 데몬 거짓말 확인 |
| 2 | `Navigation: inactive` | 파라미터 버전 불일치 — `param/humble/burger.yaml` 계열인지 확인 |
| 3 | Nav2가 목표를 거절 반복 | bringup 죽음(`/odom` 0Hz) 또는 초기 위치 미지정 |
| 4 | 명령은 나가는데 바퀴가 안 돎 | `/sensor_state`의 `torque` → `/motor_power`로 켬 |
| 5 | `There is no status packet!` → stack smashing | 모터 12V 전원 문제 (배터리/OpenCR 스위치/케이블) |
| 6 | 주행 중 "삐" 경고음 | **저전압(11V)** — 소화기 이상음과 같은 소리다. 전압부터 볼 것 |
| 7 | 스케줄러가 쐈는데 순찰 시작 안 됨 | `/patrol/enable`은 VOLATILE — 구독자 없으면 증발. 노드 떠 있는지 확인 |
| 8 | 순찰 끝났는데 다음이 안 잡힘 | `/patrol/status`에 `done`이 왔는지 확인 |
| 9 | 카메라 인덱스가 밀림 / 라이다에 가짜 벽 | USB 꽂으면 CSI 0→1. 카메라는 스캔 평면 위로. `/scan`에 12cm 미만 값 = 로봇에 붙은 물체 |
| 10 | 좌표 저장이 옛 위치로 됨 | AMCL은 움직여야 갱신 — 저장 전 20°씩 두세 번 회전 |
| 14 | 카메라를 껐는데 안 꺼짐 | launch를 죽여도 camera_node 고아가 남는다 — 이름으로 pgrep/pkill (`[c]` 트릭) |
| 15 | `/sound` 발견에 30초 | 무선 DDS 서비스 발견이 느림 — 짧은 timeout으로 "없다" 판정 금지 |
| 16 | 점검 사진에 게이지가 없음 | yaw 허용오차·자세 어긋남 — 카메라 정렬 루프가 처리. 문턱은 `HANDOFF.md` 함정 16 |
| 17 | 새 카메라가 안 뜸 (`in use by another process`) | 매달린 camera_node가 장치 점유 — SIGKILL 필요 |
| 18 | CSV가 깨짐 / 복귀가 0.1초 완료 | 같은 노드 2개 실행 중 — `ps`로 오래된 것 kill -9 |
| 19 | 바늘 안 움직였는데 "이상" | 자세 어긋남 헛경보 — 정렬 탐색(±4px)이 처리, 문턱 임의 완화 금지 |
| 신규 | **첫 구간에서 로봇이 조용히 정지** | 초기 위치 어긋남 교착 (7.4절) — 정렬 회전(5.5절) |
| 신규 | 착용인데 미착용 판정 (또는 반대) | 색 기준 재보정 — 단, ROI는 순수 영역만 (6.3절 ⚠️) |

---

## 10. 파일 구조 (2026-08-14 기준)

| 경로 | 내용 |
|---|---|
| `ros2_ws/src/patrol_core/patrol_core/` | 노드 본체: `patrol_node`(순찰) `patrol_scheduler`(랜덤 시각) `helmet_node`(안전모) `inspect_node`(점검) `gauge`(압력계 판정, ROS 무관) `shot_grab`(사진 취득, ROS 무관) `explore_node`(frontier 탐사) `cmd_vel_mux`(중재 — 현재 순찰에는 미사용) |
| `launch/` | `nav2_patrol_onboard.launch.py`(**로봇용 Nav2 — 현행**) `patrol_auto.launch.py`(VM 통합 순찰) `nav2_patrol.launch.py`(VM용 Nav2 — 구식) `inspect_once.launch.py`(점검 단독 시험) `cartographer_resumable.launch.py`(이어 매핑) `robot_camera.launch.py`(CSI) |
| `config/` | `patrol_nav2.yaml`(Nav2 파라미터, 조정 근거 주석 포함) `patrol.rviz` |
| `maps/` | 최종 지도 `patrol_map_v5.pgm/yaml` (`patrol_map.*`는 구버전) / `patrol_waypoints.yaml` / `fire_extinguisher_points.yaml`(+`_b`) / `gauge_calib.yaml` + `gauge_ref_*.png` / `helmet_calib.yaml`(08-14 재보정, `.bak_20260814`) / `state/patrol_state5.pbstream`(이어 매핑용 최신) |
| `models/` | MobileNet-SSD 2파일 (사람 검출) |
| `tools/` | `webcam_node.py`(로봇 웹캠 — 로봇 `~/launch/`에 심볼릭) `helmet_calib.py` `gauge_calib.py` `save_waypoint.py` `aim_gauge.py` `grab_shot.py` `fake_nav2.py`(로봇 없이 순찰 흐름 시험) `live_view.py` `make_report.py` 등 |
| `logs/` | 순찰·점검·안전모 CSV + 증빙 사진 (실주행 기록) |
| `backup/` | 작업 전 스냅샷 tar.gz |
| `docs/memory/` | 교훈 18개 (`~/.claude/projects/.../memory/`와 이중화 — **고치면 양쪽 동기화**) |

지도가 바뀌는 상황(새 장소 시연)이면: 그 자리에서 cartographer 재매핑이 답이다.
지도+웨이포인트 30분, 소화기·안전모 재보정까지 1~2시간.

---

## 11. 다른 컴퓨터에서 시작하기 (인수자용 셋업)

**원본은 VM의 `~/vibe/ex1` 폴더다.** 로봇 안에도 사본이 있지만 최신이 아닐 수 있다
(예: 2026-08-14의 안전모 재보정은 VM에만 반영됨). 이 폴더를 통째로 받아라.

**받은 폴더는 반드시 홈 아래 `~/vibe/ex1`에 둘 것.** 노드·도구의 기본 경로가 전부
`~/vibe/ex1/...`로 박혀 있다 (사용자 이름은 달라도 된다 — `~` 기준이라 자동으로 맞는다).

새 컴퓨터(우분투 22.04 권장, VM이면 **bridged 네트워크** — 로봇과 같은 공유기 서브넷이어야 한다):

```bash
# [새 컴퓨터] 1) ROS 2 Humble + 필요한 패키지 (sudo 필요)
sudo apt install ros-humble-desktop ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-turtlebot3 ros-humble-turtlebot3-msgs ros-humble-turtlebot3-navigation2 \
  ros-humble-turtlebot3-cartographer python3-opencv python3-colcon-common-extensions
```
```bash
# [새 컴퓨터] 2) 폴더 배치 후 빌드 (build/install은 재생성되므로 복사할 필요 없다)
cd ~/vibe/ex1/ros2_ws && source /opt/ros/humble/setup.bash && colcon build
```
```bash
# [새 컴퓨터] 3) 환경 고정 — 매 터미널 반복이 싫으면 ~/.bashrc에 넣는다
echo 'export ROS_DOMAIN_ID=3' >> ~/.bashrc
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
```
```bash
# [새 컴퓨터] 4) 로봇 ssh 키 등록 (한 번만, rpi 계정 비밀번호 필요 — 인계자에게 받을 것)
ssh-copy-id rpi@<로봇IP>
```
확인 순서: ① 4절대로 로봇 IP 찾기 → ② `ssh rpi@<IP> hostname` 이 `ros18` →
③ 로봇 bringup 상태에서 `ros2 topic list`에 `/scan` `/odom`이 보이면 통신 OK →
④ 5절의 절차대로 순찰 실행.

통신이 안 되면: `ROS_DOMAIN_ID=3`인지, `RMW_IMPLEMENTATION`이 비어 있는지(기본 fastrtps),
같은 서브넷인지(bridged), 함정 1(데몬 캐시) 순으로 확인.

**전달 패키지에서 빼도 되는 것:** `ros2_ws/build·install·log`(재빌드로 생성),
`logs/`(118MB 실주행 증빙 — 보고서·이력용이니 필요하면 별도로), `backup/`(옛 스냅샷).
**꼭 들어가야 하는 것:** `ros2_ws/src`, `launch/`, `config/`, `maps/`(캘리브레이션 전부),
`models/`, `tools/`, `docs/`, `*.md`, `순찰로봇_최종보고서.pdf`.
