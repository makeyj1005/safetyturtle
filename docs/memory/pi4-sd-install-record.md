---
name: pi4-sd-install-record
description: 2026-08-07 Pi 4 + 새 64GB SD 클린 설치 완료 기록 — 카드리더기 없이 USB 부팅으로 우회한 방법과 그때 겪은 함정들
metadata: 
  node_type: memory
  type: project
  originSessionId: 2555f1fc-1d69-43e9-adbc-c328f1a22c22
  modified: 2026-08-07T09:00:23.263Z
---

**2026-08-07 완료.** [[pi4-upgrade-plan]] 의 1~2단계(보드 교체 + 새 카드 클린 설치)가 이날 끝났다. 이 메모는 **그 과정에서 얻은, 다시 필요해질 방법과 함정**을 남긴다.

## 최종 상태 (검증됨)

- Raspberry Pi 4 Model B **Rev 1.5**, 4코어, RAM 1844MB, hostname `ros18`
- 부팅 디스크 = **새 64GB SD (`mmcblk0`)**, 루트 58G 중 53G 여유 (옛 16GB 카드는 2.8G 여유뿐이었다)
- Ubuntu 22.04.5 / 커널 5.15.0-1061-raspi / cloud-init `done`
- `eth0` 유선 + `wlan0` 무선 **둘 다 동시에 UP** (IP는 DHCP라 계속 바뀜)
- `throttled=0x0`, 38.4°C — **Pi 3 시절의 저전압 경고가 사라졌다** ([[robot-undervoltage-warning]] 대비 개선)
- 옛 16GB 카드(SC16G, 2018-10)는 **손대지 않고 백업으로 보관** — 되돌리려면 이것만 다시 끼우면 5분

## 카드리더기가 없을 때 SD를 굽는 법 (이번에 실제로 쓴 우회)

카드리더기도, 노트북 SD 슬롯도 없었다. **빈 SD를 Pi 에 꽂아도 부팅이 안 되니 원격 설치가 불가능**하다(OS 가 없으면 ssh 도 없다 — 데이터를 VM 에 갖고 있는 것과는 다른 문제다). 해결:

1. **USB 메모리에** Raspberry Pi Imager 로 Ubuntu 를 굽는다 (USB 는 노트북에 바로 꽂히니 리더기가 불필요)
2. USB 를 Pi 에 꽂고 부팅 → **Pi 4 는 SD 가 없으면 USB 로 부팅한다**
3. 부팅된 상태에서 **빈 SD 를 전원 켠 채로 꽂는다** (핫플러그 됨, `/dev/mmcblk0`)
4. 로봇이 이미지를 직접 받아 **자기 SD 에 써넣는다**: `wget` → `xz -dc | sudo dd of=/dev/mmcblk0 bs=4M conv=fsync`
5. **SD 부팅 파티션에 cloud-init 파일 3개를 복사**: `/boot/firmware/{user-data,meta-data,network-config}`. 이걸 빼먹으면 SD 로 부팅했을 때 Wi-Fi·계정이 없어서 또 접속 불가가 된다
6. USB 빼고 SD 로 부팅

**`user-data` 를 직접 편집해 SSH 키를 넣지는 않았다** — YAML 을 잘못 건드리면 계정 생성 자체가 깨져 접속 불가가 된다. 대신 SD 부팅 후 사용자가 `ssh-copy-id` 를 한 번 실행했다. sudo 는 이 이미지에서 **NOPASSWD 라 어시스턴트가 설치를 전부 원격 진행할 수 있다**(옛 카드와 다른 점).

## 이날 겪은 함정 (재발 시 즉시 확인)

1. **`b8:27:eb` MAC 스윕으로 로봇을 못 찾는다** — Pi 4 로 바뀌며 MAC 이 `88:a2:9e:ff:46:c2`(wlan0) / `...:c1`(eth0) 로 바뀌었다. 상세는 [[robot-host-identity]]
2. **공유기 DHCP 목록에 `ubuntu` 로 뜬다** — DHCP 요청이 cloud-init 의 호스트명 변경보다 먼저 나가서다. 시스템 안에서는 `ros18` 이 맞다. 목록에 이름이 다르다고 설정 실패로 오판하지 말 것
3. **DHCP 목록에 있는데 ping 이 안 된다** — 목록은 "과거 임대 기록"이고 현재 접속 중이라는 뜻이 아니다. 꺼진 기기도 임대 시간이 남으면 계속 보인다
4. **첫 부팅이 몇 분씩 무응답** — 루트 파티션 확장(resize2fs) 중이다. 468GB USB 는 **수십 분**, 64GB SD 는 1~2분. 응답이 없다고 실패로 판단하지 말 것. 큰 용량 매체를 임시로 쓰면 이 대기가 길어진다
5. **Imager 내장 다운로더가 990MB 를 못 받는다** — 무선 손실 20~30% 에서 `Recv failure` 로 끊기고, **이어받기가 안 되어 처음부터 다시**다. 브라우저로 직접 받아 `사용자 지정 이미지 사용` 으로 넣거나, 핫스팟을 쓰는 게 답. 로봇이 유선으로 받으면 20MB/s 로 4분
6. **Imager 입력칸에 글자가 안 쳐진다** — 윈도우 IME/포커스 문제. 메모장에 쓰고 복사·붙여넣기로 우회했다
7. **`shutdown -h` 후에는 원격으로 다시 켤 수 없다** — 빨간 PWR LED 는 계속 켜져 있지만 OS 는 멈춘 상태다. 전원을 물리적으로 뺐다 꽂아야 한다

## Pi 4 전원에 대해

Pi 4 는 Pi 3 보다 전력을 훨씬 많이 먹는다. 설치 중 브라운아웃 재부팅을 의심해 **USB-C 충전기(5V 3A) 직결**로 바꿔봤고, 이후 **OpenCR 어댑터 전원으로도 `throttled=0x0` 유지**가 확인됐다. 배터리로 장시간 설치 작업은 하지 말 것(11V 아래면 노드가 죽는다).

OS 설치·ROS 설치 단계에서는 **라이다·OpenCR 이 전혀 필요 없다.** 라이다 USB 를 빼면 전력 여유가 생기고, 부저 소음도 OpenCR 을 안 켜면 사라진다(부저는 펌웨어에 있어 소프트웨어로 못 끈다).

## 유선이 결정적이었다

같은 순간 측정: **eth0 유선 7.07ms / 손실 0%** vs **wlan0 무선 107ms(최대 399ms)**. 15배 차이다. [[wireless-is-the-bottleneck]] 의 "VM 부하 때문에 부풀려진 값일 수 있다"는 가설은 이날 **기각됐다** — 양쪽 다 유휴 상태에서도 무선은 40ms·손실 20~30% 였다. VM 코어 2→4 는 CPU 경쟁만 풀고 무선은 못 고친다.

**설치는 유선으로, 주행은 무선으로.** 온보드 Nav2 가 완성되면 무선이 나빠도 주행에 지장이 없어진다(무선으로 넘어가는 건 화면·로그뿐).

## 온보드 Nav2 검증 완료 (같은 날, 2026-08-07)

로봇 안에서 Nav2 라이프사이클 **9개 노드 전부 `active`** 확인 (map_server, amcl, controller_server, smoother_server, planner_server, behavior_server, bt_navigator, waypoint_follower, velocity_smoother), ERROR 로그 0.

**실측 자원 (Pi 4, 4코어 / RAM 1844MB):**

| 구성 | CPU | 비고 |
|---|---|---|
| Nav2 전체(`component_container`) | 44.8% | = 1코어의 0.45개 |
| `turtlebot3_ros` + 라이다 | 14.4% | |
| **합계** | **4코어 중 약 15%** | 메모리 362MB, 44.8°C, `throttled=0x0` |

VM(2코어)에서 Nav2+추론이 다퉈 순찰 한 구간이 16초→107초가 됐던 문제는 **이 여유로 해소될 것으로 보인다.** 2GB 로 산 판단도 실측으로 정당화됐다.

### 새로 만들어야 했던 것 — `launch/nav2_patrol_onboard.launch.py`

기존 `nav2_patrol.launch.py` 는 **로봇에서 그대로 못 쓴다.** turtlebot3_navigation2 의 `navigation2.launch.py` 가 **RViz2 를 조건 없이 실행**하고(끄는 인자가 없다) 로봇에는 rviz2 를 깔지 않았기 때문이다. 원본이 하는 일은 ① `nav2_bringup/bringup_launch.py` 포함 ② RViz 실행 **딱 두 개뿐**이라, ①만 남긴 launch 를 새로 만들었다. 지도·파라미터 파일은 기존 것을 그대로 재사용한다(로봇/VM 의 nav2 1.1.20, turtlebot3_navigation2 2.3.6 이 동일함을 확인 후).

### 초기 위치(`/initialpose`)를 코드로 줄 때의 함정

**VM 에서 `ros2 topic pub --once /initialpose` 는 AMCL 에 닿지 않는다** — 디스커버리가 끝나기 전에 한 번 쏘고 끝나서 사라진다([[patrol-progress-nav2]] 의 `/patrol/enable` VOLATILE 함정과 같은 원인). **로봇 안에서 `-r 2` 로 몇 초간 반복 발행**하니 `initialPoseReceived` → `Setting pose` 가 찍히고 `map`→`base_link` 변환이 살아났다.

그리고 **초기 위치를 주기 전까지 Nav2 활성화가 `planner_server` 에서 멈춘다** — planner_server 는 global_costmap 을, global_costmap 은 `map` 프레임을, `map` 프레임은 AMCL 초기 위치를 기다린다. 이때 나오는 `Invalid frame ID "map" ... frame does not exist` 는 **ERROR 가 아니라 INFO** 이며 정상 대기 상태다. 오류로 오판하지 말 것.

**미검증:** ~~실제 주행~~(→ **2026-08-14 실주행 1바퀴 완주 검증됨.** 온보드 Nav2 + VM patrol_auto + 안전모 동시 가동, `Failed to make progress` 2회를 복구 스핀으로 자체 회복, 4.5분/바퀴로 8-07 실측 1.9분보다 느림 — 소화기↔local costmap 가설 분석 남음), 안전모 추론을 얹었을 때의 CPU, threading(MultiThreadedExecutor) 설계.

## 다음에 반드시 손봐야 하는 것 — ssh 를 전제로 쓴 노드들

데이터 이전은 완료됐다(VM/로봇 66개 파일 일치, 없는 `logs/` 는 코드가 `os.makedirs(exist_ok=True)` 로 만든다). 하지만 **코드 일부가 "VM 에서 돌면서 로봇에 ssh 한다"를 전제로 쓰여 있어 온보드에서는 그대로 못 돈다.**

- **`helmet_node.py:308`** — `robot_host` 기본값이 **`rpi@192.168.0.67`(옛 Pi 3 주소)로 하드코딩**돼 있다. 이 노드는 `shot_grab.ssh_cmd` 로 로봇의 `~/launch/webcam_node.py` 를 띄우고 내린다. 로봇 안에서 돌면 **자기 자신에게 ssh** 하는 셈이고, `rpi@localhost` 키도 없다.
- `inspect_node.py` 도 CSI 카메라를 같은 방식(ssh)으로 켰다 끈다 — 같은 문제일 것.
- `tools/` 의 `shot_grab.py` 계열도 host 인자를 받는다.

**고치는 방향:** IP 를 바꾸는 게 아니라 **ssh 를 아예 건너뛰는 로컬 모드를 추가**하는 쪽이 맞다. 같은 기계 안이니 프로세스를 직접 띄우면 되고, 그러면 [[wireless-is-the-bottleneck]] 에 기록된 "사진 취득이 ssh 왕복 35·60초 상한을 넘겨 실패" 문제도 같이 사라진다.

**`launch/patrol_auto.launch.py` 도 아직 온보드용으로 안 고쳤다** — `nav2_patrol.launch.py` 가 RViz 때문에 못 쓴 것처럼, 이것도 VM 기준으로 쓰여 있어 점검이 필요하다.

관련: [[pi4-upgrade-plan]], [[robot-host-identity]], [[wireless-is-the-bottleneck]], [[patrol-progress-nav2]]
