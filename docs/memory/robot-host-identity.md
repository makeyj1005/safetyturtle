---
name: robot-host-identity
description: "로봇 SBC와 CSI 카메라는 같은 기기(hostname ros18) — 2026-08-07 Pi 4 로 교체되며 MAC 이 88:a2:9e:ff:46:c2 로 바뀜(옛 b8:27:eb 검색법은 이제 안 통한다)"
metadata: 
  node_type: memory
  type: project
  originSessionId: 014807c6-8861-4a69-b2db-b81e5c632cb8
  modified: 2026-08-07T13:35:22.343Z
---

TurtleBot3의 SBC와 CSI 카메라는 **같은 한 대**다: 계정 `rpi`, hostname **`ros18`** (aarch64, Ubuntu 22.04.5, ROS 2 Humble). CSI 센서는 **`ov5647`(Camera v1)** 이다 — 예전에 imx219 로 적어둔 건 틀렸다([[csi-camera-stream-tuning]]).

**보드와 MAC이 2026-08-07 에 바뀌었다** — Raspberry Pi 3 Model B Rev 1.2 → **Raspberry Pi 4 Model B Rev 1.5** (4코어, RAM 1844MB). MAC 도 `b8:27:eb:9f:73:ec` → **`88:a2:9e:ff:46:c2`** 로 바뀌었다. **`b8:27:eb` OUI 로 스윕하면 로봇을 못 찾는다** — 실제로 이 때문에 로봇이 살아있는데도 "네트워크에 없다"고 오판한 적이 있다. 88:a2:9e 는 Pi 표준 OUI 목록(b8:27:eb, dc:a6:32, e4:5f:01, 2c:cf:67)에 없으므로 **OUI 추측 자체를 믿지 말 것.**

**MAC 대신 쓸 더 튼튼한 탐색법** — ROS 토픽은 IP와 무관하게 보이므로, 살아있는 호스트에 `hostname` 을 물어 `ros18` 을 찾는 게 확실하다:
```
for i in $(seq 1 254); do (ping -c 1 -W 1 192.168.0.$i >/dev/null 2>&1 &); done; sleep 8
for ip in $(ip neigh show dev ens33 | grep -v FAILED | awk '{print $1}'); do
  echo -n "$ip "; timeout 5 ssh -o StrictHostKeyChecking=accept-new -o BatchMode=yes rpi@$ip hostname 2>&1 | head -1
done
```

**IP를 신뢰하지 말 것.** 재부팅마다 DHCP가 다른 주소를 준다. 2026-07-28 하루 동안 확인된 이력: `192.168.0.22`(옛 세션) → `192.168.0.66` → `192.168.0.67`. 그리고 로봇이 떠난 주소를 **다른 기기가 곧바로 차지한다** — `.22`는 MAC `e4:1f:d5:...`, `.66`은 `b0:38:6c:...` 가 가져갔다. 그래서 옛 IP로 접속하면 "다른 기기에 붙거나" `No route to host` 가 난다.

**How to apply — 로봇 IP 찾는 방법:**
```
for i in $(seq 1 254); do (ping -c 1 -W 1 192.168.0.$i >/dev/null 2>&1 &); done; sleep 6
ip neigh show dev ens33 | grep -i b8:27:eb
```
MAC OUI로 보드 종류도 구분된다 — `b8:27:eb`=Pi 1~3, `dc:a6:32`/`e4:5f:01`=Pi 4. VM은 `192.168.0.52`(bridged, ens33).

**ROS 2 통신은 IP와 무관하다.** 디스커버리가 멀티캐스트로 이뤄지므로 `ROS_DOMAIN_ID=3`만 같으면 IP가 바뀌어도 토픽이 그대로 보인다. IP를 갱신해야 하는 건 **SSH / scp 뿐**이다.

증상 구분: `No route to host` = 그 IP에 아무도 없음(로봇이 꺼짐 또는 IP 변경). `Connection refused` = 호스트는 살아있고 sshd 가 아직 안 뜸(부팅 중, Pi 3 는 30초~1분 걸림).

보드는 **한 대뿐**이다(SBC=카메라). 2026-08-07 부터 그 한 대가 Pi 4 다 — 예전에 "SBC = 라즈베리파이 4"를 틀린 정보로 적어둔 적이 있는데, 그건 Pi 3 시절 이야기이고 지금은 Pi 4 가 맞다. `192.168.0.22` 는 여전히 로봇이 아니다(딴 기기).

VM → 로봇 SSH는 키 인증이 설정되어 있어 비밀번호가 필요 없다(키는 계정 기준이라 IP가 바뀌어도 그대로 통한다. 새 IP는 호스트키 확인만 필요하므로 `-o StrictHostKeyChecking=accept-new` 를 붙이면 된다). 단 **sudo 는 양쪽 다 비밀번호가 필요**하므로 패키지 설치는 사용자가 직접 실행해야 한다.

관련: [[ros2-rmw-fastrtps-decision]], [[csi-camera-stream-tuning]], [[nav2-not-line-following]]
