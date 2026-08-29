---
name: wireless-is-the-bottleneck
description: 사진 실패·목표 거절·위치추정 불량의 공통 원인은 VM 쪽 무선 지연이다 — 코드부터 의심하지 말 것
metadata: 
  node_type: memory
  type: project
  originSessionId: 778ff1b0-1416-4417-b48c-f274a626fe16
  modified: 2026-08-07T14:04:57.644Z
---

> ## ★ 2026-08-07 대폭 갱신 — 로봇 쪽 원인을 찾아 고쳤다
>
> **로봇의 Wi-Fi 절전 모드(`power_save on`)가 지연·지터의 주요 원인이었다.** 이 메모에 "확인에 `iw` 설치(sudo)가 필요해 보류 중"으로 남겨뒀던 가설이 맞았다. 새 SD 카드에서는 `sudo` 가 NOPASSWD 라 확인할 수 있었다.
>
> **끄기 전/후 실측 (VM → 로봇, 각 30회):**
>
> | | 평균 | 최대 | 지터(mdev) |
> |---|---|---|---|
> | 절전 ON | 6.79ms | 26.6ms | 5.64ms |
> | 절전 OFF | **3.89ms** | **11.0ms** | **1.68ms** |
>
> **그리고 이 메모가 기록한 "100ms 초과 구멍 24초당 33회"가 0회가 됐다** — 100 샘플에서 50ms 초과조차 0회. VM→로봇 평균 5.80ms 가 VM→공유기 5.62ms 와 거의 같아졌다. 즉 **로봇 무선 구간은 더 이상 병목이 아니다.**
>
> **영구화:** `/etc/systemd/system/wifi-powersave-off.service` (부팅마다 `iw dev wlan0 set power_save off`). 기본값이 `on` 이고 재부팅·인터페이스 재시작마다 되돌아가므로 서비스가 필요하다. `enabled`+`active` 확인됨.
>
> **아래 본문의 "5ms~1500ms", "VM→공유기 200ms" 는 절전 모드가 켜져 있던 시절의 값이다.** 호스트 PC 유선 랜 권고는 여전히 유효하지만(유선 7ms vs 무선 107ms 실측), 우선순위는 내려갔다.

VM ↔ 로봇 무선이 **5ms ~ 1500ms 사이를 오간다.** 로봇→공유기는 29ms 로 멀쩡하고
로봇 신호도 −29dBm 로 강한데 **VM→공유기가 200ms** 다 → 병목은 **호스트 PC 무선 구간**.

이 하나가 서로 달라 보이는 증상을 전부 만든다:

- 사진 취득 실패 (ssh 왕복이 35·60초 상한을 넘김)
- `/sound` 서비스 발견에 11~30초, 때로 실패
- **Nav2 가 `/scan` 을 통째로 버린다** — `Message Filter dropping message: frame 'base_scan'
  ... earlier than all the data in the transform cache` → AMCL 위치추정 불량,
  `No valid trajectories out of 819`, 목표 status=6 즉시 중단

  ⚠️ **`Failed to make progress` 는 이 목록에서 빼야 한다(2026-08-07 정정).** 그건 무선이
  아니라 `progress_checker` 기본값이 좁은 방에 안 맞아서 나던 것이었다 — 상세는
  [[nav2-tuning-for-tiny-room]]. 증상이 겹쳐서 오진했다. Nav2 를 온보드로 옮겨
  무선을 제거하고 나서야 구분이 됐다.

  ⚠️ **RViz 에서 보이는 `Message Filter dropping message ... queue is full` 도 다르다.**
  그건 VM 의 RViz 가 무선으로 들어오는 스캔을 못 따라가는 **화면 문제**이고,
  온보드 Nav2 의 주행에는 영향이 없다. Nav2 가 VM 에 있던 시절과 의미가 바뀌었다.
- `ros2 topic list` 가 빈 목록을 내놓음 (데몬 캐시. `--no-daemon` 으로 확인할 것)

**Why:** 2026-08-01 세션에서 이 증상들을 따로따로 디버깅하느라 시간을 크게 썼다.
카메라 인덱스·좌표·부호를 의심했지만 상당 부분이 무선이었다.

**How to apply:** 이런 증상이 보이면 **먼저 `ping <로봇IP>` 를 10회 재고 시작한다.**
평균 300ms 를 넘으면 코드를 고치기 전에 무선부터 본다. 근본 해결은 **호스트 PC 유선 랜**.
급하면 완화책: `config/patrol_nav2.yaml` 의 `transform_tolerance` 를 costmap 0.2 → 1.0,
AMCL 1.0 → 2.0 으로 늘려 늦게 온 스캔도 받아들이게 한다(정밀도는 떨어진다).

**2026-08 추가:** 측정한 ping 값 자체가 VM 부하에 영향받을 수 있다는 게 나중에 드러났다.
VM 이 2코어뿐이고, `ping` 응답도 VM 커널 네트워크 스택이 CPU 를 받아야 나가므로 VM 이
바쁘면(Nav2+추론+RViz+Claude Desktop 26%) ping 값 자체가 부풀려질 수 있다. 실제로
안전모 추론을 VM 에서 전속으로 돌리자 순찰 한 구간이 16초→107초로 늘었는데, 이게
"무선이 나빠져서"가 아니라 "VM 이 바빠서 스캔 처리가 밀려서"였을 가능성이 있다.
**완화책 두 가지를 권했지만 이번 세션 끝까지 실행 여부 미확인**: ① VM 코어 2→4
(VMware 설정, VM 종료 후 변경) ② 이 메모에 있는 `transform_tolerance` 완화. Pi 4로
Nav2 를 로봇 안으로 옮기면([[pi4-upgrade-plan]]) 이 무선 구간 자체가 사라지므로
이 문제의 근본 해결이 된다 — 다만 그 전까지는 위 두 완화책이 유효하다.

**2026-08-07 확인:** 위 두 완화책 중 ① VM 코어 2→4 는 **실행됐다**(nproc=4 확인). ② `transform_tolerance` 완화는 **미적용**(AMCL 1.0, controller 0.2 그대로). 그리고 "ping 값이 VM 부하 탓일 수 있다"는 가설은 **기각됐다** — 양쪽이 유휴인 상태에서도 무선은 40ms·손실 20~30% 였다. 코어를 4개로 늘려도 무선은 안 고쳐진다. 진짜 원인은 위에 적은 **로봇 절전 모드**였다.

## 5GHz 는 예비 카드로 등록해뒀다 (2026-08-07)

공유기에 **`team1_5g`**(WPA2-PSK[AES], 채널폭 20/40/80, 802.11a/n/ac/ax)를 만들고 로봇 netplan 에 **`team1_5g` + `team1` 둘 다 등록**했다. 한쪽이 안 되면 다른 쪽으로 붙어 **접속 잠금 위험이 없다.**

**netplan 은 wpa_supplicant 의 `priority` 를 노출하지 않는다** — 선택은 신호 세기로 결정되고, 실측에서 로봇은 2.4GHz(−52dBm)를 골랐다. 5GHz 를 강제하려면 netplan 에서 `team1` 을 잠시 빼야 하는데, **절전 모드를 끈 뒤 2.4GHz 로도 충분히 좋아졌으므로 지금은 불필요하다.**

**주변 스캔 실측(교실):** 2.4GHz 에 AP 6개가 몰려 있고 특히 `HIMADE`(2442MHz)가 우리 `team1`(2437MHz)과 5MHz 차이로 겹친다. 5GHz 는 훨씬 한산하다. **대회장처럼 2.4GHz 가 혼잡한 곳에서 꺼내 쓸 카드다.**

⚠️ **5GHz 채널은 "자동"보다 36·40·44·48 중 하나로 고정**하는 게 좋다. 자동은 DFS 채널(5260~5700)을 고를 수 있고, 그 대역은 레이더 감지 시 AP 가 채널을 비워야 해서 **갑작스런 끊김**이 생긴다.

## netplan 을 고칠 때 (2026-08-07)

`/etc/netplan/50-cloud-init.yaml` 은 cloud-init 생성물이라 그냥 고치면 재부팅 때 되돌아간다. `/etc/cloud/cloud.cfg.d/99-disable-network-config.cfg` 에 `network: {config: disabled}` 를 넣어 재생성을 껐다. 백업은 `/root/netplan-backup-20260807.yaml`.

절차: **① 백업 → ② 수정 → ③ `netplan generate` 로 문법 검증(적용 전!) → ④ `setsid nohup` 으로 `netplan apply` 분리 실행**(ssh 가 끊겨도 적용이 끝나도록). 기존 SSID 를 남겨두면 새 설정이 실패해도 원래 망으로 돌아와 잠금되지 않는다.

관련: [[patrol-progress-nav2]], [[csi-camera-stream-tuning]], [[pi4-upgrade-plan]], [[pi4-sd-install-record]], [[nav2-tuning-for-tiny-room]]
