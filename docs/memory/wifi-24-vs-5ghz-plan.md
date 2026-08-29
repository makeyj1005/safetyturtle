---
name: wifi-24-vs-5ghz-plan
description: 2.4GHz/5GHz 둘 다 준비돼 있고 전환은 1분이다 — 안전모까지 구현한 뒤 실제 부하로 비교하기로 미뤘다. 전환 절차와 지금까지 측정값
metadata: 
  node_type: memory
  type: project
  originSessionId: 2555f1fc-1d69-43e9-adbc-c328f1a22c22
  modified: 2026-08-07T14:30:36.871Z
---

**2026-08-07.** 로봇 Wi-Fi 를 2.4GHz(`team1`)와 5GHz(`team1_5g`) 양쪽에서 쓸 수 있게 만들어놓고, **어느 쪽이 나은지 판정은 미뤘다.** 이유: 지금은 무선에 흐르는 데이터가 사실상 없어서(ping 뿐) 차이가 나타나지 않는다. **안전모 인식까지 구현되면 카메라 영상·판정 이미지·대시보드가 상시 흐르므로 그때 비교해야 유의미하다.** 사용자 판단이고 맞는 판단이다.

**현재 상태: 2.4GHz 고정** (`team1` 만 netplan 에 등록). 기준선을 먼저 확보하려는 것이다.

## 전환 절차 (1분, 검증됨)

로봇에서:
```
# 5GHz 우선 + 2.4GHz 예비 (둘 다 등록)
sudo cp /root/netplan-both.yaml /etc/netplan/50-cloud-init.yaml
sudo chmod 600 /etc/netplan/50-cloud-init.yaml
sudo netplan generate          # 반드시 적용 전에 문법 검증
sudo setsid nohup bash -c "sleep 2; netplan apply; sleep 4; systemctl restart netplan-wpa-wlan0.service" >/dev/null 2>&1 &
```
2.4GHz 로 되돌릴 때는 `access-points` 에서 `team1_5g` 를 지우면 된다. **`netplan apply` 만으로는 이미 맺어진 연결이 유지되므로 `netplan-wpa-wlan0.service` 재시작으로 재선택을 강제해야 한다.**

**위험 관리:** 새 설정으로 바꿀 때는 **자동 복구 감시**를 먼저 걸었다 — `setsid nohup` 으로 "N초 안에 `/tmp/keep5g` 가 없으면 백업본으로 되돌리고 `netplan apply`" 하는 백그라운드 작업. 실제로 한 번 작동해서 복구됐다(5GHz SSID 가 아직 전파되지 않았을 때). **접속을 잃을 수 있는 변경에는 이 패턴을 쓸 것.**

## ★ 중요: wpa_supplicant 는 5GHz 를 우선한다

**둘 다 등록하면 2.4GHz(-48dBm)가 더 센데도 5GHz(-60dBm)를 골랐다.** wpa_supplicant 는 5GHz 신호가 쓸 만하면(대략 -70dBm 이상) 가산점을 줘 우선 선택한다. netplan 이 `priority` 를 노출하지 않는 게 문제되지 않는다는 뜻이다.

**즉 "둘 다 등록" = 평소 5GHz + 5GHz 없으면 2.4GHz 자동 복귀** 로, 트레이드오프 없이 양쪽을 얻는 구성이다. 어시스턴트가 처음에 "둘 다 등록하면 신호 센 2.4 를 고른다"고 잘못 말했다가 실측으로 정정했다.

## 측정값 (2026-08-07, 부하 없는 상태 = ping 만)

모두 **Wi-Fi 절전 모드를 끈 뒤**의 값이다([[wireless-is-the-bottleneck]] 참고 — 절전이 진짜 문제였다).

| 구성 | 평균 | 최대 | 지터 | 손실 | bitrate |
|---|---|---|---|---|---|
| 2.4GHz (양쪽 2.4) | 5.80 / 7.05ms | 18.3 / 25.5ms | 3.24ms | 0% | 72.2 Mbit/s |
| 5GHz (VM 은 2.4 — **경로 혼합**) | 12.91ms | **194.4ms** | 27.47ms | 0% | 390 Mbit/s |
| 5GHz (양쪽 5GHz) | 8.80 / 7.31ms | 17.1 / 15.3ms | 3.52ms | 0% | 325 Mbit/s |
| 5GHz (양쪽, 재연결 후) | **4.32ms** | 17.9ms | **2.41ms** | 0% | 325 Mbit/s |

**핵심: 부하가 없으면 두 대역이 사실상 동등하다.** 그리고 **VM 과 로봇이 다른 대역에 있으면 안 된다** — 경로가 대역을 건너가면서 194ms 스파이크가 났고, 양쪽을 맞추자 사라졌다. **측정할 때 노트북 대역을 꼭 맞출 것.**

## 나중에 비교할 때 챙길 것

1. **실제 부하를 걸고 측정한다** — 카메라 스트림(약 2.2Mbps) + 안전모 판정 이미지 + 대시보드가 돌아가는 상태. ping 만으로는 차이가 안 난다
2. **노트북과 로봇을 같은 대역에 맞춘다**
3. 볼 지표: 평균보다 **지터와 50ms/100ms 초과 구멍 횟수**. Nav2 시절 문제를 만든 건 평균이 아니라 구멍이었다
4. **혼잡도 같이 기록** — `sudo iw dev wlan0 scan` 으로 주변 AP 수. 2026-08-07 교실 실측: 2.4GHz 에 AP 6개(특히 `HIMADE` 2442MHz 가 우리 2437MHz 와 5MHz 차이로 겹침), 5GHz 는 한산. **혼잡할 때가 5GHz 의 진짜 강점이 드러나는 조건이다**
5. 대역폭은 병목이 아니다 — 우리가 보내는 양이 약 3Mbps 이고 2.4GHz 실효 35Mbps 로도 12배 여유다

## 5GHz 채널 주의

현재 `team1_5g` 는 **ch36(5180MHz)** 이고 DFS 가 아니라 안전하다. 다만 채널이 "자동" 설정이라 **재부팅 시 DFS 채널(52~144)로 옮겨갈 수 있다** — 그 대역은 레이더 감지 시 AP 가 채널을 비워서 로봇이 갑자기 끊긴다. **공유기에서 채널을 36·40·44·48 중 하나로 고정하는 게 좋다.** ch36 은 `찌니쫑5G`·숨김 SSID 와 겹치므로 **채널폭 20MHz + ch44** 가 가장 깨끗하다(80MHz 는 ch36~48 을 한 덩어리로 쓰므로 `YNC_PUBLIC` ch48 까지 겹친다).

## netplan 편집 주의

`/etc/netplan/50-cloud-init.yaml` 은 cloud-init 생성물이다. `/etc/cloud/cloud.cfg.d/99-disable-network-config.cfg` 에 `network: {config: disabled}` 를 넣어 재생성을 껐으므로 이제 직접 수정이 유지된다. 백업: `/root/netplan-backup-20260807.yaml`(team1 만), `/root/netplan-both.yaml`(둘 다).

관련: [[wireless-is-the-bottleneck]], [[pi4-sd-install-record]], [[csi-camera-stream-tuning]], [[robot-host-identity]]
