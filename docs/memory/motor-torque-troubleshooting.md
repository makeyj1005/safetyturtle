---
name: motor-torque-troubleshooting
description: 명령은 나가는데 바퀴가 안 돌면 /sensor_state 의 torque 를 먼저 본다 — /motor_power 서비스로 켠다
metadata: 
  node_type: memory
  type: project
  originSessionId: 014807c6-8861-4a69-b2db-b81e5c632cb8
  modified: 2026-07-30T01:18:53.537Z
---

`/cmd_vel` 에 정상적인 명령이 나가는데도 로봇이 움직이지 않으면 **모터 토크가 꺼진 것**을 먼저 의심한다.

확인:
```
ros2 topic echo /sensor_state --once     # torque: false 면 이것이 원인
```

해결:
```
ros2 service call /motor_power std_srvs/srv/SetBool "{data: true}"
```
성공하면 `success=True, message='Succeeded to write data'` 가 나오고 `torque: true` 로 바뀐다. 안 되면 OpenCR 의 RESET 버튼 → bringup 재실행, 그다음 OpenCR 전원 재투입 순서로 시도한다.

**Why:** 2026-07-28 에 `turtlebot3_node` 가 `Failed transmit instruction packet` 후 `*** stack smashing detected ***` 로 강제 종료됐고(exit -6), 그 여파로 토크가 해제된 채 남았다. bringup 을 재시작해도 자동으로 켜지지 않았다. 이 상태에서는 노드가 전부 정상 동작하고 `/cmd_vel` 값도 정상인데 바퀴만 안 돌아서, 소프트웨어를 한참 잘못 의심했다.

**How to apply:** 로봇이 안 움직일 때 진단 순서 — ① `/cmd_vel` 에 값이 실제로 나가는지 ② `torque` 상태 ③ `/joint_states`·`/odom` 이 변하는지. `/odom` 이 e-11 수준이면 바퀴가 전혀 안 돈 것이다.

**배터리가 아예 없거나 모터 전원이 끊긴 경우 (2026-07-30 실측)**
bringup 로그가 이렇게 흐르면 모터 전원 문제다:
```
Succeeded to open the port(/dev/ttyACM0)!   ← 파이↔OpenCR USB 는 정상
Calibration End / Add Motors / Run!         ← 초기화도 정상
Failed to read[[TxRxResult] There is no status packet!]  ← 초당 25회 반복
*** stack smashing detected *** → exit code -6
```
**`There is no status packet`** = OpenCR↔Dynamixel 구간 무응답. OpenCR 은 USB 만으로도 논리회로가 켜져서 `/dev/ttyACM0` 이 보이고 포트도 열리지만, **모터는 배터리에서 오는 12V 가 따로 필요**하다. `stack smashing` 은 turtlebot3_node 가 읽기 실패를 처리하는 코드의 버그이고, 읽기 실패가 멈추면 같이 사라진다.
확인 순서: 배터리 연결 여부 → OpenCR 전원 스위치 → OpenCR 전원 LED → 모터 케이블(책상에 올렸다 내리며 빠질 수 있다). SMPS 어댑터로도 모터 전원이 공급된다.
이 상태에서는 Nav2 를 아무리 잘 설정해도 로봇이 움직이지 않으므로, 소프트웨어를 의심하기 전에 전원을 먼저 본다.

**주행 중 안전 경고:** `turtlebot3_node` 가 주행 중 죽으면 Dynamixel 이 **마지막 지령 속도를 유지**할 수 있다. 그때는 `Ctrl+C` 도 `/mux/enable false` 도 듣지 않는다(명령을 모터에 전달할 노드가 없으므로). **로봇을 들어올리거나 OpenCR 전원 스위치를 끄는 것만 유효**하다. 첫 주행 테스트는 항상 로봇을 집을 수 있는 자세에서 한다.

참고: `throttled` 는 라즈베리파이 5V 레일 기준이고 배터리 전압과 별개다. 배터리가 12.11V(89%)로 멀쩡한데도 `0x50000`(저전압·쓰로틀링 이력)이 떴다 — Pi 3 + CSI 카메라 + Wi-Fi 부하가 OpenCR 5V 레귤레이터에 버겁기 때문. 장시간 테스트 전에는 파이 전원을 보조 배터리로 분리하거나 SMPS 어댑터를 쓰는 것이 좋다.

관련: [[robot-undervoltage-warning]], [[nav2-not-line-following]]
