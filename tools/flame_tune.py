#!/usr/bin/env python3
"""flame_tune.py — 불꽃센서 감도(가변저항)를 맞출 때 쓰는 도구. 로봇에서 실행한다.

원시 GPIO 값을 계속 화면에 그린다. 가변저항을 돌리면서 이 화면을 보면
"몇 도 돌렸을 때 반응이 시작되는지"를 눈으로 알 수 있다.

/flame/detected 토픽은 **값이 바뀔 때만** 발행하므로 감도를 맞추는 데는
쓸 수 없다 — 변화가 없으면 아무것도 안 나와서, 센서가 죽은 건지 조용한
건지 구분이 안 된다. 그래서 원시값을 따로 본다.

[쓰는 법]
  1) 라이터 불을 센서 앞 30cm 에 **계속 대고 있는다**
  2) 이 화면을 보면서 가변저항을 천천히 돌린다
  3) ●●●●● (감지) 로 바뀌는 지점을 찾는다
  4) 그 지점에서 4분의 1 바퀴만 더 돌린다 — 딱 경계에 두면
     조금만 멀어져도 놓친다
  5) 불을 치운다. ····· (없음) 으로 돌아가야 한다.
     계속 ●●●●● 이면 너무 많이 돌린 것이다(햇빛·조명에도 반응한다).

  중단: Ctrl-C
"""
import sys
import time

import RPi.GPIO as GPIO

PIN_FLAME = 23
HZ = 10

GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN_FLAME, GPIO.IN)

print(__doc__)
print(f"GPIO{PIN_FLAME} 감시 시작 ({HZ}Hz). HIGH=감지 로 가정한다.\n")

# 최근 값들을 모아 "몇 % 시간 동안 감지됐는지"를 같이 보여준다.
# 순간값만 보면 깜빡이는 걸 눈으로 못 따라간다.
window = []
WINDOW_N = HZ * 3          # 최근 3초

try:
    while True:
        v = GPIO.input(PIN_FLAME) == GPIO.HIGH
        window.append(v)
        if len(window) > WINDOW_N:
            window.pop(0)
        pct = 100.0 * sum(window) / len(window)

        bar = "●●●●● 감지  " if v else "····· 없음  "
        # 최근 3초 중 감지된 비율을 막대로. 경계에서 깜빡이면 여기가 중간값이 된다.
        n = int(pct / 5)
        gauge = "#" * n + "-" * (20 - n)
        sys.stdout.write(f"\r{bar}[{gauge}] 최근3초 {pct:5.1f}%   ")
        sys.stdout.flush()
        time.sleep(1.0 / HZ)
except KeyboardInterrupt:
    print("\n중단")
finally:
    GPIO.cleanup(PIN_FLAME)
