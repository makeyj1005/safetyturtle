#!/usr/bin/env python3
"""qr_aim.py — 후면 카메라에 QR 이 잡히는지 실시간으로 알려주는 조준 도우미.

[왜 필요한가]
extinguisher_inspect_node 는 QR 을 못 찾아도 아무 말을 하지 않는다(조용히
다음 프레임으로 넘어간다). 그래서 "반응이 없다" 가 카메라를 못 보는 건지,
QR 이 화면 밖인지, 너무 작은지, 흐린지 구분이 안 된다.
이 도구는 매번 판독을 시도하고 결과를 그대로 찍어준다. QR 을 들고 움직이면서
언제 잡히는지 눈으로 확인할 수 있다.

[쓰는 법]
  python3 tools/qr_aim.py            # 40초 동안 확인
  python3 tools/qr_aim.py 90         # 90초
  python3 tools/qr_aim.py 40 front   # 전면(웹캠)으로

판독은 zbarimg 를 부르므로 patrol-ros2:humble 이미지 안에서 실행해야 한다
(cv2 는 QUIRC 없이 빌드돼 있어 QR 을 못 읽는다).
"""
import os
import subprocess
import sys
import tempfile
import time
import urllib.request

SECS = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0
WHICH = sys.argv[2] if len(sys.argv) > 2 else "rear"
URL = ("http://localhost:8080/snapshot_rear.jpg" if WHICH == "rear"
       else "http://localhost:8080/snapshot.jpg")

print(f"{WHICH} 카메라에서 QR 을 찾는다 ({SECS:.0f}초). QR 을 움직여 보세요.\n")

t0 = time.time()
hits = 0
tries = 0
last = None

while time.time() - t0 < SECS:
    tries += 1
    try:
        with urllib.request.urlopen(URL, timeout=5) as r:
            jpg = r.read()
    except Exception as e:
        print(f"  화면을 못 받았다: {e}")
        time.sleep(1.0)
        continue

    with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as tf:
        tf.write(jpg)
        path = tf.name
    try:
        # --quiet 는 요약을 끄고, --raw 는 내용만 낸다.
        # zbarimg 는 dbus 가 없으면 경고를 내는데 판독 자체에는 지장이 없다.
        out = subprocess.run(["zbarimg", "--quiet", "--raw", path],
                             capture_output=True, timeout=8)
        text = out.stdout.decode("utf-8", "ignore").strip()
    except Exception as e:
        text = ""
        print(f"  판독 실패: {e}")
    finally:
        os.unlink(path)

    el = time.time() - t0
    if text:
        hits += 1
        mark = "" if text == last else "  ← 새로 잡힘"
        print(f"[{el:5.1f}초] ✅ 잡힘: {text!r}{mark}")
        last = text
    else:
        print(f"[{el:5.1f}초] ·  없음  ({len(jpg)//1024}KB 화면)")
    time.sleep(1.0)

print(f"\n결과: {tries}번 중 {hits}번 잡힘")
if hits == 0:
    print("""
한 번도 못 잡았다면 순서대로 확인할 것:
  1) QR 이 화면 안에 들어와 있는가 (후면 카메라는 차체 아래쪽을 본다)
  2) 화면에서 QR 이 충분히 큰가 — 한 변이 화면 폭의 1/5 은 되어야 안정적이다
  3) 초점이 맞는가 — ov5647 는 고정초점이라 20cm 보다 가까우면 흐려진다
  4) 화면에 띄운 QR 이라면 반사광이 없는가 (종이에 인쇄한 쪽이 훨씬 잘 잡힌다)
""")
