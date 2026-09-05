#!/usr/bin/env python3
"""gauge_find.py — 사진에서 동그란 압력계 후보를 찾아 번호를 붙여 보여준다.

[왜 필요한가]
gauge_calib.py --select 는 마우스로 영역을 끌어 지정하는데, 도커 안에서
창을 띄우려면 X 를 넘겨줘야 해서 번거롭다. 사람이 화면을 보고 "몇 번"
이라고만 말하면 되도록, 후보에 번호를 붙인 사진을 만든다.

[쓰는 법]
  python3 tools/gauge_find.py <사진> [--out 표시사진.jpg]
  -> 후보 목록(번호, 중심, 반지름)을 찍고 표시사진을 남긴다.
"""
import argparse
import json

import cv2
import numpy as np


def find_circles(img):
    """동그란 것들을 찾아 (x, y, r) 목록으로 준다.

    압력계는 소화기 몸통보다 훨씬 작다. 화면 폭의 2~15% 반지름만 본다 —
    이 범위를 안 두면 소화기 몸통 윤곽이나 바닥 무늬까지 잡힌다.
    """
    h, w = img.shape[:2]
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # 살짝 흐리게 해서 잡티 때문에 생기는 가짜 원을 줄인다
    g = cv2.medianBlur(g, 5)
    rmin = max(8, int(w * 0.02))
    rmax = max(rmin + 4, int(w * 0.15))
    circles = cv2.HoughCircles(
        g, cv2.HOUGH_GRADIENT, dp=1.2, minDist=int(w * 0.04),
        param1=110, param2=45, minRadius=rmin, maxRadius=rmax)
    if circles is None:
        return []
    out = [tuple(int(v) for v in c) for c in circles[0]]
    # 큰 것부터 — 압력계는 보통 화면에서 눈에 띄는 크기다
    out.sort(key=lambda c: -c[2])
    return out[:12]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("image")
    ap.add_argument("--out", default="gauge_candidates.jpg")
    a = ap.parse_args()

    img = cv2.imread(a.image)
    if img is None:
        raise SystemExit(f"사진을 못 읽었다: {a.image}")
    h, w = img.shape[:2]
    cs = find_circles(img)

    vis = img.copy()
    rows = []
    for i, (x, y, r) in enumerate(cs, 1):
        cv2.circle(vis, (x, y), r, (0, 0, 255), 3)
        cv2.circle(vis, (x, y), 2, (0, 0, 255), 3)
        # 번호는 원 위쪽에. 배경을 깔아야 밝은 곳에서도 보인다.
        tp = (x - 18, max(24, y - r - 8))
        cv2.rectangle(vis, (tp[0] - 4, tp[1] - 24), (tp[0] + 44, tp[1] + 6),
                      (255, 255, 255), -1)
        cv2.putText(vis, str(i), tp, cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                    (0, 0, 255), 3)
        rows.append({"no": i, "x": x, "y": y, "r": r})
        print(f"  {i:2d}번  중심=({x:4d},{y:4d})  반지름={r:3d}px")

    if not rows:
        print("  동그란 후보를 하나도 못 찾았다.")
        print("  압력계가 화면에 크게, 초점이 맞은 상태로 들어와야 한다.")

    cv2.imwrite(a.out, vis)
    print(f"\n해상도 {w}x{h}, 후보 {len(rows)}개 -> {a.out}")
    print(json.dumps(rows, ensure_ascii=False))


if __name__ == "__main__":
    main()
