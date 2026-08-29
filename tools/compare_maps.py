#!/usr/bin/env python3
"""두 지도(.pgm)를 수치로 비교한다. 다시 매핑한 게 나아졌는지 눈대중 말고 숫자로 본다.

  python3 ~/vibe/ex1/tools/compare_maps.py 예전.pgm 새것.pgm

보는 것
  빈공간/장애물/미탐사 칸 수  — 넓어졌나, 벽이 더 잡혔나
  30cm 여유 영역             — 로봇(반경 10cm)이 실제로 지나갈 수 있는 면적.
                               순찰 사각형을 넓힐 수 있는지가 여기서 결정된다
  벽 두께                    — 루프 클로저가 잘 맞으면 벽선이 얇아진다.
                               어긋나면 같은 벽이 두 겹으로 그려져 두꺼워진다
"""
import sys

import numpy as np
from scipy.ndimage import distance_transform_edt

RES = 0.05          # m/px (지도 yaml 의 resolution)
CLEAR = 0.30        # 확보하려는 벽 여유(m)


def read_pgm(path):
    with open(path, "rb") as f:
        data = f.read()
    # P5 헤더: 매직, 폭, 높이, 최대값 (주석 줄은 #)
    fields, i = [], 2
    while len(fields) < 3:
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b"#":
            while data[i:i + 1] != b"\n":
                i += 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        fields.append(int(data[i:j]))
        i = j
    w, h, _ = fields
    return np.frombuffer(data[i + 1:i + 1 + w * h], dtype=np.uint8).reshape(h, w)


def stats(img):
    free = img >= 254            # 흰색 = 빈공간
    occ = img <= 1               # 검정 = 장애물
    unknown = ~free & ~occ       # 회색 = 미탐사
    # 빈공간에서 "장애물·미탐사가 아닌 곳"까지의 거리. 로봇이 갈 수 있는 여유를 본다.
    dist = distance_transform_edt(free) * RES
    clear = (dist >= CLEAR) & free
    cell = RES * RES
    return {
        "빈공간": free.sum() * cell,
        "장애물": occ.sum() * cell,
        "미탐사": unknown.sum() * cell,
        f"{int(CLEAR * 100)}cm 여유": clear.sum() * cell,
        "장애물 칸수": int(occ.sum()),
    }


def main():
    a, b = sys.argv[1], sys.argv[2]
    ia, ib = read_pgm(a), read_pgm(b)
    if ia.shape != ib.shape:
        print(f"크기가 다르다: {ia.shape} vs {ib.shape} — 원점이 같은지 확인할 것")
    sa, sb = stats(ia), stats(ib)

    print(f"{'항목':<14}{'예전':>10}{'새것':>10}{'차이':>10}")
    for k in sa:
        unit = "" if "칸수" in k else " m²"
        d = sb[k] - sa[k]
        fmt = (lambda v: f"{v:.0f}") if "칸수" in k else (lambda v: f"{v:.2f}")
        print(f"{k:<14}{fmt(sa[k]):>10}{fmt(sb[k]):>10}{('+' if d >= 0 else '') + fmt(d):>10}{unit}")

    # 두 지도에서 상태가 바뀐 칸 (벽이 새로 잡혔거나 지워진 곳)
    changed = (ia != ib).sum()
    print(f"\n값이 바뀐 칸: {changed}개 ({changed / ia.size * 100:.1f}%)")
    newly_occ = ((ia > 1) & (ib <= 1)).sum()
    gone_occ = ((ia <= 1) & (ib > 1)).sum()
    newly_free = ((ia < 254) & (ib >= 254)).sum()
    print(f"  새로 장애물이 된 칸: {newly_occ}   장애물에서 빠진 칸: {gone_occ}")
    print(f"  새로 빈공간이 된 칸: {newly_free}")


if __name__ == "__main__":
    main()
