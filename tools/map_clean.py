#!/usr/bin/env python3
"""map_clean.py — 지도의 바깥 벽만 남기고 안쪽을 깨끗한 빈 공간으로 만든다.

[언제 쓰나]
방 안이 실제로는 비어 있는데, 라이다 그늘(책상다리 뒤 등)과 잡음 때문에
지도 안쪽이 회색 얼룩과 점으로 지저분할 때 쓴다.

[안전에 대해]
정적 지도에서 장애물을 지워도 로봇이 물건에 부딪히지는 않는다. Nav2 는
주행 중 라이다로 지역 코스트맵(local costmap)을 실시간으로 만들어 눈앞의
장애물을 피한다. 정적 지도는 "어디가 방인가" 를 알려주는 용도다.
**단, 실제로 안이 비어 있을 때만 쓸 것.** 고정 장애물이 있는데 지우면
전역 경로가 그 장애물을 통과하도록 잡혀 매번 돌아가느라 헤맨다.

[쓰는 법]
  python3 tools/map_clean.py maps/venue5_map.pgm maps/venue5_clean.pgm
  (yaml 은 원본 것을 복사해 쓰면 된다 — 해상도·원점이 그대로다)
"""
import argparse

import cv2
import numpy as np

FREE, UNKNOWN, WALL = 254, 205, 0


def clean(img, keep_inside=False):
    """(정리된 지도, 통계) 를 돌려준다."""
    wall = (img <= 50).astype(np.uint8)

    # 벽에 난 1~2px 틈을 메운다. 안 메우면 아래 flood fill 이 그 틈으로
    # 새어 나가서 "안쪽" 을 못 찾는다.
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    sealed = cv2.morphologyEx(wall, cv2.MORPH_CLOSE, k)

    # 바깥에서 물을 부어 닿는 곳을 표시한다. 닿지 않은 빈칸이 곧 방 안이다.
    free = (sealed == 0).astype(np.uint8)
    h, w = free.shape
    ff = free.copy()
    mask = np.zeros((h + 2, w + 2), np.uint8)
    # 네 귀퉁이에서 모두 부어야 한다 — 지도가 기울어져 있으면 한 귀퉁이가
    # 벽 안쪽에 들어가 있을 수 있다.
    for seed in ((0, 0), (w - 1, 0), (0, h - 1), (w - 1, h - 1)):
        if ff[seed[1], seed[0]] == 1:
            cv2.floodFill(ff, mask, seed, 2)
    outside = (ff == 2)
    inside = (free == 1) & (~outside)

    # 안쪽 덩어리가 여러 개면 가장 큰 것만 방으로 본다(작은 것은 잡음).
    n, lab, stats, _ = cv2.connectedComponentsWithStats(
        inside.astype(np.uint8), connectivity=4)
    if n <= 1:
        raise SystemExit("방 안쪽을 찾지 못했다 — 벽이 닫혀 있지 않은 것 같다")
    biggest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    room = (lab == biggest)

    # 방을 **감싼** 벽만 남긴다.
    # 처음엔 "방에서 몇 칸 부풀린 자리의 벽" 으로 잡았는데, 그러면 방 한가운데
    # 떠 있는 잡음 덩어리도 방에 닿아 있으니 같이 살아남았다.
    # 외벽의 진짜 특징은 **바깥과 이어져 있다**는 것이다. 그래서 벽 덩어리를
    # 하나씩 보고, 바깥에 닿은 덩어리만 남긴다. 안에 떠 있는 조각은 버려진다.
    outside_grown = cv2.dilate(outside.astype(np.uint8),
                               cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
    nw, wlab = cv2.connectedComponents(sealed, connectivity=8)
    keep_wall = np.zeros_like(sealed, bool)
    for wi in range(1, nw):
        blob = (wlab == wi)
        if (blob & (outside_grown == 1)).any():
            keep_wall |= blob

    if keep_inside:
        # 방 테두리 안에 있는 벽은 전부 남긴다 — 소화기 같은 실제 장애물이다.
        ys, xs = np.where(room)
        y0, y1, x0, x1 = ys.min(), ys.max(), xs.min(), xs.max()
        box = np.zeros_like(room)
        box[y0:y1 + 1, x0:x1 + 1] = True
        keep_wall = keep_wall | (box & (sealed == 1))

    out = np.full_like(img, UNKNOWN)
    out[room] = FREE
    out[keep_wall] = WALL

    stat = {
        "before_free": float((img >= 230).mean() * 100),
        "before_wall": float((img <= 50).mean() * 100),
        "after_free": float((out == FREE).mean() * 100),
        "after_wall": float((out == WALL).mean() * 100),
        "room_px": int(room.sum()),
    }
    return out, stat


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("dst")
    ap.add_argument("--keep-inside", action="store_true",
                    help="방 안의 장애물(소화기 등)도 남긴다. "
                         "이걸 안 주면 안쪽 벽 조각을 잡음으로 보고 지운다 — "
                         "실제 장애물이 있는데 지우면 전역 경로가 그것을 "
                         "통과하도록 잡혀 좁은 방에서 길을 못 찾는다(2026-09-06 실측).")
    a = ap.parse_args()

    img = cv2.imread(a.src, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise SystemExit(f"지도를 못 읽었다: {a.src}")
    out, st = clean(img, keep_inside=a.keep_inside)
    cv2.imwrite(a.dst, out)

    res = 0.05
    print(f"정리 전: 빈공간 {st['before_free']:.1f}%  벽 {st['before_wall']:.1f}%")
    print(f"정리 후: 빈공간 {st['after_free']:.1f}%  벽 {st['after_wall']:.1f}%")
    print(f"방 넓이: {st['room_px'] * res * res:.1f}m^2")
    print(f"저장: {a.dst}")


if __name__ == "__main__":
    main()
