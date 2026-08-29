#!/usr/bin/env python3
"""helmet_calib.py — 실제 안전모를 찍어 '이게 안전모다' 라고 기준을 등록한다.

[VM 에서 실행]
  웹캠으로 바로 찍어 등록 (로봇에서 webcam_node 가 떠 있어야 한다):
    python3 ~/vibe/ex1/tools/helmet_calib.py --grab --name 노란안전모 --select

  이미 찍어둔 사진으로 등록:
    python3 ~/vibe/ex1/tools/helmet_calib.py --image <사진.jpg> --name 노란안전모 --select

  화면 없이 영역을 숫자로 줄 때:
    python3 ~/vibe/ex1/tools/helmet_calib.py --image <사진.jpg> --name 노란안전모 \
        --roi 250,60,140,90

  등록된 기준 보기 / 지우기:
    python3 ~/vibe/ex1/tools/helmet_calib.py --list
    python3 ~/vibe/ex1/tools/helmet_calib.py --remove 노란안전모

[무엇을 저장하나]  maps/helmet_calib.yaml
  hsv          안전모 색 범위 [H하한,S하한,V하한, H상한,S상한,V상한] (여러 개 가능)
  achromatic   흰색·회색 안전모인가 (색상 H 로는 구분이 안 돼 채도·명도로만 본다)
  cover        기준 영역에서 이 범위가 덮은 비율 — 1.0 에 가까울수록 잘 잡은 것
  ref_image    근거 사진. 나중에 왜 이 값인지 되짚을 수 있어야 한다

[왜 코드에 색을 박지 않고 파일로 두나]
안전모 색은 현장마다 다르고, 같은 안전모도 조명에 따라 HSV 값이 꽤 움직인다.
코드에 박아두면 안전모를 바꿀 때마다 코드를 고쳐야 한다. 압력계 기준을
gauge_calib.yaml 로 뺀 것과 같은 이유다 — **코드는 그대로 두고 기준만 다시 잡는다.**
helmet_node 는 이 파일이 있으면 여기 값을 쓰고, 없으면 내장 일반값(흰/노랑/파랑/빨강)
으로 돌아간다. 그래서 보정 전에도 노드는 동작한다.

[⚠️ 무엇을 찍어야 하나]
**실제 순찰에서 보게 될 거리·조명에서** 사람이 안전모를 쓴 사진을 찍는다. 안전모만
따로 책상 위에서 찍으면 조명이 달라 현장에서 안 맞는다. 선택 영역은 안전모 **껍데기만**
잡는다 — 챙 아래 그림자나 얼굴이 들어가면 범위가 넓어져 머리카락까지 안전모로 본다.
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np
import yaml

EX1 = os.path.join(os.path.expanduser("~"), "vibe", "ex1")
CALIB = os.path.join(EX1, "maps", "helmet_calib.yaml")
DEFAULT_TOPIC = "/webcam/image_raw/compressed"

# 채도가 이보다 낮으면 색상(H)이 의미 없다 = 흰색·회색 안전모로 본다.
ACHROMATIC_S = 60
# 범위를 잡을 백분위. 양끝 10%를 버려 반사광·그림자에 범위가 끌려가지 않게 한다.
LO_PCT, HI_PCT = 10, 90
# 백분위로 좁힌 범위에 주는 여유. 조명이 조금 변해도 견디게 한다.
PAD_H, PAD_S, PAD_V = 6, 40, 45

# 무채색(흰·회색) 안전모는 여유를 훨씬 좁게 준다 — 2026-08-02 실측 근거.
# 흰 안전모는 색상(H)으로 구를 수 없어 "밝다"는 것만으로 판단하는데, 실내에는
# 밝은 것이 많다. 실측값(V 백분위): 안전모 198~235 / 천장 107~169 / 크림벽 96~127 /
# 머리카락 66~163 / 형광등 248~254. 유채색과 같은 여유(±45)를 주면 하한이 153 까지
# 내려가 천장이 들어오고, 상한은 255 라 형광등까지 삼킨다(실제로 흰 천장을 흰
# 안전모로 보고 미착용자를 통과시켰다). 그래서 아래를 쓴다:
ACHRO_PAD_V_LO = 12      # 하한 여유 — 천장과 벌어지도록 좁게
ACHRO_PAD_V_HI = 12      # 상한 여유
ACHRO_V_CAP = 245        # 상한 한계. 형광등·창 직사광(248~254)을 잘라낸다
# 채도 상한 한계 45 — 실측으로 나온 분리선이다(2026-08-02, 실제 운용 구도:
# 카메라를 바닥에 두고 서 있는 사람을 올려다봄). 흰 안전모는 S 16~45(중앙 28)인데
# 실내에서 안전모가 아닌 것은 전부 그 위였다: 머리 위 천장 48~61, 셔츠 47~76,
# 얼굴 86~126, 나무 몰딩 131~149. 이 값으로 머리 위 천장이 0.00 만 잡힌다
# (60 으로 두면 천장이 들어와 미착용자가 착용으로 통과한다).
ACHRO_S_CAP = 45


def grab_frame(topic, timeout):
    """로봇 webcam_node 가 보내는 프레임 한 장을 받아 온다.

    게이지 때와 달리 여기서는 스트림을 그대로 받아도 된다 — 640x480 jpeg50 은
    프레임이 20KB 라 무선으로도 유실 없이 온다(고해상도 177KB 가 문제였다).
    """
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage

    rclpy.init()
    node = Node("helmet_calib_grab")
    got = []
    node.create_subscription(
        CompressedImage, topic,
        lambda m: got.append(bytes(m.data)) if len(got) < 1 else None,
        qos_profile_sensor_data,
    )
    print(f"{topic} 에서 한 장 기다리는 중... (로봇 webcam_node 확인)")
    t0 = time.time()
    while not got and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    if not got:
        print(f"{timeout:.0f}초 안에 프레임이 오지 않았다 — 로봇에서 webcam_node 가 "
              "떠 있는지, ROS_DOMAIN_ID 가 3 인지 확인할 것", file=sys.stderr)
        return None
    img = cv2.imdecode(np.frombuffer(got[0], np.uint8), cv2.IMREAD_COLOR)
    out = os.path.join(EX1, "logs", "helmet_calib")
    os.makedirs(out, exist_ok=True)
    path = os.path.join(out, f"calib_{time.strftime('%m%d_%H%M%S')}.jpg")
    cv2.imwrite(path, img)
    print(f"저장: {path}")
    return path


def pick_roi(img):
    """드래그로 안전모 영역을 고른다. 창이 안 열리면 --roi 를 쓰라고 알린다."""
    try:
        r = cv2.selectROI("안전모 영역을 드래그 (Enter 확정, c 취소)", img,
                          showCrosshair=False)
        cv2.destroyAllWindows()
    except cv2.error as e:
        print(f"창을 열 수 없다({e}). --roi x,y,w,h 로 직접 지정할 것", file=sys.stderr)
        return None
    if r[2] < 3 or r[3] < 3:
        print("영역이 너무 작다", file=sys.stderr)
        return None
    return tuple(int(v) for v in r)


def ranges_from_patch(patch):
    """선택 영역의 HSV 분포에서 색 범위를 만든다. (범위목록, 무채색여부)"""
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[:, :, 0].ravel(), hsv[:, :, 1].ravel(), hsv[:, :, 2].ravel()

    s_lo = max(int(np.percentile(s, LO_PCT)) - PAD_S, 0)
    s_hi = min(int(np.percentile(s, HI_PCT)) + PAD_S, 255)
    v_lo = max(int(np.percentile(v, LO_PCT)) - PAD_V, 0)
    v_hi = min(int(np.percentile(v, HI_PCT)) + PAD_V, 255)

    if float(np.median(s)) < ACHROMATIC_S:
        # 흰색·회색 안전모. H 는 잡음이라 버리고 "채도 낮고 밝다" 로만 잡는다.
        # 여유는 위 상수 설명대로 좁게 준다 — 넓히면 천장·형광등이 들어온다.
        a_lo = max(int(np.percentile(v, LO_PCT)) - ACHRO_PAD_V_LO, 0)
        a_hi = min(int(np.percentile(v, HI_PCT)) + ACHRO_PAD_V_HI, ACHRO_V_CAP)
        return [[0, 0, a_lo, 179, min(s_hi, ACHRO_S_CAP), a_hi]], True

    # 빨강은 H 가 0 과 179 양끝에 걸쳐 분포가 두 덩어리로 갈린다. 그대로 백분위를
    # 내면 0~179 전체가 잡히므로, 그럴 때는 H 를 90 만큼 돌려서 재고 되돌린다.
    if np.percentile(h, LO_PCT) < 15 and np.percentile(h, HI_PCT) > 165:
        hs = (h.astype(np.int16) + 90) % 180
        lo = int(np.percentile(hs, LO_PCT)) - PAD_H
        hi = int(np.percentile(hs, HI_PCT)) + PAD_H
        lo, hi = (lo - 90) % 180, (hi - 90) % 180
        # 되돌리면 lo > hi 가 된다 = 0 을 넘어가는 범위. 두 구간으로 쪼갠다.
        return [[lo, s_lo, v_lo, 179, s_hi, v_hi],
                [0, s_lo, v_lo, hi, s_hi, v_hi]], False

    h_lo = max(int(np.percentile(h, LO_PCT)) - PAD_H, 0)
    h_hi = min(int(np.percentile(h, HI_PCT)) + PAD_H, 179)
    return [[h_lo, s_lo, v_lo, h_hi, s_hi, v_hi]], False


def cover_ratio(patch, ranges):
    """만든 범위가 선택 영역을 얼마나 덮는지. 낮으면 영역을 잘못 잡은 것이다."""
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for r in ranges:
        mask |= cv2.inRange(hsv, np.array(r[:3], np.uint8), np.array(r[3:], np.uint8))
    return float(np.count_nonzero(mask)) / mask.size


def load():
    if not os.path.exists(CALIB):
        return {"helmets": {}}
    with open(CALIB) as f:
        return yaml.safe_load(f) or {"helmets": {}}


def save(data):
    os.makedirs(os.path.dirname(CALIB), exist_ok=True)
    with open(CALIB, "w") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
    print(f"기록: {CALIB}")


def show_list():
    data = load()
    hs = data.get("helmets") or {}
    if not hs:
        print(f"등록된 안전모 기준이 없다 ({CALIB})")
        return 0
    print(f"{CALIB}")
    for name, e in hs.items():
        kind = "무채색(흰/회색)" if e.get("achromatic") else "유채색"
        print(f"  {name}: {kind}, 덮은비율 {e.get('cover', 0):.2f}, "
              f"근거 {os.path.basename(str(e.get('ref_image', '')))}")
        for r in e.get("hsv", []):
            print(f"      H {r[0]}~{r[3]}  S {r[1]}~{r[4]}  V {r[2]}~{r[5]}")
    return 0


def main():
    p = argparse.ArgumentParser(description="안전모 색 기준 등록")
    p.add_argument("--image", help="쓸 사진 경로")
    p.add_argument("--grab", action="store_true", help="웹캠에서 한 장 받아 쓴다")
    p.add_argument("--topic", default=DEFAULT_TOPIC)
    p.add_argument("--wait", type=float, default=20.0, help="--grab 대기 시간(초)")
    p.add_argument("--name", default="기본", help="등록 이름")
    p.add_argument("--select", action="store_true", help="드래그로 영역 선택")
    p.add_argument("--roi", help="영역을 숫자로: x,y,w,h")
    p.add_argument("--list", action="store_true", help="등록된 기준 보기")
    p.add_argument("--remove", help="이름을 지운다")
    a = p.parse_args()

    if a.list:
        return show_list()
    if a.remove:
        data = load()
        if a.remove not in (data.get("helmets") or {}):
            print(f"'{a.remove}' 은(는) 등록돼 있지 않다", file=sys.stderr)
            return 1
        del data["helmets"][a.remove]
        save(data)
        return 0

    path = grab_frame(a.topic, a.wait) if a.grab else a.image
    if not path:
        return 1
    if not os.path.exists(path):
        print(f"사진이 없다: {path}", file=sys.stderr)
        return 1
    img = cv2.imread(path)
    if img is None:
        print(f"사진을 읽지 못했다: {path}", file=sys.stderr)
        return 1

    if a.roi:
        roi = tuple(int(v) for v in a.roi.split(","))
        if len(roi) != 4:
            print("--roi 는 x,y,w,h 네 개다", file=sys.stderr)
            return 1
    elif a.select:
        roi = pick_roi(img)
        if roi is None:
            return 1
    else:
        print("--select 또는 --roi 로 안전모 영역을 지정할 것 "
              "(자동으로 찾지 않는다 — 옷·벽을 안전모로 잡으면 판정이 통째로 틀어진다)",
              file=sys.stderr)
        return 1

    x, y, w, h = roi
    patch = img[y:y + h, x:x + w]
    if patch.size == 0:
        print(f"영역이 사진 밖이다: {roi} (사진 {img.shape[1]}x{img.shape[0]})",
              file=sys.stderr)
        return 1

    ranges, achro = ranges_from_patch(patch)
    cover = cover_ratio(patch, ranges)

    data = load()
    data.setdefault("helmets", {})[a.name] = {
        "hsv": [[int(v) for v in r] for r in ranges],
        "achromatic": bool(achro),
        "cover": round(cover, 3),
        "roi": [int(v) for v in roi],
        "resolution": [int(img.shape[1]), int(img.shape[0])],
        "ref_image": os.path.abspath(path),
    }
    save(data)

    print(f"\n[{a.name}] {'무채색(흰/회색)' if achro else '유채색'} 안전모")
    for r in ranges:
        print(f"  H {r[0]}~{r[3]}  S {r[1]}~{r[4]}  V {r[2]}~{r[5]}")
    print(f"  선택 영역의 {cover * 100:.0f}% 를 덮는다")
    if cover < 0.8:
        print("  ⚠️ 80% 미만이다 — 영역에 안전모가 아닌 것(얼굴·그림자·배경)이 "
              "섞였을 수 있다. 껍데기만 다시 잡아볼 것")

    # 같은 사진 전체에서 이 범위가 얼마나 잡히는지도 본다. 배경까지 넓게 잡히면
    # 사람이 없어도 '안전모 있음' 이 되므로 여기서 걸러야 한다.
    whole = cover_ratio(img, ranges)
    print(f"  사진 전체로는 {whole * 100:.0f}% 가 이 색이다"
          + ("  ⚠️ 배경에도 같은 색이 많다 — 오탐이 잦을 수 있다" if whole > 0.25 else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
