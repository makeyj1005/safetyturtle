#!/usr/bin/env python3
"""gauge_calib.py — 소화기 압력계의 '정상' 기준을 등록한다.

[VM 에서 실행]
  자동 (사진에서 게이지를 찾아 기준으로 삼는다):
    python3 ~/vibe/ex1/tools/gauge_calib.py --image ~/vibe/ex1/logs/shots/shot_112513_f01.jpg \
        --name FE-01 --radius 40

  ROI 를 손으로 지정 (화면이 있을 때, 드래그로 선택):
    python3 ~/vibe/ex1/tools/gauge_calib.py --image <사진> --name FE-01 --select

  로봇에서 바로 찍어 등록:
    python3 ~/vibe/ex1/tools/gauge_calib.py --grab --name FE-01

[무엇을 저장하나]  maps/gauge_calib.yaml
  roi            게이지가 나타나는 화면 영역. 라벨의 동그란 로고와 구분하려면 필수다
  radius_px      게이지 반지름(px)
  normal_angle   지금 바늘 각도 = 정상 기준
  resolution     이 값들이 유효한 해상도 (다르면 판정이 거부된다)

[전제]
지금 보이는 게이지가 **정상 상태**여야 한다. 압력이 빠진 소화기로 캘리브레이션하면
그 상태가 '정상'으로 등록된다. 라벨의 초록 구간에 바늘이 있는지 눈으로 먼저 확인할 것.
"""
import argparse
import os
import subprocess
import sys

import cv2

sys.path.insert(0, os.path.join(os.path.expanduser("~"), "vibe", "ex1",
                                "ros2_ws", "src", "patrol_core"))
from patrol_core import gauge as G          # noqa: E402

CALIB = os.path.join(os.path.expanduser("~"), "vibe", "ex1", "maps", "gauge_calib.yaml")


def grab_one(out_dir):
    """grab_shot.py 로 로봇에서 한 장 가져온다(무선 유실을 피하려고 파일로 받는다)."""
    tool = os.path.join(os.path.dirname(os.path.abspath(__file__)), "grab_shot.py")
    r = subprocess.run([sys.executable, tool, "--n", "1", "--out", out_dir],
                       capture_output=True, text=True, timeout=180)
    print(r.stdout.strip())
    files = sorted(os.path.join(out_dir, f) for f in os.listdir(out_dir)
                   if f.endswith(".jpg"))
    return files[-1] if files else None


def auto_roi(img, radius_px, tol=0.3):
    """사진 위쪽 절반에서 기대 반경에 맞는 원을 찾아 ROI 를 제안한다.

    게이지는 소화기 밸브 옆에 있어 화면 위쪽에 나타난다. 아래쪽 라벨의 로고를
    피하려고 위쪽 60% 만 본다.
    """
    h, w = img.shape[:2]
    top = img[: int(h * 0.6), :]
    g = cv2.GaussianBlur(cv2.cvtColor(top, cv2.COLOR_BGR2GRAY), (7, 7), 1.5)
    rmin = max(int(radius_px * (1 - tol)), 5)
    rmax = int(radius_px * (1 + tol)) + 1
    c = cv2.HoughCircles(g, cv2.HOUGH_GRADIENT, dp=1.2, minDist=max(rmin, 10),
                         param1=120, param2=30, minRadius=rmin, maxRadius=rmax)
    if c is None:
        return None
    cx, cy, r = min(c[0], key=lambda t: abs(t[2] - radius_px))
    pad = r * 1.6
    return (max(int(cx - pad), 0), max(int(cy - pad), 0), int(pad * 2), int(pad * 2))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", required=True, help="지점 이름 (예: FE-01)")
    ap.add_argument("--image", help="사진 경로")
    ap.add_argument("--grab", action="store_true", help="로봇에서 바로 한 장 가져온다")
    ap.add_argument("--radius", type=float, default=40.0, help="게이지 반지름 실측(px)")
    ap.add_argument("--select", action="store_true", help="ROI 를 드래그로 지정")
    ap.add_argument("--roi", help="ROI 를 직접 입력: x,y,w,h")
    ap.add_argument("--max-change", type=float, default=None,
                    help="기준 대비 변화량 임계값(기본 0.9). 실측: 정상 최대 0.39 / 이상 최소 1.65")
    ap.add_argument("--calib", default=CALIB)
    args = ap.parse_args()

    path = args.image
    if args.grab or not path:
        path = grab_one(os.path.join(os.path.expanduser("~"), "vibe", "ex1", "logs", "shots"))
    if not path or not os.path.exists(path):
        raise SystemExit("사진이 없다. --image 로 지정하거나 --grab 을 쓸 것")
    img = cv2.imread(path)
    if img is None:
        raise SystemExit(f"사진을 읽지 못했다: {path}")
    print(f"사진: {path}  ({img.shape[1]}x{img.shape[0]})")

    if args.roi:
        roi = tuple(int(v) for v in args.roi.split(","))
    elif args.select:
        print("게이지를 감싸는 사각형을 드래그하고 Enter. 취소는 c")
        show = img if img.shape[1] <= 1200 else cv2.resize(img, (1200, int(1200 * img.shape[0] / img.shape[1])))
        k = img.shape[1] / show.shape[1]
        r = cv2.selectROI("select gauge", show, showCrosshair=True)
        cv2.destroyAllWindows()
        if r[2] == 0:
            raise SystemExit("선택이 취소됐다")
        roi = tuple(int(v * k) for v in r)
    else:
        roi = auto_roi(img, args.radius)
        if roi is None:
            raise SystemExit("게이지를 자동으로 찾지 못했다. --select 또는 --roi 로 지정할 것")
        print(f"자동 제안 ROI: {roi}")

    # 캘리브레이션 때만 Hough 로 게이지를 찾는다(기준 패치를 아직 만들기 전이므로).
    # 판정 단계에서는 이 패치로 템플릿 정합을 하므로 Hough 의 흔들림이 문제되지 않는다.
    circ = G.find_gauge_hough(img, roi, args.radius)
    if circ is None:
        raise SystemExit("ROI 안에서 게이지 원을 찾지 못했다. --radius 를 실측값으로 조정할 것")
    cx, cy, r = circ
    ang, con = G.needle_angle(img, cx, cy, r)
    refl = G.reflection_pct(img, cx, cy, r)
    red = G.red_pct(img, roi)
    print(f"검출: 중심({cx:.0f},{cy:.0f}) 반지름 {r:.0f}px")
    print(f"바늘 {ang:.0f}°  대비 {con:.0f}  반사 {refl:.1f}%  주변 빨강 {red:.1f}%")

    if con < G.DEF["min_contrast"]:
        print(f"경고: 대비 {con:.0f} 가 낮다(기준 {G.DEF['min_contrast']:.0f}). "
              "바늘을 제대로 못 본 상태로 등록될 수 있다")
    if refl > G.DEF["max_reflection"]:
        print(f"경고: 반사 {refl:.0f}% 가 높다. 자세를 조정해 반사를 줄이고 다시 등록할 것")

    points = G.load_calib(args.calib) if os.path.exists(args.calib) else {}
    points[args.name] = {
        "roi": [int(v) for v in roi],
        "radius_px": round(float(r), 1),
        "normal_angle": round(float(ang), 1),
        "resolution": [img.shape[1], img.shape[0]],
        "ref_image": os.path.basename(path),
        "ref_contrast": round(float(con), 1),
    }
    if args.max_change is not None:
        points[args.name]["max_change"] = args.max_change
    os.makedirs(os.path.dirname(args.calib), exist_ok=True)
    G.save_calib(args.calib, points)

    # 기준 패치를 저장한다. 판정 때 이 패치로 위치를 잡는다 — Hough 는 중심이 흔들려
    # 같은 자세 사진에서도 각도가 뒤집혔다(gauge.py 주석 참고).
    tw = int(r * 2.4)
    patch = img[max(int(cy - tw / 2), 0):int(cy + tw / 2),
                max(int(cx - tw / 2), 0):int(cx + tw / 2)]
    cv2.imwrite(G.ref_path(args.calib, args.name), patch)
    print(f"\n저장: {args.calib}  [{args.name}]")
    print(f"기준 패치: {G.ref_path(args.calib, args.name)}  ({patch.shape[1]}x{patch.shape[0]})")

    res = G.judge(img, points[args.name], ref=patch)
    print(f"자기검증 판정: {res['status']} — {res['reason']}")
    out = os.path.splitext(path)[0] + f"_calib_{args.name}.png"
    cv2.imwrite(out, G.annotate(img, res, roi))
    print(f"표시 사진: {out}")


if __name__ == "__main__":
    sys.exit(main())
