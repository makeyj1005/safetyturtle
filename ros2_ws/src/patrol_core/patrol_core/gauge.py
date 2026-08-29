#!/usr/bin/env python3
"""gauge.py — 소화기 압력계 사진 한 장으로 정상/이상을 판정한다.

ROS 의존이 없는 순수 모듈이다(cv2, numpy, yaml 만 쓴다). inspect_node 와
캘리브레이션 도구가 같은 기준을 쓰도록 판정 로직을 여기 한 곳에 둔다.

[판정 방법 — 기준 사진과의 차이를 본다]
① 캘리브레이션 때 저장한 게이지 패치로 **템플릿 정합**해 위치와 크기를 찾는다.
② 다이얼 고리(반지름의 45~80%)의 **각도별 밝기 분포**를 기준 사진과 같은 방식으로 구한다.
③ 두 분포의 **차이**를 본다. 차이가 작으면 바늘이 그대로 = 정상,
   특정 각도에 큰 음의 변화가 있으면 바늘이 그쪽으로 옮겨간 것 = 이상.

[왜 '차이'를 보는가 — 이게 핵심이다]
바늘의 절대 각도를 매번 새로 찾으려 하면 실패한다. 다이얼에는 바늘 말고도 어두운 것이
많다 — 가운데 육각 너트와 그 그림자, 아래쪽 금속, 테두리 반사. 실측에서 이런 방식은
같은 자세 사진에서도 175°/145°/180° 로 오판했다. 고리 반지름이 몇 % 만 어긋나도
넓은 그림자를 바늘로 착각한다.
기준 사진과 비교하면 **고정된 어두운 것들은 양쪽에 똑같이 있어 상쇄**되고 움직인
바늘만 남는다. 실측 결과(2026-07-31):
  바늘 안 움직임: 변화량 0.00~0.39 (같은 자세 / 위치 ±10cm / 거리 ±35%)
  바늘 움직임   : 변화량 1.65~3.53, 그리고 최대 변화 각도가 실제 바늘 각도와 일치
                  (45°→45°, 90°→90°, 135°→135°, 270°→265°)
그래서 임계값을 0.9 로 두면 양쪽에 두 배 가까운 여유가 있다.

[왜 원 검출(Hough)이 아니라 템플릿 정합으로 위치를 잡나 — 실측]
같은 자세 3연속 프레임에서 Hough 가 잡은 원이 흔들렸다(중심 10px, 반경 4px). 그만큼만
어긋나도 고리가 육각 너트 그림자를 물어 판정이 뒤집혔다. 템플릿 정합은 같은 3장에서
중심이 동일(872,266)하고 점수 0.996~1.000 이었다. 정합 점수는 '소화기가 그 자리에
있는지'를 판단하는 신뢰도로도 쓴다.

[왜 ROI 가 필요한가]
라벨의 "OK" 로고 원이 93px, 게이지가 80px 로 크기 차이가 14% 뿐이라 크기로는 구분되지
않는다. 로고 위치에서도 각도와 대비가 멀쩡히 계산되므로 위치로 걸러야 한다.

[판정 결과]
  정상       기준 사진과 바늘 위치가 같다
  이상       바늘이 옮겨갔다 (압력 부족/과다)
  판정불가   반사가 심하거나, 게이지 정합이 낮거나, 바늘 대비가 없다
  부재       게이지도 못 찾고 빨간 영역도 없다 = 소화기가 치워졌다
"""
import os

import cv2
import numpy as np
import yaml

# ---- 기본 임계값 (캘리브레이션 파일에서 항목별로 덮어쓸 수 있다) ----
DEF = {
    # 기준 사진 대비 변화량 임계값. 실측 정상 최대 0.39 / 이상 최소 1.65.
    "max_change": 0.9,
    "min_contrast": 40.0,   # 고리의 최대-최소 밝기 차(원래 밝기 단위). 실측 102.
    "max_reflection": 8.0,  # 게이지 안 포화 화소 비율(%). 실측 0.7%.
    "min_red_pct": 1.0,     # ROI 주변 빨강 비율. 이보다 낮으면 소화기 자체가 없다.
    "min_score": 0.55,      # 템플릿 정합 점수 하한. 정상 자세는 0.92~1.00 이었다.
    # 탐색 범위는 Nav2 정지 오차를 견뎌야 한다. 40cm 거리에서 좌우 10cm = 약 340px,
    # 전후 10cm = 크기 ±25%. 여기에 각도 오차가 더 크게 들어온다 —
    # 2026-08-01 실측: yaw 가 10° 어긋나자 게이지가 492px 옮겨가 420px 밖으로 나갔다
    # (그 좌표에서는 정합 0.39 = 부재 오판, 화면 전체를 뒤지면 0.64 로 찾힌다).
    # 700 이면 명령 각도 기준 ±14° 를 덮는다. 더 키우면 라벨의 "OK" 로고 같은
    # 엉뚱한 원에 걸릴 위험이 커지므로 min_score(0.55)로 거른다.
    "search_pad": 700,
    "scales": [0.70, 0.78, 0.85, 0.92, 1.0, 1.08, 1.18, 1.30, 1.42],
    # 찾은 게이지가 기준 자리(ROI 중심)에서 이만큼 넘게 벗어나면 판정하지 않는다.
    # 판정법이 "같은 자세에서 찍은 기준 사진과의 차이"라 자세가 틀어지면 무너진다.
    # 2026-08-01 실측: 492px 어긋난 사진을 그대로 판정했더니 바늘이 그대로인데도
    # 변화량 2.16 이 나와 "이상" 으로 오판했다. 놓치는 것보다 나쁜 게 헛경보다.
    # 정렬 탐색이 들어간 뒤로는 93px 어긋나도 0.53 이라 여유가 있다. 이 문턱은
    # "이 정도면 아예 다른 자세다" 를 거르는 용도로만 남긴다(그 경우 보정 재시도).
    "max_offset_px": 120,
    # 화면 오프셋을 로봇 각도로 되돌리는 환산.
    #   필요한 각도(도) = -dx(px) / px_per_deg
    #   화면에서 오른쪽(dx>0)이면 yaw 를 **줄인다**(시계방향).
    #
    # [부호를 이렇게 정한 근거 — 2026-08-01]
    # 처음엔 반대로 뒀다가 실제 주행에서 틀린 게 확인됐다. 사용자가 로봇을 보고 있었고,
    # 보정으로 **왼쪽(CCW)으로 돌자 대상이 화면 밖으로 더 나갔다**. 기하학적으로도
    # 그쪽이 맞다 — 카메라를 CCW 로 돌리면 장면은 화면에서 오른쪽으로 흐른다
    # (전방·후방 장착 모두 같다. 축과 '오른쪽' 방향이 함께 돌기 때문이다).
    # 반대 부호를 잠깐 믿었던 이유는 "기준 사진이 저장된 yaw(-81°)에서 찍혔다"고
    # 가정하고 한 점짜리 추론을 했기 때문이다. 그 가정에 근거가 없었다.
    # ⚠️ 그래서 inspect_node 는 이 부호를 맹신하지 않는다 — 보정 후 오프셋이 오히려
    #    커지면 스스로 부호를 뒤집는다.
    #
    # 크기는 **기하학적 값**을 쓴다: 화각 62.2° / 1640px = 26.4px/도.
    # 한때 실측이라며 49.2 를 썼는데, 그건 "기준 사진이 저장된 yaw 에서 찍혔다"는
    # 근거 없는 가정 위에서 한 점으로 뽑은 값이라 폐기했다.
    # ⚠️ 보정량 = dx / px_per_deg 이므로 **작게 잡을수록 크게 돈다**. 이 값이 실제보다
    #    작으면 지나쳐서 발산한다(시뮬레이션: 실제 49.2 인데 26.4 로 잡으면 성공률 0%).
    #    그래서 inspect_node 는 계산값의 **절반만** 준다 — 실제가 이 값의 2배까지
    #    벌어져도 안전하고, 맞으면 두세 번에 수렴한다.
    "px_per_deg": 26.4,
    "r_in": 0.45,           # 고리 안쪽 (가운데 육각 너트를 피한다)
    "r_out": 0.80,          # 고리 바깥쪽 (테두리 금속을 피한다)
    "nbin": 72,             # 각도 분해능 5°
    # 비교 전에 정합 위치를 조금씩 흔들어 보고 **가장 잘 맞는 자리**의 값을 쓴다.
    # 2026-08-01 실측: 바늘은 그대로인데 화면이 93px 어긋난 것만으로 변화량이
    # 0.18 -> 1.27 로 뛰어 헛경보가 났다. 템플릿 정합이 몇 px만 빗나가도 고리가
    # 다른 곳을 훑기 때문이다. 바늘이 진짜 움직였으면 어떻게 맞춰도 값이 안 내려가므로
    # (구조가 통째로 바뀐다) 이 탐색은 헛경보만 걷어낸다.
    # 실측으로 고른 값(2026-08-01). 넓게 흔들수록 헛경보는 줄지만 **진짜 이상까지
    # 깎아먹는다** — ±8px 로 넓히자 바늘 240° 이동이 2.32 -> 1.28 로 내려갔다.
    #   ±4px/2 배율3 : 헛경보 0.49~0.53 / 진짜이상 1.60~2.68  (1.8초)  <- 채택
    #   ±6px/2 배율5 : 헛경보 0.52      / 진짜이상 1.41~2.25  (2.3초)
    #   탐색 없음    : 헛경보 1.25      / 진짜이상 2.42~3.61  (1.8초)
    "align_px": 4,          # 중심을 ±이만큼 흔들어 본다
    "align_step": 2,
    "align_scales": [0.96, 1.0, 1.04],   # 반지름도 조금 흔든다
}


# ---------------- 파일 ----------------
def ref_path(calib_path, name):
    """지점별 게이지 기준 패치 경로. 캘리브레이션 파일과 같은 폴더에 둔다."""
    return os.path.join(os.path.dirname(calib_path) or ".", f"gauge_ref_{name}.png")


def load_ref(calib_path, name):
    p = ref_path(calib_path, name)
    return cv2.imread(p) if os.path.exists(p) else None


def load_calib(path):
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    return data.get("points", {})


def save_calib(path, points):
    with open(path, "w") as f:
        yaml.safe_dump({"points": points}, f, allow_unicode=True, sort_keys=False)


def params(cal):
    p = dict(DEF)
    p.update({k: v for k, v in (cal or {}).items() if k in DEF})
    return p


# ---------------- 위치 찾기 ----------------
def find_gauge_hough(img, roi, radius_px, radius_tol=0.25):
    """ROI 안에서 원을 찾는다. **캘리브레이션 전용** — 기준 패치가 아직 없을 때만 쓴다.

    판정에는 쓰지 말 것. 프레임마다 중심이 흔들려 판정이 뒤집힌다(모듈 설명 참고).
    """
    x, y, w, h = [int(v) for v in roi]
    x, y = max(x, 0), max(y, 0)
    sub = img[y:y + h, x:x + w]
    if sub.size == 0:
        return None
    g = cv2.GaussianBlur(cv2.cvtColor(sub, cv2.COLOR_BGR2GRAY), (7, 7), 1.5)
    rmin = max(int(radius_px * (1 - radius_tol)), 5)
    rmax = int(radius_px * (1 + radius_tol)) + 1
    c = cv2.HoughCircles(g, cv2.HOUGH_GRADIENT, dp=1.2, minDist=max(rmin, 10),
                         param1=120, param2=30, minRadius=rmin, maxRadius=rmax)
    if c is None:
        return None
    best = min(c[0], key=lambda t: abs(t[2] - radius_px))
    return float(best[0] + x), float(best[1] + y), float(best[2])


# 화면 전체 탐색용 배율. 자세가 크게 틀어지면 거리도 달라지므로 넓게 잡는다.
WIDE_SCALES = [0.6, 0.7, 0.8, 0.85, 0.9, 0.95, 1.0, 1.05, 1.1, 1.2, 1.3, 1.45]


def find_anywhere(img, ref, scales=None):
    """기준 패치를 **화면 전체**에서 찾는다. 반환 (score, cx, cy, scale).

    판정용 locate_gauge 는 ROI 주변만 본다(엉뚱한 원에 속지 않으려고). 이 함수는
    자세를 맞추려고 "게이지가 지금 화면 어디에 있나"를 찾는 용도라 범위를 넓게 둔다.
    찾은 위치를 그대로 판정에 쓰지는 않는다 — 정렬한 뒤 다시 판정한다.
    """
    best = None
    for s in (scales or WIDE_SCALES):
        t = cv2.resize(ref, None, fx=s, fy=s,
                       interpolation=cv2.INTER_AREA if s < 1 else cv2.INTER_CUBIC)
        if t.shape[0] >= img.shape[0] or t.shape[1] >= img.shape[1]:
            continue
        _, sc, _, loc = cv2.minMaxLoc(cv2.matchTemplate(img, t, cv2.TM_CCOEFF_NORMED))
        if best is None or sc > best[0]:
            best = (sc, loc[0] + t.shape[1] / 2.0, loc[1] + t.shape[0] / 2.0, s)
    return best


def roi_center(cal):
    """기준 자세에서 게이지가 있던 화면 위치."""
    x, y, w, h = [float(v) for v in cal["roi"]]
    return x + w / 2.0, y + h / 2.0


def locate_gauge(img, cal, ref):
    """기준 패치로 게이지 위치를 찾는다. 반환 (cx, cy, r, score) 또는 None.

    여러 배율로 정합해 거리 변화를 흡수한다. 배율이 곧 반지름 배율이다.
    """
    p = params(cal)
    x, y, w, h = [int(v) for v in cal["roi"]]
    pad = int(p["search_pad"])
    H, W = img.shape[:2]
    x0, y0 = max(x - pad, 0), max(y - pad, 0)
    sub = img[y0:min(y + h + pad, H), x0:min(x + w + pad, W)]
    if sub.size == 0:
        return None

    best = None
    for s in p["scales"]:
        interp = cv2.INTER_AREA if s < 1 else cv2.INTER_CUBIC
        t = cv2.resize(ref, None, fx=s, fy=s, interpolation=interp)
        if t.shape[0] >= sub.shape[0] or t.shape[1] >= sub.shape[1]:
            continue
        _, score, _, loc = cv2.minMaxLoc(cv2.matchTemplate(sub, t, cv2.TM_CCOEFF_NORMED))
        if best is None or score > best[0]:
            best = (score, x0 + loc[0] + t.shape[1] / 2.0,
                    y0 + loc[1] + t.shape[0] / 2.0, float(cal["radius_px"]) * s)
    if best is None:
        return None
    score, cx, cy, r = best
    return cx, cy, r, score


# ---------------- 각도 분포 ----------------
def angular_profile(img, cx, cy, r, p=None):
    """다이얼 고리의 각도별 밝기 분포. 반환 (정규화 분포, 원래 밝기 분포).

    각 반지름 링마다 평균을 빼고 표준편차로 나눈다 — 반경 방향 밝기 기울기와
    전체 조명 변화를 없애 기준 사진과 직접 비교할 수 있게 만든다. 반지름을
    상대값(r 의 45~80%)으로 잡으므로 크기가 달라도 같은 축에서 비교된다.
    """
    p = p or DEF
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.float32)
    H, W = g.shape
    nbin = int(p["nbin"])
    radii = np.arange(r * p["r_in"], r * p["r_out"], max(r * 0.04, 0.7))
    if len(radii) == 0:
        return None, None
    th = np.arange(nbin) * 2 * np.pi / nbin - np.pi / 2      # 12시=0°, 시계방향
    xs = cx + radii[:, None] * np.cos(th)[None, :]
    ys = cy + radii[:, None] * np.sin(th)[None, :]
    ok = (xs >= 0) & (xs < W) & (ys >= 0) & (ys < H)
    M = np.full(xs.shape, np.nan, np.float32)
    M[ok] = g[np.clip(ys.astype(int), 0, H - 1)[ok], np.clip(xs.astype(int), 0, W - 1)[ok]]
    if np.all(np.isnan(M)):
        return None, None
    with np.errstate(invalid="ignore"):
        Z = M - np.nanmean(M, axis=1, keepdims=True)
        Z = Z / np.maximum(np.nanstd(M, axis=1, keepdims=True), 1e-3)
        norm = np.nan_to_num(np.nanmean(Z, axis=0))
        raw = np.nan_to_num(np.nanmean(M, axis=0))
    return norm, raw


def needle_angle(img, cx, cy, r, p=None):
    """가장 어두운 각도를 바늘로 본다. 캘리브레이션 화면 표시용.

    판정에는 쓰지 않는다 — 다이얼의 다른 어두운 부분에 속기 때문이다(모듈 설명 참고).
    """
    norm, raw = angular_profile(img, cx, cy, r, p)
    if norm is None:
        return None, 0.0
    nbin = len(norm)
    return int(np.argmin(norm)) * 360.0 / nbin, float(raw.max() - raw.min())


def change_vs_ref(img, cx, cy, r, ref, cal):
    """기준 사진과 각도 분포를 비교한다. 반환 (변화량, 변화 각도, 원래 대비).

    변화량이 크면 바늘이 움직인 것이고, 변화 각도가 지금 바늘 위치다.

    중심과 반지름을 조금씩 흔들어 보고 **가장 잘 맞는 자리**의 값을 돌려준다.
    정합이 몇 px만 빗나가도 고리가 다른 곳을 훑어 헛경보가 나기 때문이다(DEF 주석).
    """
    p = params(cal)
    rp, _ = angular_profile(ref, ref.shape[1] / 2.0, ref.shape[0] / 2.0,
                            float(cal["radius_px"]), p)
    if rp is None:
        return None, None, 0.0

    pad = int(p.get("align_px", 0))
    step = max(int(p.get("align_step", 3)), 1)
    shifts = range(-pad, pad + 1, step) if pad else [0]
    scales = p.get("align_scales", [1.0])

    best = None
    for ddx in shifts:
        for ddy in shifts:
            for sc in scales:
                cur, raw = angular_profile(img, cx + ddx, cy + ddy, r * sc, p)
                if cur is None:
                    continue
                d = cur - rp
                i = int(np.argmin(d))
                val = float(-d[i])
                if best is None or val < best[0]:
                    best = (val, i * 360.0 / len(d), float(raw.max() - raw.min()))
    if best is None:
        return None, None, 0.0
    return best


# ---------------- 보조 신호 ----------------
def reflection_pct(img, cx, cy, r):
    """게이지 안에서 흰색으로 포화된 화소 비율(%). 반사로 바늘이 묻혔는지 본다."""
    m = np.zeros(img.shape[:2], np.uint8)
    cv2.circle(m, (int(cx), int(cy)), max(int(r) - 2, 1), 255, -1)
    px = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)[m > 0]
    return float((px > 245).mean() * 100) if px.size else 0.0


def red_pct(img, roi, grow=2.0):
    """ROI 를 넓힌 영역의 빨강 비율(%). 소화기가 그 자리에 있는지 보는 보조 신호."""
    x, y, w, h = [int(v) for v in roi]
    cx, cy = x + w / 2, y + h / 2
    x0, y0 = max(int(cx - w * grow / 2), 0), max(int(cy - h * grow / 2), 0)
    sub = img[y0:int(cy + h * grow / 2), x0:int(cx + w * grow / 2)]
    if sub.size == 0:
        return 0.0
    hsv = cv2.cvtColor(sub, cv2.COLOR_BGR2HSV)
    red = cv2.inRange(hsv, (0, 120, 60), (10, 255, 255)) | \
          cv2.inRange(hsv, (170, 120, 60), (180, 255, 255))
    return float(red.mean() / 255 * 100)


# ---------------- 판정 ----------------
def judge(img, cal, ref):
    """사진 한 장을 판정한다.

    cal : 캘리브레이션 항목(roi, radius_px, resolution, 임계값 덮어쓰기)
    ref : 기준 게이지 패치 (gauge_ref_<이름>.png). 없으면 판정할 수 없다.

    반환 dict: status, change, angle, contrast, reflection, red, score, circle,
              offset(기준 자리에서 벗어난 (dx, dy) px), yaw_fix_deg(보정할 각도), reason
    """
    p = params(cal)
    out = {"status": None, "change": None, "angle": None, "contrast": None,
           "reflection": None, "red": None, "score": None, "circle": None,
           "offset": None, "yaw_fix_deg": None, "reason": ""}

    if ref is None:
        out["status"] = "판정불가"
        out["reason"] = "기준 패치가 없다 — gauge_calib.py 로 먼저 등록할 것"
        return out

    want = cal.get("resolution")
    if want and (img.shape[1], img.shape[0]) != tuple(want):
        out["status"] = "판정불가"
        out["reason"] = (f"해상도가 캘리브레이션({want[0]}x{want[1]})과 다르다 — "
                         f"지금 {img.shape[1]}x{img.shape[0]}. ROI 가 픽셀 좌표라 맞아야 한다")
        return out

    out["red"] = red_pct(img, cal["roi"])
    found = locate_gauge(img, cal, ref)
    out["score"] = None if found is None else found[3]
    if found is None or found[3] < p["min_score"]:
        s = out["score"] or 0.0
        if out["red"] < p["min_red_pct"]:
            out["status"] = "부재"
            out["reason"] = f"게이지 정합 {s:.2f}, 빨강 {out['red']:.1f}% — 소화기가 없다"
        else:
            out["status"] = "판정불가"
            out["reason"] = (f"소화기는 보이는데(빨강 {out['red']:.1f}%) 게이지 정합이 "
                             f"낮다({s:.2f}) — 자세가 틀렸거나 가려졌다")
        return out

    cx, cy, r, _ = found
    out["circle"] = (cx, cy, r)

    # 기준 자세에서 게이지가 있던 자리(ROI 중심)와 얼마나 어긋났나.
    x, y, w, h = [float(v) for v in cal["roi"]]
    dx, dy = cx - (x + w / 2), cy - (y + h / 2)
    out["offset"] = (dx, dy)
    out["yaw_fix_deg"] = -dx / float(p["px_per_deg"])
    if abs(dx) > p["max_offset_px"] or abs(dy) > p["max_offset_px"]:
        # 자세가 틀어졌다. 여기서 판정하면 바늘이 그대로여도 "이상" 이 나온다.
        out["status"] = "판정불가"
        out["reason"] = (f"자세가 어긋났다 (dx {dx:+.0f}px, dy {dy:+.0f}px) — "
                         f"yaw 를 {out['yaw_fix_deg']:+.1f}° 돌려 다시 볼 것")
        return out

    out["reflection"] = reflection_pct(img, cx, cy, r)
    if out["reflection"] > p["max_reflection"]:
        out["status"] = "판정불가"
        out["reason"] = f"반사가 심하다({out['reflection']:.0f}%) — 바늘이 묻혔다"
        return out

    change, angle, contrast = change_vs_ref(img, cx, cy, r, ref, cal)
    out["change"], out["contrast"] = change, contrast
    if change is None:
        out["status"] = "판정불가"
        out["reason"] = "각도 분포를 만들 수 없었다"
        return out
    if contrast < p["min_contrast"]:
        out["status"] = "판정불가"
        out["reason"] = f"바늘 대비가 낮다({contrast:.0f} < {p['min_contrast']:.0f})"
        return out

    if change < p["max_change"]:
        out["status"] = "정상"
        out["reason"] = f"기준과 같다 (변화량 {change:.2f} < {p['max_change']:.2f})"
    else:
        out["angle"] = angle
        out["status"] = "이상"
        out["reason"] = (f"바늘이 {angle:.0f}° 로 옮겨갔다 "
                         f"(변화량 {change:.2f} >= {p['max_change']:.2f})")
    return out


# 사진에 얹는 글자는 영문으로 쓴다 — cv2.putText 는 한글을 못 그려 "??????" 가 된다
# (2026-08-01 보고서에서 발견). 한글이 필요하면 PIL 로 TTF 를 써야 한다.
LABEL_EN = {"정상": "OK", "이상": "ABNORMAL", "판정불가": "UNKNOWN", "부재": "MISSING"}


def annotate(img, res, roi=None):
    """판정 결과를 그림으로 표시한다(증빙 사진용)."""
    out = img.copy()
    if roi is not None:
        x, y, w, h = [int(v) for v in roi]
        cv2.rectangle(out, (x, y), (x + w, y + h), (200, 200, 0), 2)
    col = {"정상": (0, 200, 0), "이상": (0, 0, 255), "판정불가": (0, 200, 255),
           "부재": (0, 0, 255)}.get(res["status"], (255, 255, 255))
    if res.get("circle"):
        cx, cy, r = [int(v) for v in res["circle"]]
        cv2.circle(out, (cx, cy), r, col, 2)
        if res.get("angle") is not None:
            a = np.radians(res["angle"] - 90)
            cv2.line(out, (cx, cy), (int(cx + r * np.cos(a)), int(cy + r * np.sin(a))), col, 2)
    txt = LABEL_EN.get(res["status"], "?")
    if res.get("change") is not None:
        txt += f"  change {res['change']:.2f}"
    if res.get("score") is not None:
        txt += f"  match {res['score']:.2f}"
    if res.get("reflection") is not None:
        txt += f"  refl {res['reflection']:.0f}%"
    # 글자 뒤에 어두운 띠를 깐다 — 밝은 배경(흰 벽·나무판)에서 흰 글자가 안 보였다
    # (2026-08-01 보고서에서 발견).
    (tw, th), base = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 2)
    cv2.rectangle(out, (6, 8), (18 + tw, 20 + th + base), (0, 0, 0), -1)
    cv2.putText(out, txt, (12, 14 + th), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
    return out
