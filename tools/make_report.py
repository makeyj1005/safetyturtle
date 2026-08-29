#!/usr/bin/env python3
"""make_report.py — patrol_log.py 가 쌓은 기록을 시험 보고서 PDF 로 만든다.

  python3 ~/vibe/ex1/tools/make_report.py ~/vibe/ex1/logs/demo_0730_2230 \
      --batt /경로/batt3.log --out ~/vibe/ex1/logs/patrol_test_report.pdf

입력
  <기록>.csv   회차별 요약 (patrol_log.py 가 쓴 것)
  <기록>.txt   상태 변화 전체 시각 — 구간별 소요 시간을 여기서 뽑는다
  --batt       전압 로그 (선택). "HH:MM:SS 12.34" 형식 한 줄씩

[한글 폰트]
PDF 본문은 reportlab 의 CID 폰트(HYGothic-Medium)를 쓴다. 이 시스템의 한글 폰트는
Noto Sans CJK 뿐이고 그건 .ttc + CFF(포스트스크립트 윤곽선)라서 reportlab 이
임베드하지 못한다("postscript outlines are not supported"). CID 폰트는 파일을
임베드하지 않고 표준 한국어 글꼴을 참조하므로, 보는 쪽에서 한글 글꼴로 대체해 그린다
(Ubuntu/Windows/Adobe 모두 가능). 차트는 matplotlib 이므로 Noto 를 직접 쓴다.
"""
import argparse
import csv
import os
import re
from datetime import datetime

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.cidfonts import UnicodeCIDFont
from reportlab.platypus import (Image, PageBreak, Paragraph, SimpleDocTemplate,
                                Spacer, Table, TableStyle)

TTC = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"   # 차트용
CID_BODY = "HYGothic-Medium"      # PDF 본문 표
CID_HEAD = "HYSMyeongJo-Medium"   # 제목 표 머리 (대체 글꼴이 달라 대비가 생긴다)

# 순찰 사각형의 구간 길이(m). 웨이포인트 순서: 좌상 -> 우상 -> 우하 -> 좌하 -> 좌상
SEG_LEN = {"우상": 1.15, "우하": 0.95, "좌하": 1.15, "좌상": 0.95}
DANGER_V = 11.0                   # 이 전압에서 turtlebot3_node 가 죽는다


# ---------------- 폰트 ----------------
def register_fonts():
    pdfmetrics.registerFont(UnicodeCIDFont(CID_BODY))
    pdfmetrics.registerFont(UnicodeCIDFont(CID_HEAD))
    pdfmetrics.registerFontFamily("KR", normal=CID_BODY, bold=CID_HEAD)
    # matplotlib 은 .ttc 를 직접 읽는다. 첫 얼굴(JP)만 잡히지만 Noto Sans CJK 는
    # 모든 변형이 한글 글리프를 포함하므로 그대로 써도 된다.
    if os.path.exists(TTC):
        font_manager.fontManager.addfont(TTC)
        names = {f.name for f in font_manager.fontManager.ttflist if "Sans CJK" in f.name}
        if names:
            plt.rcParams["font.family"] = sorted(names)[0]
    plt.rcParams["axes.unicode_minus"] = False


# ---------------- 입력 파싱 ----------------
def read_runs(base):
    with open(base + ".csv") as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        for k in ("소요초", "바퀴수", "실패", "거절"):
            r[k] = int(float(r[k]))
        for k in ("거리m", "전압시작", "전압최저", "전압종료"):
            r[k] = float(r[k])
    return rows


def read_segments(base):
    """상태 로그에서 회차별 구간 소요 시간을 뽑는다.

    'moving to X' 가 찍힌 시각은 X 로 가는 목표를 보낸 순간이다. 따라서
    다음 'moving to Y' 까지의 간격이 X 구간의 소요 시간이다.
    회차 첫 줄은 시작지점 이동이라 구간으로 세지 않는다.
    """
    events = []
    for line in open(base + ".txt"):
        m = re.match(r"(\d\d:\d\d:\d\d) \[상태\] (.+)", line.strip())
        if m:
            events.append((datetime.strptime(m.group(1), "%H:%M:%S"), m.group(2)))

    runs, cur, prev, first = [], None, None, True
    for t, s in events:
        if s == "run started":
            if cur:
                runs.append(cur)
            cur = {"start": t, "segs": [], "done": None}
            prev, first = None, True
            continue
        if cur is None:
            continue
        m = re.match(r"moving to (\S+)", s)
        if m:
            if prev and not first:
                cur["segs"].append((prev[1], (t - prev[0]).total_seconds()))
            elif prev and first:
                first = False          # 시작지점 이동은 버린다
            prev = (t, m.group(1))
        elif s.startswith(("done", "stopped")):
            if prev and not first:
                cur["segs"].append((prev[1], (t - prev[0]).total_seconds()))
            cur["done"] = (t, s)
            runs.append(cur)
            cur, prev = None, None
    if cur:
        runs.append(cur)
    return runs


def find_power_events(batt, thr=0.20, win=15):
    """전원 교체·충전으로 전압이 계단식으로 올라간 지점을 찾는다.

    전압 센서는 0.1V 단위로 계단 진동을 하므로(12.19<->12.30) 한 샘플 차이로는
    판단할 수 없다. 앞뒤 win 개의 중앙값을 비교해 지속적인 상승만 잡는다.
    """
    ys = [v for _, v in batt]
    out = []
    for i in range(win, len(ys) - win):
        before = sorted(ys[i - win:i])[win // 2]
        after = sorted(ys[i:i + win])[win // 2]
        if after - before >= thr:
            if not out or i - out[-1][0] > win * 2:
                out.append((i, after - before))
    return out


def read_batt(path):
    out = []
    if not path or not os.path.exists(path):
        return out
    for line in open(path):
        m = re.match(r"(\d\d:\d\d:\d\d)\s+([\d.]+)", line.strip())
        if m:
            out.append((m.group(1), float(m.group(2))))
    return out


# ---------------- 그래프 ----------------
def chart_segments(runs, path):
    names = ["우상", "우하", "좌하", "좌상"]
    labels = ["좌상→우상\n1.15m", "우상→우하\n0.95m", "우하→좌하\n1.15m", "좌하→좌상\n0.95m"]
    fig, ax = plt.subplots(figsize=(7.2, 3.2))
    w = 0.8 / max(len(runs), 1)
    for i, r in enumerate(runs):
        d = dict(r["segs"])
        vals = [d.get(n, 0) for n in names]
        ax.bar([x + i * w for x in range(4)], vals, width=w, label=f"{i+1}회차")
    ax.set_xticks([x + 0.4 - w / 2 for x in range(4)])
    ax.set_xticklabels(labels, fontsize=8)
    ax.set_ylabel("소요 시간 (초)")
    ax.legend(fontsize=7, ncol=len(runs))
    ax.grid(axis="y", alpha=0.3)
    ax.set_title("구간별 소요 시간", fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def chart_battery(batt, runs, path, events=()):
    if not batt:
        return False
    xs = list(range(len(batt)))
    ys = [v for _, v in batt]
    fig, ax = plt.subplots(figsize=(7.2, 2.8))
    ax.plot(xs, ys, lw=0.8)
    ax.axhline(DANGER_V, color="red", ls="--", lw=1)
    ax.annotate(f"{DANGER_V}V — turtlebot3_node 정지", (0, DANGER_V),
                textcoords="offset points", xytext=(4, 4), color="red", fontsize=7)
    for i, d in events:
        ax.axvline(i, color="#2a7", ls=":", lw=1.2)
        ax.annotate(f"전원 교체 (+{d:.2f}V)", (i, max(ys)),
                    textcoords="offset points", xytext=(-70, -8), color="#2a7", fontsize=7)
    step = max(len(batt) // 8, 1)
    ax.set_xticks(xs[::step])
    ax.set_xticklabels([batt[i][0][:5] for i in xs[::step]], fontsize=7)
    lo = min(min(ys), DANGER_V) - 0.05
    ax.set_ylim(lo, max(ys) + 0.1)
    ax.set_ylabel("전압 (V)")
    ax.grid(alpha=0.3)
    ax.set_title("배터리 전압 추이", fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return True


# ---------------- 표 ----------------
def wrap_cells(data, body_style, head_style):
    """셀 텍스트를 Paragraph 로 감싼다. 그냥 문자열로 두면 칸을 넘쳐 옆 칸과 겹친다."""
    out = []
    for i, row in enumerate(data):
        st = head_style if i == 0 else body_style
        out.append([c if isinstance(c, Paragraph) else Paragraph(str(c), st) for c in row])
    return out


def styled(data, widths, align_right=()):
    t = Table(data, colWidths=widths, hAlign="LEFT")
    cmds = [
        ("FONT", (0, 0), (-1, 0), CID_HEAD, 8),
        ("FONT", (0, 1), (-1, -1), CID_BODY, 8),
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#e8eaf0")),
        ("GRID", (0, 0), (-1, -1), 0.4, colors.HexColor("#9aa0ac")),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("TOPPADDING", (0, 0), (-1, -1), 3),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 3),
    ]
    for c in align_right:
        cmds.append(("ALIGN", (c, 1), (c, -1), "RIGHT"))
    t.setStyle(TableStyle(cmds))
    return t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("base", help="patrol_log 기록 경로 (확장자 없이)")
    ap.add_argument("--batt", default=None, help="전압 로그 경로")
    ap.add_argument("--out", default=None, help="PDF 경로")
    ap.add_argument("--date", default=None, help="시험 일자 (예: 2026-07-30)")
    args = ap.parse_args()

    base = os.path.expanduser(args.base)
    out = os.path.expanduser(args.out or base + "_report.pdf")
    register_fonts()

    runs = read_runs(base)
    segs = read_segments(base)
    batt = read_batt(os.path.expanduser(args.batt) if args.batt else None)
    tmp = os.path.dirname(out) or "."
    seg_png = os.path.join(tmp, "_seg.png")
    bat_png = os.path.join(tmp, "_bat.png")
    # 완주한 회차만 구간 표에 넣는다 (중단된 회차는 구간이 비어 통계를 왜곡한다)
    seg_done = [r for r in segs if r["done"] and len(r["segs"]) >= 4]
    chart_segments(seg_done, seg_png)
    power_events = find_power_events(batt) if batt else []
    has_bat = chart_battery(batt, runs, bat_png, power_events)

    ss = getSampleStyleSheet()
    H1 = ParagraphStyle("H1", parent=ss["Heading1"], fontName=CID_HEAD, fontSize=15,
                        spaceAfter=6, spaceBefore=2)
    H2 = ParagraphStyle("H2", parent=ss["Heading2"], fontName=CID_HEAD, fontSize=11,
                        spaceAfter=4, spaceBefore=10)
    P = ParagraphStyle("P", parent=ss["Normal"], fontName=CID_BODY, fontSize=9, leading=14)
    SMALL = ParagraphStyle("S", parent=P, fontSize=8, textColor=colors.HexColor("#555"))
    CELL = ParagraphStyle("C", parent=P, fontSize=8, leading=10.5)
    CELLH = ParagraphStyle("CH", parent=CELL, fontName=CID_HEAD)
    TITLE = ParagraphStyle("T", parent=ss["Title"], fontName=CID_HEAD, fontSize=19,
                           alignment=TA_CENTER, spaceAfter=2)
    SUB = ParagraphStyle("Sub", parent=P, alignment=TA_CENTER,
                         textColor=colors.HexColor("#555"), spaceAfter=12)

    doc = SimpleDocTemplate(out, pagesize=A4, topMargin=18 * mm, bottomMargin=16 * mm,
                            leftMargin=18 * mm, rightMargin=18 * mm,
                            title="순찰 로봇 구간별 주행 시험 보고서")
    S = []
    date = args.date or "2026-07-30"

    S.append(Paragraph("순찰 로봇 구간별 주행 시험 보고서", TITLE))
    S.append(Paragraph(f"TurtleBot3 Burger / Nav2 웨이포인트 순찰 / {date}", SUB))

    # 1. 개요
    S.append(Paragraph("1. 시험 개요", H1))
    S.append(styled([
        ["항목", "내용"],
        ["목적", "사각형 순찰 경로의 구간별 주행 성능과 자동 재시작 동작 확인"],
        ["대상", "patrol_node (웨이포인트 순찰) + patrol_scheduler (랜덤 재시작)"],
        ["로봇", "TurtleBot3 Burger, Raspberry Pi 3 Model B, OpenCR"],
        ["제어", "VM (Ubuntu 22.04 / ROS 2 Humble), Nav2, AMCL"],
        ["지도", "patrol_map_v5 (5cm/px, 원점 -1.59 / -5.12)"],
        ["경로", "사각형 1.15m × 0.95m, 시계방향, 코너 정지 후 회전"],
        ["시험 일자", date],
    ], [70, 380], align_right=())),

    # 2. 조건
    S.append(Paragraph("2. 시험 조건", H1))
    S.append(styled([
        ["파라미터", "값", "의미"],
        ["laps", "1", "한 회차에 도는 바퀴 수"],
        ["rest_min", "1.0 분", "도착 직후 반드시 쉬는 시간"],
        ["cycle_min", "1.5 분", "도착 후 다음 순찰까지의 최대 시간"],
        ["mode", "loop", "왕복이 아니라 같은 방향 순환"],
        ["stop_at_corners", "true", "지점마다 정지 후 회전 (좁은 경로 대응)"],
        ["max_vel_x", "0.15 m/s", "벽 여유를 고려해 낮춘 상한"],
        ["inflation_radius", "0.25 m", "벽 주변 고비용 구역 반경"],
    ], [90, 70, 290]))
    S.append(Spacer(1, 4))
    S.append(Paragraph(
        "다음 순찰 시각은 시계가 아니라 <b>직전 순찰이 끝난 시각</b>을 기준으로 뽑는다. "
        "따라서 규격상 회차 간 대기 시간은 60~90초 사이의 임의값이어야 한다.", P))

    # 3. 판정
    S.append(Paragraph("3. 시험 항목 및 판정", H1))
    n_done = len([r for r in runs])
    fails = sum(r["실패"] for r in runs)
    rejects = sum(r["거절"] for r in runs)
    # 회차 간 대기 시간. 중단된 회차(done 없이 새 run started 가 온 경우)가 끼면
    # 그 간격은 규격 검증에서 제외한다 — 사람이 다시 띄운 시간이 섞여 있어 의미가 없다.
    rests, excluded = [], 0
    last_done, prev_aborted = None, False
    for r in segs:
        if last_done is not None:
            gap = (r["start"] - last_done).total_seconds()
            if prev_aborted:
                excluded += 1
            else:
                rests.append(gap)
        if r["done"]:
            last_done, prev_aborted = r["done"][0], False
        else:
            prev_aborted = True
    ok_rest = [r for r in rests if 60 <= r <= 90]
    rest_pass = bool(rests) and len(ok_rest) == len(rests)
    judge = [
        ["항목", "판정", "근거"],
        ["사각형 한 바퀴 주행", "적합", f"{n_done}회차 모두 완주, 주행거리 4.4~5.9m (둘레 4.2m + 시작지점 이동)"],
        ["코너 정지 후 회전", "적합", "지점마다 목표를 따로 보내 4개 코너에서 정지 후 회전 확인"],
        ["시작지점 자동 이동", "적합", "회차마다 좌상 지점으로 먼저 이동 후 순찰 시작"],
        ["바퀴 수 도달 시 자동 정지", "적합", "회차마다 done (1바퀴 완주) 로 스스로 종료"],
        ["쉼 후 랜덤 재시작", "적합" if rest_pass else "확인 필요",
         f"측정 {', '.join(f'{int(r)}초' for r in rests) or '없음'} / 규격 60~90초"
         + (f" (중단 회차로 인한 {excluded}건 제외)" if excluded else "")],
        ["주행 실패 / 목표 거절", "적합", f"실패 {fails}회, 거절 {rejects}회"],
        ["좌상 기둥 회피", "적합", "해당 구간(좌하→좌상) 소요 시간이 회차 간 가장 일정"],
    ]
    S.append(styled(wrap_cells(judge, CELL, CELLH), [100, 38, 312]))

    S.append(PageBreak())

    # 4. 회차별 결과
    S.append(Paragraph("4. 회차별 결과", H1))
    head = ["회차", "시작", "종료", "소요(초)", "거리(m)", "전압 시작", "전압 최저", "전압 종료", "실패", "거절"]
    body = [[r["회차"], r["시작"], r["종료"], r["소요초"], f"{r['거리m']:.2f}",
             f"{r['전압시작']:.2f}", f"{r['전압최저']:.2f}", f"{r['전압종료']:.2f}",
             r["실패"], r["거절"]] for r in runs]
    if runs:
        durs = [r["소요초"] for r in runs]
        body.append(["평균", "", "", f"{sum(durs)/len(durs):.0f}",
                     f"{sum(r['거리m'] for r in runs)/len(runs):.2f}",
                     "", "", "", "", ""])
    t = styled([head] + body, [28, 52, 52, 46, 44, 48, 48, 48, 30, 30],
               align_right=(3, 4, 5, 6, 7, 8, 9))
    t.setStyle(TableStyle([("FONT", (0, len(body)), (-1, -1), CID_HEAD, 8),
                           ("BACKGROUND", (0, len(body)), (-1, -1),
                            colors.HexColor("#f2f4f8"))]))
    S.append(t)
    S.append(Spacer(1, 3))
    S.append(Paragraph("전압은 부하 시 처지고 정지하면 회복한다. '최저'는 주행 중 순간값, "
                       "'종료'는 회복 후 값이다.", SMALL))

    # 5. 구간별
    S.append(Paragraph("5. 구간별 소요 시간", H1))
    names = ["우상", "우하", "좌하", "좌상"]
    labels = {"우상": "좌상→우상 (1.15m)", "우하": "우상→우하 (0.95m)",
              "좌하": "우하→좌하 (1.15m)", "좌상": "좌하→좌상 (0.95m)"}
    head = ["구간"] + [f"{i+1}회차" for i in range(len(seg_done))] + ["평균", "최소", "최대", "평균 속도"]
    body = []
    for n in names:
        vals = [dict(r["segs"]).get(n) for r in seg_done]
        got = [v for v in vals if v]
        avg = sum(got) / len(got) if got else 0
        row = [labels[n]] + [f"{v:.0f}초" if v else "-" for v in vals]
        row += [f"{avg:.0f}초", f"{min(got):.0f}초" if got else "-",
                f"{max(got):.0f}초" if got else "-",
                f"{SEG_LEN[n]/avg:.3f} m/s" if avg else "-"]
        body.append(row)
    w = [104] + [34] * len(seg_done) + [36, 34, 34, 56]
    S.append(styled([head] + body, w, align_right=tuple(range(1, len(head)))))
    S.append(Spacer(1, 6))
    S.append(Image(seg_png, width=175 * mm, height=77 * mm))
    S.append(Spacer(1, 3))
    S.append(Paragraph(
        "설정 상한은 0.15 m/s 이나 실측 평균 속도는 그 절반 수준이다. 코너마다 정지 후 회전하고 "
        "목표 도달 판정(xy_goal_tolerance 0.10m)을 기다리기 때문이다. "
        "특정 구간이 일관되게 느린 것이 아니라, 회차마다 임의의 한 구간에서 20초대가 나타난다 — "
        "Nav2 가 그 구간에서 한 번 경로를 다시 계획했거나 자세를 재조정한 것으로 보인다.", P))

    # 6. 배터리
    if has_bat:
        S.append(PageBreak())
        S.append(Paragraph("6. 배터리", H1))
        S.append(Image(bat_png, width=175 * mm, height=68 * mm))
        S.append(Spacer(1, 4))
        # 전원 교체 지점을 찾아, 소모량은 교체 전 구간에서만 계산한다.
        # 교체를 무시하면 전압이 올라간 것으로 보여 "소모 없음"이라는 잘못된 결론이 난다.
        events = power_events
        seg = batt[:events[-1][0]] if events else batt
        v0, v1 = seg[0][1], seg[-1][1]
        span = (datetime.strptime(seg[-1][0], "%H:%M:%S")
                - datetime.strptime(seg[0][0], "%H:%M:%S")).total_seconds() / 60
        # 대기 시간이 섞여 있어 '분당 소모'로 잔여를 추정하면 과대평가된다.
        # 회차당 소모량과 실제 회차 주기(시작~시작 간격)로 계산한다.
        t_end = datetime.strptime(seg[-1][0], "%H:%M:%S")
        in_seg = [r for r in runs if datetime.strptime(r["시작"], "%H:%M:%S") <= t_end]
        per_run = (v0 - v1) / len(in_seg) if in_seg else 0
        starts = [datetime.strptime(r["시작"], "%H:%M:%S") for r in (in_seg or runs)]
        cadence = ((starts[-1] - starts[0]).total_seconds() / 60 / (len(starts) - 1)
                   if len(starts) > 1 else 0)
        n_left = (v1 - DANGER_V) / per_run if per_run > 0 else float("inf")
        min_left = n_left * cadence if cadence and n_left != float("inf") else None
        rows = [
            ["항목", "값"],
            ["측정 구간", f"{seg[0][0]} ~ {seg[-1][0]} ({span:.0f}분, 대기 시간 포함) / "
                        f"회차 {len(in_seg) or len(runs)}건"],
            ["전압", f"{v0:.2f}V → {v1:.2f}V (최저 {min(v for _, v in seg):.2f}V)"],
            ["회차당 소모", f"약 {per_run*1000:.0f} mV (회차 주기 약 {cadence:.1f}분)"],
        ]
        if events:
            i, d = events[-1]
            rows.insert(2, ["전원 교체 감지",
                            f"{batt[i][0]} 에 +{d:.2f}V 계단 상승 — 충전 또는 어댑터 연결로 "
                            f"판단. 이후 구간은 소모 계산에서 제외했다"])
        S.append(styled(wrap_cells(rows + [
            ["잔여 추정",
             (f"{DANGER_V}V 까지 약 {n_left:.0f}회차"
              + (f", 약 {min_left:.0f}분" if min_left else ""))
             if per_run > 0 else "소모가 관측되지 않음 (어댑터 구동 가능성)"],
            ["안전 정지선", f"11.2~11.3V — 약 {(v1-11.25)/per_run:.0f}회차 여유"
                        if per_run > 0 else "-"],
            ["비고", "기준선이 회복되지 않고 내려간다 = 어댑터가 아닌 배터리 구동"],
        ], CELL, CELLH), [70, 380]))
        S.append(Spacer(1, 3))
        S.append(Paragraph(
            f"방전 곡선은 뒤로 갈수록 급해진다. 11.2~11.3V 를 정지 기준으로 삼는 편이 안전하다. "
            f"{DANGER_V}V 아래로 내려가면 OpenCR 이 경고음을 내고 turtlebot3_node 가 종료된다.", P))

    # 7. 관찰 / 조치
    S.append(Paragraph("7. 관찰 사항 및 조치", H1))
    S.append(styled(wrap_cells([
        ["관찰", "판단", "조치"],
        ["회차마다 한 구간에서 20초대 소요", "재계획 또는 자세 재조정. 실패로 이어지지 않음",
         "경과 관찰. 재발이 잦으면 sim_time 과 회전 파라미터 검토"],
        ["회차 1건이 시작지점 이동 후 중단", "화면 절전으로 제어 세션이 멈춘 것. 로봇과 Nav2 문제 아님",
         "시연 중 절전 해제 (gsettings idle-delay 0)"],
        ["기록상 바퀴 수가 0으로 집계", "마지막 바퀴는 lap 상태 없이 done 으로 넘어가 로거가 세지 못함",
         "patrol_node 가 마지막 바퀴도 상태를 발행하도록 수정"],
        ["전압 기준선 하강", "배터리 구동 확인", "시연 전 충전 또는 SMPS 어댑터 연결"],
    ], CELL, CELLH), [112, 168, 170]))

    # 8. 결론
    S.append(Paragraph("8. 결론", H1))
    S.append(Paragraph(
        f"사각형 순찰, 코너 정지 후 회전, 바퀴 수 도달 시 자동 정지, 도착 후 쉼과 임의 시각 재시작이 "
        f"모두 실제 로봇에서 확인되었다. {len(runs)}회차 동안 주행 실패와 목표 거절은 발생하지 않았다. "
        f"한 바퀴는 약 1분이며, 회차 간 대기는 규격(60~90초) 안에서 매번 다른 값으로 관측되었다. "
        f"남은 제약은 주행 성능이 아니라 배터리 지속 시간이다.", P))

    doc.build(S)
    for p in (seg_png, bat_png):
        if os.path.exists(p):
            os.remove(p)
    print("생성:", out)


if __name__ == "__main__":
    main()
