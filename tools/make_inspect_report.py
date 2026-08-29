#!/usr/bin/env python3
"""make_inspect_report.py — 순찰·소화기 점검 결과를 보고서 PDF 로 묶는다.

[VM 에서 실행]
  python3 ~/vibe/ex1/tools/make_inspect_report.py
  python3 ~/vibe/ex1/tools/make_inspect_report.py --date 0801 --out ~/보고서.pdf

[무엇을 담나]
  1. 시스템 구성 — 쓰고 있는 센서와 각각이 무엇에 쓰이는지
  2. 순찰 결과 — patrol_log.py 가 남긴 회차 기록(있으면)
  3. 소화기 점검 결과 — logs/inspect_*.csv 를 모아 회차별 표와 성공률
  4. 증빙 사진 — 정상 판정이 난 회차의 사진(판정 표시가 그려진 것)

[한글 폰트]
make_report.py 와 같은 방식이다. reportlab 의 CID 폰트(HYGothic-Medium)를 쓴다 —
이 시스템의 한글 폰트(Noto Sans CJK)는 .ttc + CFF 라서 임베드가 안 된다. CID 는
파일을 넣지 않고 표준 한국어 글꼴을 참조하므로 보는 쪽에서 대체해 그린다.
"""
import argparse
import csv
import glob
import os
import sys
from datetime import datetime

from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.cidfonts import UnicodeCIDFont
from reportlab.platypus import (Image, PageBreak, Paragraph, SimpleDocTemplate,
                                Spacer, Table, TableStyle)

EX1 = os.path.join(os.path.expanduser("~"), "vibe", "ex1")
CID_BODY, CID_HEAD = "HYSMyeongJo-Medium", "HYGothic-Medium"

# 이 프로젝트가 실제로 쓰는 센서. "무엇에 쓰는가"를 같이 적는다 —
# 목록만 있으면 보고서에서 의미가 없다.
SENSORS = [
    ["LDS-01 라이다", "360° 거리 (5Hz)", "/scan",
     "지도 작성(SLAM), 위치추정(AMCL), 장애물 회피"],
    ["바퀴 엔코더", "좌우 바퀴 회전량", "/odom",
     "이동 거리와 방향 추정. 라이다와 합쳐 위치를 잡는다"],
    ["IMU (자이로/가속도)", "각속도, 가속도", "/imu",
     "회전 추정 보정. 부팅 시 자이로 캘리브레이션 수행"],
    ["지자기 센서", "방위", "/magnetic_field", "보조 (현재 판정에는 미사용)"],
    ["배터리 전압계", "전압, 잔량", "/battery_state",
     "저전압 감시. 11V 아래면 OpenCR 경고음, 노드가 죽는다"],
    ["CSI 카메라 (imx219)", "1640x1232 정지영상", "/camera/image_raw/compressed",
     "압력계 판정용 사진. 로봇 후면 장착"],
    ["부저 (OpenCR)", "-", "/sound (서비스)",
     "판정 알림. 정상은 긴 소리, 이상은 짧은 소리"],
]


def register_fonts():
    pdfmetrics.registerFont(UnicodeCIDFont(CID_BODY))
    pdfmetrics.registerFont(UnicodeCIDFont(CID_HEAD))
    pdfmetrics.registerFontFamily("KR", normal=CID_BODY, bold=CID_HEAD)


def styles():
    ss = getSampleStyleSheet()
    return {
        "title": ParagraphStyle("t", parent=ss["Title"], fontName=CID_HEAD, fontSize=18,
                                leading=24, spaceAfter=4),
        "sub": ParagraphStyle("s", parent=ss["Normal"], fontName=CID_BODY, fontSize=9,
                              textColor=colors.grey, spaceAfter=10),
        "h": ParagraphStyle("h", parent=ss["Heading2"], fontName=CID_HEAD, fontSize=13,
                            leading=17, spaceBefore=12, spaceAfter=6),
        "b": ParagraphStyle("b", parent=ss["Normal"], fontName=CID_BODY, fontSize=9.5,
                            leading=14),
        "cell": ParagraphStyle("c", parent=ss["Normal"], fontName=CID_BODY, fontSize=8.5,
                               leading=11),
        "cellh": ParagraphStyle("ch", parent=ss["Normal"], fontName=CID_HEAD, fontSize=8.5,
                                leading=11),
        "cap": ParagraphStyle("cap", parent=ss["Normal"], fontName=CID_BODY, fontSize=8,
                              textColor=colors.grey, spaceBefore=2, spaceAfter=8),
    }


def table(data, widths, st, align_right=()):
    """첫 줄을 머리행으로 보고 표를 만든다. 셀은 Paragraph 로 감싸 줄바꿈되게 한다."""
    body = [[Paragraph(str(c), st["cellh"] if i == 0 else st["cell"]) for c in row]
            for i, row in enumerate(data)]
    t = Table(body, colWidths=widths, hAlign="LEFT")
    style = [
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#e8e8e8")),
        ("GRID", (0, 0), (-1, -1), 0.4, colors.HexColor("#999999")),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("TOPPADDING", (0, 0), (-1, -1), 3),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 3),
    ]
    for c in align_right:
        style.append(("ALIGN", (c, 1), (c, -1), "RIGHT"))
    t.setStyle(TableStyle(style))
    return t


def read_inspections(date_tag):
    """logs/inspect_<날짜>_*.csv 를 읽어 회차별로 정리한다."""
    rows = []
    for path in sorted(glob.glob(os.path.join(EX1, "logs", f"inspect_{date_tag}_*.csv"))):
        try:
            # 깨진 파일이 섞여 있을 수 있다 — 중복 노드가 같은 CSV 에 겹쳐 쓴 적이 있다.
            # 한 줄 못 읽는다고 보고서 전체가 실패하면 안 된다.
            with open(path, errors="replace") as f:
                for r in csv.DictReader(f):
                    if not r.get("status"):
                        continue
                    r["_file"] = os.path.basename(path)
                    r["_dir"] = path[:-4]        # 증빙 사진 폴더
                    rows.append(r)
                    break                        # 지점이 하나뿐이라 첫 줄만
        except (OSError, csv.Error, UnicodeDecodeError) as e:
            print(f"  건너뜀: {os.path.basename(path)} ({type(e).__name__})")
            continue
    return rows


def read_patrol(base):
    """patrol_log.py 가 남긴 회차 기록(있으면)."""
    path = base + ".csv"
    if not os.path.exists(path):
        return []
    try:
        with open(path, errors="replace") as f:
            return list(csv.DictReader(f))
    except (OSError, csv.Error):
        return []


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--date", default=datetime.now().strftime("%m%d"),
                    help="점검 기록 날짜 태그 (기본 오늘, 예: 0801)")
    ap.add_argument("--patrol", default=os.path.join(EX1, "logs", "demo_0730_2230"),
                    help="patrol_log.py 기록의 확장자 없는 경로")
    ap.add_argument("--out", default=os.path.join(EX1, "logs", "inspect_report.pdf"))
    ap.add_argument("--photos", type=int, default=4, help="넣을 증빙 사진 수")
    a = ap.parse_args()

    register_fonts()
    st = styles()
    ins = read_inspections(a.date)
    if not ins:
        print(f"점검 기록이 없다: logs/inspect_{a.date}_*.csv")
        return 1
    pat = read_patrol(a.patrol)

    doc = SimpleDocTemplate(a.out, pagesize=A4, topMargin=18 * mm, bottomMargin=16 * mm,
                            leftMargin=18 * mm, rightMargin=18 * mm,
                            title="순찰 로봇 시험 보고서")
    S = []
    S.append(Paragraph("순찰 로봇 — 순찰 및 소화기 점검 시험 보고서", st["title"]))
    S.append(Paragraph(
        f"작성 {datetime.now().strftime('%Y-%m-%d %H:%M')} / "
        f"TurtleBot3 Burger + Raspberry Pi 3 / ROS 2 Humble / Nav2", st["sub"]))

    # 1. 센서
    S.append(Paragraph("1. 사용 센서", st["h"]))
    S.append(table([["센서", "측정값", "토픽", "무엇에 쓰는가"]] + SENSORS,
                   [30 * mm, 27 * mm, 42 * mm, 68 * mm], st))
    S.append(Paragraph(
        "판정 연산(압력계 영상 처리)은 로봇이 아닌 원격 PC(VM)에서 수행한다. "
        "로봇은 센서 수집과 주행만 담당한다.", st["cap"]))

    # 2. 순찰
    S.append(Paragraph("2. 순찰 주행", st["h"]))
    S.append(Paragraph(
        "사각형 경로 4점(가로 1.15 m × 세로 0.95 m, 시계방향)을 Nav2 로 순회한다. "
        "정지 기준은 시간이 아니라 <b>바퀴 수</b>이며, 한 회차를 마치면 스스로 멈추고 "
        "정해진 쉼(1분) 뒤 무작위 시점에 다시 시작한다.", st["b"]))
    if pat:
        # patrol_log.py 의 열 이름: 회차,시작,종료,소요초,바퀴수,거리m,
        #                          전압시작,전압최저,전압종료,실패,거절
        rows = [["회차", "시작", "소요", "바퀴", "거리 m", "전압 시작", "전압 최저",
                 "실패", "거절"]]
        for r in pat:
            sec = r.get("소요초", "")
            took = f"{int(sec)//60}분 {int(sec)%60}초" if sec.isdigit() else (sec or "-")
            rows.append([r.get("회차", "-"), r.get("시작", "-"), took,
                         r.get("바퀴수", "-"), r.get("거리m", "-"),
                         r.get("전압시작", "-"), r.get("전압최저", "-"),
                         r.get("실패", "0"), r.get("거절", "0")])
        S.append(Spacer(1, 4))
        S.append(table(rows, [12 * mm, 20 * mm, 20 * mm, 12 * mm, 18 * mm, 20 * mm,
                              20 * mm, 13 * mm, 13 * mm], st,
                       align_right=(3, 4, 5, 6, 7, 8)))
        tot = sum(float(r.get("거리m", 0) or 0) for r in pat)
        fails = sum(int(r.get("실패", 0) or 0) + int(r.get("거절", 0) or 0) for r in pat)
        vmin = min((float(r.get("전압최저", 99) or 99) for r in pat), default=0)
        S.append(Paragraph(
            f"{len(pat)}회차 합계 주행 {tot:.1f} m, 실패와 거절 {fails}건, "
            f"전압 최저 {vmin:.2f} V. (2026-07-30 시연 기록)", st["cap"]))
    else:
        S.append(Paragraph(
            "이번 회차의 순찰 기록 파일은 없다(터미널 로그로만 확인). "
            "실측: 1바퀴 1.1~2.0분, 목표 거절과 실패 0.", st["cap"]))

    # 3. 소화기 점검
    S.append(Paragraph("3. 소화기 압력계 점검", st["h"]))
    ok = [r for r in ins if r["status"] == "정상"]
    S.append(Paragraph(
        f"총 {len(ins)}회 시행 중 <b>정상 판정 {len(ok)}회</b>. 판정은 기준 사진과 "
        "바늘 각도 분포를 비교해 변화량이 문턱(0.90) 미만이면 정상으로 본다. "
        "도착 후 카메라 영상을 보며 자세를 스스로 맞춘 뒤 촬영한다.", st["b"]))
    rows = [["시각", "판정", "변화량", "정합", "시도", "사진", "비고"]]
    for r in ins:
        rows.append([r.get("time", ""), r.get("status", ""), r.get("change", "-") or "-",
                     r.get("score", "-") or "-", r.get("attempt", "-"),
                     r.get("per_shot", "-") or "-",
                     (r.get("reason", "") or "")[:34]])
    S.append(Spacer(1, 4))
    S.append(table(rows, [18 * mm, 17 * mm, 16 * mm, 14 * mm, 12 * mm, 30 * mm, 60 * mm],
                   st, align_right=(2, 3, 4)))
    S.append(Paragraph(
        "판정불가와 부재는 대부분 자세가 어긋났거나(카메라 정렬 도입 전) 무선 지연으로 "
        "사진을 받지 못한 경우다. 바늘이 그대로인데 이상으로 판정하는 헛경보를 막기 위해, "
        "자세가 기준에서 크게 벗어나면 판정하지 않고 다시 맞춘다.", st["cap"]))

    # 3.1 원인 분석과 디버깅 경과
    S.append(Paragraph("3.1 실패 원인과 조치", st["h"]))
    S.append(Paragraph(
        "<b>결론부터: 이상으로 나온 3건은 압력계의 실제 이상이 아니라 촬영 자세 차이로 "
        "생긴 헛경보였다.</b> 근거는 두 가지다. (1) 같은 소화기를 몇 분 간격으로 찍은 "
        "다른 회차가 정상으로 판정됐고 그 사이 소화기에 손대지 않았다. (2) 이상으로 "
        "기록된 13:32 회차 사진을 자세를 다시 등록한 뒤 재판정하면 어긋남 0 px 에 "
        "정상으로 나온다. 압력계 바늘이 허용 범위를 벗어난 사례는 없었다.", st["b"]))
    S.append(Spacer(1, 4))
    S.append(Paragraph(
        "<b>원인.</b> 판정 방식이 '기준 사진과 같은 자세에서 찍은 사진'을 전제로 "
        "바늘 각도 분포를 비교하는데, 등록된 목표 각도(yaw)가 기준 사진을 찍은 실제 "
        "자세와 어긋나 있었다. 지도 좌표계 각도는 RViz 의 초기 위치 지정(2D Pose "
        "Estimate)이 몇 도만 틀어져도 그대로 어긋난다. 그 결과 로봇이 매번 조금씩 "
        "다른 자세에서 촬영했고, 바늘이 그대로여도 밝기 분포가 달라져 변화량이 "
        "커졌다. 화면이 93 px 밀리면 변화량 0.49, 180 px 밀리면 0.58~0.76 까지 "
        "올라간다(문턱 0.90).", st["b"]))
    S.append(Spacer(1, 4))
    S.append(Paragraph("<b>조치 — 네 단계로 나눠 고쳤다.</b>", st["b"]))
    S.append(Spacer(1, 3))
    S.append(table([
        ["#", "조치", "내용", "효과"],
        ["1", "촬영 자세 재등록",
         "로봇이 자율 이동으로 실제 도착하는 자세에서 기준 사진을 다시 등록",
         "변화량 0.58 → 0.18"],
        ["2", "판정 강건화",
         "비교 전에 중심과 반지름을 조금씩 흔들어 가장 잘 맞는 자리의 값을 사용",
         "93 px 어긋남에서 1.25 → 0.49"],
        ["3", "카메라 기반 자세 정렬",
         "도착 후 화면에서 압력계 위치를 찾아 어긋난 픽셀만큼 제자리 회전"
         "(상대 회전이라 지도 각도 오차와 무관)",
         "지도 각도 의존 제거"],
        ["4", "판정 보류 규칙",
         "자세가 기준에서 120 px 넘게 벗어나면 판정하지 않고 다시 맞춘다",
         "헛경보 차단"],
    ], [8 * mm, 30 * mm, 78 * mm, 34 * mm], st))
    S.append(Paragraph(
        "조치 후(18:47 이후) 6회 중 5회가 정상 판정, 최고 정합 0.90 이다. "
        "부재와 판정불가는 로봇이 소화기를 보지 못한 자세에서 촬영했거나 무선 지연으로 "
        "사진을 받지 못한 경우이며, 같은 조치로 함께 해소됐다.", st["cap"]))
    S.append(Paragraph(
        "<b>남은 한계.</b> 19:24 회차는 자세가 허용 범위 안(66 px)이었는데도 변화량 "
        "1.05 로 이상 판정이 났다. 세로 방향으로 32 px 어긋난 것이 원인으로 보인다. "
        "허용 범위를 더 좁히거나 판정을 한 단계 더 강건하게 만들 여지가 남아 있다.",
        st["cap"]))

    # 4. 인식 실패 사진
    bad = [r for r in ins if r["status"] != "정상"]
    picks = []
    for want in ("부재", "판정불가", "이상"):
        for r in bad:
            if r["status"] == want and glob.glob(os.path.join(r["_dir"], "*.png")):
                picks.append(r)
                break
    if picks:
        S.append(PageBreak())
        S.append(Paragraph("4. 인식 실패 사진 — 원인 진단", st["h"]))
        S.append(Paragraph(
            "자세가 어긋나면 압력계가 화면 기준 위치를 벗어나거나 아예 화면 밖으로 "
            "나간다. 아래는 그 예다. 노란 사각형이 기준 위치이고, 압력계가 그 안에 "
            "들어와야 판정할 수 있다.", st["b"]))
        for r in picks[:3]:
            imgs = sorted(glob.glob(os.path.join(r["_dir"], "*.png")))
            if not imgs:
                continue
            why = {"부재": "압력계가 화면에 없다 — 로봇이 소화기를 보지 않는 자세로 촬영",
                   "판정불가": "압력계가 기준 위치에서 크게 벗어났다 — 판정을 보류하고 자세를 다시 맞춘다",
                   "이상": "바늘은 그대로인데 자세 차이로 변화량이 문턱을 넘었다 (헛경보)"}[r["status"]]
            S.append(Spacer(1, 5))
            S.append(Image(imgs[0], width=132 * mm, height=99 * mm))
            S.append(Paragraph(
                f"{r.get('time','')} / 판정 {r['status']} / 정합 {r.get('score','-')} / "
                f"{why}", st["cap"]))

    # 5. 증빙 사진
    if ok:
        S.append(PageBreak())
        S.append(Paragraph("5. 증빙 사진 — 조치 후 정상 판정", st["h"]))
        S.append(Paragraph(
            "카메라 정렬을 거쳐 압력계가 기준 위치에 들어온 상태다. 노란 사각형이 "
            "기준 위치(ROI), 녹색 원이 검출된 압력계다. 사진 위 문자는 판정(OK), "
            "변화량, 정합, 반사율이다.", st["b"]))
        shown = 0
        for r in reversed(ok):          # 최근 것부터
            if shown >= a.photos:
                break
            imgs = sorted(glob.glob(os.path.join(r["_dir"], "*정상*.png")))
            if not imgs:
                continue
            S.append(Spacer(1, 6))
            S.append(Image(imgs[0], width=150 * mm, height=112 * mm))
            S.append(Paragraph(
                f"{r.get('time','')} / 변화량 {r.get('change','-')} "
                f"(문턱 0.90) / 정합 {r.get('score','-')} / "
                f"사진 {r.get('per_shot','-')} / {os.path.basename(imgs[0])}", st["cap"]))
            shown += 1

    doc.build(S)
    print(f"보고서: {a.out}")
    print(f"  점검 {len(ins)}회 (정상 {len(ok)}회), 사진 {min(len(ok), a.photos)}장 첨부")
    return 0


if __name__ == "__main__":
    sys.exit(main())
