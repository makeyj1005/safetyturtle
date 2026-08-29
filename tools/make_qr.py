#!/usr/bin/env python3
"""make_qr.py — 소화기 점검용 QR 코드를 만들어 인쇄용 PDF/PNG 로 저장한다.

  python3 ~/vibe/ex1/tools/make_qr.py FE-01 FE-02 --mm 100 --out ~/vibe/ex1/logs/qr

[왜 직접 만드나]
이 VM 에는 QR 생성 수단이 하나도 없다 — cv2.QRCodeEncoder 는 contrib 빌드에만 있고,
qrencode CLI 도 qrcode 모듈도 없고 pip 조차 설치되어 있지 않다(설치는 sudo 필요).
그래서 QR 부호화를 직접 구현했다. 대신 **만든 코드를 zbarimg 로 다시 읽어 원문과
일치하는지 확인**한 뒤에만 파일로 내보낸다. 인코더 버그로 못 읽는 QR 을 인쇄해
붙이는 사고를 막는 장치다. (판독에 cv2 를 쓰지 않는 이유는 decode() 주석 참고)

[범위]
버전 1 (21x21), 오류정정 M, 바이트 모드 — 페이로드 14바이트까지.
소화기 식별자("FE-01" 등)에는 충분하다. 더 긴 문자열이 필요하면 버전 2 이상과
정렬 패턴 처리가 필요하므로, 그때는 오류를 내고 멈춘다.

[인쇄]
PDF 는 실제 물리 크기를 지정해 만든다(기본 100mm). 프린터 설정에서 '실제 크기'
또는 배율 100% 로 인쇄해야 크기가 맞는다. '용지에 맞춤'을 쓰면 작아진다.
"""
import argparse
import os
import subprocess
import tempfile
import sys

import cv2
import numpy as np

# ---------------- 갈루아 체 GF(256) — 리드-솔로몬 오류정정용 ----------------
GF_EXP = [0] * 512
GF_LOG = [0] * 256
_x = 1
for _i in range(255):
    GF_EXP[_i] = _x
    GF_LOG[_x] = _i
    _x <<= 1
    if _x & 0x100:          # 원시 다항식 x^8+x^4+x^3+x^2+1
        _x ^= 0x11D
for _i in range(255, 512):
    GF_EXP[_i] = GF_EXP[_i - 255]


def gmul(a, b):
    if a == 0 or b == 0:
        return 0
    return GF_EXP[GF_LOG[a] + GF_LOG[b]]


def poly_mul(a, b):
    r = [0] * (len(a) + len(b) - 1)
    for i, ai in enumerate(a):
        for j, bj in enumerate(b):
            r[i + j] ^= gmul(ai, bj)
    return r


def rs_ecc(data, n_ec):
    """데이터 코드워드에 대한 오류정정 코드워드 n_ec 개를 만든다."""
    g = [1]
    for i in range(n_ec):
        g = poly_mul(g, [1, GF_EXP[i]])
    msg = list(data) + [0] * n_ec
    for i in range(len(data)):
        c = msg[i]
        if c:
            for j, gj in enumerate(g):
                msg[i + j] ^= gmul(gj, c)
    return msg[len(data):]


# ---------------- 버전 1 / EC M 상수 ----------------
SIZE = 21
DATA_CW = 16          # 데이터 코드워드 수
EC_CW = 10            # 오류정정 코드워드 수
MAX_BYTES = 14        # 바이트 모드 최대 페이로드 (16*8 - 4(모드) - 8(길이) = 116비트)
EC_M = 0b00           # 오류정정 레벨 M


def bitstream(payload: bytes):
    bits = []

    def put(val, n):
        for i in range(n - 1, -1, -1):
            bits.append((val >> i) & 1)

    put(0b0100, 4)              # 모드: 바이트
    put(len(payload), 8)        # 길이 (버전 1~9 는 8비트)
    for b in payload:
        put(b, 8)
    put(0, min(4, DATA_CW * 8 - len(bits)))        # 종료자
    while len(bits) % 8:                           # 바이트 경계 맞춤
        bits.append(0)
    pad = [0xEC, 0x11]
    i = 0
    while len(bits) < DATA_CW * 8:                 # 규격이 정한 채움 바이트
        put(pad[i % 2], 8)
        i += 1
    return bits


def codewords(payload: bytes):
    bits = bitstream(payload)
    data = [int("".join(str(b) for b in bits[i:i + 8]), 2) for i in range(0, len(bits), 8)]
    return data + rs_ecc(data, EC_CW)


# ---------------- 기능 패턴 ----------------
def function_matrix():
    """모듈 값과 '여기는 데이터가 아니다' 표시를 만든다."""
    m = [[None] * SIZE for _ in range(SIZE)]
    reserved = [[False] * SIZE for _ in range(SIZE)]

    def finder(r0, c0):
        for dr in range(-1, 8):
            for dc in range(-1, 8):
                r, c = r0 + dr, c0 + dc
                if not (0 <= r < SIZE and 0 <= c < SIZE):
                    continue
                reserved[r][c] = True
                inside = 0 <= dr <= 6 and 0 <= dc <= 6
                if not inside:
                    m[r][c] = 0                     # 분리자(흰 테두리)
                else:
                    # 7x7 핀더: 가장 바깥 검은 테두리(3) / 흰 링(2) / 가운데 3x3 검정(0,1)
                    ring = max(abs(dr - 3), abs(dc - 3))
                    m[r][c] = 1 if ring in (0, 1, 3) else 0

    finder(0, 0)
    finder(0, SIZE - 7)
    finder(SIZE - 7, 0)

    for i in range(SIZE):                           # 타이밍 패턴
        if m[6][i] is None:
            m[6][i] = 1 if i % 2 == 0 else 0
            reserved[6][i] = True
        if m[i][6] is None:
            m[i][6] = 1 if i % 2 == 0 else 0
            reserved[i][6] = True

    m[SIZE - 8][8] = 1                              # 항상 검은 모듈
    reserved[SIZE - 8][8] = True

    for r, c in FORMAT_POS_1 + FORMAT_POS_2:        # 형식 정보 자리 예약
        reserved[r][c] = True
    return m, reserved


# 형식 정보 15비트가 놓이는 좌표 (비트 14 = MSB 부터 순서대로)
FORMAT_POS_1 = [(8, 0), (8, 1), (8, 2), (8, 3), (8, 4), (8, 5), (8, 7), (8, 8),
                (7, 8), (5, 8), (4, 8), (3, 8), (2, 8), (1, 8), (0, 8)]
FORMAT_POS_2 = [(20, 8), (19, 8), (18, 8), (17, 8), (16, 8), (15, 8), (14, 8),
                (8, 13), (8, 14), (8, 15), (8, 16), (8, 17), (8, 18), (8, 19), (8, 20)]


def format_bits(mask):
    """오류정정 레벨 + 마스크 번호에 BCH(15,5) 를 붙이고 규격 마스크로 XOR 한다."""
    data = (EC_M << 3) | mask
    v = data << 10
    for i in range(4, -1, -1):
        if v & (1 << (i + 10)):
            v ^= 0b10100110111 << i
    return ((data << 10) | v) ^ 0b101010000010010


MASK_FN = [
    lambda r, c: (r + c) % 2 == 0,
    lambda r, c: r % 2 == 0,
    lambda r, c: c % 3 == 0,
    lambda r, c: (r + c) % 3 == 0,
    lambda r, c: (r // 2 + c // 3) % 2 == 0,
    lambda r, c: (r * c) % 2 + (r * c) % 3 == 0,
    lambda r, c: ((r * c) % 2 + (r * c) % 3) % 2 == 0,
    lambda r, c: ((r + c) % 2 + (r * c) % 3) % 2 == 0,
]


def build(payload: bytes, mask: int):
    m, reserved = function_matrix()
    bits = []
    for cw in codewords(payload):
        for i in range(7, -1, -1):
            bits.append((cw >> i) & 1)

    # 오른쪽 아래부터 두 열씩 지그재그로 채운다
    idx = 0
    col = SIZE - 1
    upward = True
    while col > 0:
        if col == 6:                 # 타이밍 열은 건너뛴다
            col -= 1
        rows = range(SIZE - 1, -1, -1) if upward else range(SIZE)
        for r in rows:
            for c in (col, col - 1):
                if reserved[r][c] or m[r][c] is not None:
                    continue
                bit = bits[idx] if idx < len(bits) else 0
                idx += 1
                if MASK_FN[mask](r, c):
                    bit ^= 1
                m[r][c] = bit
        col -= 2
        upward = not upward

    fmt = format_bits(mask)
    for i, (r, c) in enumerate(FORMAT_POS_1):
        m[r][c] = (fmt >> (14 - i)) & 1
    for i, (r, c) in enumerate(FORMAT_POS_2):
        m[r][c] = (fmt >> (14 - i)) & 1
    m[SIZE - 8][8] = 1

    return np.array([[0 if v else 255 for v in row] for row in m], dtype=np.uint8)


def render(mat, scale=8, quiet=4):
    """모듈 행렬을 이미지로. quiet 은 규격이 요구하는 흰 여백(모듈 단위)."""
    n = mat.shape[0]
    img = np.full(((n + quiet * 2) * scale, (n + quiet * 2) * scale), 255, np.uint8)
    big = np.kron(mat, np.ones((scale, scale), np.uint8))
    img[quiet * scale:quiet * scale + big.shape[0],
        quiet * scale:quiet * scale + big.shape[1]] = big
    return img


def decode(img):
    """zbarimg 로 판독한다.

    cv2.QRCodeDetector 를 쓰지 않는 이유: 이 시스템의 OpenCV 는 QUIRC 가 링크되지
    않아 QR 을 '탐지'만 하고 '판독'은 못 한다("Library QUIRC is not linked").
    그래서 판독은 zbar 로 한다 (sudo apt install zbar-tools).
    """
    path = os.path.join(tempfile.gettempdir(), f"_qrchk_{os.getpid()}.png")
    cv2.imwrite(path, img)
    try:
        r = subprocess.run(["zbarimg", "--quiet", "--raw", path],
                           capture_output=True, text=True, timeout=15)
        return r.stdout.strip()
    finally:
        if os.path.exists(path):
            os.remove(path)


def make_verified(text):
    """마스크를 바꿔가며 만들고, 디코더가 원문을 읽어내는 것만 반환한다."""
    payload = text.encode("iso-8859-1")
    if len(payload) > MAX_BYTES:
        raise SystemExit(f"'{text}' 는 {len(payload)}바이트다. 버전 1 최대 {MAX_BYTES}바이트를 넘는다")
    for mask in range(8):
        mat = build(payload, mask)
        if decode(render(mat, scale=10)) == text:
            return mat, mask
    raise SystemExit(f"'{text}' QR 을 만들었지만 디코더가 읽지 못했다 — 인코더를 점검할 것")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("texts", nargs="+", help="QR 내용 (예: FE-01 FE-02)")
    ap.add_argument("--mm", type=float, default=100.0, help="인쇄 시 QR 한 변 길이(mm)")
    ap.add_argument("--out", default=os.path.expanduser("~/vibe/ex1/logs/qr"),
                    help="출력 경로(확장자 없이)")
    args = ap.parse_args()

    out = os.path.expanduser(args.out)
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)

    made = []
    for t in args.texts:
        mat, mask = make_verified(t)
        png = f"{out}_{t}.png"
        cv2.imwrite(png, render(mat, scale=20))
        made.append((t, mat, png))
        print(f"  {t}: 검증 통과 (마스크 {mask}) -> {png}")

    # 인쇄용 PDF — 물리 크기를 지정한다
    from reportlab.lib.pagesizes import A4
    from reportlab.lib.units import mm as MM
    from reportlab.lib.utils import ImageReader
    from reportlab.pdfgen import canvas

    pdf = out + ".pdf"
    c = canvas.Canvas(pdf, pagesize=A4)
    W, H = A4
    for t, mat, png in made:
        side = args.mm * MM
        x = (W - side) / 2
        y = (H - side) / 2 + 10 * MM
        c.drawImage(ImageReader(png), x, y, side, side)
        c.setFont("Helvetica-Bold", 16)
        c.drawCentredString(W / 2, y - 12 * MM, t)
        c.setFont("Helvetica", 9)
        c.drawCentredString(W / 2, y - 18 * MM,
                            f"QR {args.mm:.0f} x {args.mm:.0f} mm  /  print at 100% scale")
        c.showPage()
    c.save()
    print(f"인쇄용 PDF: {pdf}  ({len(made)}장, 각 {args.mm:.0f}mm)")


if __name__ == "__main__":
    sys.exit(main())
