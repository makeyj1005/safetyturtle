#!/usr/bin/env python3
"""grab_shot.py — 로봇에서 고해상도 프레임을 '로컬 저장' 시킨 뒤 파일로 가져온다.

[VM 에서 실행]
  python3 ~/vibe/ex1/tools/grab_shot.py --host 192.168.0.67
  python3 ~/vibe/ex1/tools/grab_shot.py --n 3 --host 192.168.0.67 --camera 1640x1232
  python3 ~/vibe/ex1/tools/grab_shot.py --judge 소화기1        # 가져오면서 압력계 판정까지

[왜 스트리밍이 아니라 파일로 가져오나 — 2026-07-31 실측]
1640x1232 프레임은 120~180KB 다. 무선으로 DDS 로 보내면:
  best_effort  20초에 0장   (조각 하나 잃으면 프레임 전체를 버린다)
  reliable     20초에 13장  (0.65fps, 최대 5.5초 공백 — 재전송으로 대역폭을 다 쓴다)
반면 로봇 안에서는 2.6fps 로 멀쩡히 흐른다. 그래서 판정용 프레임은 로봇에서 로컬로
저장하고 가져온다. 점검은 정지 상태에서 몇 장만 필요하므로 스트리밍일 필요가 없다.

조준용 실시간 화면은 live_view.py 로 640x480 저해상도를 쓰면 부드럽다.

[scp 버그를 어떻게 고쳤나 — 2026-07-31]
이전 판은 `scp host:/tmp/grab/*.jpg` 로 받았다. 원격 글로브 확장에 의존하는 방식이라
직접 셸에서는 되는데 도구에서는 실패했고, 실패 이유가 stderr 한 줄로만 남아 원인을
찾기 어려웠다. 지금은 ssh 한 번으로
  stdout = tar 스트림(바이너리),  stderr = 진행 로그(몇 장 저장했는지)
를 나눠 받는다. 글로브가 없고 연결도 한 번이다. 취득 코드는 inspect_node 와 공유한다
(patrol_core/shot_grab.py) — 점검 노드와 손으로 찍는 도구가 같은 사진을 얻어야 한다.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.expanduser("~"), "vibe", "ex1",
                                "ros2_ws", "src", "patrol_core"))

import cv2                                     # noqa: E402
from patrol_core import gauge as G             # noqa: E402
from patrol_core import shot_grab as SG        # noqa: E402

EX1 = os.path.join(os.path.expanduser("~"), "vibe", "ex1")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="rpi@192.168.0.67")
    ap.add_argument("--n", type=int, default=3, help="가져올 프레임 수")
    ap.add_argument("--topic", default="/camera/image_raw/compressed")
    ap.add_argument("--out", default=os.path.join(EX1, "logs", "shots"))
    ap.add_argument("--domain", default="3", help="ROS_DOMAIN_ID")
    ap.add_argument("--wait", type=float, default=15.0, help="프레임 대기 한도(초)")
    ap.add_argument("--camera", default="",
                    help="찍는 동안만 로봇에서 카메라를 띄운다. 예: 1640x1232 "
                         "(비우면 이미 떠 있는 카메라를 구독한다)")
    ap.add_argument("--camera-index", type=int, default=0,
                    help="CSI 인덱스. USB 웹캠을 꽂으면 1, 빼면 0 (함정 9)")
    ap.add_argument("--fps", type=float, default=3.0)
    ap.add_argument("--jpeg", type=int, default=85)
    ap.add_argument("--judge", default="", help="이 지점 이름의 기준으로 압력계를 판정한다")
    ap.add_argument("--calib", default=os.path.join(EX1, "maps", "gauge_calib.yaml"))
    args = ap.parse_args()

    cam = None
    if args.camera:
        try:
            w, h = (int(v) for v in args.camera.lower().split("x"))
        except ValueError:
            print(f"--camera 형식이 잘못됐다: {args.camera} (예: 1640x1232)")
            return 1
        cam = SG.camera_args(w, h, fps=args.fps, jpeg_quality=args.jpeg,
                             index=args.camera_index)
        print(f"카메라를 찍는 동안만 띄운다: {w}x{h} fps={args.fps} jpeg={args.jpeg}")

    files, log = SG.grab(args.host, args.out, n=args.n, topic=args.topic,
                         domain=args.domain, camera=cam, wait_sec=args.wait)
    if log:
        print(log)
    if not files:
        print("사진을 가져오지 못했다. 카메라가 떠 있는지 / 로봇 IP 가 맞는지 확인할 것")
        return 1
    print(f"가져옴: {len(files)}장 -> {args.out}")

    cal = ref = None
    if args.judge:
        points = G.load_calib(args.calib) if os.path.exists(args.calib) else {}
        cal = points.get(args.judge)
        if cal is None:
            print(f"'{args.judge}' 의 캘리브레이션이 없다: {args.calib}")
            return 1
        ref = G.load_ref(args.calib, args.judge)

    print(f"\n{'파일':<24}{'해상도':>12}{'판정':>10}{'변화량':>9}{'정합':>7}{'반사':>7}")
    for p in files:
        img = cv2.imread(p)
        if img is None:
            continue
        line = f"{os.path.basename(p):<24}{img.shape[1]}x{img.shape[0]:<7}"
        if cal is None:
            print(line)
            continue
        r = G.judge(img, cal, ref)

        def num(key, fmt):
            v = r.get(key)
            return "-" if v is None else format(v, fmt)

        print(line + f"{r['status']:>10}{num('change', '.2f'):>9}"
              f"{num('score', '.2f'):>7}{num('reflection', '.0f'):>7}")
        if r["status"] != "정상":
            print(f"    {r['reason']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
