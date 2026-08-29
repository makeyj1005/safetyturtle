#!/usr/bin/env python3
"""aim_gauge.py — teleop 으로 로봇을 몰 때, 게이지가 기준 자리에 오도록 수치로 안내한다.

[VM 에서 실행]  (Nav2 가 떠 있어야 좌표 저장이 가능하다. patrol_auto 는 꺼둘 것)
  python3 ~/vibe/ex1/tools/aim_gauge.py --name 소화기1 --host 192.168.0.67

  카메라를 이미 띄워뒀다면:
  python3 ~/vibe/ex1/tools/aim_gauge.py --name 소화기1 --no-camera

[무엇을 하나]
몇 초마다 로봇에서 사진을 한 장 받아, 기준 패치(gauge_ref_<이름>.png)를 화면 전체에서
찾고 **기준 자리에서 몇 px 벗어났는지**를 보여준다. 눈대중이 아니라 수치로 맞출 수 있다.

  dx  +480px  ->  게이지가 오른쪽으로 밀렸다
  dy   -20px  ->  살짝 위
  배율  1.10  ->  기준보다 10% 크다 = 조금 가깝다

[왜 필요한가 — 2026-08-01]
자율 이동으로 저장된 좌표에 정확히(오차 5mm) 갔는데도 게이지가 480px 밀려 있었다.
`fire_extinguisher_points.yaml` 의 yaw 가 **기준 사진을 찍은 자세와 다르다**는 뜻이다.
점검 노드의 보정 루프가 매번 그걸 따라잡을 수는 있지만 성공률이 떨어진다(계통오차
20°면 60~75%). 자세를 제대로 다시 등록하면 첫 시도에 판정된다(96~100%).

[회전 방향은 이 도구가 배운다]
카메라가 로봇 **후면**에 달려 있어 "왼쪽으로 돌리면 화면이 어디로 가는지"를 머리로
따지면 틀린다(실제로 두 번 틀렸다). 그래서 /odom 의 yaw 를 같이 읽어, 당신이 조금
돌릴 때마다 **1도에 화면이 몇 px 움직이는지(부호 포함)** 를 스스로 잰다. 재고 나면
"왼쪽으로 3.2도" 처럼 방향과 양을 같이 알려준다.

/odom 을 쓰는 이유: AMCL 은 11.5° 이상 돌아야 갱신돼서(update_min_a) 미세 조준에는
못 쓴다. 짧은 조준 시간 동안 odom 의 각도 드리프트는 무시할 수 있다.

[다 맞추면]
"저장할 준비가 됐다"가 뜨면 그 자리에서 좌표를 덮어쓴다(저장 전 제자리 회전
두세 번 — AMCL 갱신 + 방향 수렴, HANDOFF 함정 10):

  python3 ~/vibe/ex1/tools/save_waypoint.py --name 소화기1 \\
      --file ~/vibe/ex1/maps/fire_extinguisher_points.yaml
"""
import argparse
import csv
import math
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.join(os.path.expanduser("~"), "vibe", "ex1",
                                "ros2_ws", "src", "patrol_core"))

import cv2                                     # noqa: E402
import rclpy                                   # noqa: E402
from nav_msgs.msg import Odometry              # noqa: E402
from patrol_core import gauge as G             # noqa: E402
from patrol_core import shot_grab as SG        # noqa: E402
from rclpy.node import Node                    # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402

EX1 = os.path.join(os.path.expanduser("~"), "vibe", "ex1")
CALIB = os.path.join(EX1, "maps", "gauge_calib.yaml")
# 이 안에 들면 자세가 맞은 것으로 본다. gauge.judge 의 판정 문턱과 같은 값을 쓰되
# 여유를 두려고 절반으로 잡는다 — 등록은 한 번이고, 정확할수록 이후가 편하다.
OK_PX = 70
# 배율(거리)이 이 범위를 벗어나면 앞뒤로 움직이라고 안내한다. 기준 대비 ±12%.
OK_SCALE = (0.88, 1.12)
SCALES = [0.6, 0.7, 0.8, 0.85, 0.9, 0.95, 1.0, 1.05, 1.1, 1.2, 1.3, 1.45]


def yaw_of(q):
    return math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)


class OdomYaw(Node):
    """/odom 의 yaw 만 읽는다. 사진을 찍은 순간의 각도를 알기 위해서다."""

    def __init__(self):
        super().__init__("aim_gauge")
        self.yaw = None
        self.create_subscription(Odometry, "/odom", self.cb, qos_profile_sensor_data)

    def cb(self, m):
        self.yaw = yaw_of(m.pose.pose.orientation)

    def read(self, timeout=2.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.yaw is not None:
                return math.degrees(self.yaw)
        return None


def find(img, ref, cal):
    """기준 패치를 화면 전체에서 찾는다. 반환 (score, cx, cy, scale)."""
    best = None
    for s in SCALES:
        t = cv2.resize(ref, None, fx=s, fy=s,
                       interpolation=cv2.INTER_AREA if s < 1 else cv2.INTER_CUBIC)
        if t.shape[0] >= img.shape[0] or t.shape[1] >= img.shape[1]:
            continue
        _, sc, _, loc = cv2.minMaxLoc(cv2.matchTemplate(img, t, cv2.TM_CCOEFF_NORMED))
        if best is None or sc > best[0]:
            best = (sc, loc[0] + t.shape[1] / 2.0, loc[1] + t.shape[0] / 2.0, s)
    return best


def ssh(host, cmd, timeout=40):
    # 사진 취득과 같은 통로를 쓴다(SG.SSH_OPTS = ControlMaster). 접속 왕복을 아낀다.
    return subprocess.run(["ssh", *SG.SSH_OPTS, host, cmd],
                          capture_output=True, text=True, timeout=timeout)


def stop_camera(host, tries=3):
    """카메라를 확실히 없앤다. 남으면 다음 실행이 장치를 못 잡는다.

    ⚠️ 매달린(hung) 카메라는 SIGTERM 으로 안 죽는다 — 실측: futex 에서 멈춘 채
    25분간 /dev/media0 을 붙들고 있었고, 그 동안 프레임은 한 장도 안 나왔다.
    새로 띄운 카메라는 `Pipeline handler in use by another process` 로 죽었다.
    그래서 SIGTERM -> SIGKILL 순서로 확인하며 없앤다.
    """
    for i in range(tries):
        try:
            r = ssh(host, "pkill -f '[c]amera_ros/camera_node'; sleep 2; "
                          "pkill -9 -f '[c]amera_ros/camera_node'; sleep 1; "
                          "pgrep -f '[c]amera_ros/camera_node' >/dev/null "
                          "&& echo LEFT || echo GONE", timeout=45)
            if "GONE" in r.stdout:
                return True
        except subprocess.SubprocessError:
            pass                        # 무선이 끊겼을 수 있다. 다시 시도한다
        time.sleep(2)
    return False


def start_camera(host, w, h, fps, jpeg, index):
    """카메라를 계속 켜둔다. 매번 켰다 끄면 한 장에 35초씩 걸려 조준을 못 한다.

    index 는 CSI 가 1, USB 웹캠이 0 이다(USB 를 꽂으면 libcamera 가 카메라를 두 대로
    보게 되어 인덱스가 밀린다 — HANDOFF 함정 9). 이 도구는 CSI(1)가 기본이다.

    먼저 남아있는 카메라를 없앤다 — 장치를 하나만 잡을 수 있어서, 묵은 게 있으면
    새 카메라가 그냥 죽는다.
    """
    if not stop_camera(host):
        print("  ⚠️ 로봇에 남아있는 카메라를 없애지 못했다. 무선 상태를 확인할 것")
        return False
    opts = " ".join(f"{k}:={v}" for k, v in
                    SG.camera_args(w, h, fps=fps, jpeg_quality=jpeg, index=index).items())
    cmd = (f"source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=3; "
           f"setsid nohup ros2 launch {SG.REMOTE_CAM_LAUNCH} {opts} "
           f">/tmp/aim_cam.log 2>&1 < /dev/null & echo started")
    try:
        r = ssh(host, cmd)
    except subprocess.SubprocessError as e:
        print(f"  카메라 실행 실패: {e}")
        return False
    return "started" in r.stdout


def camera_died(host):
    """카메라가 죽었는지 로그로 확인한다. 이유까지 돌려준다."""
    try:
        r = ssh(host, "pgrep -f '[c]amera_ros/camera_node' >/dev/null && echo ALIVE "
                      "|| (echo DEAD; tail -3 /tmp/aim_cam.log 2>/dev/null)", timeout=30)
    except subprocess.SubprocessError:
        return None, "로봇 응답 없음"
    if r.returncode != 0:
        # ssh 자체가 실패했다. 카메라 상태는 알 수 없다 — 카메라 탓으로 몰면 안 된다.
        return None, f"로봇에 접속하지 못했다: {r.stderr.strip()[-120:]}"
    if "ALIVE" in r.stdout:
        return False, ""
    return True, r.stdout.replace("DEAD", "").strip()[-200:]


def guide(dx, dy, scale, k):
    """사람이 바로 따라할 수 있는 문장으로 바꾼다."""
    out = []
    if abs(dx) <= OK_PX:
        out.append("좌우 OK")
    elif k is None:
        # 아직 1도에 몇 px 인지 모른다. 방향을 알려주지 않고 조금 돌려보게 한다.
        out.append(f"좌우 {dx:+.0f}px — 아무 쪽으로 5도쯤 돌려보라(방향을 재는 중)")
    else:
        need = -dx / k          # k 의 부호 안에 회전 방향이 들어 있다
        out.append(f"{'왼쪽' if need > 0 else '오른쪽'}으로 {abs(need):.1f}도")
    if abs(dy) > OK_PX:
        out.append(f"{'카메라가 높다(뒤로)' if dy > 0 else '카메라가 낮다(앞으로)'} {dy:+.0f}px")
    if scale < OK_SCALE[0]:
        out.append(f"조금 앞으로 (배율 {scale:.2f})")
    elif scale > OK_SCALE[1]:
        out.append(f"조금 뒤로 (배율 {scale:.2f})")
    return " / ".join(out)


def summarize(rows, out_dir):
    """각도 훑기 결과를 CSV 로 남기고, 두 가지 수치를 뽑아준다.

    ① 1도에 화면이 몇 px 움직이는가(부호 포함) — 보정 계산의 근거
    ② 몇 px 틀어지면 판정이 무너지는가 — max_offset_px 의 근거
       (판정법이 "같은 자세" 전제라, 틀어지면 바늘이 그대로인데도 변화량이 커진다)
    """
    path = os.path.join(out_dir, "sweep.csv")
    with open(path, "w", newline="") as f:
        wcsv = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        wcsv.writeheader()
        wcsv.writerows(rows)
    print(f"\n기록 {len(rows)}장 -> {path}")

    pts = [(float(r["yaw_odom"]), float(r["dx"])) for r in rows if r["yaw_odom"]]
    if len(pts) >= 3:
        ys = [p[0] for p in pts]; xs = [p[1] for p in pts]
        my, mx = sum(ys) / len(ys), sum(xs) / len(xs)
        den = sum((y - my) ** 2 for y in ys)
        if den > 1e-6:
            k = sum((y - my) * (x - mx) for y, x in pts) / den
            span = max(ys) - min(ys)
            print(f"① 1도당 화면 이동: {k:+.1f} px/도   "
                  f"(각도 {span:.1f}° 범위, {len(pts)}점에서 맞춤)")
            print(f"   -> gauge.py 의 px_per_deg 를 {abs(k):.0f} 로,"
                  f" 부호는 {'그대로' if k < 0 else '반대로'} 두면 된다")

    ok = [r for r in rows if r["change"] and float(r["change"]) < 0.9]
    bad = [r for r in rows if r["change"] and float(r["change"]) >= 0.9]
    if ok and bad:
        lim = max(abs(float(r["dx"])) for r in ok)
        first = min(abs(float(r["dx"])) for r in bad)
        print(f"② 판정이 무너지는 경계: 정상 유지 최대 {lim:.0f}px / "
              f"헛경보 최소 {first:.0f}px")
        print(f"   -> max_offset_px 는 {min(lim, first) * 0.7:.0f} 쯤이 안전하다"
              f" (지금 {G.DEF['max_offset_px']})")
    elif ok:
        print(f"② 훑은 범위(최대 {max(abs(float(r['dx'])) for r in ok):.0f}px) 안에서는 "
              f"판정이 무너지지 않았다 — 더 크게 틀어 봐야 경계를 안다")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default="소화기1", help="캘리브레이션에 등록된 지점 이름")
    ap.add_argument("--host", default="192.168.0.67")
    ap.add_argument("--calib", default=CALIB)
    ap.add_argument("--no-camera", action="store_true",
                    help="카메라를 이미 띄워뒀다(켜고 끄지 않는다)")
    ap.add_argument("--fps", type=float, default=3.0)
    ap.add_argument("--jpeg", type=int, default=85)
    ap.add_argument("--camera-index", type=int, default=0,
                    help="CSI 인덱스. USB 웹캠을 꽂으면 1, 빼면 0 (함정 9)")
    ap.add_argument("--interval", type=float, default=1.0, help="사진 사이 쉬는 시간(초)")
    ap.add_argument("--save-dir", default=os.path.join(EX1, "logs", "aim"))
    ap.add_argument("--record", action="store_true",
                    help="각도별로 사진과 측정값을 남긴다(자세 허용범위 조사용)")
    args = ap.parse_args()
    host = args.host if "@" in args.host else f"rpi@{args.host}"

    points = G.load_calib(args.calib) if os.path.exists(args.calib) else {}
    cal = points.get(args.name)
    if cal is None:
        print(f"'{args.name}' 의 캘리브레이션이 없다: {args.calib}")
        return 1
    ref = G.load_ref(args.calib, args.name)
    if ref is None:
        print(f"기준 패치가 없다: {G.ref_path(args.calib, args.name)}")
        return 1
    w, h = cal.get("resolution", [1640, 1232])
    x, y, rw, rh = [float(v) for v in cal["roi"]]
    ex, ey = x + rw / 2, y + rh / 2        # 기준 자세에서 게이지가 있던 자리

    print(f"지점 {args.name} — 기준 자리 ({ex:.0f}, {ey:.0f}), 해상도 {w}x{h}")
    if not args.no_camera:
        print("로봇 카메라를 켠다 (계속 켜둔다. 끝내면 자동으로 끈다)...")
        if not start_camera(host, w, h, args.fps, args.jpeg, args.camera_index):
            print("카메라를 켜지 못했다. ssh 와 로봇 상태를 확인할 것")
            return 1
        time.sleep(9)
        # 정말 프레임이 나오는지 받아본다. 여기서 걸러야 조준하다 헤매지 않는다.
        # ⚠️ 무선이 끊겼다 붙었다 하므로 한 번 실패했다고 포기하지 않는다(실측: 손실이
        #    0%와 20% 사이를 오간다). 네트워크 문제와 카메라 문제를 구분해 안내한다.
        ok = False
        for attempt in range(1, 4):
            probe, plog = SG.grab(host, args.save_dir, n=1, camera=None, wait_sec=10.0,
                                  prefix="probe")
            if probe:
                os.remove(probe[0])
                print("카메라 확인 완료 — 프레임이 들어온다")
                ok = True
                break
            net = any(w in plog for w in ("connect to host", "ssh 시간초과",
                                          "Connection timed out", "ssh 실행 실패",
                                          "로봇 응답 없음"))
            if net:
                print(f"  [{attempt}/3] 로봇에 닿지 않는다(무선). 다시 시도한다 — "
                      f"{plog[:70]}")
                time.sleep(5)
                continue
            dead, why = camera_died(host)
            if dead is None:
                print(f"  [{attempt}/3] {why} — 다시 시도한다")
                time.sleep(5)
                continue
            if dead:
                print(f"  카메라가 죽었다: {why}")
                print("  USB 카메라를 꽂았다면 인덱스가 밀린다 -> --camera-index 0")
                break
            print(f"  [{attempt}/3] 카메라는 떠 있는데 프레임이 안 온다 — 다시 시도한다")
            time.sleep(5)
        if not ok:
            print("프레임을 받지 못했다. 무선 상태(ping)와 로봇 카메라를 확인할 것")
            stop_camera(host)
            return 1

    rclpy.init()
    node = OdomYaw()
    prev = None            # (dx, yaw) — 1도에 몇 px 인지 재기 위한 직전 값
    fails = 0              # 연달아 사진을 못 받은 횟수
    k = None               # px/도 (부호 포함)
    n = 0
    os.makedirs(args.save_dir, exist_ok=True)
    rows = []
    sweep_dir = os.path.join(args.save_dir, time.strftime("sweep_%m%d_%H%M%S"))
    if args.record:
        os.makedirs(sweep_dir, exist_ok=True)
        print(f"기록 모드 — 사진과 측정값을 {sweep_dir} 에 남긴다.")
        print("자세를 맞춘 뒤 좌우로 천천히 돌려라(±15도쯤). Ctrl+C 로 끝내면 정리해준다.")
    print("\nteleop 으로 움직여라. Ctrl+C 로 끝낸다.\n")
    try:
        while True:
            yaw0 = node.read()
            files, log = SG.grab(host, args.save_dir, n=1, camera=None, wait_sec=8.0,
                                 prefix="aim")
            if not files:
                fails += 1
                print(f"  사진 실패({fails}) — {log[:90]}")
                # 두 번 연달아 실패하면 카메라가 죽었거나 매달린 것이다. 다시 띄운다.
                if fails >= 2 and not args.no_camera:
                    dead, why = camera_died(host)
                    if dead:
                        print(f"  카메라가 죽어 있다: {why}")
                    print("  카메라를 다시 띄운다...")
                    if start_camera(host, w, h, args.fps, args.jpeg, args.camera_index):
                        time.sleep(9)
                        fails = 0
                    else:
                        print("  다시 띄우지 못했다. 무선/로봇 상태를 확인할 것")
                        time.sleep(5)
                time.sleep(args.interval)
                continue
            fails = 0
            img = cv2.imread(files[0])
            keep = files[0]
            if args.record:
                keep = os.path.join(sweep_dir, f"s{n + 1:03d}.jpg")
                os.replace(files[0], keep)
            else:
                os.remove(files[0])
            n += 1
            if img is None or (img.shape[1], img.shape[0]) != (int(w), int(h)):
                print(f"  해상도가 다르다: {None if img is None else img.shape} — "
                      f"카메라를 {w}x{h} 로 띄워야 한다")
                time.sleep(args.interval)
                continue

            sc, cx, cy, scale = find(img, ref, cal)
            dx, dy = cx - ex, cy - ey
            yaw1 = node.read()

            # 사진과 각도의 짝을 맞춘다. 한 장 받는 데 8초가 걸리므로, 그 사이에
            # 로봇이 움직였으면 "이 사진을 찍을 때의 각도"를 알 수 없다.
            # (실측 2026-08-01: 이걸 안 맞춰서 1도당 8px 이라는 엉뚱한 값이 나왔다.
            #  기하학적으로 26px/도쯤 이어야 한다.)
            still = (yaw0 is not None and yaw1 is not None
                     and abs((yaw1 - yaw0 + 180) % 360 - 180) <= 1.0)
            yaw_at_shot = (yaw0 + yaw1) / 2.0 if still else None

            # 직전 프레임과 비교해 1도에 몇 px 움직였는지 잰다(부호 포함).
            if prev is not None and yaw_at_shot is not None:
                d_yaw = (yaw_at_shot - prev[1] + 180) % 360 - 180
                if abs(d_yaw) >= 2.0:
                    cand = (dx - prev[0]) / d_yaw
                    if 8.0 <= abs(cand) <= 150.0:
                        k = cand
                        print(f"    [측정] {d_yaw:+.1f}도 돌렸더니 화면이 "
                              f"{dx - prev[0]:+.0f}px 움직였다 -> 1도당 {k:+.0f}px")
            if yaw_at_shot is not None:
                prev = (dx, yaw_at_shot)
            elif prev is not None and yaw1 is not None:
                # 움직이는 중이었다. 이 사진은 각도 재기에 쓰지 않지만, 다음 비교의
                # 기준으로도 쓰면 안 되므로 버린다.
                prev = None

            if args.record and sc >= 0.45:
                # 자세 문턱(max_offset_px)을 무시하고 변화량을 직접 구한다 —
                # "몇 px 틀어지면 판정이 무너지는가"를 재는 게 목적이기 때문이다.
                r_px = float(cal["radius_px"]) * scale
                change, angle, contrast = G.change_vs_ref(img, cx, cy, r_px, ref, cal)
                refl = G.reflection_pct(img, cx, cy, r_px)
                rows.append({
                    "yaw_odom": "" if yaw1 is None else f"{yaw1:.2f}",
                    "dx": f"{dx:.0f}", "dy": f"{dy:.0f}", "scale": f"{scale:.2f}",
                    "score": f"{sc:.3f}",
                    "change": "" if change is None else f"{change:.2f}",
                    "angle": "" if angle is None else f"{angle:.0f}",
                    "contrast": f"{contrast:.0f}", "reflection": f"{refl:.1f}",
                    "verdict": "정상" if (change is not None and change < 0.9) else "이상",
                    "image": os.path.basename(keep),
                })

            if sc < 0.45:
                print(f"[{n:3d}] 게이지를 못 찾겠다 (정합 {sc:.2f}) — 소화기가 화면에 있나?")
            else:
                ready = abs(dx) <= OK_PX and abs(dy) <= OK_PX and \
                    OK_SCALE[0] <= scale <= OK_SCALE[1]
                mark = "  ✅ 저장할 준비가 됐다" if ready else ""
                moving = "" if still else "  (찍는 동안 움직임 — 각도 측정 건너뜀)"
                print(f"[{n:3d}] dx {dx:+5.0f}px  dy {dy:+5.0f}px  배율 {scale:.2f}  "
                      f"정합 {sc:.2f}   {guide(dx, dy, scale, k)}{mark}{moving}")
                if ready:
                    print("       제자리에서 20도씩 두세 번 회전시킨 뒤(AMCL 갱신), "
                          "다른 터미널에서:")
                    print(f"       python3 ~/vibe/ex1/tools/save_waypoint.py "
                          f"--name {args.name} \\\n"
                          f"           --file ~/vibe/ex1/maps/fire_extinguisher_points.yaml")
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\n끝낸다.")
        if rows:
            summarize(rows, sweep_dir)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if not args.no_camera:
            print("로봇 카메라를 끈다...")
            if stop_camera(host):
                print("완료 — 카메라가 남아있지 않다")
            else:
                print("⚠️ 카메라를 못 껐다. 주행 전에 반드시 손으로 지울 것:")
                print(f"   ssh {host} \"pkill -9 -f camera_node\"")
    return 0


if __name__ == "__main__":
    sys.exit(main())
