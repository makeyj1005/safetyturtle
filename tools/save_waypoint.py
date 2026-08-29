#!/usr/bin/env python3
"""
save_waypoint.py — 로봇의 현재 위치를 순찰 웨이포인트로 파일에 기록한다.

[VM에서 실행]
  python3 ~/vibe/ex1/tools/save_waypoint.py                 # 현재 위치를 목록에 추가
  python3 ~/vibe/ex1/tools/save_waypoint.py --name 소화기앞  # 이름 붙여서 추가
  python3 ~/vibe/ex1/tools/save_waypoint.py --list           # 저장된 목록 보기
  python3 ~/vibe/ex1/tools/save_waypoint.py --clear          # 전부 지우기

  소화기 점검 지점은 순찰 웨이포인트와 파일을 분리한다 (같은 파일에 넣으면
  순찰 경로에 엉뚱한 점이 추가된다):
  python3 ~/vibe/ex1/tools/save_waypoint.py --name 소화기1 \
      --file ~/vibe/ex1/maps/fire_extinguisher_points.yaml

[원리]
Nav2 의 AMCL 이 발행하는 /amcl_pose 를 읽어서 좌표(x, y)와 방향(yaw)을 저장한다.
직선 경로를 만들려면 로봇을 직선 위로 이동시키면서 1m 내외 간격으로 이 명령을 반복한다.
Nav2 에게 먼 목표 하나만 주면 알아서 우회하지만, 촘촘한 웨이포인트를 순서대로 주면
각 구간이 짧아 딴 길로 갈 여지가 없어 거의 직선으로 이동한다.

[주의]
저장 전에 RViz 에서 위치 추정이 잘 맞는지 확인할 것. 라이다 점이 지도 벽선과 어긋나 있으면
엉뚱한 좌표가 저장된다. covariance 값이 크면 경고를 띄운다.
"""
import argparse
import math
import os
import sys

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

MAPS_DIR = os.path.join(os.path.expanduser("~"), "vibe", "ex1", "maps")
WAYPOINT_FILE = os.path.join(MAPS_DIR, "patrol_waypoints.yaml")
# 벽 간격 검사에 쓰는 지도. patrol_map_v5 는 순찰로 여러 번 돌며 다듬은 것으로,
# 원점(-1.59, -5.12)과 해상도가 예전 patrol_map 과 같아 좌표는 그대로 유효하다.
MAP_YAML = os.path.join(MAPS_DIR, "patrol_map_v5.yaml")

# 이 값보다 위치 불확실성이 크면 저장을 경고한다.
# 잘 수렴한 상태는 0.05 이하, 0.5 이상이면 오차가 ±0.7m 수준으로 커진다.
COV_WARN = 0.25

# 방향(yaw) 불확실성 경고 기준(rad^2). 0.15 = 표준편차 약 22도.
#
# 처음에 0.03(±10도)으로 잡았다가 0.15 로 완화했다. 근거:
# 이 로봇에서 실측되는 값은 코너 회전을 포함한 순찰 2바퀴 뒤에도 0.24~0.44 였다.
# TurtleBot3 기본 AMCL 은 모션 노이즈(alpha 0.2)가 커서 보고되는 yaw 불확실성이
# 실제 오차보다 크게 나온다 — 실제로 ±28도 틀어져 있었다면 순찰 2바퀴가 실패 없이
# 돌 수 없다. 즉 0.03 은 이 장비에서 도달 불가능한 기준이었다.
# 0.15 는 '완전히 발산한 경우(±40도 이상)'만 걸러내는 안전망으로 남긴다.
#
# 점검 지점의 진짜 합격 기준은 covariance 가 아니라 **카메라**다:
# 저장한 좌표로 자율 이동한 뒤 게이지 템플릿 정합 점수가 0.9 이상이면 된다.
# 그게 "게이지가 기대한 자리에 온다"는 것을 직접 측정한 값이다.
YAW_COV_WARN = 0.15

# TurtleBot3 Burger 의 robot_radius (Nav2 파라미터와 동일). 로봇 중심에서 몸체 끝까지.
ROBOT_RADIUS = 0.10
# 벽 간격이 이 값보다 좁으면 경고한다. 30cm 순찰이 목표이고,
# 로봇 반경 10cm 를 빼면 실제 여유가 20cm 밖에 안 되므로 이보다 좁으면 위험하다.
CLEARANCE_WARN = 0.30


def quat_to_yaw(q):
    """쿼터니언에서 yaw(z축 회전, 라디안)만 뽑는다."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))


class ClearanceMap:
    """지도에서 임의 좌표의 '가장 가까운 벽까지 거리'를 조회한다.

    왜 필요한가: /amcl_pose 의 x,y 는 지도 원점 기준 좌표일 뿐이어서
    그 지점이 벽에 얼마나 가까운지는 알 수 없다. 실제로 벽에서 10cm 밖에
    떨어지지 않은 좌표를 저장한 적이 있는데(2026-07-30), robot_radius 가 10cm 라
    로봇 몸체가 벽에 닿는 위치였고 Nav2 가 경로를 만들 수 없었다.
    저장 시점에 바로 걸러내기 위해 넣었다.

    미탐사(unknown) 영역도 장애물로 취급한다. 그쪽은 뭐가 있는지 모르므로
    보수적으로 보는 것이 안전하다.
    """

    def __init__(self, map_yaml=MAP_YAML):
        self.ok = False
        try:
            import cv2
            import numpy as np
        except ImportError:
            return
        if not os.path.exists(map_yaml):
            return
        with open(map_yaml) as f:
            meta = yaml.safe_load(f)
        pgm = os.path.join(os.path.dirname(map_yaml), meta["image"])
        img = cv2.imread(pgm, cv2.IMREAD_UNCHANGED)
        if img is None:
            return
        self.res = float(meta["resolution"])
        self.ox, self.oy = float(meta["origin"][0]), float(meta["origin"][1])
        self.h, self.w = img.shape
        occ = img < 100
        free = img > 250
        blocked = occ | (~free & ~occ)          # 장애물 + 미탐사
        self.dist = cv2.distanceTransform(
            (~blocked).astype(np.uint8), cv2.DIST_L2, 5
        ) * self.res
        self.ok = True

    def at(self, mx, my):
        """지도좌표(m) -> 벽까지 거리(m). 범위 밖이면 None."""
        if not self.ok:
            return None
        px = int((mx - self.ox) / self.res)
        # pgm 은 이미지 좌표가 위에서 아래로 증가하므로 y 를 뒤집는다.
        py = self.h - 1 - int((my - self.oy) / self.res)
        if not (0 <= px < self.w and 0 <= py < self.h):
            return None
        return float(self.dist[py, px])


class PoseGrabber(Node):
    def __init__(self):
        super().__init__("save_waypoint")
        # AMCL 은 TRANSIENT_LOCAL 로 발행하므로 구독도 맞춰야 마지막 값을 즉시 받을 수 있다.
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pose = None
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.on_pose, qos
        )

    def on_pose(self, msg):
        self.pose = msg


def load(path=None):
    path = path or WAYPOINT_FILE
    if not os.path.exists(path):
        return {"waypoints": []}
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    data.setdefault("waypoints", [])
    return data


def save(data, path=None):
    """--file 로 다른 파일에 저장할 수 있다.

    소화기 점검 지점을 순찰 웨이포인트와 같은 파일에 넣으면 순찰 경로에 엉뚱한
    점이 하나 추가된다. 용도가 다른 좌표는 파일을 나눈다.
    """
    path = path or WAYPOINT_FILE
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)


def show(data, cmap=None):
    wps = data["waypoints"]
    if not wps:
        print("저장된 웨이포인트 없음")
        return
    print(f"{'#':>3}  {'이름':<14} {'x':>8} {'y':>8} {'yaw(도)':>8} {'구간거리':>8} {'벽간격':>8}")
    prev = None
    for i, w in enumerate(wps):
        d = ""
        if prev is not None:
            d = f"{math.dist((prev['x'], prev['y']), (w['x'], w['y'])):.2f}m"
        cl = ""
        if cmap is not None and cmap.ok:
            c = cmap.at(w["x"], w["y"])
            if c is not None:
                mark = "" if c >= CLEARANCE_WARN else " !"
                cl = f"{c*100:.0f}cm{mark}"
        print(f"{i:>3}  {w.get('name',''):<14} {w['x']:>8.3f} {w['y']:>8.3f} "
              f"{math.degrees(w['yaw']):>8.1f} {d:>8} {cl:>8}")
        prev = w


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default="", help="웨이포인트 이름(선택)")
    ap.add_argument("--list", action="store_true", help="목록만 보기")
    ap.add_argument("--clear", action="store_true", help="전부 삭제")
    ap.add_argument("--force", action="store_true", help="불확실성·벽간격 경고 무시")
    ap.add_argument("--replace", action="store_true",
                    help="같은 이름이 이미 있으면 덮어쓴다(기본은 뒤에 추가)")
    ap.add_argument("--file", default=None,
                    help="저장할 파일 (기본: maps/patrol_waypoints.yaml). "
                         "소화기 지점은 maps/fire_extinguisher_points.yaml 처럼 따로 둘 것")
    a = ap.parse_args()

    cmap = ClearanceMap()

    if a.list:
        show(load(a.file), cmap)
        return 0
    if a.clear:
        save({"waypoints": []}, a.file)
        print("웨이포인트 전부 삭제됨")
        return 0

    rclpy.init()
    node = PoseGrabber()
    print("/amcl_pose 대기 중...")
    import time
    end = time.monotonic() + 10.0
    while rclpy.ok() and node.pose is None and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    pose = node.pose
    node.destroy_node()
    rclpy.shutdown()

    if pose is None:
        print("[실패] /amcl_pose 를 못 받았다. Nav2 가 켜져 있고 초기 위치를 지정했는지 확인.")
        return 1

    p = pose.pose.pose.position
    yaw = quat_to_yaw(pose.pose.pose.orientation)
    cov_x = pose.pose.covariance[0]
    cov_y = pose.pose.covariance[7]
    cov_yaw = pose.pose.covariance[35]

    print(f"현재 위치: x={p.x:.3f}  y={p.y:.3f}  yaw={math.degrees(yaw):.1f}도")
    print(f"위치 불확실성: x={cov_x:.3f}  y={cov_y:.3f}")
    print(f"방향 불확실성: yaw={cov_yaw:.3f}  (표준편차 약 {math.degrees(math.sqrt(cov_yaw)):.0f}도)")

    clear = cmap.at(float(p.x), float(p.y)) if cmap.ok else None
    if clear is None:
        print("벽 간격: 확인 불가 (지도 범위 밖이거나 지도를 못 읽음)")
    else:
        margin = clear - ROBOT_RADIUS
        print(f"벽 간격: {clear*100:.1f} cm  "
              f"(로봇 반경 {ROBOT_RADIUS*100:.0f}cm 제외하면 실제 여유 {margin*100:.1f} cm)")

    if cov_yaw > YAW_COV_WARN and not a.force:
        print(f"\n[경고] 방향 불확실성이 {YAW_COV_WARN} 보다 크다 "
              f"(표준편차 약 {math.degrees(math.sqrt(cov_yaw)):.0f}도).")
        print("       이 상태로 저장하면 로봇이 그만큼 틀어진 방향으로 서게 되고,")
        print("       점검 지점에서는 게이지가 화면을 벗어난다.")
        print("       제자리 회전을 몇 번 하거나 순찰을 한 바퀴 돌려 수렴시킬 것.")
        print("       그래도 저장하려면 --force.")
        return 2

    if max(cov_x, cov_y) > COV_WARN and not a.force:
        print(f"\n[경고] 불확실성이 {COV_WARN} 보다 크다. 지금 저장하면 좌표가 부정확할 수 있다.")
        print("       로봇을 조금 움직여 위치를 수렴시킨 뒤 다시 시도하거나,")
        print("       그래도 저장하려면 --force 를 붙일 것.")
        return 2

    if clear is not None and clear < CLEARANCE_WARN and not a.force:
        print(f"\n[경고] 벽 간격이 {CLEARANCE_WARN*100:.0f}cm 보다 좁다 ({clear*100:.1f}cm).")
        if clear <= ROBOT_RADIUS:
            print("       로봇 몸체가 벽에 닿는 위치다. Nav2 가 경로를 만들 수 없어 반드시 옮겨야 한다.")
        else:
            print("       Nav2 가 이 지점을 피하려 해서 경로가 곡선이 되거나 복구 동작에 빠질 수 있다.")
        print("       로봇을 벽에서 더 떨어뜨린 뒤 다시 시도하거나, 그래도 저장하려면 --force 를 붙일 것.")
        return 3

    data = load(a.file)
    idx = len(data["waypoints"])
    entry = {
        "name": a.name or f"wp{idx}",
        "x": round(float(p.x), 3),
        "y": round(float(p.y), 3),
        "yaw": round(float(yaw), 4),
    }
    # 같은 이름이 있는데 그냥 추가하면 지점이 둘이 된다. 점검 노드는 등록된 지점을
    # 모두 돌기 때문에 옛 자리까지 들르게 된다(자세를 다시 등록할 때 실제로 걸린다).
    same = [i for i, w in enumerate(data["waypoints"]) if w.get("name") == entry["name"]]
    if same and not a.replace:
        print(f"\n[경고] '{entry['name']}' 이(가) 이미 있다 (#{same[0]}). "
              "그냥 저장하면 같은 이름이 둘이 된다.")
        print("       덮어쓰려면 --replace, 그래도 추가하려면 이름을 바꿀 것.")
        old = data["waypoints"][same[0]]
        print(f"       기존: x={old['x']:.3f} y={old['y']:.3f} "
              f"yaw={math.degrees(old['yaw']):.1f}도")
        print(f"       지금: x={entry['x']:.3f} y={entry['y']:.3f} "
              f"yaw={math.degrees(entry['yaw']):.1f}도")
        return 4
    if same:
        old = data["waypoints"][same[0]]
        print(f"\n'{entry['name']}' 덮어쓴다: "
              f"x {old['x']:.3f}->{entry['x']:.3f}  y {old['y']:.3f}->{entry['y']:.3f}  "
              f"yaw {math.degrees(old['yaw']):.1f}->{math.degrees(entry['yaw']):.1f}도")
        data["waypoints"][same[0]] = entry
        for i in reversed(same[1:]):          # 중복이 더 있으면 정리한다
            del data["waypoints"][i]
    else:
        data["waypoints"].append(entry)
    save(data, a.file)
    print(f"\n저장됨 -> {a.file or WAYPOINT_FILE}")
    print()
    show(data, cmap)
    return 0


if __name__ == "__main__":
    sys.exit(main())
