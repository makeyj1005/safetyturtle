#!/usr/bin/env python3
"""스케줄러의 시각 선택 규칙을 실제 노드 코드로 검증한다 (로봇·Nav2 불필요).

  source ~/vibe/ex1/ros2_ws/install/setup.bash
  python3 ~/vibe/ex1/tools/test_schedule.py

순찰 길이를 여러 값으로 가정하고 여러 시간 분량의 회차를 시뮬레이션한다.
스케줄러를 고칠 때마다 먼저 이걸 돌릴 것 — 실제로 기다리지 않고 초 단위로 확인된다.

검사 항목
  ① 순찰이 끝난 뒤 rest_min 안에는 절대 시작하지 않는가 (무조건 쉬는 시간)
  ② 끝난 뒤 cycle_min 을 넘겨 시작하지 않는가 (5분 안에 다시 돈다)
  ③ 순찰이 길어져도 ①②가 깨지지 않는가 (시계 기준이 아니라 종료 기준이므로)
  ④ 랜덤이 실제로 흩어지는가 (같은 지연으로 몰리지 않는가)
"""
import rclpy

from patrol_core.patrol_scheduler import PatrolScheduler

rclpy.init()

FAILS = []


def check(cond, msg):
    if not cond:
        FAILS.append(msg)


def simulate(cycle_min, rest_min, run_sec, n_runs, seed):
    """회차를 n_runs 번 돌린 것으로 가정하고 시작 시각들을 뽑는다.

    실제 노드는 /patrol/status 의 done 을 받은 시각을 기준으로 schedule_after 를
    호출한다. 여기서는 그 기준 시각만 직접 넘겨 같은 계산을 반복한다.
    """
    import random as _r

    node = PatrolScheduler()
    node.cycle = cycle_min * 60.0
    node.rest = rest_min * 60.0
    node.rng = _r.Random(seed)

    t = 0.0            # 노드를 켠 시각(= 첫 기준점)
    delays = []        # 기준점에서 시작까지의 지연(초)
    for _ in range(n_runs):
        node.next_fire = None
        node.schedule_after(t, "테스트")
        start = node.next_fire
        delay = start - t
        delays.append(delay)
        check(delay >= rest_min * 60.0 - 1e-6,
              f"쉬는 시간 위반: {delay / 60:.2f}분 < {rest_min}분")
        check(delay <= cycle_min * 60.0 + 1e-6,
              f"창 초과: {delay / 60:.2f}분 > {cycle_min}분")
        t = start + run_sec      # 이번 순찰이 끝난 시각이 다음 기준점

    node.destroy_node()
    return delays


def report(title, cycle_min, rest_min, run_sec, n_runs, seed):
    d = simulate(cycle_min, rest_min, run_sec, n_runs, seed)
    lo, hi = min(d) / 60, max(d) / 60
    avg = sum(d) / len(d) / 60
    # 뽑을 수 있는 폭을 10칸으로 나눠, 몇 칸에 걸쳐 흩어졌는지 센다.
    # 칸 크기를 30초처럼 고정하면 폭이 좁은 설정(쉼 4.5분/창 5분)에서 무조건 실패한다.
    bucket = max((cycle_min - rest_min) * 60.0 / 10.0, 1.0)
    spread = len({int(x / bucket) for x in d})
    check(spread >= 3, f"랜덤이 흩어지지 않는다: {bucket:.0f}초 칸 기준 {spread}칸")
    print(f"  {title}")
    print(f"    끝난 뒤 시작까지: 최소 {lo:.2f}분 / 평균 {avg:.2f}분 / 최대 {hi:.2f}분")
    print(f"    순찰 {run_sec / 60:.1f}분 가정 -> 시작~시작 간격 평균 "
          f"{(avg + run_sec / 60):.2f}분, 지연이 {spread}칸에 흩어짐")


print("=== 기본값 (cycle 5분 / rest 1분) — 순찰 길이를 바꿔가며 ===")
report("순찰 1.0분 (1바퀴 정도)", 5.0, 1.0, 60, 40, 1)
report("순찰 2.5분 (2바퀴 정도)", 5.0, 1.0, 150, 40, 2)
report("순찰 6.0분 (창보다 길다)", 5.0, 1.0, 360, 40, 3)

print("\n=== 빨리 확인용 (cycle 1.5분 / rest 0.5분) ===")
report("순찰 0.3분", 1.5, 0.5, 20, 30, 4)

print("\n=== 경계 조건 ===")
report("쉼 0분 (쉬지 않음)", 5.0, 0.0, 60, 30, 5)
report("쉼이 창의 90%", 5.0, 4.5, 60, 30, 6)

if FAILS:
    print(f"\n실패 {len(FAILS)}건:")
    for m in FAILS[:10]:
        print("  -", m)
else:
    print("\n모든 검사 통과")

rclpy.shutdown()
