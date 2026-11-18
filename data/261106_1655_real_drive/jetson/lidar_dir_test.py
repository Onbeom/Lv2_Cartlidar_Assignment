#!/usr/bin/env python3
"""라이다 방향 시험 (Jetson, bringup 이 떠 있을 때, 11-15). 앞 · 뒤 라이다의 '지금 TF' 가 실물 방향과 맞는지 물체로 가려서 본다.
  python3 lidar_dir_test.py [기록 폴더=.]   (기준 · 자리마다 3 s 스캔을 raw_*.npz 로 남김)
엔터로 진행: 처음 아무것도 대지 않고 엔터 → 기준 3 s. 그다음 화면이 말하는 자리를 상자로 가리고 엔터 → 3 s 읽고 결과.
- 3 s 통계로 기준에 없던 0.4 m 안 묶음을 넓은 순으로 라이다마다 찍음 (detect 설명)
- 로봇 기준 방향: 0 = 앞(+x), +90 = 왼쪽, −90 = 오른쪽, ±180 = 뒤. 지금 TF = robot.urdf.xacro (roll π · 뒤 yaw π)
- 판정: 가린 쪽 라이다가 반응하고 그 방향이 '맞으면' 각 ±30° 안이면 그 자리 통과
"""
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

YAW = {'/scan_front': 0.0, '/scan_rear': math.pi}   # robot.urdf.xacro laser_front · laser_rear 요
NAME = {'/scan_front': '앞', '/scan_rear': '뒤'}
# (이름, 가릴 곳, 반응해야 할 라이다, 맞으면 각) — 로봇 앞 = 앞 D435 가 보는 쪽, 왼쪽 = 앞을 보고 섰을 때 왼손
STEPS = [('F_front', '앞 라이다 정면 (로봇 앞쪽 끝에서 15 cm)', '/scan_front', 0),
         ('F_left', '앞 라이다 왼쪽 옆 (로봇 왼쪽 면, 앞 라이다 높이 15 cm)', '/scan_front', 90),
         ('F_right', '앞 라이다 오른쪽 옆 (로봇 오른쪽 면, 앞 라이다 높이 15 cm)', '/scan_front', -90),
         ('R_back', '뒤 라이다 정면 (로봇 뒤쪽 끝에서 15 cm)', '/scan_rear', 180),
         ('R_left', '뒤 라이다 왼쪽 옆 (로봇 왼쪽 면, 뒤 라이다 높이 15 cm)', '/scan_rear', 90),
         ('R_right', '뒤 라이다 오른쪽 옆 (로봇 오른쪽 면, 뒤 라이다 높이 15 cm)', '/scan_rear', -90)]
TOL_DEG = 30.0
NEAR = 0.4   # m. 상자는 라이다에서 0.1 ~ 0.3 m, 상자를 든 사람 몸은 이보다 멀게 (11-15)


def wrap(d):
    return (d + 180.0) % 360.0 - 180.0


def cmean(v):   # 원형 평균 (±180° 근처에서 갈라지지 않게)
    a = np.radians(v)
    return math.degrees(math.atan2(float(np.mean(np.sin(a))), float(np.mean(np.cos(a)))))


class Col:
    def __init__(self, node):
        self.buf = None
        self.ang = {}
        for k in YAW:
            node.create_subscription(LaserScan, k, lambda m, k=k: self.on(k, m), qos_profile_sensor_data)

    def on(self, k, m):
        r = np.array(m.ranges, dtype=float)
        # 이 드라이버는 반사가 없는 각을 0 으로 냄 (11-15 18:44) → 잴 수 없는 값(무한 · range_min 미만)은 멀리로
        r[~np.isfinite(r) | (r < max(float(m.range_min), 0.02))] = 12.0
        # 스캔마다 점 수가 다름 (11-15 18:52) → 센서 각 1° 칸 360 개로 모음 (칸마다 가장 가까운 값)
        a = np.degrees(m.angle_min + m.angle_increment * np.arange(len(r)))
        idx = np.floor(a).astype(int) % 360
        binned = np.full(360, 12.0)
        np.minimum.at(binned, idx, r)
        self.ang[k] = (math.radians(0.5), math.radians(1.0), 360)   # 칸 i 의 가운데 = i + 0.5°
        if self.buf is not None:
            self.buf[k].append(binned)


def collect(node, col, sec):
    t = time.time() + 0.5   # 엔터 누르기 전에 쌓인 옛 스캔 버림
    while time.time() < t:
        rclpy.spin_once(node, timeout_sec=0.05)
    col.buf = {k: [] for k in YAW}
    t = time.time() + sec
    while time.time() < t:
        rclpy.spin_once(node, timeout_sec=0.05)
    out, col.buf = col.buf, None
    return out


def groups_of(mask):
    """원형(0~359°) 칸에서 이어진 True 묶음들 (2° 빈칸까지는 이어 봄)"""
    idx = np.where(mask)[0]
    if idx.size == 0:
        return []
    gs = np.split(idx, np.where(np.diff(idx) > 3)[0] + 1)
    if len(gs) > 1 and gs[0][0] <= 2 and gs[-1][-1] >= 357:   # 359° ↔ 0° 이음
        gs = [np.concatenate([gs[-1], gs[0]])] + gs[1:-1]
    return gs


def detect(base_scans, cur_scans, k):
    """3 s 통계로 '새로 생긴 가까운 것' 묶음 (11-15 19:00: 한 장씩 보면 몸체 · 브라켓의 깜빡이는 0.10 m 점을 상자로 잘못 잡음).
    칸이 바뀜 = 지금 70 % 넘게 NEAR(0.4 m) 안 · 기준에서는 30 % 아래였거나 중앙값이 0.15 m 넘게 가까워짐.
    돌려줌: 넓은 순 [(로봇 방향 °, 센서 각 °, 가장 가까운 중앙값 m, 폭 °)]"""
    B, C = np.array(base_scans), np.array(cur_scans)
    occ_b, occ_c = (B < NEAR).mean(0), (C < NEAR).mean(0)
    med_b, med_c = np.median(B, 0), np.median(C, 0)
    # 기준에서 비어 있던 칸에 새로 생김, 또는 기준에서도 늘 있던 것보다 0.15 m 넘게 앞에 생김.
    # 기준에서 깜빡이던 칸 (0.3 < 기준 점유 < 0.7: 차체 · 브라켓 가장자리 등) 은 아예 보지 않음
    ch = (occ_c >= 0.7) & (med_c < NEAR) & ((occ_b <= 0.3) | ((occ_b >= 0.7) & (med_b - med_c > 0.15)))
    # 깜빡이던 칸이라도 지금 70 % 넘게 가까우면 이웃 묶음을 잇는 데만 씀 (상자가 깜빡이던 칸을 덮으면 둘로 갈리지 않게).
    # 묶음은 진짜 바뀐 칸 (ch) 이 3 개 이상이어야 함 → 깜빡이는 칸만으로는 묶음이 안 생김
    flick = (occ_b > 0.3) & (occ_b < 0.7)
    out = []
    for g in groups_of(ch | (flick & (occ_c >= 0.7) & (med_c < NEAR))):
        if int(ch[g].sum()) < 3:
            continue
        sa = cmean(g + 0.5)
        out.append((wrap(-sa + math.degrees(YAW[k])), sa, float(np.min(med_c[g])), len(g)))   # roll π → 센서 각 부호 반전, 그다음 요
    # 넓은 순: 차체 · 브라켓처럼 늘 있는 것은 기준에 들어 있어 빠지고, 새로 생긴 것 중 15 cm 앞 상자가 각도로 가장 넓음 (20 cm 상자 ≈ 60°)
    return sorted(out, key=lambda x: -x[3])


def main():
    odir = sys.argv[1] if len(sys.argv) > 1 else '.'
    rclpy.init()
    node = rclpy.create_node('lidar_dir_test')
    col = Col(node)
    input('로봇 둘레 60 cm 안에 아무것도 없게 하고 엔터 (기준 3 s) ')
    b = collect(node, col, 3.0)
    np.savez(f'{odir}/raw_base.npz', **{NAME[k]: np.array(v) for k, v in b.items()})
    print('기준: ' + ' · '.join(f'{NAME[k]} {len(b[k])} 장' for k in YAW), flush=True)
    rows = []
    for name, where, want_k, want in STEPS:
        while True:
            input(f'\n▶ {name}: 상자로 {where} 을 가리고 엔터 ')
            s = collect(node, col, 3.0)
            np.savez(f'{odir}/raw_{name}.npz', **{NAME[k]: np.array(v) for k, v in s.items()})
            got = {}
            for k in YAW:
                if not s.get(k) or not b.get(k):
                    print(f'  {NAME[k]} 라이다: 스캔 없음', flush=True)
                    continue
                d = detect(b[k], s[k], k)
                if d:
                    got[k] = d[0]
                print(f'  {NAME[k]} 라이다: ' + (' / '.join(f'로봇 {x[0]:+6.1f}° (센서 {x[1]:6.1f}°, {x[2]:.2f} m, 폭 {x[3]}°)' for x in d[:3]) or '변화 없음'), flush=True)
            if want_k in got:
                break
            if input(f'  {NAME[want_k]} 라이다가 반응하지 않음 — 다시 하려면 엔터, 넘어가려면 s 엔터 ').strip() == 's':
                break
        m = got.get(want_k)
        ok = m is not None and abs(wrap(m[0] - want)) <= TOL_DEG
        rows.append((name, want, want_k, m, [k for k in got if k != want_k], ok))
    print('\n== 요약 (판정: 가린 쪽 라이다의 가장 넓은 새 묶음이 지금 TF 로 맞으면 각 ±30° 안)')
    for name, want, want_k, m, other, ok in rows:
        got = f'{m[0]:+6.1f}° (폭 {m[3]}°, {m[2]:.2f} m)' if m else '반응 없음'
        oth = f' · 다른 라이다도 반응({", ".join(NAME[k] for k in other)})' if other else ''
        print(f'{name}: 맞으면 {want:+4d}° · {NAME[want_k]} 라이다 {got}{oth} → {"통과" if ok else "불통과"}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
