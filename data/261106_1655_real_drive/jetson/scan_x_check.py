#!/usr/bin/env python3
"""라이다 요 PR 머지 전 실물 확인 (Jetson, 11-15). 합친 스캔(base_link 프레임)에서 상자가 맞는 쪽에 찍히는지 본다.
  python3 scan_x_check.py [기록 폴더=.]
엔터로 진행: 기준 3 s → 로봇 앞에 상자 → 로봇 뒤에 상자. 토픽 /scan (scan_merger) · /scan_merged (scan_merge) 둘 다 봄.
판정: 앞 상자 = base_link 각 0° ±30° (+x 쪽), 뒤 상자 = ±180° ±30°. 변화 판정은 lidar_dir_test.py 와 같음 (3 s 통계, 0.5 m 안, 넓은 묶음).
"""
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

TOPICS = ['/scan', '/scan_merged']
NEAR = 0.5   # m (base_link 원점에서: 차체 앞 끝 0.25 m + 상자 15 cm)
STEPS = [('front', '로봇 앞쪽 끝에서 15 cm 앞', 0.0), ('rear', '로봇 뒤쪽 끝에서 15 cm 뒤', 180.0)]


def wrap(d):
    return (d + 180.0) % 360.0 - 180.0


def cmean(v):
    a = np.radians(v)
    return math.degrees(math.atan2(float(np.mean(np.sin(a))), float(np.mean(np.cos(a)))))


class Col:
    def __init__(self, node):
        self.buf, self.frame = None, {}
        for k in TOPICS:
            node.create_subscription(LaserScan, k, lambda m, k=k: self.on(k, m), qos_profile_sensor_data)

    def on(self, k, m):
        r = np.array(m.ranges, dtype=float)
        r[~np.isfinite(r) | (r < max(float(m.range_min), 0.02))] = 12.0
        a = np.degrees(m.angle_min + m.angle_increment * np.arange(len(r)))
        b = np.full(360, 12.0)
        np.minimum.at(b, np.floor(a).astype(int) % 360, r)
        self.frame[k] = m.header.frame_id
        if self.buf is not None:
            self.buf[k].append(b)


def collect(node, col, sec):
    t = time.time() + 0.5
    while time.time() < t:
        rclpy.spin_once(node, timeout_sec=0.05)
    col.buf = {k: [] for k in TOPICS}
    t = time.time() + sec
    while time.time() < t:
        rclpy.spin_once(node, timeout_sec=0.05)
    out, col.buf = col.buf, None
    return out


def detect(B, C):
    B, C = np.array(B), np.array(C)
    ob, oc = (B < NEAR).mean(0), (C < NEAR).mean(0)
    mb, mc = np.median(B, 0), np.median(C, 0)
    ch = (oc >= 0.7) & (mc < NEAR) & ((ob <= 0.3) | ((ob >= 0.7) & (mb - mc > 0.15)))
    idx = np.where(ch)[0]
    if idx.size == 0:
        return None
    gs = np.split(idx, np.where(np.diff(idx) > 3)[0] + 1)
    if len(gs) > 1 and gs[0][0] <= 2 and gs[-1][-1] >= 357:
        gs = [np.concatenate([gs[-1], gs[0]])] + gs[1:-1]
    g = max(gs, key=len)
    return cmean(g + 0.5), float(np.min(mc[g])), len(g)


def main():
    odir = sys.argv[1] if len(sys.argv) > 1 else '.'
    rclpy.init()
    node = rclpy.create_node('scan_x_check')
    col = Col(node)
    input('로봇 둘레를 비우고 엔터 (기준 3 s) ')
    base = collect(node, col, 3.0)
    np.savez(f'{odir}/raw_base.npz', **{k.strip('/'): np.array(v) for k, v in base.items()})
    print('기준: ' + ' · '.join(f'{k} {len(base[k])} 장 (frame {col.frame.get(k, "?")})' for k in TOPICS), flush=True)
    res = []
    for name, where, want in STEPS:
        input(f'\n▶ 상자를 {where} 에 두고 엔터 ')
        s = collect(node, col, 3.0)
        np.savez(f'{odir}/raw_{name}.npz', **{k.strip('/'): np.array(v) for k, v in s.items()})
        for k in TOPICS:
            if not base.get(k) or not s.get(k):
                print(f'  {k}: 스캔 없음', flush=True)
                continue
            d = detect(base[k], s[k])
            ok = d is not None and abs(wrap(d[0] - want)) <= 30.0
            res.append((name, k, want, d, ok))
            print(f'  {k}: ' + (f'base_link {d[0]:+6.1f}° · {d[1]:.2f} m · 폭 {d[2]}°' if d else '변화 없음') + f' → {"통과" if ok else "불통과"} (맞으면 {want:+.0f}°)', flush=True)
    print('\n== 요약')
    for name, k, want, d, ok in res:
        print(f'{name} {k}: ' + (f'{d[0]:+6.1f}°' if d else '없음') + f' (맞으면 {want:+.0f}°) → {"통과" if ok else "불통과"}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
