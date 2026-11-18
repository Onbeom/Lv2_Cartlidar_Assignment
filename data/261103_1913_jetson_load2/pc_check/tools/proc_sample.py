#!/usr/bin/env python3
"""프로세스별 CPU · 메모리 기록(1 초, /proc 만 씀 — psutil 없어도 됨). 이름에 아래 낱말이 든 프로세스만 남긴다.
열: wall_s, mono_s, key, pid, cpu_pct(코어 하나 = 100), rss_mb. key 'ALL' 행 = 시스템 전체 CPU %(모든 코어 평균, tegrastats 와 같은 뜻).
사용: python3 proc_sample.py --out proc.csv --seconds 1900"""
import argparse
import os
import time

KEYS = ['ekf_node', 'amcl', 'scan_body_mask', 'scan_merge', 'loc_monitor', 'map_server', 'lifecycle_manager', 'ros2 bag', 'load_test', 'component_container']
ap = argparse.ArgumentParser()
ap.add_argument('--out', required=True)
ap.add_argument('--seconds', type=float, default=1900)
a = ap.parse_args()
TCK = os.sysconf('SC_CLK_TCK')
PAGE_MB = os.sysconf('SC_PAGE_SIZE') / 1e6


def key_of(pid):
    try:
        cmd = open(f'/proc/{pid}/cmdline', 'rb').read().replace(b'\0', b' ').decode(errors='replace')
    except OSError:
        return None
    for k in KEYS:
        if k in cmd and 'proc_sample' not in cmd:
            return k
    return None


def ticks(pid):
    try:
        f = open(f'/proc/{pid}/stat').read().rsplit(')', 1)[1].split()
        return int(f[11]) + int(f[12]), int(f[21]) * PAGE_MB
    except (OSError, IndexError):
        return None


def total():
    v = [int(x) for x in open('/proc/stat').readline().split()[1:]]
    return sum(v), v[3] + v[4]


f = open(a.out, 'w')
f.write('wall_s,mono_s,key,pid,cpu_pct,rss_mb\n')
prev, (pt, pi), t_prev = {}, total(), time.monotonic()
t_end = t_prev + a.seconds
while time.monotonic() < t_end:
    time.sleep(1.0)
    now, wall = time.monotonic(), time.time()
    dt = now - t_prev
    tt, ti = total()
    f.write(f'{wall:.3f},{now:.3f},ALL,0,{100.0 * (1 - (ti - pi) / max(tt - pt, 1)):.1f},0\n')
    pt, pi, t_prev = tt, ti, now
    cur = {}
    for d in os.listdir('/proc'):
        if not d.isdigit():
            continue
        k = key_of(d)
        if k is None:
            continue
        r = ticks(d)
        if r is None:
            continue
        cur[d] = r[0]
        if d in prev:
            f.write(f'{wall:.3f},{now:.3f},{k},{d},{100.0 * (r[0] - prev[d]) / TCK / dt:.1f},{r[1]:.1f}\n')
    prev = cur
    f.flush()
