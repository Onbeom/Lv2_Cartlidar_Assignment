#!/usr/bin/env python3
"""J1: place_box 처리 시간 (장당, 프로세스 시작 · 영상 읽기 포함 — PC 287 / 920 ms 와 같은 재기). j1_time.py <도구> [manifest.tsv]
앞 D435 = 배율 1.0 (노드 front_scales), 팔 = 기본 배율 (노드 arm.place_box.scales). 출력: 종류별 장 수 · 찾음 · 중앙 · p95 · 최대 ms."""
import os, subprocess, sys, time
tool = sys.argv[1]; man = sys.argv[2] if len(sys.argv) > 2 else 'manifest.tsv'
res = {}
for line in open(man):
    kind, scales, args = line.rstrip('\n').split('\t')
    env = dict(os.environ); env.pop('PLACE_BOX_SCALES', None)
    if scales: env['PLACE_BOX_SCALES'] = scales
    t = time.perf_counter(); o = subprocess.run([tool] + args.split(), capture_output=True, text=True, env=env); ms = (time.perf_counter() - t) * 1e3
    ok = o.stdout.split()[4] == '1' if len(o.stdout.split()) > 4 else False
    res.setdefault(kind, []).append((ms, ok))
for k, v in res.items():
    ms = sorted(m for m, _ in v); n = len(ms)
    print(f"{k}: 장 {n}, 찾음 {sum(o for _, o in v)}, 중앙 {ms[n // 2]:.0f} ms, p95 {ms[int(n * 0.95) - 1]:.0f} ms, 최대 {ms[-1]:.0f} ms")
