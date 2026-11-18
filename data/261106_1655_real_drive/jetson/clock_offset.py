#!/usr/bin/env python3
"""PC 에서: Jetson 시계 − PC 시계 [s]. ssh 로 n 번 재서 왕복이 가장 짧은 표본을 쓴다(불확실 ± 왕복/2).
clock_offset.py [n=10]  → 한 줄 JSON {offset_s, rtt_s, n}"""
import json, subprocess, sys, time
n = int(sys.argv[1]) if len(sys.argv) > 1 else 10
best = None
for _ in range(n):
    a = time.time()
    j = float(subprocess.run(['ssh', '-o', 'ControlPath=~/.ssh/lv2_jetson.sock', 'cart11@cart11.local', 'date +%s.%N'],
                             capture_output=True, text=True, timeout=10).stdout)
    b = time.time()
    if best is None or b - a < best[1]:
        best = (j - (a + b) / 2, b - a)
print(json.dumps(dict(offset_s=round(best[0], 4), rtt_s=round(best[1], 4), n=n)))
