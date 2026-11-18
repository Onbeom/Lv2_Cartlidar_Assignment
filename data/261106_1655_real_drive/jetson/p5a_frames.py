#!/usr/bin/env python3
"""P5a 프레임 판정 (PC): p5a_frames.py <결과 폴더> <probe jsonl> [--guard 0.15]

판 k 마다 truth_k.log(연습 신호등 시리얼: 'ON i C 보드ms', 켜짐 1 s) 의 정답과 /perception/signal 판단을 촬영 시각으로 맞춘다.
  - 판단 시각 = 메시지 stamp(Jetson 시계, 마지막으로 쓴 영상 촬영 시각) − 시계 차이(clock.json 의 Jetson−PC, 없으면 0)
  - 정답 시각 = 보드 ms 를 PC 시각으로: 시리얼 줄을 받은 PC 시각 − 보드 ms 의 최솟값(가장 빨리 온 줄 = 지연 최소)을 기준으로 씀
  - 정답이 바뀌는 순간 ±guard 초 안 메시지와 고정 전(SEARCHING) 메시지는 뺀다
판정(README P5a):
  켜짐 검출 = 정답 색 구간 메시지 중 같은 색으로 판단한 비율(모름 제외) ≥ 95 % — 색마다
  오검출 = 꺼짐 구간에서 켜짐으로 판단 + 켜짐 구간에서 다른 색으로 판단, 0 이어야 함
  모름(신호등 자리 화면 밖 · 가림) 비율은 따로 적음(기준 없음)
"""
import argparse
import collections
import glob
import json
import os
import re

ap = argparse.ArgumentParser()
ap.add_argument('dir')
ap.add_argument('probe')
ap.add_argument('--guard', type=float, default=0.15)
ap.add_argument('--on_s', type=float, default=1.0, help='연습 신호등 켜짐 길이 [s] (signal_light_12.ino)')
a = ap.parse_args()

off = 0.0
offs = []
for f in ('clock_before.json', 'clock_after.json', 'clock.json'):
    p = os.path.join(a.dir, f)
    if os.path.exists(p):
        offs.append(json.load(open(p))['offset_s'])
if offs:
    off = sum(offs) / len(offs)
rows = [json.loads(l) for l in open(a.probe) if '"kind"' not in l]
if rows and 'stamp' not in rows[0]:
    raise SystemExit('probe 기록에 stamp 가 없음 (p5_probe.py 새 판으로 기록할 것)')
resets = []
for f in glob.glob(os.path.join(a.dir, 'reset_*.txt')):
    k = int(re.findall(r'reset_(\d+)\.txt', f)[0])
    resets.append((k, float(open(f).read().split()[-1])))
resets.sort()
LT = {'R': 'R', 'Y': 'Y', 'G': 'G', 'OFF': 'OFF'}
total = collections.Counter()
ok_all = True
print(f'시계 차이 Jetson−PC {off * 1e3:.1f} ms (표본 {len(offs)}), 경계 제외 ±{a.guard:.2f} s')
for i, (k, t0j) in enumerate(resets):
    t1j = resets[i + 1][1] if i + 1 < len(resets) else float('inf')
    ons = []
    for l in open(os.path.join(a.dir, f'truth_{k}.log')):
        m = re.match(r'(\d+\.\d+) ON \d+ ([RYG]) (\d+)\s*$', l.strip())
        if m:
            ons.append((float(m.group(1)), m.group(2), int(m.group(3))))
    if not ons:
        print(f'판 {k}: 정답 줄 없음')
        continue
    base = min(pc - ms / 1000 for pc, _, ms in ons)
    iv = sorted((base + ms / 1000, base + ms / 1000 + a.on_s, c) for _, c, ms in ons)
    edges = [e for s, t, _ in iv for e in (s, t)]

    def truth(t):
        for s, e, c in iv:
            if s <= t < e:
                return c
        return 'OFF'

    tab = collections.Counter()
    for r in rows:
        if not (t0j <= r['t'] < t1j) or r['status'] == 'SEARCHING':
            continue
        tp = r['stamp'] - off
        if tp < iv[0][0] - 1.0 or tp > iv[-1][1] + 1.0:
            continue
        if any(abs(tp - e) < a.guard for e in edges):
            continue
        tab[(truth(tp), LT.get(r['light'], r['light']) if r['visible'] else '?')] += 1
    total.update(tab)
    line = []
    for c in 'RYG':
        n = sum(v for (t, _), v in tab.items() if t == c and _ != '?')
        hit = tab[(c, c)]
        line.append(f'{c} {hit}/{n}={100 * hit / max(1, n):.1f}%')
    fp_off = sum(v for (t, g), v in tab.items() if t == 'OFF' and g in 'RYG')
    fp_col = sum(v for (t, g), v in tab.items() if t in 'RYG' and g in 'RYG' and g != t)
    unk = sum(v for (t, g), v in tab.items() if g == '?')
    allm = sum(tab.values())
    print(f'판 {k}: 켜짐 검출 ' + ' · '.join(line) + f' | 꺼짐 오검출 {fp_off} · 다른 색 {fp_col} | 모름 {unk}/{allm}={100 * unk / max(1, allm):.1f}%')
    miss = {kk: v for kk, v in tab.items() if kk[0] != kk[1] and kk[1] != '?'}
    if miss:
        print('   틀린 판단(정답, 판단): ' + ', '.join(f'{kk}:{v}' for kk, v in sorted(miss.items())))

print('--- 모든 판')
verdict = []
for c in 'RYG':
    n = sum(v for (t, g), v in total.items() if t == c and g != '?')
    hit = total[(c, c)]
    p = 100 * hit / max(1, n)
    verdict.append(p >= 95.0 and n > 0)
    print(f'  {c}: 같은 색 {hit}/{n} = {p:.1f} % ({"통과" if p >= 95 and n > 0 else "불통과"}, 기준 ≥ 95 %)')
fp_off = sum(v for (t, g), v in total.items() if t == 'OFF' and g in 'RYG')
fp_col = sum(v for (t, g), v in total.items() if t in 'RYG' and g in 'RYG' and g != t)
off_n = sum(v for (t, g), v in total.items() if t == 'OFF' and g != '?')
unk = sum(v for (t, g), v in total.items() if g == '?')
print(f'  꺼짐 오검출 {fp_off}/{off_n} · 다른 색 오검출 {fp_col} ({"통과" if fp_off == 0 and fp_col == 0 else "불통과"}, 기준 0)')
print(f'  모름 {unk}/{sum(total.values())} = {100 * unk / max(1, sum(total.values())):.1f} % (참고)')
print('P5a 판정:', '통과' if all(verdict) and fp_off == 0 and fp_col == 0 else '불통과')
