#!/usr/bin/env python3
"""P5e 분석 (PC): p5e_analyze.py <결과 폴더> <probe jsonl>

판 k 마다 reset_k.txt(Jetson 초기화 시각) ~ 다음 판 초기화 사이 /perception/signal 기록에서
고정 시각, 후보 · 불일치 · 확정 순서와 시각(초기화 기준 s), 확정 때 묶음 수, 모름 메시지 % 를 내고
truth_k.log 의 SEQ(정답)와 비교한다.
"""
import glob
import json
import os
import re
import sys

D, probe = sys.argv[1], sys.argv[2]
rows = [json.loads(l) for l in open(probe) if '"kind"' not in l]
resets = []
for f in sorted(glob.glob(os.path.join(D, 'reset_*.txt')), key=lambda p: int(re.findall(r'(\d+)\.txt', p)[0])):
    k = int(re.findall(r'(\d+)\.txt', f)[0])
    resets.append((k, float(open(f).read().split()[-1])))
verdict = []
for i, (k, t0) in enumerate(resets):
    t1 = resets[i + 1][1] if i + 1 < len(resets) else float('inf')
    m = re.search(r'SEQ ([RYG]{12}) (R\d+ Y\d+ G\d+)', open(os.path.join(D, f'truth_{k}.log')).read())
    truth, counts = (m.group(1), m.group(2)) if m else ('?', '?')
    sel = [r for r in rows if t0 <= r['t'] < t1]
    locked = [r for r in sel if r['status'] != 'SEARCHING']
    unk = sum(not r['visible'] for r in locked)
    print(f'판 {k}: 정답 {truth} ({counts}), 메시지 {len(sel)}, 고정 뒤 {len(locked)}, 모름 {100 * unk / max(1, len(locked)):.1f} %')
    if locked:
        print(f'  고정 {locked[0]["t"] - t0:.1f} s')
    seen = set()
    for r in sel:
        key = (r['status'], r['seq'])
        if r['status'] in ('CANDIDATE', 'MISMATCH', 'CONFIRMED') and key not in seen:
            seen.add(key)
            ok = '정답' if r['seq'] == truth else '틀림'
            print(f'  {r["t"] - t0:6.1f} s {r["status"]:9s} {r["seq"]} R{r["r"]} Y{r["y"]} G{r["g"]} 묶음 {r["cycles"]} ({ok})')
    conf = [r for r in sel if r['status'] == 'CONFIRMED']
    good = bool(conf) and all(r['seq'] == truth for r in conf) and conf[0]['cycles'] <= 2
    verdict.append(good)
    print(f'  → {"통과" if good else "불통과"} (확정 = 정답 · 틀린 확정 0 · 확정까지 묶음 ≤ 2)')
print('P5b 판정:', '통과' if verdict and all(verdict) else '불통과', f'({sum(verdict)}/{len(verdict)} 판)')
