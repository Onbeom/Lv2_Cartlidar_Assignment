#!/usr/bin/env python3
"""read_all.sh 결과를 촬영 폴더마다 센다 (Jetson 또는 PC). python3 summarize.py <clips> <results>

폴더 이름 = <찍은 것>_<거리>cm_<시각>. 찍은 것에 따라 정답:
  tag00 ~ tag11 → 태그 ID 0 ~ 11, L1 · L2 · L3 → 태그 ID 12 · 13 · 14, P2-01 ~ P2-04 → 바코드 문자열 그대로
센 것 (장마다):
  바코드 폴더: 읽음(ok) 비율, 틀린 문자열(ok 인데 정답과 다름) 수, 읽기 시간 중앙 · p95 · 최대 [ms]
  태그 폴더  : 정답 ID 를 본 비율, 다른 ID 를 본 장 수, 검출 시간 중앙 · p95 · 최대 [ms]
  모든 폴더  : 바코드가 아닌 폴더에서 바코드를 읽은 장 · 태그가 아닌 폴더에서 태그를 본 장 (잘못 읽기)
"""
import json
import re
import sys
from collections import defaultdict
from pathlib import Path


def pct(v, q):
    v = sorted(v)
    return v[min(len(v) - 1, int(round(q * (len(v) - 1))))] if v else float('nan')


def truth(clip):
    what = clip.split('_')[0]
    m = re.fullmatch(r'tag(\d{1,2})', what)
    if m:
        return 'tag', int(m.group(1))
    if what in ('L1', 'L2', 'L3'):
        return 'tag', 11 + int(what[1])
    if re.fullmatch(r'P2-0[1-4]', what):
        return 'barcode', what
    return 'unknown', what


clips, res = Path(sys.argv[1]), Path(sys.argv[2])
bc, tg = defaultdict(list), defaultdict(list)
for line in open(res / 'barcode.jsonl'):
    if line.strip():
        r = json.loads(line); bc[r['name'].split('/')[0]].append(r)
for line in open(res / 'tag.jsonl'):
    if line.strip():
        r = json.loads(line); tg[r['name'].split('/')[0]].append(r)

rows, allbc, alltag = [], [], []
print('| 폴더 | 정답 | 장 | 맞음 | 맞음 비율 | 틀림 장 | 다른 쪽 잘못 읽기 | 시간 중앙 · p95 · 최대 [ms] |')
print('|---|---|---|---|---|---|---|---|')
for clip in sorted(set(bc) | set(tg)):
    kind, ans = truth(clip)
    meta = clips / clip / 'meta.json'
    b, t = bc.get(clip, []), tg.get(clip, [])
    n = max(len(b), len(t))
    if kind == 'barcode':
        ok = sum(1 for r in b if r.get('ok') and r.get('text') == ans)
        wrong = sum(1 for r in b if r.get('ok') and r.get('text') != ans)
        other = sum(1 for r in t if r.get('ids'))
        ms = [r['ms'] for r in b if 'ms' in r]; allbc += ms
    elif kind == 'tag':
        ok = sum(1 for r in t if ans in r.get('ids', []))
        wrong = sum(1 for r in t if any(i != ans for i in r.get('ids', [])))
        other = sum(1 for r in b if r.get('ok'))
        ms = [r['ms'] for r in t if 'ms' in r]; alltag += ms
    else:
        ok = wrong = other = 0; ms = []
    rate = ok / n if n else float('nan')
    print(f'| {clip} | {ans} | {n} | {ok} | {rate:.3f} | {wrong} | {other} | {pct(ms, .5):.1f} · {pct(ms, .95):.1f} · {max(ms) if ms else float("nan"):.1f} |')
    rows.append(dict(clip=clip, kind=kind, truth=ans, frames=n, ok=ok, rate=rate, wrong=wrong, cross=other,
                     ms_p50=pct(ms, .5), ms_p95=pct(ms, .95), ms_max=max(ms) if ms else None,
                     meta=json.loads(meta.read_text()) if meta.exists() else None))
print(f'\n바코드 한 장 읽기 전체: 중앙 {pct(allbc, .5):.1f} · p95 {pct(allbc, .95):.1f} · 최대 {max(allbc) if allbc else float("nan"):.1f} ms ({len(allbc)} 장)')
print(f'태그 한 장 검출 전체:   중앙 {pct(alltag, .5):.1f} · p95 {pct(alltag, .95):.1f} · 최대 {max(alltag) if alltag else float("nan"):.1f} ms ({len(alltag)} 장)')
(res / 'summary.json').write_text(json.dumps(dict(rows=rows, barcode_ms=dict(p50=pct(allbc, .5), p95=pct(allbc, .95)),
                                                  tag_ms=dict(p50=pct(alltag, .5), p95=pct(alltag, .95))), ensure_ascii=False, indent=1))
