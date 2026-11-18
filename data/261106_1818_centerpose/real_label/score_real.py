#!/usr/bin/env python3
"""실물 채점: 정답은 여기서만 읽는다. 성공 = 중심 <= 10 mm + 긴 축 <= 10°(부호 무시). 판정 구간 = 거리 < 500 mm.
    score_real.py --data data/final --cp x.jsonl --out x.json"""
import argparse, json
from pathlib import Path
import numpy as np
p = argparse.ArgumentParser(); p.add_argument('--data', required=True); p.add_argument('--cp', required=True); p.add_argument('--out', type=Path, required=True); a = p.parse_args()
J = json.loads((Path(a.data) / 'truth.json').read_text()); P = {json.loads(x)['id']: json.loads(x) for x in open(a.cp)}
def iou(a_, b):
    lo = np.maximum(a_[:2], b[:2]); hi = np.minimum(a_[2:], b[2:]); i = np.prod(np.maximum(hi - lo, 0)); return float(i / ((a_[2:] - a_[:2]).prod() + (b[2:] - b[:2]).prod() - i + 1e-9))
rows = []; fp = 0
for im in J['images']:
    r = P[im['id']]; A = [np.array(b, float) for b in im['all_boxes']]; used = {}
    for d in sorted([d for d in r['dets'] if d['valid']], key=lambda d: -d['score']):
        ov = [iou(np.array(d['box']), b) for b in A]; j = int(np.argmax(ov))
        if ov[j] < .3 or j in used: fp += 1; continue
        used[j] = d
    for q in im['pucks']:
        j = next(k for k, b in enumerate(im['all_boxes']) if b == q['box']); c = np.array(q['center_cam']); z = np.array(q['R_cam'])[:, 2]; o = dict(img=im['id'], d=float(np.linalg.norm(c)), tag=q['n_tags'] > 0, cyl=q['cls'] < 2, human=q['human'], out=j in used)
        if j in used:
            d = used[j]; o['e'] = float(np.linalg.norm(np.array(d['center']) - c)) * 1e3; zz = np.array(d['z']); o['a'] = float(np.degrees(np.arccos(min(1, abs(zz @ z) / np.linalg.norm(zz)))))
            o['ok'] = o['e'] <= 10 and o['a'] <= 10
        else: o['ok'] = False
        rows.append(o)
def wil(k, n, z=1.96):
    p_ = k / n; d = 1 + z * z / n; c = p_ + z * z / (2 * n); m = z * np.sqrt(p_ * (1 - p_) / n + z * z / (4 * n * n)); return round(100 * (c - m) / d, 1), round(100 * (c + m) / d, 1)
def tb(S):
    if not S: return dict(n=0)
    O = [r for r in S if r['out']]; k = sum(r['ok'] for r in S)
    return dict(n=len(S), output=len(O), success=k, rate=round(100 * k / len(S), 1), ci95=wil(k, len(S)), center_mm_p50_p95=[round(float(np.percentile([r['e'] for r in O], q)), 2) for q in (50, 95)] if O else None, axis_deg_p50_p95=[round(float(np.percentile([r['a'] for r in O], q)), 2) for q in (50, 95)] if O else None)
N = [r for r in rows if r['d'] < .5]; ms = np.array([r['ms_all'] for r in P.values()])
R = dict(C1_500mm=tb(N), C3_500mm_no_tag=tb([r for r in N if not r['tag']]), ref_tag=tb([r for r in N if r['tag']]), ref_300mm=tb([r for r in N if r['d'] < .3]), ref_300_500=tb([r for r in N if r['d'] >= .3]), ref_cyl=tb([r for r in N if r['cyl']]), ref_box=tb([r for r in N if not r['cyl']]), ref_over_500=tb([r for r in rows if r['d'] >= .5]), ref_false_out=fp, ms_all_p50_p95=[round(float(np.percentile(ms, q)), 2) for q in (50, 95)], n_frames=len(ms))
a.out.parent.mkdir(parents=True, exist_ok=True); a.out.write_text(json.dumps(dict(summary=R, rows=rows), ensure_ascii=False, indent=1))
for k, v in R.items(): print(k, v)
