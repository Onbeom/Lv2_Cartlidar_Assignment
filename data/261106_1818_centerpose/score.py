"""점수: 정답은 여기서만 읽는다. 대상 = visible >= .5 인 퍽. 성공 = 중심 <= 10 mm + 긴 축 <= 10°(부호 무시).
    score.py --data data/eval --tag results/tag.jsonl --cp results/cp_x.jsonl ... --out results/summary.json
"""
import argparse, json
from pathlib import Path
import numpy as np
p = argparse.ArgumentParser(); p.add_argument('--data', required=True); p.add_argument('--tag'); p.add_argument('--cp', nargs='*', default=[]); p.add_argument('--out', type=Path, required=True)
a = p.parse_args(); J = json.loads((Path(a.data) / 'truth.json').read_text())
def cond(q):
    if not q['lying']: return 'standing_' + q['shape']
    return 'lying_tag_up' if q['tag6'] else 'lying_no_up_tag'
targets = []
for im in J['images']:
    for q in im['pucks']:
        if q['visible'] >= .5:
            targets.append(dict(img=im['id'], tag_id=q['tag_id'], cond=cond(q), tag_seen=q['tag_visible_max'] >= .9, c=np.array(q['center_cam']), z=np.array(q['R_cam'])[:, 2], uv=np.array(q['corners_px']), vis=q['visible'], dist=q['dist_m'], lab=(q['color'], q['shape'])))
def err(t, c, z):
    e = float(np.linalg.norm(np.array(c) - t['c'])); ang = None if z is None else float(np.degrees(np.arccos(min(1, abs(np.dot(z, t['z']) / np.linalg.norm(z))))))
    return e, ang
def iou(a_, b):
    lo = np.maximum(a_[:2], b[:2]); hi = np.minimum(a_[2:], b[2:]); i = np.prod(np.maximum(hi - lo, 0)); return float(i / ((a_[2:] - a_[:2]).prod() + (b[2:] - b[:2]).prod() - i + 1e-9))
def stats(v, q=(50, 95)): return [round(float(np.percentile(v, x)), 3) for x in q] if len(v) else None
def table(rows):
    """rows: 대상마다 (t, 값 또는 None)"""
    def one(sel):
        S = [r for r in rows if sel(r[0])]; O = [r[1] for r in S if r[1] is not None]; F = [o for o in O if o['ang'] is not None]
        return dict(n=len(S), output=len(O), output_rate=round(len(O) / max(1, len(S)), 4), orientation=len(F), success=sum(o['e'] <= .01 and o['ang'] <= 10 for o in F), success_rate=round(sum(o['e'] <= .01 and o['ang'] <= 10 for o in F) / max(1, len(S)), 4),
                    center_mm_p50_p95=stats([1e3 * o['e'] for o in O]), axis_deg_p50_p95=stats([o['ang'] for o in F]))
    out = {'all': one(lambda t: True), 'tag_seen': one(lambda t: t['tag_seen']), 'tag_not_seen': one(lambda t: not t['tag_seen'])}
    for c in sorted({r[0]['cond'] for r in rows}): out[c] = one(lambda t, c=c: t['cond'] == c)
    return out
R = {'n_images': len(J['images']), 'synthetic_only': True}
byimg = {}
for t in targets: byimg.setdefault(t['img'], []).append(t)
if a.tag:
    P = [json.loads(x) for x in open(a.tag)]; rows = []; wrong = extra = 0; allids = {im['id']: {q['tag_id'] for q in im['pucks']} for im in J['images']}
    for r in P:
        ts = {t['tag_id']: t for t in byimg.get(r['name'], [])}; got = {}
        for q in r['pucks']:
            if q['id'] not in allids[r['name']]: wrong += 1
            elif q['id'] in ts: got[q['id']] = q
        for tid, t in ts.items():
            if tid in got:
                e, ang = err(t, got[tid]['center'], np.array(got[tid]['z']) if got[tid]['orientation_valid'] else None); rows.append((t, dict(e=e, ang=ang)))
            else: rows.append((t, None))
    R['tag'] = dict(table=table(rows), wrong_id=wrong, ms_all_p50_p95=stats([r['ms_all'] for r in P]), ms_max=float(max(r['ms_all'] for r in P)))
for path in a.cp:
    P = [json.loads(x) for x in open(path)]; rows = []; fp = 0; conf_ok = conf_n = 0
    NAMES = None
    for r in P:
        ts = byimg.get(r['id'], []); used = {}; 
        im = next(i for i in J['images'] if i['id'] == r['id']); alls = im['pucks']
        boxes = [np.array([*np.clip(np.array(q['corners_px']), [0, 0], [J['width'], J['height']]).min(0), *np.clip(np.array(q['corners_px']), [0, 0], [J['width'], J['height']]).max(0)]) for q in alls]
        for d in sorted([d for d in r['dets'] if d['valid']], key=lambda d: -d['score']):
            ov = [iou(np.array(d['box']), b) for b in boxes]; j = int(np.argmax(ov)) if ov else -1
            if j < 0 or ov[j] < .3 or j in used: fp += 1; continue
            used[j] = d
        for t in ts:
            j = next(k for k, q in enumerate(alls) if q['tag_id'] == t['tag_id'])
            if j in used:
                e, ang = err(t, used[j]['center'], np.array(used[j]['z'])); rows.append((t, dict(e=e, ang=ang)))
            else: rows.append((t, None))
    R[Path(path).stem] = dict(table=table(rows), false_outputs=fp, ms_all_p50_p95=stats([r['ms_all'] for r in P]), ms_det_p50_p95=stats([r['ms_det'] for r in P]), ms_pose_p50_p95=stats([r['ms_pose'] for r in P]), ms_pose_per_det_p50=stats([r['ms_pose'] / max(1, len(r['dets'])) for r in P if r['dets']], (50,)))
a.out.write_text(json.dumps(R, ensure_ascii=False, indent=1))
for k, v in R.items():
    if isinstance(v, dict): print(k, json.dumps(v['table']['all'], ensure_ascii=False), json.dumps({x: v[x] for x in v if x.startswith('ms_') or x in ('wrong_id', 'false_outputs')}))
