#!/usr/bin/env python3
"""저장한 코너로 PnP 만 다시: 코너 퍼짐 보정(s 배) + 유효 판정(재투영 <= max(10 px, rel x 상자 긴 변)).  repnp.py in.jsonl out.jsonl data_dir --s 1.03 --rel .05"""
import argparse, json
from pathlib import Path
import cv2, numpy as np
C = np.array([[x, y, z] for x in (-.015, .015) for y in (-.015, .015) for z in (-.03, .03)], float)
def redo(d, K, D, s, rel):
    if 'corners_px' not in d: return d
    p = np.array(d['corners_px'], float); m = p.mean(0); q = m + s * (p - m)
    ok, rv, tv = cv2.solvePnP(C, q, K, D, flags=cv2.SOLVEPNP_EPNP)
    if not ok: d['valid'] = False; return d
    rv, tv = cv2.solvePnPRefineLM(C, q, K, D, rv, tv); pr = cv2.projectPoints(C, rv, tv, K, D)[0].reshape(-1, 2); err = float(np.linalg.norm(pr - q, axis=1).mean()); R = cv2.Rodrigues(rv)[0]; b = d['box']; L = max(b[2] - b[0], b[3] - b[1])
    d.update(center=tv.ravel().tolist(), z=R[:, 2].tolist(), R=R.tolist(), reproj_px=err, valid=bool(tv[2, 0] > 0 and np.isfinite(tv).all() and err <= max(10., rel * L) and d['score'] >= .25)); return d
if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('inp'); p.add_argument('out'); p.add_argument('data'); p.add_argument('--s', type=float, default=1.); p.add_argument('--rel', type=float, default=.05); a = p.parse_args()
    J = json.loads((Path(a.data) / 'truth.json').read_text()); K = np.array(J['K'], float); D = np.array(J['D'], float)
    with open(a.out, 'w') as f:
        for x in open(a.inp):
            r = json.loads(x); [redo(d, K, D, a.s, a.rel) for d in r['dets']]; f.write(json.dumps(r) + '\n')
