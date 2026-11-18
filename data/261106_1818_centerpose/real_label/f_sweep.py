#!/usr/bin/env python3
"""f 후보마다 태그 PnP → 퍽 모양 투영 상자가 사람이 고친 YOLO 상자와 얼마나 맞는지 본다. 보정을 대신하지 않는 단서."""
import json, sys
from pathlib import Path
import numpy as np
import puck3d as P
ROOT = Path('/home/pa4/git/lv2_next_module')
SETS = {'train3': (ROOT / '261104_1618_E3/train3_arm', 'tags_train3.jsonl'), 'final': (ROOT / '261105_2047_final_eval/final_arm', 'tags_final.jsonl')}
def pairs(setdir, tagf):
    T = {}
    for ln in open(tagf):
        r = json.loads(ln); T[(r['video'], r['frame'])] = r['tags']
    out = []
    for it in json.load(open(setdir / 'auto_label.json')):
        s = it['source'].split('/'); v, fr = s[-4], s[-1]
        lab = setdir / 'labels' / (Path(it['image']).stem + '.txt')
        if not lab.exists(): continue
        B = P.read_yolo(lab)
        for tg in T.get((v, fr), []):
            cx = np.mean(tg['c'], 0)
            hit = [(c, b) for c, b in B if b[0] <= cx[0] <= b[2] and b[1] <= cx[1] <= b[3]]
            if len(hit) == 1: out.append((it['image'], tg, hit[0][0], hit[0][1]))
    return out
def best(tg, cls, box, K):
    b = (0, None)
    for R, t, e in P.tag_solutions(tg['c'], K):
        if t[2] <= 0: continue
        for name, Rp, c in P.hypotheses(R, t, cls):
            if (c[2] - P.HL) <= 0.005: continue
            i = P.iou(P.bbox(P.project(P.model_pts(cls), Rp, c, K)), box)
            if i > b[0]: b = (i, name)
    return b
if __name__ == '__main__':
    for name, (d, tf) in SETS.items():
        Q = pairs(d, tf); print(name, '태그-상자 쌍', len(Q), '태그 변 중앙', np.median([q[1]['side_px'] for q in Q]) if Q else None)
        for lo in (0, 40):
            S = [q for q in Q if q[1]['side_px'] >= lo and not (q[3][0] < 3 or q[3][1] < 3 or q[3][2] > 1277 or q[3][3] > 717)]
            print(f' 태그 변 >= {lo} px, 화면 안 상자 {len(S)}개')
            for f in (280, 350, 424, 500, 566, 640, 720, 800, 900, 1000):
                K = np.array([[f, 0, 640], [0, f, 360], [0, 0, 1.]])
                r = [best(q[1], q[2], q[3], K) for q in S]; I = np.array([x[0] for x in r])
                ends = sum(x[1] == 'end' for x in r if x[1])
                print(f'  f {f:5d}: IoU 중앙 {np.median(I):.3f} p25 {np.percentile(I,25):.3f} >=.8 {np.mean(I>=.8)*100:5.1f} % 끝면 해 {ends}')
