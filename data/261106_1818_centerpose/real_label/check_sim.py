#!/usr/bin/env python3
"""라벨 도구 확인(모의): data/val 합성 영상에서 라벨 도구가 정답을 얼마나 되찾는지. 판정 아님.
    check_sim.py <출력 json> [--fscale 1.0] [--max 150] [--tag_off]"""
import argparse, json, sys
from multiprocessing import Pool
from pathlib import Path
import cv2, numpy as np
sys.path.insert(0, str(Path(__file__).parent))
import puck3d as P, fit_pose as F
ROOT = Path(__file__).resolve().parent.parent / 'data/val'
pa = cv2.aruco.DetectorParameters(); pa.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
DET = None
def work(a_):
    global DET
    im, Ktrue, fs, fo = a_
    cv2.setNumThreads(1)
    if DET is None: DET = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), pa)
    img = cv2.imread(str(ROOT / im['image'])); c, ids, _ = DET.detectMarkers(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)); K = np.array(Ktrue, float); K[0, 0] *= fs; K[1, 1] *= fs
    tags = [] if ids is None else [(int(i), q.reshape(4, 2)) for q, i in zip(c, ids.ravel())]; out = []
    for q in im['pucks']:
        if q['visible'] < .5: continue
        uv = np.array(q['corners_px']); box = P.bbox(uv) if q['shape'] == 'cuboid' else P.bbox(P.project(P.CYL, np.array(q['R_cam']), np.array(q['center_cam']), np.array(Ktrue, float)))
        if box[2] - box[0] < 8 or box[3] - box[1] < 8: continue
        cls = (2 if q['shape'] == 'cuboid' else 0) + (0 if q['color'] == 'red' else 1)
        qs = [t for i, t in tags if i == q['tag_id'] and box[0] <= t[:, 0].mean() <= box[2] and box[1] <= t[:, 1].mean() <= box[3]]
        r = F.fit_puck(img, box, cls, qs, K, free_off=fo)
        if r is None: out.append(dict(img=im['id'], n_tags=len(qs), ok=False)); continue
        e = float(np.linalg.norm(r['c'] - np.array(q['center_cam']))); ax = float(np.degrees(np.arccos(min(1, abs(r['R'][:, 2] @ np.array(q['R_cam'])[:, 2])))))
        out.append(dict(img=im['id'], ok=True, n_tags=r['n_tags'], e_mm=e * 1e3, ax=ax, sil=r.get('sil_iou'), tag_px=r.get('tag_px'), box_iou=r['box_iou'], hyp=r['hyp'], dist=q['dist_m'], lying=q['lying'], shape=q['shape']))
    return out
if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('out'); p.add_argument('--fscale', type=float, default=1.); p.add_argument('--max', type=int, default=150); p.add_argument('--tag_off', action='store_true'); a = p.parse_args()
    J = json.loads((ROOT / 'truth.json').read_text()); rows = []
    with Pool(16) as po:
        for o in po.imap(work, [(im, J['K'], a.fscale, a.tag_off) for im in J['images'][:a.max]]): rows += o
    json.dump(rows, open(a.out, 'w'))
    def st(S, name):
        S = [r for r in S if r['ok']]
        if not S: print(name, 0); return
        E = np.array([r['e_mm'] for r in S]); A = np.array([r['ax'] for r in S])
        print(f"{name}: n {len(S)} 중심 중앙 {np.median(E):.2f} p95 {np.percentile(E,95):.2f} mm · 긴 축 중앙 {np.median(A):.2f} p95 {np.percentile(A,95):.2f}° · 중심<=5 mm & 축<=5° {np.mean((E<=5)&(A<=5))*100:.1f} %")
    print('f 배율', a.fscale, '실패', sum(not r['ok'] for r in rows), '/', len(rows))
    st([r for r in rows if r['n_tags'] > 0], '태그 있음'); st([r for r in rows if r['n_tags'] > 1], ' 태그 둘 이상'); st([r for r in rows if r['n_tags'] == 0], '태그 없음')
    for t in (.95, .9, .8): st([r for r in rows if r.get('sil', 0) >= t and r['n_tags'] > 0], f' 태그 있음, 윤곽 IoU>={t}')
