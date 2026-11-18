#!/usr/bin/env python3
"""실물 3D 자동 라벨 초안: 사람이 고친 YOLO 상자(사진) + 태그 코너 + 색 윤곽 + 내부값 → 퍽마다 자세 · 코너 2D.
    auto_label3d.py --set train3 --calib <camera_info.yaml> --out labels_train3.json   (보정 뒤 정식)
    auto_label3d.py --set train3 --f 800 --out x.json                                  (도구 확인만, 왜곡 0)
"""
import argparse, json, sys
from multiprocessing import Pool
from pathlib import Path
import cv2, numpy as np
sys.path.insert(0, str(Path(__file__).parent))
import puck3d as P, fit_pose as F
ROOT = Path('/home/pa4/git/lv2_next_module'); HERE = Path(__file__).parent
SETS = {'train3': (ROOT / '261104_1618_E3/train3_arm', HERE / 'tags_train3.jsonl'), 'final': (ROOT / '261105_2047_final_eval/final_arm', HERE / 'tags_final.jsonl')}
OK_SIL, OK_TAG_MM, OK_BOX = .90, 1.0, .85    # 이 값을 넘으면 '자동 통과', 아니면 사람이 먼저 본다. 태그 차는 mm(퍽이 가까워 px 기준은 너무 엄함, 11-09 사용자 승인)

def read_calib(path):
    """ROS camera_info yaml (calibrate_charuco.py 결과) → K, D"""
    import re
    t = Path(path).read_text()
    def arr(name):
        m = re.search(name + r'\s*:\s*\n\s*rows:.*\n\s*cols:.*\n\s*data:\s*\[([^\]]*)\]', t)
        return np.array([float(v) for v in m.group(1).split(',')])
    K = arr('camera_matrix').reshape(3, 3); D = arr('distortion_coefficients')
    return K, D

def tag_mm(r, fx):
    """태그 코너 차(px)를 퍽 거리에서 mm 로 바꾼 값."""
    return r.get('tag_px', 99) * float(np.linalg.norm(r['c'])) * 1000 / fx

def status(r, fx):
    if r is None: return 'fail'
    if r['n_tags'] == 0: return 'no_tag'
    return 'auto' if r.get('sil_iou', 0) >= OK_SIL and tag_mm(r, fx) <= OK_TAG_MM and r['box_iou'] >= OK_BOX else 'check'

def work(a_):
    d, it, tags, K, D, fo = a_
    cv2.setNumThreads(1)
    lab = d / 'labels' / (Path(it['image']).stem + '.txt')
    if not lab.exists(): return []
    img = cv2.imread(str(d / 'images' / it['image'])); H, W = img.shape[:2]; out = []
    B = P.read_yolo(lab, W, H)
    for bi, (cls, box) in enumerate(B):
        qs = []
        for t in tags:
            q = np.array(t['c']); m = q.mean(0)
            inside = box[0] <= m[0] <= box[2] and box[1] <= m[1] <= box[3]
            # 상자가 겹치면 가장 작은 상자의 태그로 본다
            if inside and min(((b[2] - b[0]) * (b[3] - b[1]), j) for j, (_, b) in enumerate(B) if b[0] <= m[0] <= b[2] and b[1] <= m[1] <= b[3])[1] == bi: qs.append(q)
        r = F.fit_puck(img, box, cls, qs, K, D, W=W, H=H, free_off=fo)
        o = dict(image=it['image'], source=it['source'], box_index=bi, cls=cls, box=np.round(box, 1).tolist(), tags_2d=[np.round(q, 2).tolist() for q in qs], status=status(r, K[0, 0]))
        if r is not None:
            o.update(R=r['R'].tolist(), c=r['c'].tolist(), corners_px=np.round(P.project(P.CORNERS, r['R'], r['c'], K, D), 2).tolist(), sil_iou=r.get('sil_iou'), tag_px=r.get('tag_px'), box_iou=r['box_iou'], n_tags=r['n_tags'], hyp=r['hyp'], tag_off=r['tag_off'], tags_body=[np.round(T, 5).tolist() for _, T in r['tags']])
        out.append(o)
    return out

def run(setn, K, D, pool, fo=False):
    d, tf = SETS[setn]; T = {}
    for ln in open(tf):
        r = json.loads(ln); T[(r['video'], r['frame'])] = r['tags']
    items = json.load(open(d / 'auto_label.json')); jobs = []
    for it in items:
        s = it['source'].split('/'); jobs.append((d, it, T.get((s[-4], s[-1]), []), K, D, fo))
    rows = []
    for o in pool.imap(work, jobs, chunksize=2): rows += o
    return rows

def summary(rows):
    import collections
    c = collections.Counter(r['status'] for r in rows); tg = [r for r in rows if r.get('n_tags', 0) > 0]
    med = lambda k, S: float(np.median([r[k] for r in S if r.get(k) is not None])) if S else float('nan')
    return dict(n=len(rows), **c, tag_n=len(tg), two_tags=sum(r.get('n_tags', 0) > 1 for r in rows), sil_med=med('sil_iou', tg), tag_px_med=med('tag_px', tg), box_med=med('box_iou', tg))

if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('--set', required=True, choices=SETS); p.add_argument('--f', type=float); p.add_argument('--calib'); p.add_argument('--out', required=True)
    p.add_argument('--tag_off', action='store_true', help='태그 자리 어긋남(면 안 2방향, 한계는 fit_pose.LIM_* 실측값)도 푼다. 실물 퍽은 태그를 손으로 붙여 켬'); a = p.parse_args()
    if a.calib: K, D = read_calib(a.calib)
    else: K, D = np.array([[a.f, 0, 640], [0, a.f, 360], [0, 0, 1.]]), None
    with Pool(16) as po: rows = run(a.set, K, D, po, a.tag_off)
    json.dump(dict(K=K.tolist(), D=None if D is None else D.tolist(), set=a.set, calibrated=bool(a.calib), tag_off=a.tag_off, rows=rows), open(a.out, 'w'), ensure_ascii=False)
    print(summary(rows))
