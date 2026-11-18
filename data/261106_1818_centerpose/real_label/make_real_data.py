#!/usr/bin/env python3
"""실물 라벨(자동 통과 + 사람 수정)을 truth.json 형식으로 바꾼다. 뺌: 모형 상자(화면 안으로 자름)와 사람 상자 IoU < 0.6.
    train = train3_arm v2~v5, val = train3_arm v1(영상 통째로), final = final_arm(채점만, 학습 금지).  make_real_data.py"""
import json, sys
from pathlib import Path
import cv2, numpy as np
HERE = Path(__file__).resolve().parent; sys.path.insert(0, str(HERE))
import auto_label3d as A
CORNERS = np.array([[x, y, z] for x in (-.015, .015) for y in (-.015, .015) for z in (-.03, .03)], float)
MIN_IOU, W, H = .6, 1280, 720
def canon(R, c):
    """원기둥은 긴 축 둘레 각도가 안 보이므로 가상 상자 x 축을 시선과 긴 축에 수직으로 정한다(자세 채점 값인 중심 · 긴 축은 그대로)."""
    z = R[:, 2]; x = np.cross(c / np.linalg.norm(c), z); x /= np.linalg.norm(x); return np.c_[x, np.cross(z, x), z]
def iou(a, b):
    lo = np.maximum(a[:2], b[:2]); hi = np.minimum(a[2:], b[2:]); i = np.prod(np.maximum(hi - lo, 0)); return float(i / ((a[2:] - a[:2]).prod() + (b[2:] - b[:2]).prod() - i + 1e-9))
def build(lab, img_root, keep_video, out):
    J = json.loads((HERE / lab).read_text()); K = np.array(J['K'], float); D = np.array(J['D'], float).ravel(); by = {}; drop = 0
    for r in J['rows']: by.setdefault(r['image'], []).append(r)
    imgs = []
    for name, S in sorted(by.items()):
        if not keep_video(name.split('__')[0]): continue
        P = []
        for r in S:
            if not (r['status'] == 'auto' or 'human' in r): continue
            R = np.array(r['R'], float); c = np.array(r['c'], float); R = canon(R, c) if r['cls'] < 2 else R; rv = cv2.Rodrigues(R)[0]; uv = cv2.projectPoints(CORNERS, rv, c, K, D)[0].reshape(-1, 2)
            ct = cv2.projectPoints(np.zeros((1, 3)), rv, c, K, D)[0].ravel()
            from puck3d import CYL
            M = CYL if r['cls'] < 2 else CORNERS; v = cv2.projectPoints(M, rv, c, K, D)[0].reshape(-1, 2)
            if iou(np.clip(np.r_[v.min(0), v.max(0)], 0, [W, H, W, H]), np.array(r['box'], float)) < MIN_IOU: drop += 1; continue
            P.append(dict(box=r['box'], corners_px=uv.tolist(), center_px=ct.tolist(), center_cam=c.tolist(), R_cam=R.tolist(), cls=r['cls'], n_tags=r['n_tags'], human='human' in r, visible=1.0))
        if P: imgs.append(dict(id=Path(name).stem, image=str(img_root / name), pucks=P, all_boxes=[r['box'] for r in S]))
    (out).mkdir(parents=True, exist_ok=True); (out / 'truth.json').write_text(json.dumps(dict(width=W, height=H, K=K.tolist(), D=D.tolist(), images=imgs)))
    print(out.name, '사진', len(imgs), '퍽', sum(len(i['pucks']) for i in imgs), '뺌', drop, '영상별', {v: sum(len(i['pucks']) for i in imgs if i['id'].startswith(v)) for v in sorted({i['id'].split('__')[0] for i in imgs})})
if __name__ == '__main__':
    tr = A.SETS['train3'][0] / 'images'; fi = A.SETS['final'][0] / 'images'; O = HERE / 'data'
    build('labels_train3.json', tr, lambda v: v != 'train3_arm_v1', O / 'train')
    build('labels_train3.json', tr, lambda v: v == 'train3_arm_v1', O / 'val')
    build('labels_final.json', fi, lambda v: True, O / 'final')
