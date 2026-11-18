#!/usr/bin/env python3
"""render_real.py 결과(바늘구멍 1920x1080)를 실측 렌즈(K, D, 1280x720)로 바꾸고 truth 를 실물 형식으로 쓴다. 모의.  warp_sim.py <렌더 폴더> <출력 폴더> <실물 truth.json>"""
import json, sys
from pathlib import Path
import cv2, numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))
from make_real_data import CORNERS, canon
from puck3d import CYL
src, out, ref = map(Path, sys.argv[1:4]); S = json.loads((src / 'truth.json').read_text()); Rf = json.loads(ref.read_text()); K = np.array(Rf['K'], float); D = np.array(Rf['D'], float); W, H = 1280, 720; Kr = np.array(S['K'], float)
g = np.stack(np.meshgrid(np.arange(W, dtype=np.float64), np.arange(H, dtype=np.float64)), -1).reshape(-1, 2); n = cv2.undistortPoints(g[None], K, D).reshape(-1, 2)
mx = (Kr[0, 0] * n[:, 0] + Kr[0, 2]).reshape(H, W).astype(np.float32); my = (Kr[1, 1] * n[:, 1] + Kr[1, 2]).reshape(H, W).astype(np.float32)
(out / 'images').mkdir(parents=True, exist_ok=True); imgs = []; kept = drop = 0
for im in S['images']:
    I = cv2.imread(str(src / im['image'])); I = cv2.GaussianBlur(I, (0, 0), .6); J_ = cv2.remap(I, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE); fn = out / 'images' / Path(im['image']).name; cv2.imwrite(str(fn), J_); P = []
    for p in im['pucks']:
        c = np.array(p['center_cam'], float); R = np.array(p['R_cam'], float)
        if p['shape'] == 'cylinder': R = canon(R, c)
        Q = CORNERS @ R.T + c
        if p['visible'] < .3 or Q[:, 2].min() < .03 or np.abs(Q[:, :2] / Q[:, 2, None]).max() > 1.3: drop += 1; continue
        rv = cv2.Rodrigues(R)[0]; uv = cv2.projectPoints(CORNERS, rv, c, K, D)[0].reshape(-1, 2); ct = cv2.projectPoints(np.zeros((1, 3)), rv, c, K, D)[0].ravel(); v = cv2.projectPoints(CYL if p['shape'] == 'cylinder' else CORNERS, rv, c, K, D)[0].reshape(-1, 2)
        a = np.r_[v.min(0), v.max(0)]; b = np.clip(a, 0, [W, H, W, H]); area = (b[2:] - b[:2]).prod()
        if area < .6 * (a[2:] - a[:2]).prod() or area < 400: drop += 1; continue
        P.append(dict(box=b.tolist(), corners_px=uv.tolist(), center_px=ct.tolist(), center_cam=c.tolist(), R_cam=R.tolist(), cls=(0 if p['shape'] == 'cylinder' else 2) + (p['color'] != 'red'), n_tags=int(p['tag_visible_max'] >= .9), human=False, visible=p['visible'])); kept += 1
    if P: imgs.append(dict(id=im['id'], image=str(fn), pucks=P, all_boxes=[q['box'] for q in P]))
(out / 'truth.json').write_text(json.dumps(dict(width=W, height=H, K=K.tolist(), D=D.tolist(), images=imgs, synthetic_only=True))); print(out.name, '사진', len(imgs), '퍽', kept, '뺌', drop)
