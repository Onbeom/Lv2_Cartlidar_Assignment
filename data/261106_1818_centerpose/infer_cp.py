"""CenterPose 추론 + 시간 측정. 상자 = E3(배포와 같은 입력 384x640, conf .25) 또는 정답 투영 상자(oracle, 검출 영향 분리용). 출력은 점수와 분리해 jsonl 로 낸다.
    infer_cp.py --data data/eval --ckpt runs/s128/best.pt --box e3 --out results/cp_s128_e3.jsonl [--device cpu]
"""
import argparse, json, sys, time
from pathlib import Path
import cv2, numpy as np, torch
HERE = Path(__file__).resolve().parent; sys.path.insert(0, str(HERE))
from train_crop import CORNERS, PERMS, crop_matrix, make_model, to_tensor
p = argparse.ArgumentParser(); p.add_argument('--data', required=True); p.add_argument('--ckpt', required=True); p.add_argument('--box', choices=['e3', 'oracle'], required=True); p.add_argument('--out', type=Path, required=True); p.add_argument('--device', default='cuda'); p.add_argument('--limit', type=int, default=0); p.add_argument('--tta', action='store_true'); p.add_argument('--agnostic', action='store_true', help='설계 3: 클래스가 달라도 겹친 E3 상자를 하나로')
a = p.parse_args(); torch.set_num_threads(4); dev = a.device
J = json.loads((Path(a.data) / 'truth.json').read_text()); K = np.array(J['K']); DIST = np.array(J.get('D', [0.] * 5)); imgs = J['images'][:a.limit or None]
ck = torch.load(a.ckpt, map_location='cpu', weights_only=False); size = ck['size']; grid = size // 4
model = make_model(); model.load_state_dict(ck['model']); model.to(dev).eval()
yolo = None
if a.box == 'e3':
    from ultralytics import YOLO
    yolo = YOLO('/home/pa4/git/lv2_next_module/261104_1618_E3/runs/E3/weights/last.pt')  # export_e3.py 와 같은 원본
def solve(pix, s, r):
    ok, rv, tv = cv2.solvePnP(CORNERS, pix.astype(np.float64), K, DIST, flags=cv2.SOLVEPNP_EPNP)
    if ok:
        rv, tv = cv2.solvePnPRefineLM(CORNERS, pix.astype(np.float64), K, DIST, rv, tv)
        pr, _ = cv2.projectPoints(CORNERS, rv, tv, K, DIST); err = float(np.linalg.norm(pr.reshape(-1, 2) - pix, axis=1).mean()); R = cv2.Rodrigues(rv)[0]
        r.update(center=tv.ravel().tolist(), z=R[:, 2].tolist(), R=R.tolist(), reproj_px=err, corners_px=pix.tolist(), valid=bool(tv[2, 0] > 0 and np.isfinite(tv).all() and err < 10 and s >= .25))
    return r
def pose_tta(im, box):
    # 교안 v2 방식: 상자 0.9 · 1.0 · 1.1 배, 대칭을 맞춰 코너 평균 → PnP. 추론 3번
    base = pose1(im, box); x1, y1, x2, y2 = box; cx, cy = (x1 + x2) / 2, (y1 + y2) / 2; ref = np.array(base['corners_px']); A = [ref]
    for k in (.9, 1.1):
        w, h = (x2 - x1) * k / 2, (y2 - y1) * k / 2; p2 = np.array(pose1(im, [cx - w, cy - h, cx + w, cy + h])['corners_px'])
        A.append(min((p2[q] for q in PERMS.numpy()), key=lambda v: np.linalg.norm(v - ref)))
    return solve(np.mean(A, axis=0), base['score'], dict(base, valid=False))
def pose(im, box): return pose_tta(im, box) if a.tta else pose1(im, box)
@torch.inference_mode()
def pose1(im, box):
    m = crop_matrix(box, size); crop = cv2.warpAffine(im, m, (size, size), flags=cv2.INTER_LINEAR, borderValue=(114, 114, 114))
    o = model(to_tensor(crop)[None].to(dev))[0]; h = o['hm'][0, 0].sigmoid(); f = int(h.argmax()); y, x = divmod(f, grid); s = float(h[y, x])
    ij = np.array([x, y], np.float32); ct = (ij + o['reg'][0, :, y, x].cpu().numpy()) * 4; c = (o['hps'][0, :, y, x].cpu().numpy().reshape(8, 2) + ij) * 4
    inv = cv2.invertAffineTransform(m); pix = c @ inv[:, :2].T + inv[:, 2]; cpx = ct @ inv[:, :2].T + inv[:, 2]
    r = dict(score=s, center_px=cpx.tolist(), corners_px=pix.tolist(), valid=False)
    ok, rv, tv = cv2.solvePnP(CORNERS, pix.astype(np.float64), K, DIST, flags=cv2.SOLVEPNP_EPNP)
    if ok:
        rv, tv = cv2.solvePnPRefineLM(CORNERS, pix.astype(np.float64), K, DIST, rv, tv)
        pr, _ = cv2.projectPoints(CORNERS, rv, tv, K, DIST); err = float(np.linalg.norm(pr.reshape(-1, 2) - pix, axis=1).mean())
        R = cv2.Rodrigues(rv)[0]
        r.update(center=tv.ravel().tolist(), z=R[:, 2].tolist(), R=R.tolist(), reproj_px=err, valid=bool(tv[2, 0] > 0 and np.isfinite(tv).all() and err < 10 and s >= .25))
    return r
def sync():
    if dev == 'cuda': torch.cuda.synchronize()
# 데우기: 시간 표본에서 뺀다
warm = cv2.imread(str(Path(a.data) / imgs[0]['image']))
for _ in range(5):
    pose(warm, [600, 300, 700, 400])
    if yolo: yolo.predict(warm, imgsz=(384, 640), conf=.25, iou=.45, device=0 if dev == 'cuda' else 'cpu', verbose=False)
sync()
a.out.parent.mkdir(parents=True, exist_ok=True)
with a.out.open('w') as f:
    for row in imgs:
        im = cv2.imread(str(Path(a.data) / row['image'])); t0 = time.perf_counter(); dets = []
        if yolo:
            r = yolo.predict(im, imgsz=(384, 640), conf=.25, iou=.45, agnostic_nms=a.agnostic, device=0 if dev == 'cuda' else 'cpu', verbose=False)[0]
            dets = [dict(box=b.xyxy[0].cpu().tolist(), cls=int(b.cls), conf=float(b.conf)) for b in r.boxes]
        else:  # oracle: 시간 재기 전에 정답 상자를 정해 두는 것과 같음, 추정값에는 상자만 들어감
            for p_ in row['pucks']:
                uv = np.array(p_['corners_px']); dets.append(dict(box=[float(max(0, uv[:, 0].min())), float(max(0, uv[:, 1].min())), float(min(J['width'], uv[:, 0].max())), float(min(J['height'], uv[:, 1].max()))], cls=-1, conf=1.0))
        sync(); t1 = time.perf_counter()
        for d in dets: d.update(pose(im, d['box']))
        sync(); t2 = time.perf_counter()
        f.write(json.dumps(dict(id=row['id'], ms_det=1e3 * (t1 - t0), ms_pose=1e3 * (t2 - t1), ms_all=1e3 * (t2 - t0), dets=dets)) + '\n')
