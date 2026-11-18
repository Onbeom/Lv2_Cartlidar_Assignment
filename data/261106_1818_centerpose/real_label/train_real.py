#!/usr/bin/env python3
"""실물 미세 조정: 시뮬레이션 모델에서 이어 학습. 왜곡이 있으므로 중심은 truth 의 center_px 를 쓴다. 코너 대칭 손실은 train_crop 과 같다.
    train_real.py --data data/train --val data/val --out runs/r1 --resume ../runs/d2/best.pt [--sim_data ../data/train --sim_n 200]"""
import argparse, json, random, sys, time
from pathlib import Path
import cv2, numpy as np, torch
from torch import nn
from torch.utils.data import DataLoader, Dataset
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import train_crop as T
class Real(Dataset):
    def __init__(self, data, size, train, rot, dets=None, sim=None, sim_n=0, real_rep=1):
        J = json.loads((Path(data) / 'truth.json').read_text()); self.size = size; self.train = train; self.grid = size // 4; self.rot = rot; self.items = []
        for im in J['images']:
            for p in im['pucks']:
                gt = np.array(p['box'], float); e3 = None; best = .3
                for b in (dets or {}).get(im['id'], []):
                    b = np.array(b); lo = np.maximum(gt[:2], b[:2]); hi = np.minimum(gt[2:], b[2:]); i = np.prod(np.maximum(hi - lo, 0)); v = i / (np.prod(gt[2:] - gt[:2]) + np.prod(b[2:] - b[:2]) - i + 1e-9)
                    if v > best: best = v; e3 = b.tolist()
                self.items.append((im['image'], np.array(p['corners_px'], np.float32), np.array(p['center_px'], np.float32), p['box'], e3))
        self.items *= real_rep; self.n_real = len(self.items)
        if sim and sim_n:
            S = json.loads((Path(sim) / 'truth.json').read_text())
            it = [(im['image'], np.array(p['corners_px'], np.float32), np.array(p['center_px'], np.float32), p['box'], None) for im in S['images'] for p in im['pucks']]
            self.items += random.Random(0).sample(it, min(sim_n, len(it)))
    def __len__(self): return len(self.items)
    def __getitem__(self, i):
        name, uv, ct, box, e3 = self.items[i]; im = cv2.imread(name)
        r = random.Random(np.random.randint(1 << 30)) if self.train else None
        if e3 is not None and r is not None and r.random() < .5: box = e3
        m = T.crop_matrix(box, self.size, jit=r)
        if r is not None and self.rot:
            G = cv2.getRotationMatrix2D((self.size / 2, self.size / 2), r.uniform(-self.rot, self.rot), 1.); m = (np.vstack([G, [0, 0, 1]]) @ np.vstack([m, [0, 0, 1]]))[:2].astype(np.float32)
        crop = cv2.warpAffine(im, m, (self.size, self.size), flags=cv2.INTER_LINEAR, borderValue=(114, 114, 114))
        if self.train: crop = T.color_aug(crop, r)
        c2 = (m[:, :2] @ ct + m[:, 2]) / 4; pts = (uv @ m[:, :2].T + m[:, 2]) / 4
        ij = np.clip(c2.astype(int), 0, self.grid - 1); yy, xx = np.mgrid[:self.grid, :self.grid]
        heat = np.exp(-((xx - ij[0]) ** 2 + (yy - ij[1]) ** 2) / 2).astype(np.float32)[None]
        return T.to_tensor(crop), torch.from_numpy(heat), torch.tensor(ij), torch.from_numpy((pts - ij).astype(np.float32)), torch.tensor(c2 - ij, dtype=torch.float32)
if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('--data', required=True); p.add_argument('--val', required=True); p.add_argument('--out', type=Path, required=True); p.add_argument('--resume', required=True); p.add_argument('--epochs', type=int, default=80); p.add_argument('--batch', type=int, default=12); p.add_argument('--lr', type=float, default=1e-4); p.add_argument('--rot', type=float, default=25.); p.add_argument('--dets'); p.add_argument('--sim_data'); p.add_argument('--sim_n', type=int, default=0); p.add_argument('--real_rep', type=int, default=1)
    a = p.parse_args(); a.out.mkdir(parents=True, exist_ok=False); torch.set_num_threads(4); torch.manual_seed(317); np.random.seed(317); random.seed(317)
    ck = torch.load(a.resume, map_location='cpu', weights_only=False); size = ck['size']
    tr = Real(a.data, size, True, a.rot, dets=json.loads(Path(a.dets).read_text()) if a.dets else None, sim=a.sim_data, sim_n=a.sim_n, real_rep=a.real_rep); va = Real(a.val, size, False, 0)
    print('학습', tr.n_real, '시뮬', len(tr) - tr.n_real, '확인', len(va), flush=True)
    dl = DataLoader(tr, batch_size=a.batch, shuffle=True, num_workers=4, pin_memory=True, drop_last=True, persistent_workers=True); vl = DataLoader(va, batch_size=a.batch, num_workers=2, persistent_workers=True)
    model = T.make_model(); model.load_state_dict(ck['model']); model.cuda(); opt = torch.optim.Adam(model.parameters(), lr=a.lr); sch = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=a.epochs, eta_min=a.lr * .1); sc = torch.amp.GradScaler('cuda')
    best = 1e9; H = []; t0 = time.perf_counter(); (a.out / 'config.json').write_text(json.dumps({k: str(v) for k, v in vars(a).items()}, indent=1) + '\n')
    def ev():
        model.eval(); V = []; K_ = []
        with torch.inference_mode():
            for b in vl:
                x, heat, ij, hps, reg = [v.cuda() for v in b]; l, k = T.loss_for(model(x)[0], heat, ij, hps, reg); V.append(float(l)); K_.append(float(k))
        return float(np.mean(V)), float(np.mean(K_))
    print('시작', ev(), flush=True)
    for e in range(1, a.epochs + 1):
        model.train(); L = []
        for b in dl:
            x, heat, ij, hps, reg = [v.cuda(non_blocking=True) for v in b]; opt.zero_grad(set_to_none=True)
            with torch.autocast('cuda', dtype=torch.float16): l, _ = T.loss_for(model(x)[0], heat, ij, hps, reg)
            sc.scale(l).backward(); sc.unscale_(opt); nn.utils.clip_grad_norm_(model.parameters(), 10); sc.step(opt); sc.update(); L.append(float(l.detach()))
        v, k = ev(); row = dict(epoch=e, train=float(np.mean(L)), val=v, val_kps=k, elapsed_s=time.perf_counter() - t0); H.append(row); print(json.dumps(row), flush=True)
        if v < best: best = v; torch.save(dict(model=model.state_dict(), epoch=e, size=size), a.out / 'best.pt')
        (a.out / 'history.json').write_text(json.dumps(H, indent=1)); sch.step()
