"""교안 방식 crop CenterPose 학습: 상자 → 정사각 crop → DLA34(TAO 공식 구조) → 중심 + 코너 8개 → PnP. 추론 입력에 정답을 쓰지 않는다.

대칭: 몸체 D4(긴 축 4회전 x 뒤집기 2 = 8가지) 중 가장 가까운 전체 순열에 손실. 코너를 따로따로 고르지 않음.
    train_crop.py --data data/train --val data/val --out runs/s128 --size 128 --epochs 40
"""
import argparse, json, random, sys, time
from pathlib import Path
from types import SimpleNamespace
import cv2, numpy as np, torch
from torch import nn
from torch.utils.data import DataLoader, Dataset
ROOT = Path('/home/pa4/git/lv2_next_module')
sys.path.insert(0, str(ROOT / '261030_2202_course/yolov8-ubuntu24-practice'))
from third_party.centerpose.centerpose import CenterPose_DLA34

CORNERS = np.array([[x, y, z] for x in (-.015, .015) for y in (-.015, .015) for z in (-.03, .03)], np.float64)
MEAN = np.array([.485, .456, .406], np.float32); STD = np.array([.229, .224, .225], np.float32)

def perms():
    out = []
    for flip in (1, -1):
        for k in range(4):
            a = k * np.pi / 2; R = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
            if flip < 0: R = R @ np.diag([1., -1., -1.])
            out.append(np.linalg.norm((CORNERS @ R.T)[:, None] - CORNERS[None], axis=-1).argmin(1))
    return torch.tensor(np.array(out))
PERMS = perms()

def make_model(head=64, pretrained=False):
    m = CenterPose_DLA34(SimpleNamespace(down_ratio=4, final_kernel=1, last_level=5, head_conv=head, out_channel=0, use_convGRU=False, use_pretrained=False))
    if pretrained:  # ImageNet DLA34 (첫 시도와 같은 파일, 격리 의존성 tao_deps)
        sys.path.insert(0, str(ROOT / '261030_1325/tao_deps'))
        from safetensors.torch import load_file
        w = load_file(str(ROOT / '261030_1325/dla34_imagenet_timm.safetensors')); own = m.base.state_dict()
        w = {k: v for k, v in w.items() if k in own and own[k].shape == v.shape}
        assert sum(v.numel() for v in w.values()) / sum(v.numel() for v in own.values()) > .99
        m.base.load_state_dict(w, strict=False)
    return m
def crop_matrix(box, size, margin=1.5, jit=None):
    x1, y1, x2, y2 = box; side = max(x2 - x1, y2 - y1, 8.0) * margin; cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    if jit is not None:
        cx += jit.uniform(-.08, .08) * side; cy += jit.uniform(-.08, .08) * side; side *= jit.uniform(.85, 1.2)
    s = size / side
    return np.array([[s, 0, size / 2 - cx * s], [0, s, size / 2 - cy * s]], np.float32)
def to_tensor(crop):
    x = (cv2.cvtColor(crop, cv2.COLOR_BGR2RGB).astype(np.float32) / 255 - MEAN) / STD
    return torch.from_numpy(x.transpose(2, 0, 1).copy())
def color_aug(c, r):
    c = c.astype(np.float32) * r.uniform(.7, 1.3) + r.uniform(-15, 15)
    if r.random() < .5:
        c = c + np.array([r.uniform(-12, 12), r.uniform(-12, 12), r.uniform(-12, 12)], np.float32)
    if r.random() < .3:
        c = cv2.GaussianBlur(c, (0, 0), r.uniform(.4, 1.0))
    if r.random() < .3:
        c = c + np.random.default_rng(r.randrange(1 << 30)).normal(0, r.uniform(2, 6), c.shape).astype(np.float32)
    return np.clip(c, 0, 255).astype(np.uint8)
class Samples(Dataset):
    def __init__(self, data, size, train, min_vis=.3, dets=None):
        J = json.loads((Path(data) / 'truth.json').read_text()); self.root = Path(data); self.size = size; self.train = train; self.grid = size // 4; self.items = []
        for im in J['images']:
            for p in im['pucks']:
                if p['visible'] >= min_vis:
                    uv = np.array(p['corners_px'], np.float32); gt = np.array([*np.clip(uv, 0, [J['width'], J['height']]).min(0), *np.clip(uv, 0, [J['width'], J['height']]).max(0)])
                    e3 = None
                    if dets:  # 설계 2: 그 퍽과 가장 많이 겹치는 E3 상자(겹침 0.3 이상)도 후보로 둔다
                        best = 0.3
                        for b in dets.get(im['id'], []):
                            b = np.array(b); lo = np.maximum(gt[:2], b[:2]); hi = np.minimum(gt[2:], b[2:]); i = np.prod(np.maximum(hi - lo, 0)); v = i / (np.prod(gt[2:] - gt[:2]) + np.prod(b[2:] - b[:2]) - i + 1e-9)
                            if v > best: best = v; e3 = b.tolist()
                    self.items.append((im['image'], uv, np.array(p['center_cam'], np.float32), np.array(J['K'], np.float32), e3))
    def __len__(self): return len(self.items)
    def __getitem__(self, i):
        name, uv, c, K, e3 = self.items[i]; im = cv2.imread(str(self.root / name)); box = [*uv.min(0), *uv.max(0)]
        box = [max(0, box[0]), max(0, box[1]), min(im.shape[1], box[2]), min(im.shape[0], box[3])]
        r = random.Random(np.random.randint(1 << 30)) if self.train else None
        if e3 is not None and r is not None and r.random() < .5: box = e3
        m = crop_matrix(box, self.size, jit=r); crop = cv2.warpAffine(im, m, (self.size, self.size), flags=cv2.INTER_LINEAR, borderValue=(114, 114, 114))
        if self.train: crop = color_aug(crop, r)
        ct = K @ c; ct = ct[:2] / ct[2]; ct = (m[:, :2] @ ct + m[:, 2]) / 4; pts = (uv @ m[:, :2].T + m[:, 2]) / 4
        ij = np.clip(ct.astype(int), 0, self.grid - 1); yy, xx = np.mgrid[:self.grid, :self.grid]
        heat = np.exp(-((xx - ij[0]) ** 2 + (yy - ij[1]) ** 2) / 2).astype(np.float32)[None]
        return to_tensor(crop), torch.from_numpy(heat), torch.tensor(ij), torch.from_numpy((pts - ij).astype(np.float32)), torch.tensor(ct - ij, dtype=torch.float32)
def loss_for(out, heat, ij, hps, reg):
    prob = out['hm'].sigmoid().clamp(1e-5, 1 - 1e-5); pos = heat.eq(1); neg = heat.lt(1)
    hm = -(torch.log(prob) * (1 - prob) ** 2 * pos + torch.log(1 - prob) * prob ** 2 * (1 - heat) ** 4 * neg).sum() / max(1, len(ij))
    idx = torch.arange(len(ij), device=ij.device); pred = out['hps'][idx, :, ij[:, 1], ij[:, 0]].reshape(-1, 8, 2); off = out['reg'][idx, :, ij[:, 1], ij[:, 0]]
    v = torch.stack([nn.functional.smooth_l1_loss(pred, hps[:, p], reduction='none').mean((1, 2)) for p in PERMS.to(ij.device)], 1)
    return hm + v.min(1).values.mean() + nn.functional.l1_loss(off, reg), v.min(1).values.mean().detach()
if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('--data', required=True); p.add_argument('--val', required=True); p.add_argument('--out', type=Path, required=True)
    p.add_argument('--size', type=int, default=128); p.add_argument('--epochs', type=int, default=40); p.add_argument('--batch', type=int, default=32); p.add_argument('--lr', type=float, default=5e-4); p.add_argument('--limit', type=int, default=0); p.add_argument('--resume'); p.add_argument('--dets')
    a = p.parse_args(); a.out.mkdir(parents=True, exist_ok=False); torch.set_num_threads(4); torch.manual_seed(317); np.random.seed(317); random.seed(317)
    tr = Samples(a.data, a.size, True, dets=json.loads(Path(a.dets).read_text()) if a.dets else None); va = Samples(a.val, a.size, False)
    if a.limit: tr.items = random.Random(0).sample(tr.items, min(a.limit, len(tr.items)))
    dl = DataLoader(tr, batch_size=a.batch, shuffle=True, num_workers=4, pin_memory=True, drop_last=True, persistent_workers=True); vl = DataLoader(va, batch_size=a.batch, num_workers=2, persistent_workers=True)
    model = make_model(pretrained=not a.resume)
    if a.resume: model.load_state_dict(torch.load(a.resume, map_location='cpu', weights_only=False)['model'])
    model.cuda(); opt = torch.optim.Adam(model.parameters(), lr=a.lr); sch = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=a.epochs, eta_min=a.lr * .1); sc = torch.amp.GradScaler('cuda')
    best = 1e9; H = []; t0 = time.perf_counter()
    (a.out / 'config.json').write_text(json.dumps({k: str(v) for k, v in vars(a).items()}, indent=1) + '\n')
    for e in range(1, a.epochs + 1):
        model.train(); L = []
        for b in dl:
            x, heat, ij, hps, reg = [v.cuda(non_blocking=True) for v in b]; opt.zero_grad(set_to_none=True)
            with torch.autocast('cuda', dtype=torch.float16): l, _ = loss_for(model(x)[0], heat, ij, hps, reg)
            sc.scale(l).backward(); sc.unscale_(opt); nn.utils.clip_grad_norm_(model.parameters(), 10); sc.step(opt); sc.update(); L.append(float(l.detach()))
        model.eval(); V = []; K_ = []
        with torch.inference_mode():
            for b in vl:
                x, heat, ij, hps, reg = [v.cuda() for v in b]; l, k = loss_for(model(x)[0], heat, ij, hps, reg); V.append(float(l)); K_.append(float(k))
        row = dict(epoch=e, train=float(np.mean(L)), val=float(np.mean(V)), val_kps=float(np.mean(K_)), lr=opt.param_groups[0]['lr'], elapsed_s=time.perf_counter() - t0); H.append(row); print(json.dumps(row), flush=True)
        if row['val'] < best:
            best = row['val']; torch.save(dict(model=model.state_dict(), epoch=e, size=a.size), a.out / 'best.pt')
        (a.out / 'history.json').write_text(json.dumps(H, indent=1)); sch.step()
