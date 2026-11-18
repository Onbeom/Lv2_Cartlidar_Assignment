"""놓친 장면 진단: 거르기를 하나씩 풀어 정답 라벨 30 mm 안 후보가 생기는지 본다.
사용: python3 diag.py <render 폴더> <장면 이름> [noise seed]"""
import json, sys, zlib
from pathlib import Path
import cv2, numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'proto_r3'))
from locate_v2 import CFG2, locate
root = Path(sys.argv[1]); name = sys.argv[2]
noise = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0; seed = int(sys.argv[4]) if len(sys.argv) > 4 else 0
rep = json.loads((root / 'render_r3.json').read_text()); K = rep['d435']['K']
labs = {l['id']: l for l in rep['labels']}
p = [q for q in rep['poses'] if q.get('name') == name][0]
bgr = cv2.imread(str(root / p['images']['color'])); dep = np.load(root / p['images']['depth'])
if noise > 0:
    rng = np.random.default_rng(zlib.crc32(p['name'].encode()) + 7919 * seed)
    e = rng.normal(0, 1, dep.shape).astype(np.float32); e = cv2.GaussianBlur(e, (0, 0), 3); e /= e.std()
    dep = (dep + e * noise * dep ** 2).astype(np.float32)
Twc = np.array(p['T_world_cam']); Twb = np.array(p['T_world_base_footprint']); c = np.array(labs[p['label']]['center'])
OFF = dict(thr=0.3, rescore_m=0.0, lift_max=99.0, sat_max=999.0, sat_hi_max=999.0, bw_min=0.0, white_ring_min=0.0, bw_max=99.0,
           con_max=99.0, ring_max=99.0, ring_rel_max=99.0, resw_min=-99.0, sharp_min=-99.0, side_min=-99.0, img_margin=0)
def near(cfg):
    out = []
    for f in locate(bgr, dep, K, np.linalg.inv(Twb) @ Twc, cfg):
        w = (Twc @ np.append(f['center'], 1))[:3]
        if np.linalg.norm(w - c) < 0.03:
            out.append({k: (round(v, 3) if isinstance(v, float) else v) for k, v in f.items() if k not in ('center', 'crop')})
    return out
base = dict(CFG2)
print('기본값:', [ (d['score'],) for d in near(base)])
allo = dict(base, **OFF); r = near(allo)
print('모두 풂:', r[:2])
for k in OFF:
    cfg = dict(allo); cfg[k] = base[k]
    print(f'  {k}={base[k]} 만 켬 ->', '있음' if near(cfg) else '없음')
