"""지도 어긋남 확인(읽기만, 실물 없음). 자료 셋을 발제 좌표에서 비교: USD(Isaac 충돌) · SDF(발제 v5, Gazebo 원본) · slam_walls_001(/map) · map_amcl(SDF 에서 만든 지도).
A→B: A 의 점마다 B 의 가장 가까운 점까지 거리. 영역(1 m 칸)별 최적 평행 이동도 잼(어긋남이 늘어나는지 = 축척 · 누적 오차 구분)."""
import json, re, sys, xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np, yaml
from scipy.spatial import cKDTree
H = Path(__file__).resolve().parent; S = H / 'src'
def load_map(name):
    y = yaml.safe_load(open(S / f'{name}.yaml'))
    raw = (S / y['image']).read_bytes()
    m = re.match(rb'P5\s+(?:#[^\n]*\n\s*)*(\d+)\s+(?:#[^\n]*\n\s*)*(\d+)\s+(?:#[^\n]*\n\s*)*(\d+)\s', raw)
    w, h = int(m[1]), int(m[2])
    img = np.frombuffer(raw[m.end():], np.uint8)[:w * h].reshape(h, w)
    neg = y.get('negate', 0)
    occ = ((img / 255.0 if neg else (255 - img.astype(float)) / 255.0) > y['occupied_thresh'])
    r, (ox, oy) = y['resolution'], y['origin'][:2]
    j, i = np.nonzero(occ)
    return np.c_[ox + (i + .5) * r, oy + (h - 1 - j + .5) * r]
def load_sdf(zl=0.06, zh=0.11, step=0.0025):
    """<collision> box · cylinder 를 라이다 높이에서 잘라 윤곽 점으로. arena 모델 pose 는 0."""
    pts = []
    root = ET.parse(S / 'dice_mission.sdf').getroot()
    names = []
    for m in root.iter('model'):
        mp = [float(v) for v in (m.findtext('pose') or '0 0 0 0 0 0').split()]
        for c in m.iter('collision'):
            pp = [float(v) for v in (c.findtext('pose') or '0 0 0 0 0 0').split()]
            g = c.find('geometry')
            x, y, z, yaw = mp[0] + pp[0], mp[1] + pp[1], mp[2] + pp[2], mp[5] + pp[5]
            b = g.find('box')
            if b is not None:
                sx, sy, sz = [float(v) for v in b.findtext('size').split()]
                if z - sz / 2 > zh or z + sz / 2 < zl: continue
                cs = np.array([[-sx/2,-sy/2],[sx/2,-sy/2],[sx/2,sy/2],[-sx/2,sy/2]])
                R = np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
                cs = cs @ R.T + [x, y]
                for k in range(4):
                    a, e = cs[k], cs[(k + 1) % 4]
                    n = max(2, int(np.hypot(*(e - a)) / step) + 1)
                    pts.append(a + (e - a) * np.linspace(0, 1, n)[:, None])
                names.append(c.get('name'))
            elif g.find('cylinder') is not None:
                r = float(g.find('cylinder').findtext('radius')); l = float(g.find('cylinder').findtext('length'))
                if z - l / 2 > zh or z + l / 2 < zl: continue
                th = np.linspace(0, 2 * np.pi, max(16, int(2 * np.pi * r / step)), endpoint=False)
                pts.append(np.c_[x + r * np.cos(th), y + r * np.sin(th)]); names.append(c.get('name'))
    return np.concatenate(pts), names
D = {'USD': np.load(H / 'usd_pts.npy'), 'slam': load_map('slam_walls_001'), 'map_amcl': load_map('map_amcl')}
D['SDF'], names = load_sdf()
print('SDF 라이다 높이 충돌', len(names), '개, 이름 앞부분 종류', sorted({re.sub(r'_?\d.*', '', n) for n in names}))
for k, v in D.items():
    print(f'{k:9s} n {len(v):7d} x {v[:,0].min():.3f}~{v[:,0].max():.3f} y {v[:,1].min():.3f}~{v[:,1].max():.3f}')
def stats(a, b):
    d, _ = cKDTree(b).query(a)
    return dict(mean=round(float(d.mean()), 4), p50=round(float(np.median(d)), 4), p95=round(float(np.percentile(d, 95)), 4), max=round(float(d.max()), 4), frac_gt_2cm=round(float((d > .02).mean()), 3), frac_gt_5cm=round(float((d > .05).mean()), 3))
res = {}
for a, b in [('slam', 'USD'), ('USD', 'slam'), ('slam', 'SDF'), ('SDF', 'slam'), ('USD', 'SDF'), ('SDF', 'USD'), ('map_amcl', 'SDF')]:
    res[f'{a}->{b}'] = stats(D[a], D[b]); print(f'{a:9s}->{b:5s}', res[f'{a}->{b}'])
def local_shift(a, b, cell=1.0, trim=0.15):
    """1 m 칸마다 a 를 평행 이동해 b 에 맞추는 값(잘라낸 평균 ICP). 벽 방향이 한쪽뿐이면 그 축은 약하므로 칸 내 점 수 · 전 · 후 오차도 씀."""
    t = cKDTree(b); out = []
    for xc in np.arange(0, 5, cell):
        for yc in np.arange(0, 7, cell):
            m = (a[:,0] >= xc) & (a[:,0] < xc + cell) & (a[:,1] >= yc) & (a[:,1] < yc + cell)
            if m.sum() < 30: continue
            p = a[m]; s = np.zeros(2); d0 = t.query(p)[0]
            for _ in range(40):
                d, i = t.query(p + s); keep = d <= trim
                if keep.sum() < 10: break
                s += (b[i[keep]] - (p[keep] + s)).mean(0)
            d1 = t.query(p + s)[0]
            out.append(dict(x=float(xc + cell/2), y=float(yc + cell/2), n=int(m.sum()), dx=round(float(s[0]), 4), dy=round(float(s[1]), 4), before=round(float(np.median(d0)), 4), after=round(float(np.median(d1)), 4)))
    return out
for a, b in [('slam', 'USD'), ('USD', 'SDF'), ('slam', 'SDF')]:
    r = local_shift(D[a], D[b]); res[f'shift {a}->{b}'] = r
    print(f'--- 칸별 이동 {a}->{b} (x y n dx dy 중앙값 전→후)')
    for c in r: print(f"  {c['x']:.1f} {c['y']:.1f} {c['n']:6d} {c['dx']:+.3f} {c['dy']:+.3f}  {c['before']:.3f}→{c['after']:.3f}")
def global_fit(a, b, it=60, trim=0.1):
    """a → b 축척 · 회전 · 이동(축별 축척) 맞춤"""
    t = cKDTree(b); A = np.eye(3)
    ph = np.c_[a, np.ones(len(a))]
    for _ in range(it):
        q = ph @ A.T; d, i = t.query(q[:, :2]); k = d < trim
        X, *_ = np.linalg.lstsq(ph[k], b[i[k]], rcond=None)
        A = np.r_[X.T, [[0, 0, 1]]]
    return A, float(np.median(t.query((ph @ A.T)[:, :2])[0]))
for a, b in [('USD', 'SDF'), ('slam', 'USD'), ('slam', 'SDF')]:
    A, m = global_fit(D[a], D[b]); res[f'affine {a}->{b}'] = dict(A=np.round(A, 5).tolist(), median_after=round(m, 4))
    print(f'affine {a}->{b}', np.round(A[:2], 5).tolist(), 'median', round(m, 4))
json.dump(res, open(H / 'compare.json', 'w'), ensure_ascii=False, indent=1)
np.savez(H / 'pts.npz', **D)
