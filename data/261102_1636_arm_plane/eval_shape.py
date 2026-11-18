"""평면 교차 + 퍽 형상 보정 (도구 확인). 퍽 모양(서 있음: 반지름 15 mm · 높이 60 mm 원기둥으로 근사, 누움: 60 × 30 × 30)을 알면
   후보 중심을 투영한 외곽 상자 중심이 관측 상자 중심과 같아지게 중심을 고친다(고정점 반복 5회). 정답 높이 / D435 높이 두 경우."""
import json, sys, math, numpy as np, collections
render, rows = sys.argv[1], sys.argv[2]
R = json.load(open(render)); K = R['arm']['K']; W, H = R['arm']['width'], R['arm']['height']
fx, fy, cx, cy = K['fx'], K['fy'], K['cx'], K['cy']
scenes = {s['name']: s for s in R['scenes']}
def ray_plane(T, ox, oy, z):
    u, v = W / 2 * (1 + ox), H / 2 * (1 + oy)
    d = T[:3, :3] @ np.array([(u - cx) / fx, (v - cy) / fy, 1.0]); o = T[:3, 3]
    t = (z - o[2]) / d[2]
    return o + t * d if t > 0 else None
a = np.linspace(0, 2 * np.pi, 24, endpoint=False)
STAND = np.array([[.015 * math.cos(t), .015 * math.sin(t), h] for t in a for h in (-.03, .03)])
def box_center(T, c, pts):
    P = (np.linalg.inv(T) @ np.c_[pts + c, np.ones(len(pts))].T)[:3]
    u = fx * P[0] / P[2] + cx; v = fy * P[1] / P[2] + cy
    return ((u.min() + u.max()) / 2 / (W / 2) - 1, (v.min() + v.max()) / 2 / (H / 2) - 1)
out = collections.defaultdict(list); kinds = collections.Counter()
for l in open(rows):
    r = json.loads(l)
    if not r['offset_valid'] or r.get('pointed') is None: continue
    sc = scenes[r['scene']]; T = np.array(sc['arm_frames'][r['k']]['T_world_cam'])
    pk = {p['name']: p for p in sc['pucks']}[r['pointed']]; c = np.array(pk['center']); kinds[(pk['shape'], pk['standing'])] += 1
    if not pk['standing']: continue
    Tb = np.array(sc['T_world_base_link'])
    for name, z in (('truth_z', c[2]), ('d435_z', (Tb @ np.r_[r['seed_object'], 1])[2] if r.get('seed_object') else None)):
        if z is None: continue
        p = ray_plane(T, *r['offset'][:2], z)
        if p is None: continue
        out['plain_' + name].append(math.hypot(*(p[:2] - c[:2])))
        q = p.copy()
        for _ in range(5):
            bx = box_center(T, q, STAND); pb = ray_plane(T, *bx, z)
            q = q + (p - pb); q[2] = z
        out['shape_' + name].append(math.hypot(*(q[:2] - c[:2])))
def st(v):
    v = np.array(v) * 1000; return f"n {len(v)}  중앙 {np.median(v):.1f}  p95 {np.percentile(v, 95):.1f}  최대 {v.max():.1f} mm"
print('퍽 종류(모양, 서 있음):', dict(kinds))
for k in ('plain_truth_z', 'shape_truth_z', 'plain_d435_z', 'shape_d435_z'): print(f"{k:14s} {st(out[k])}")
