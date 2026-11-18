"""평면 교차 오프라인 평가 (도구 확인). A1 렌더(정답 퍽 중심)와 그 재생에서 perception_node 가 낸 팔 박스 중심(arm_offset)으로
   시선 ∩ 수평면(높이 = D435 추적 목표 z) 을 구해 정답 중심과 비교한다. 비교용으로 D435 추적 위치(seed_object) 오차도 같이 잰다."""
import json, sys, math, numpy as np, collections
render, rows = sys.argv[1], sys.argv[2]
R = json.load(open(render)); K = R['arm']['K']; W, H = R['arm']['width'], R['arm']['height']
fx, fy, cx, cy = K['fx'], K['fy'], K['cx'], K['cy']
scenes = {s['name']: s for s in R['scenes']}
def ray_plane(T, ox, oy, z):
    u, v = W / 2 * (1 + ox), H / 2 * (1 + oy)
    d = T[:3, :3] @ np.array([(u - cx) / fx, (v - cy) / fy, 1.0]); o = T[:3, 3]
    if abs(d[2]) < 1e-6: return None
    t = (z - o[2]) / d[2]
    return o + t * d if t > 0 else None
out = collections.defaultdict(list); detail = []
for l in open(rows):
    r = json.loads(l)
    if not r['offset_valid'] or r.get('pointed') is None: continue
    sc = scenes[r['scene']]; f = sc['arm_frames'][r['k']]; T = np.array(f['T_world_cam'])
    pk = {p['name']: p for p in sc['pucks']}[r['pointed']]; c = np.array(pk['center'])
    Tb = np.array(sc['T_world_base_link'])
    tag = 'target' if r['pointed'] == sc['target'] else 'other'
    for name, z in (('plane_truth_z', c[2]), ('plane_d435_z', (Tb @ np.r_[r['seed_object'], 1])[2] if r.get('seed_object') else None)):
        if z is None: continue
        p = ray_plane(T, r['offset'][0], r['offset'][1], z)
        if p is None: continue
        out[name].append(math.hypot(*(p[:2] - c[:2])))
    if r.get('seed_object') and tag == 'target':
        s = (Tb @ np.r_[r['seed_object'], 1])[:3]; out['d435_seed'].append(math.hypot(*(s[:2] - c[:2])))
    cam_dist = np.linalg.norm(T[:3, 3] - c); out['cam_dist'].append(cam_dist)
    detail.append(dict(scene=r['scene'], k=r['k'], stage=r['stage'], cam_dist=round(cam_dist, 3)))
def st(a):
    a = np.array(a) * 1000
    return f"n {len(a)}  중앙 {np.median(a):.1f}  p95 {np.percentile(a, 95):.1f}  최대 {a.max():.1f} mm"
for k in ('plane_truth_z', 'plane_d435_z', 'd435_seed'): print(f"{k:14s} 수평 오차 {st(out[k])}")
a = np.array(out['cam_dist']); print(f"카메라-퍽 거리 {a.min():.3f}~{a.max():.3f} m (중앙 {np.median(a):.3f})")
