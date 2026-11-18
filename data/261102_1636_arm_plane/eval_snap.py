"""평면 높이를 '받침 높이 목록 + 퍽 반높이(누움 0.015 · 서 있음 0.03)' 중 D435 목표 z 에 가장 가까운 조합으로 고르는 방식 (도구 확인).
   받침 높이는 바닥 0 과 출발 상자 바닥 0.07 m (A1 장면 정답에서 구한 값, base_footprint 기준)."""
import json, sys, math, numpy as np, collections
render, rows = sys.argv[1], sys.argv[2]; LEVELS = [0.0, 0.07]; HALF = [0.015, 0.03]
R = json.load(open(render)); K = R['arm']['K']; W, H = R['arm']['width'], R['arm']['height']
fx, fy, cx, cy = K['fx'], K['fy'], K['cx'], K['cy']; scenes = {s['name']: s for s in R['scenes']}
def ray_plane(T, ox, oy, z):
    u, v = W / 2 * (1 + ox), H / 2 * (1 + oy)
    d = T[:3, :3] @ np.array([(u - cx) / fx, (v - cy) / fy, 1.0]); o = T[:3, 3]; t = (z - o[2]) / d[2]
    return o + t * d if t > 0 else None
E_ = collections.defaultdict(list); wrong = 0; n = 0
for l in open(rows):
    r = json.loads(l)
    if not r['offset_valid'] or r.get('pointed') != scenes[r['scene']]['target'] or not r.get('seed_object'): continue
    sc = scenes[r['scene']]; T = np.array(sc['arm_frames'][r['k']]['T_world_cam']); pk = {p['name']: p for p in sc['pucks']}[r['pointed']]
    c = np.array(pk['center']); zs = (np.array(sc['T_world_base_link']) @ np.r_[r['seed_object'], 1])[2]
    z = min((lv + h for lv in LEVELS for h in HALF), key=lambda v: abs(v - zs)); n += 1; wrong += abs(z - c[2]) > 1e-6
    p = ray_plane(T, *r['offset'][:2], z); E_['snap'].append(math.hypot(*(p[:2] - c[:2])))
    s = (np.array(sc['T_world_base_link']) @ np.r_[r['seed_object'], 1])[:3]; E_['d435'].append(math.hypot(*(s[:2] - c[:2])))
    E_['dist'].append(np.linalg.norm(T[:3, 3] - c))
def st(v):
    v = np.array(v) * 1000; return f"n {len(v)}  중앙 {np.median(v):.1f}  p95 {np.percentile(v, 95):.1f}  최대 {v.max():.1f} mm"
print(f"높이 고르기 틀림 {wrong}/{n}"); print('평면 교차(높이 고름)', st(E_['snap'])); print('D435 추적 위치     ', st(E_['d435']))
d = np.array(E_['dist']); print(f"카메라-퍽 거리 {d.min():.2f}~{d.max():.2f} m")
