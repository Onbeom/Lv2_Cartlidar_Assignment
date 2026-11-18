"""노드가 낸 /perception/arm_target 을 정답 퍽 중심과 비교 (모의 재생 결과 채점). 오프라인 계산(eval_snap.py)과 같은지도 본다."""
import json, sys, math, numpy as np
render, rows = sys.argv[1], sys.argv[2]
R = json.load(open(render)); scenes = {s['name']: s for s in R['scenes']}
import collections
e = []; n_lock = n_pub = n_rng = 0; wrong = 0; seedless = 0; by = collections.defaultdict(list); locked_by = collections.Counter()
for l in open(rows):
    r = json.loads(l)
    if not r['offset_valid'] or r.get('pointed') != scenes[r['scene']]['target']: continue
    n_lock += 1; locked_by[bool(r.get('arm_only'))] += 1
    if r.get('plane') is None: continue
    n_pub += 1; n_rng += r.get('plane_range') is not None; seedless += r.get('seed_object') is None
    sc = scenes[r['scene']]; Tb = np.array(sc['T_world_base_link'])
    p = (Tb @ np.r_[r['plane'][:3], 1])[:3]; c = np.array({q['name']: q for q in sc['pucks']}[r['pointed']]['center'])
    e.append(math.hypot(*(p[:2] - c[:2]))); wrong += abs(p[2] - c[2]) > 1e-4; by[bool(r.get('arm_only'))].append(e[-1])
v = np.array(e) * 1000
print(f"목표를 잠근 장 {n_lock}, 평면 교차 발행 {n_pub} (그중 그 장에 D435 목표 없음 {seedless}), 거리 발행 {n_rng}")
print(f"높이 틀림 {wrong}/{n_pub}")
print(f"퍽 중심 수평 오차: 중앙 {np.median(v):.1f}  p95 {np.percentile(v, 95):.1f}  최대 {v.max():.1f} mm")
for k in (False, True):
    if locked_by[k]:
        v = np.array(by[k]) * 1000
        print(f"  {'팔 카메라만(D435 끔)' if k else 'D435 + 팔'}: 잠근 장 {locked_by[k]}, 발행 {len(v)}" + (f", 중앙 {np.median(v):.1f} p95 {np.percentile(v, 95):.1f} mm" if len(v) else ''))
