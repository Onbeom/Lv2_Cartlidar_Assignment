"""K2d(261101_0016_isaac_live/results/K2d_escape_v45) G3 겹침 원인 오프라인 분석(모의 bag, 도구):
겹친 구간(참값 clear_body ≤ 0) 앞뒤로 ① 그때 쓰던 전역 경로(/plan)를 base_link footprint 로 따라갈 때 기둥까지 여유(계획이 이미 닿는지)
② 참값 자세와 경로 사이 옆 어긋남(추종 오차) ③ 명령(cmd_vel_nav) 을 본다. /plan frame = mission_map, 자세 = rotation_center(x_icr 점) → base_link = rc ∘ (−x_icr, 0).
사용: python3 plan_vs_truth.py <결과 폴더> [x_icr]"""
import csv
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, '/home/pa4/git/lv2_next_module/261101_0016_isaac_live')
sys.path.insert(0, '/home/pa4/git/lv2_next_module/261031_1325_nav2/nodes')
import bagutil  # noqa: E402
import geom  # noqa: E402

run = Path(sys.argv[1]); XI = float(sys.argv[2]) if len(sys.argv) > 2 else -0.0786
body = geom.load_world()['body']
rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(open(run / 'truth.csv'))]
bad = [r for r in rows if r['clear_body'] <= 0]
t0, t1 = bad[0]['t'], bad[-1]['t']
print(f'겹침 {len(bad)} 표본, t {t0:.2f}~{t1:.2f}, 최소 {min(r["clear_body"] for r in bad):.4f}')
plans = []
cmds = []
for topic, m, tb in bagutil.read_bag(str(run / 'out_bag'), ['/plan', '/cmd_vel_nav']):
    if topic == '/plan':
        ts = bagutil.stamp(m.header)
        if t0 - 15 <= ts <= t1 + 3:
            plans.append((ts, m.header.frame_id, [(p.pose.position.x, p.pose.position.y, bagutil.yaw_of(p.pose.orientation)) for p in m.poses]))
    else:
        ts = bagutil.stamp(m.header) if hasattr(m, 'header') else tb
        tw = m.twist if hasattr(m, 'twist') else m
        if t0 - 8 <= ts <= t1 + 2:
            cmds.append((round(ts, 2), round(tw.linear.x, 3), round(tw.angular.z, 3)))
print('경로 수', len(plans), 'frame', {p[1] for p in plans})


def bl(p):   # rotation_center 자세 → base_link 자세
    x, y, yaw = p
    return x - XI * math.cos(yaw), y - XI * math.sin(yaw), yaw


out = {'overlap': [t0, t1, len(bad)], 'plans': []}
for ts, fr, poses in plans:
    cl = [geom.clearance(*bl(p), body) for p in poses]
    i = min(range(len(cl)), key=lambda k: cl[k]) if cl else None
    rev = sum(1 for a, b in zip(poses, poses[1:]) if (b[0] - a[0]) * math.cos(a[2]) + (b[1] - a[1]) * math.sin(a[2]) < -1e-4)
    rec = dict(t=round(ts, 2), n=len(poses), min_clear=round(cl[i], 4) if cl else None, at=[round(v, 3) for v in poses[i]] if cl else None, n_reverse_steps=rev)
    out['plans'].append(rec)
    print(rec)
# 겹침 구간의 참값 자세와 그 직전 경로의 가장 가까운 점 사이 거리(rotation_center 기준)
last = [p for p in plans if p[0] <= t0]
ref = last[-1] if last else plans[0]
dev = []
for r in rows:
    if t0 - 4 <= r['t'] <= t1 + 1:
        rx, ry = r['x'] + XI * math.cos(r['yaw']), r['y'] + XI * math.sin(r['yaw'])
        d, k = min((math.hypot(rx - p[0], ry - p[1]), k) for k, p in enumerate(ref[2]))
        p = ref[2][k]
        dyaw = (r['yaw'] - p[2] + math.pi) % (2 * math.pi) - math.pi
        dev.append((round(r['t'], 2), round(d, 4), round(dyaw, 3), r['clear_body'], round(geom.clearance(*bl(p), body), 4)))
out['ref_plan_t'] = ref[0]
out['deviation'] = dev[::6]
out['cmd'] = cmds[::3]
print('기준 경로 t', ref[0])
print('t, 경로까지 거리(rc), yaw 차, 참값 여유, 가장 가까운 경로 점 여유')
for d in dev[::12]:
    print(d)
(Path(__file__).parent / (run.name + '_plan_vs_truth.json')).write_text(json.dumps(out, ensure_ascii=False, indent=1))
