"""PC: Jetson results/(dets_E3J16 · summary.json)를 PC pt E3 결과와 비교(11-04). 기준(README): K1 같은 장 ≥ 98 %, K2 경기장 판정 차 ≤ 0.01, K3 속도 D 480 · 384 FP16 과 ±10 %."""
import json, os, shutil, subprocess
from pathlib import Path
HERE = Path(__file__).resolve().parent; R = HERE / 'results'; A = Path('/home/pa4/git/lv2_next_module/261104_1211_arena_eval')
J = json.load(open(R / 'dets_E3J16.json')); P = json.load(open(HERE / 'dets_E3pt_1478.json')) | json.load(open(A / 'dets_E3.json'))


def iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0])); iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - ix * iy
    return ix * iy / u if u > 0 else 0.0


same, diff = 0, []
for k in J:
    a = [d for d in P[k] if d[1] >= 0.6]; b = [d for d in J[k] if d[1] >= 0.6]; used = set(); ok = len(a) == len(b)
    for d in a:
        m = max(((iou(d[2:], e[2:]), j) for j, e in enumerate(b) if j not in used and e[0] == d[0]), default=(0, None))
        if m[0] >= 0.9: used.add(m[1])
        else: ok = False
    same += ok
    if not ok: diff.append(k)
shutil.copy(R / 'dets_E3J16.json', A / 'dets_E3J16.json')
env = dict(os.environ, JUDGE_S0_FIXED='114', JUDGE_AGNOSTIC='arena_rear', JUDGE_CAMS='arena_front,arena_rear,arena_arm', JUDGE_OUT='judge_arena_E3J16.json')
subprocess.run(['/home/pa4/yolo-venv/bin/python', 'judge.py', 'E2', 'E3', 'E3J16'], cwd=A, env=env, check=True, capture_output=True)
T = json.load(open(A / 'judge_arena_E3J16.json')); cams = ['arena_front', 'arena_rear', 'arena_arm']
rp = {n: {c: [T[n]['0.6'][c]['recall'], T[n]['0.6'][c]['precision']] for c in cams} for n in T}
k2 = all(abs(rp['E3J16'][c][i] - rp['E3'][c][i]) <= 0.01 for c in cams for i in (0, 1))
S = json.load(open(R / 'summary.json'))['engines']; S0 = json.load(open('/home/pa4/git/lv2_next_module/261104_1005_yolo_D_480/results/summary.json'))['engines']
sp = {s: {'E3_gpu': S[s]['trtexec_gpu_ms']['median'], 'D_gpu': S0[f'{s}_16']['trtexec_gpu_ms']['median']} for s in ('480', '384')}
k3 = all(abs(v['E3_gpu'] / v['D_gpu'] - 1) <= 0.10 for v in sp.values())
out = {'K1': {'same': same, 'n': len(J), 'ratio': round(same / len(J), 4), 'pass': same / len(J) >= 0.98, 'diff': diff},
       'K2': {'recall_precision@0.6': rp, 'pass': k2}, 'K3': {'ms': sp, 'pass': k3}}
json.dump(out, open(HERE / 'result_e3.json', 'w'), ensure_ascii=False, indent=1)
print(json.dumps({k: ({x: v[x] for x in v if x != 'diff'} if k == 'K1' else v) for k, v in out.items()}, ensure_ascii=False, indent=1))
