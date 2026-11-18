"""PC: Jetson results/(dets_N32 · dets_N16 · summary.json)로 R1~R3 계산(11-04). 비교: PC pt(all_D) · 정사각 640 FP32(J32) · FP16(J16)."""
import json, os, shutil, subprocess
from pathlib import Path
HERE = Path(__file__).resolve().parent; R = HERE / 'results'; V = Path('/home/pa4/git/lv2_next_module/261103_1053_video_eval')
F = Path('/home/pa4/git/lv2_next_module/261104_0941_fp16_check')
N32, N16 = json.load(open(R / 'dets_N32.json')), json.load(open(R / 'dets_N16.json'))


def iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0])); iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - ix * iy
    return ix * iy / u if u > 0 else 0.0


same = 0
for k in N32:
    a = [d for d in N32[k] if d[1] >= 0.6]; b = [d for d in N16[k] if d[1] >= 0.6]; used = set(); ok = len(a) == len(b)
    for d in a:
        m = max(((iou(d[2:], e[2:]), j) for j, e in enumerate(b) if j not in used and e[0] == d[0]), default=(0, None))
        if m[0] >= 0.9: used.add(m[1])
        else: ok = False
    same += ok
for n in ('N32', 'N16'): shutil.copy(R / f'dets_{n}.json', V / f'dets_{n}.json')
for n in ('J32', 'J16'): shutil.copy(F / f'results/dets_{n}.json', V / f'dets_{n}.json')
env = dict(os.environ, JUDGE_STACKED='stacked.json', JUDGE_MIN_SHARP='0', JUDGE_AGNOSTIC='judge_rear,video_rear',
           JUDGE_CAMS='judge_front,judge_rear,video_front,video_rear,video_arm', JUDGE_OUT='judge_480.json')
subprocess.run(['/home/pa4/yolo-venv/bin/python', 'judge.py', 'all_D', 'J32', 'J16', 'N32', 'N16'], cwd=V, env=env, check=True, capture_output=True)
J = json.load(open(V / 'judge_480.json')); cams = ['judge_front', 'judge_rear', 'video_front', 'video_rear', 'video_arm']
T = {n: {c: [J[n]['0.6'][c]['recall'], J[n]['0.6'][c]['precision']] for c in cams} for n in J}
r1 = all(abs(T['N32'][c][i] - T['all_D'][c][i]) <= 0.01 for c in cams for i in (0, 1))
r2 = same / len(N32) >= 0.98 and all(abs(T['N16'][c][i] - T['N32'][c][i]) <= 0.01 for c in cams for i in (0, 1)) and min(T['N16']['video_arm']) >= 0.95
S = json.load(open(R / 'summary.json'))['engines']; S0 = json.load(open(F / 'results/summary.json'))['engines']
sp = {k: {'trtexec_gpu_median': v['trtexec_gpu_ms']['median'], 'infer_median': v['python_ms']['infer']['median']} for k, v in S.items()}
sp['640_16'] = {'trtexec_gpu_median': S0['J16']['trtexec_gpu_ms']['median'], 'infer_median': S0['J16']['python_ms']['infer']['median']}
sp['640_32'] = {'trtexec_gpu_median': S0['J32']['trtexec_gpu_ms']['median'], 'infer_median': S0['J32']['python_ms']['infer']['median']}
r3 = sp['480_16']['infer_median'] <= sp['640_16']['infer_median'] and sp['384_16']['infer_median'] <= sp['640_16']['infer_median']
out = {'R1_N32_eq_pt': r1, 'R2_fp16': {'same_images': same, 'n': len(N32), 'pass': r2}, 'R3_speed': {'ms': sp, 'pass': r3}, 'recall_precision@0.6': T}
json.dump(out, open(HERE / 'result_480.json', 'w'), ensure_ascii=False, indent=1); print(json.dumps(out, ensure_ascii=False, indent=1))
