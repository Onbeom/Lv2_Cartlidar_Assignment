"""PC: Jetson results/ → 정밀도별 정확도(FP32 기준 같은 장 비율 · 판정) + 실물 부하(GPU · CPU 평균, YOLO 주기 · 처리 시간) 표(11-04). 결과 → precision_result.json."""
import glob, json, os, shutil, subprocess
from pathlib import Path
HERE = Path(__file__).resolve().parent; R = Path(os.environ.get('PREC_RESULTS', HERE / 'results')); W = Path('/home/pa4/git/lv2_next_module')
V, A = W / '261103_1053_video_eval', W / '261104_1211_arena_eval'; PY = '/home/pa4/yolo-venv/bin/python'
names = ['P32', 'P16', 'I8', 'I8F16']; DS = {n: json.load(open(R / f'dets_{n}.json')) for n in names}


def iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0])); iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - ix * iy
    return ix * iy / u if u > 0 else 0.0


def same_ratio(P, Q):
    same = 0
    for k in P:
        a = [d for d in P[k] if d[1] >= 0.6]; b = [d for d in Q[k] if d[1] >= 0.6]; used = set(); ok = len(a) == len(b)
        for d in a:
            m = max(((iou(d[2:], e[2:]), j) for j, e in enumerate(b) if j not in used and e[0] == d[0]), default=(0, None))
            if m[0] >= 0.9: used.add(m[1])
            else: ok = False
        same += ok
    return round(same / len(P), 4)


out = {'same_vs_P32': {n: same_ratio(DS['P32'], DS[n]) for n in names[1:]}}
for n in names:
    shutil.copy(R / f'dets_{n}.json', V / f'dets_{n}.json'); shutil.copy(R / f'dets_{n}.json', A / f'dets_{n}.json')
env = dict(os.environ, JUDGE_STACKED='stacked.json', JUDGE_MIN_SHARP='0', JUDGE_AGNOSTIC='judge_rear,video_rear',
           JUDGE_CAMS='judge_front,judge_rear,video_front,video_rear,video_arm', JUDGE_OUT='judge_precision.json')
subprocess.run([PY, 'judge.py', *names], cwd=V, env=env, check=True, capture_output=True)
T = json.load(open(V / 'judge_precision.json'))
out['judge_11_03'] = {n: {c: [T[n]['0.6'][c]['recall'], T[n]['0.6'][c]['precision']] for c in ['judge_front', 'judge_rear', 'video_front', 'video_rear', 'video_arm']} for n in names}
if any(k.startswith('arena_') for k in DS['P32']):
    env = dict(os.environ, JUDGE_S0_FIXED='114', JUDGE_AGNOSTIC='arena_rear', JUDGE_CAMS='arena_front,arena_rear,arena_arm', JUDGE_OUT='judge_precision.json')
    subprocess.run([PY, 'judge.py', *names], cwd=A, env=env, check=True, capture_output=True)
    T = json.load(open(A / 'judge_precision.json'))
    out['judge_arena'] = {n: {c: [T[n]['0.6'][c]['recall'], T[n]['0.6'][c]['precision']] for c in ['arena_front', 'arena_rear', 'arena_arm']} for n in names}
out['trtexec_gpu_median_ms'] = {k: v['trtexec_gpu_median_ms'] for k, v in json.load(open(R / 'acc_summary.json')).items()}
out['load'] = {}
for d in sorted(glob.glob(str(R / 'load_*/*/'))):
    n = Path(d).parent.name[5:]
    subprocess.run([PY, str(W / '261103_1544_jetson_load/analyze_load.py'), d], check=True, capture_output=True)
    a = json.load(open(Path(d) / 'analysis.json')); t = a['tegrastats']
    out['load'][n] = {'gpu_mean': t['gpu_pct']['mean'], 'gpu_p95': t['gpu_pct']['p95'], 'cpu_mean': t['cpu_avg_pct']['mean'], 'vdd_in_mw_mean': t['vdd_in_mw']['mean'],
                      'temp_max': t['temp_max_c']['max'], 'cams': a['cams']}
json.dump(out, open(HERE / 'precision_result.json', 'w'), ensure_ascii=False, indent=1); print(json.dumps(out, ensure_ascii=False, indent=1)[:6000])
