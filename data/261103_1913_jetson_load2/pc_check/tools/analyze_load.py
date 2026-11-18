"""load_test.py 결과 분석(PC 에서): frames.csv + tegrastats.log → 카메라별 YOLO 주기 · 지연 p50/p95/p99 · 결과 나이 · 미스율 · 연속 미스, CPU · GPU · RAM · 온도 · 전력, 초반 5분 대 마지막 5분. 사용: analyze_load.py <결과 폴더> [기준 json]"""
import csv, json, re, sys
from pathlib import Path
import numpy as np
D = Path(sys.argv[1]); rows = list(csv.DictReader(open(D / 'frames.csv'))); S = json.load(open(D / 'summary.json'))
done = [r for r in rows if r['stale'] == '0']; T0 = min(float(r['t_start']) for r in done); dur = S['seconds']
out = {'seconds': dur, 'cams': {}, 'errors': S['errors']}
for c in ('front', 'rear', 'arm'):
    R = [r for r in done if r['cam'] == c]; lat = np.array([float(r['infer_ms']) for r in R]); age = np.array([float(r['age_ms']) for r in R])
    miss = age > 300; run = best = 0
    for m in miss: run = run + 1 if m else 0; best = max(best, run)
    st = S['cams'].get(c, {})
    q = lambda v: {p: round(float(np.percentile(v, p)), 2) for p in (50, 95, 99)} | {'max': round(float(v.max()), 2)} if len(v) else {}
    early = [float(r['age_ms']) for r in R if float(r['t_start']) - T0 < 300]; late = [float(r['age_ms']) for r in R if float(r['t_start']) - T0 > dur - 300]
    out['cams'][c] = {'yolo_hz': round(len(R) / dur, 2), 'stale_slots': sum(1 for r in rows if r['cam'] == c and r['stale'] == '1'),
                      'cam_fps': round(st.get('frames', 0) / dur, 2), 'cam_gaps': st.get('gaps'), 'infer_ms': q(lat), 'age_ms': q(age),
                      'miss_rate_age300': round(float(miss.mean()), 5) if len(miss) else None, 'longest_miss_run': best,
                      'age_p99_first5min': round(float(np.percentile(early, 99)), 1) if early else None, 'age_p99_last5min': round(float(np.percentile(late, 99)), 1) if late else None}
L = open(D / 'tegrastats.log').read().splitlines(); cpu, gpu, ram, temp, pw = [], [], [], [], []
for l in L:
    m = re.search(r'RAM (\d+)/(\d+)MB', l); ram.append(int(m.group(1))) if m else None
    m = re.search(r'CPU \[([^\]]+)\]', l)
    if m: cpu.append(np.mean([int(x.split('%')[0]) for x in m.group(1).split(',') if '%' in x]))
    m = re.search(r'GR3D_FREQ (\d+)%', l); gpu.append(int(m.group(1))) if m else None
    t = [float(x) for x in re.findall(r'(?:cpu|gpu|tj|soc\d?)@([\d.]+)C', l)]; temp.append(max(t)) if t else None
    m = re.search(r'VDD_IN (\d+)mW', l); pw.append(int(m.group(1))) if m else None
q2 = lambda v: {'mean': round(float(np.mean(v)), 1), 'p95': round(float(np.percentile(v, 95)), 1), 'max': round(float(np.max(v)), 1)} if v else None
out['tegrastats'] = {'cpu_avg_pct': q2(cpu), 'gpu_pct': q2(gpu), 'ram_mb': q2(ram), 'ram_growth_mb_last_minus_first_min': round(float(np.mean(ram[-60:]) - np.mean(ram[:60])), 1) if len(ram) > 120 else None,
                     'temp_max_c': q2(temp), 'vdd_in_mw': q2(pw), 'samples': len(L)}
json.dump(out, open(D / 'analysis.json', 'w'), ensure_ascii=False, indent=1); print(json.dumps(out, ensure_ascii=False, indent=1))
