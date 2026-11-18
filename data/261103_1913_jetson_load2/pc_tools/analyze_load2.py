"""Jetson 부하 2단계 분석(PC). 사용: python3 analyze_load2.py <결과 폴더(Jetson 에서 가져온 results/<시각>)> [--ref <PC 재생 결과 폴더>]  → <결과 폴더>/analysis2.json
- 재생 구간(passes.csv 의 play ~ play_end)만 본다. 카메라 · YOLO(cam/frames.csv, monotonic) · tegrastats(벽시계) · proc.csv 를 그 구간으로 자른다.
- 판마다 out_bag(모의 시각 수신): /amcl_pose 주기 · (수신 − stamp), /scan_merged · /odometry/filtered 주기, loc_events.jsonl 검사 주기 · SUSPECT/LOST,
  /amcl_pose 와 정답(모의 K0 판 truth.csv, base_link 발제 좌표) 위치 차이.
- 같은 계산을 원래 모의 실시간 판(K0_pkg_amcl out_bag /amcl_pose)에도 해 'live_pc' 로 둔다."""
import argparse
import bisect
import csv
import json
import math
import re
import sys
from datetime import datetime
from pathlib import Path

import numpy as np

sys.path.insert(0, '/home/pa4/git/lv2_next_module/261101_0016_isaac_live')
import bagutil as bu  # noqa: E402

TRUTH = Path('/home/pa4/git/lv2_next_module/261103_1502_loc_pkg/results/K0_pkg_amcl/truth.csv')
LIVE_BAG = '/home/pa4/git/lv2_next_module/261103_1502_loc_pkg/results/K0_pkg_amcl/out_bag'
ap = argparse.ArgumentParser()
ap.add_argument('run')
ap.add_argument('--ref', default='')
a = ap.parse_args()
D = Path(a.run).resolve()

tr = np.array([[float(r['t']), float(r['x']), float(r['y'])] for r in csv.DictReader(open(TRUTH))])


def q(v, ps=(50, 95, 99)):
    v = np.asarray(v, dtype=float)
    if not len(v):
        return None
    return {**{f'p{p}': round(float(np.percentile(v, p)), 4) for p in ps}, 'mean': round(float(v.mean()), 4), 'max': round(float(v.max()), 4)}


def amcl_vs_truth(bag):
    st, rx, err = [], [], []
    for _, m, r in bu.read_bag(bag, ['/amcl_pose']):
        s = bu.stamp(m.header)
        j = min(np.searchsorted(tr[:, 0], s), len(tr) - 1)
        if s < 3.0:                                   # 출발 정지 구간(초기 자세 그대로)은 빼지 않아도 되지만 판끼리 같게 3 s 뒤부터
            continue
        st.append(s); rx.append(r)
        err.append(math.hypot(m.pose.pose.position.x - tr[j, 1], m.pose.pose.position.y - tr[j, 2]))
    st, rx, err = map(np.array, (st, rx, err))
    dur = st[-1] - st[0] if len(st) > 1 else float('nan')
    return {'n': int(len(st)), 'hz': round(float((len(st) - 1) / dur), 3) if len(st) > 1 else None,
            'rx_minus_stamp_s': q(rx - st), 'pos_err_m': {'rmse': round(float(np.sqrt((err ** 2).mean())), 4), 'p95': round(float(np.percentile(err, 95)), 4),
                                                          'max': round(float(err.max()), 4)} if len(err) else None}


def rate(bag, topic):
    t = [r for _, _, r in bu.read_bag(bag, [topic])]
    return round((len(t) - 1) / (t[-1] - t[0]), 3) if len(t) > 1 else None


def events(path):
    if not path.exists():
        return None
    L = [json.loads(x) for x in path.read_text().splitlines() if x.strip()]
    rep = [d for d in L if d.get('event') == 'report']
    st = [d['state'] for d in L if d.get('event') == 'state']
    return {'check_hz': q([d['n_check'] / 10.0 for d in rep], (50,)) | {'min': round(min(d['n_check'] for d in rep) / 10.0, 2)} if rep else None,
            'res_max': max((d['res_max'] for d in rep if d['res_max'] is not None), default=None), 'n_check_total': sum(d['n_check'] for d in rep), 'n_skip_total': sum(d['n_skip'] for d in rep), 'suspect': st.count('SUSPECT'), 'lost': st.count('LOST'), 'states': st}


def ros_part(run):
    out = {}
    for p in sorted(run.glob('pass_*'), key=lambda x: int(x.name.split('_')[1])):
        bag = str(p / 'out_bag')
        try:
            out[p.name] = {'amcl': amcl_vs_truth(bag), 'scan_merged_hz': rate(bag, '/scan_merged'), 'ekf_hz': rate(bag, '/odometry/filtered'),
                           'loc_monitor': events(p / 'loc_events.jsonl')}
        except Exception as e:  # noqa: BLE001 — 판 하나가 깨져도 나머지는 봄
            out[p.name] = {'error': str(e)}
    return out


res = {'run': str(D), 'live_pc_amcl': amcl_vs_truth(LIVE_BAG), 'passes': ros_part(D)}
if a.ref:
    res['ref_passes'] = ros_part(Path(a.ref).resolve())

# 재생 구간
pas = {}
for r in csv.DictReader(open(D / 'passes.csv')):
    pas.setdefault(r['pass'], {})[r['event']] = (float(r['wall_s']), float(r['mono_s']))
win = [(v['play'], v['play_end']) for v in pas.values() if 'play' in v and 'play_end' in v]
in_wall = lambda t: any(w[0][0] <= t <= w[1][0] for w in win)
in_mono = lambda t: any(w[0][1] <= t <= w[1][1] for w in win)
res['windows_s'] = [round(w[1][0] - w[0][0], 1) for w in win]
T_ROS = sum(res['windows_s'])

# proc.csv
if (D / 'proc.csv').exists():
    # proc_sample 의 이름 맞춤은 명령줄 낱말이라 Jetson 경로(~/load_test2/…)에서는 ROS 노드가 모두 'load_test' 로 잡힘 → launch.log 의 pid 로 노드 이름을 다시 붙임
    pid_name = {}
    for lg in D.glob('pass_*/launch.log'):
        for m in re.finditer(r'\[INFO\] \[(\w+?)-\d+\]: process started with pid \[(\d+)\]', lg.read_text(errors='replace')):
            pid_name[m.group(2)] = m.group(1)
    allrows = list(csv.DictReader(open(D / 'proc.csv')))
    life = {}
    for r in allrows:
        life.setdefault(r['pid'], []).append(float(r['wall_s']))
    def name_of(r):
        if r['pid'] in pid_name:
            return pid_name[r['pid']]
        if r['key'] in ('load_test', 'load_test.py') and max(life[r['pid']]) - min(life[r['pid']]) < 500:   # 판마다 새로 뜨는 것 = ros2 launch 자신
            return 'ros2_launch'
        return r['key']
    rows = [dict(r, key=name_of(r)) for r in allrows if in_wall(float(r['wall_s']))]
    per = {}
    for r in rows:
        per.setdefault(r['key'], {}).setdefault(r['wall_s'], [0.0, 0.0])
        e = per[r['key']][r['wall_s']]
        e[0] += float(r['cpu_pct']); e[1] += float(r['rss_mb'])
    res['proc_by_pass'] = {}
    for n, (w0, w1) in enumerate(win, 1):
        pp = {}
        for r in rows:
            if w0[0] <= float(r['wall_s']) <= w1[0]:
                pp.setdefault(r['key'], {}).setdefault(r['wall_s'], 0.0)
                pp[r['key']][r['wall_s']] += float(r['cpu_pct'])
        res['proc_by_pass'][f'pass_{n}'] = {k: round(float(np.mean(list(v.values()))), 1) for k, v in sorted(pp.items())}
    res['proc'] = {k: {'cpu_pct': q([x[0] for x in v.values()], (50, 95)), 'rss_mb_max': round(max(x[1] for x in v.values()), 1)} for k, v in per.items()}

# 카메라 · YOLO(1단계 analyze_load.py 와 같은 계산, 재생 구간만)
cam = D / 'cam'
cdirs = sorted(cam.glob('*/frames.csv')) if cam.exists() else []
if cdirs:
    C = cdirs[-1].parent
    rows = [r for r in csv.DictReader(open(C / 'frames.csv')) if r['t_start'] and in_mono(float(r['t_start']))]
    stale = [r for r in csv.DictReader(open(C / 'frames.csv')) if r['stale'] == '1']
    S = json.load(open(C / 'summary.json')) if (C / 'summary.json').exists() else {}
    res['cams'] = {'errors': S.get('errors'), 'summary_cams': S.get('cams'), 'seconds_total': S.get('seconds')}
    for c in ('front', 'rear', 'arm'):
        R = [r for r in rows if r['cam'] == c]
        age = np.array([float(r['age_ms']) for r in R])
        miss = age > 300
        run = best = 0
        for mm in miss:
            run = run + 1 if mm else 0
            best = max(best, run)
        first = [win[0][0][1], win[0][0][1] + 300]
        last = [win[-1][1][1] - 300, win[-1][1][1]]
        res['cams'][c] = {'yolo_hz': round(len(R) / T_ROS, 2) if T_ROS else None, 'infer_ms': q([float(r['infer_ms']) for r in R]), 'age_ms': q(age),
                          'miss_rate_age300': round(float(miss.mean()), 5) if len(miss) else None, 'longest_miss_run': best,
                          'stale_slots_total': sum(1 for r in stale if r['cam'] == c),
                          'age_p99_first5min': round(float(np.percentile([float(r['age_ms']) for r in R if first[0] <= float(r['t_start']) <= first[1]], 99)), 1) if R else None,
                          'age_p99_last5min': round(float(np.percentile([float(r['age_ms']) for r in R if last[0] <= float(r['t_start']) <= last[1]], 99)), 1) if R else None}
    teg = C / 'tegrastats.log'
else:
    teg = D / 'tegrastats.log'

# tegrastats(재생 구간만)
if teg.exists():
    cpu, gpu, ram, temp, pw, ram_t = [], [], [], [], [], []
    for ln in teg.read_text().splitlines():
        m = re.match(r'(\d\d-\d\d-\d{4} \d\d:\d\d:\d\d)', ln)
        if not m or not in_wall(datetime.strptime(m.group(1), '%m-%d-%Y %H:%M:%S').timestamp()):
            continue
        g = re.search(r'RAM (\d+)/(\d+)MB', ln)
        if g:
            ram.append(int(g.group(1)))
        g = re.search(r'CPU \[([^\]]+)\]', ln)
        if g:
            cpu.append(np.mean([int(x.split('%')[0]) for x in g.group(1).split(',') if '%' in x]))
        g = re.search(r'GR3D_FREQ (\d+)%', ln)
        if g:
            gpu.append(int(g.group(1)))
        t = [float(x) for x in re.findall(r'(?:cpu|gpu|tj|soc\d?)@([\d.]+)C', ln)]
        if t:
            temp.append(max(t))
        g = re.search(r'VDD_IN (\d+)mW', ln)
        if g:
            pw.append(int(g.group(1)))
    res['tegrastats'] = {'samples': len(cpu), 'cpu_avg_pct': q(cpu, (95,)), 'gpu_pct': q(gpu, (95,)), 'ram_mb': q(ram, (95,)),
                         'ram_growth_mb_last_minus_first_min': round(float(np.mean(ram[-60:]) - np.mean(ram[:60])), 1) if len(ram) > 120 else None,
                         'temp_max_c': max(temp) if temp else None, 'vdd_in_mw': q(pw, (95,))}
(D / 'analysis2.json').write_text(json.dumps(res, ensure_ascii=False, indent=1))
print(json.dumps(res, ensure_ascii=False, indent=1))
