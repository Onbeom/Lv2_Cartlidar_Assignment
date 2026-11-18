"""run_live_period.sh의 도중 지도(snapshots)를 정답 벽과 비교해 루프 폐합 주기별로 정리한다.

실행마다 저장 시점별 유령 벽 비율·벽 위치 오차를 구해 시간 평균을 내고, 반복 실행의 중앙값으로 비교한다.
측정 정의·정렬 방식은 evaluate_maps.py와 같다. 관측 영역은 첫 설정 첫 실행의 마지막 지도 빈칸으로 정한다.

사용: python3 evaluate_snapshots.py <결과 폴더> [maze model.sdf]
"""
import csv
import glob
import importlib.util
import os
import re
import statistics
import sys

from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location('evaluate_maps', os.path.join(HERE, 'evaluate_maps.py'))
ev = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ev)


def main():
    out = sys.argv[1]
    sdf = sys.argv[2] if len(sys.argv) > 2 else ev.DEFAULT_SDF
    gt = ev.world_to_map(ev.gt_points(sdf))
    gt_tree = cKDTree(gt)
    runs = sorted(glob.glob(os.path.join(out, 'runs', '*_r*')))
    first_snaps = sorted(glob.glob(os.path.join(runs[0], 'snapshots', '*.yaml')))
    _, free, _ = ev.load_map(first_snaps[-1][:-5])
    region_tree = cKDTree(gt[cKDTree(free).query(gt)[0] <= ev.REGION_M])

    rows, per_run = [], []
    for run in runs:
        cfg = re.sub(r'_r\d+$', '', os.path.basename(run))
        series = []
        for y in sorted(glob.glob(os.path.join(run, 'snapshots', '*.yaml'))):
            occ, _, _ = ev.load_map(y[:-5])
            if len(occ) == 0:
                continue
            dx, dy, dth = ev.align(occ, gt_tree)
            m = ev.metrics(ev.transform(occ, dx, dy, dth), gt_tree, region_tree)
            t = float(re.search(r'map_t(\d+\.\d+)', y).group(1))
            rows.append({'config': cfg, 'run': os.path.basename(run), 'sim_t': t, **m})
            series.append(m)
        cpu, rss = ev.read_time(os.path.join(run, 'time.txt'))
        if series:
            per_run.append({'config': cfg, 'run': os.path.basename(run), 'n_snap': len(series),
                            'ghost_mean': statistics.mean(s['ghost_ratio'] for s in series),
                            'err_mean_cm': statistics.mean(s['mean_err_cm'] for s in series),
                            'ghost_last': series[-1]['ghost_ratio'], 'err_last_cm': series[-1]['mean_err_cm'],
                            'cpu_s': cpu, 'max_rss_mib': rss})
        print(os.path.basename(run), per_run[-1] if series else 'no snapshots')

    with open(os.path.join(out, 'snapshot_metrics.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    lines = ['# 루프 폐합 주기 비교 (실시간 재생, 도중 지도 시간 평균의 실행 간 중앙값)', '',
             '| 설정 | 실행 수 | 도중 유령 벽 비율 (평균) | 도중 벽 위치 오차 [cm] (평균) | 마지막 유령 벽 비율 | CPU [s] |',
             '|---|---|---|---|---|---|']
    for cfg in sorted({r['config'] for r in per_run}, key=lambda c: -int(re.search(r'opt(\d+)', c).group(1))):
        rs = [r for r in per_run if r['config'] == cfg]
        med = lambda k: statistics.median(r[k] for r in rs)
        lines.append(f"| {cfg} | {len(rs)} | {med('ghost_mean'):.3f} | {med('err_mean_cm'):.2f} | "
                     f"{med('ghost_last'):.3f} | {med('cpu_s'):.1f} |")
    with open(os.path.join(out, 'summary.md'), 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
