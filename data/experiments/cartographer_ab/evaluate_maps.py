"""run_offline.sh가 만든 지도들을 미로의 정답 벽과 비교해 summary.md·metrics.csv를 쓴다.

정답 벽(기본): 미로의 보이는 모양(maze.stl)을 LiDAR 높이(LIDAR_Z)에서 자른 단면을 0.01 m 간격 점으로 바꾼 것.
  Gazebo의 gpu_lidar는 보이는 메시를 보므로 이것이 LiDAR가 실제로 본 벽이다.
  (model.sdf의 충돌 박스는 x 방향 판넬 40개·기둥 83개뿐이고 y 방향 벽이 없어 정답으로 쓰지 않는다.
   .sdf를 인자로 주면 예전처럼 충돌 박스를 정답으로 쓴다.)
좌표 맞추기: Cartographer map 원점 = 녹화 시작 때 로봇 위치. 녹화 직전 Gazebo를 새로 띄워 로봇이 시작 위치
(SPAWN)에 있다고 가정한다. 시작 자세의 작은 차이를 없애려고 ±ALIGN_XY m, ±ALIGN_DEG° 안에서 강체 정렬을
한 번 더 찾는다(정렬 전 값도 함께 남긴다).

판정용 측정값(시험 전에 사용자와 정한 정의):
  유령 벽 비율   = 지도의 벽 칸 중 정답 벽에서 GHOST_M보다 먼 칸의 비율
  벽 위치 오차   = 지도의 벽 칸에서 가장 가까운 정답 벽까지 거리의 평균 [cm]
  벽 채움률      = 관측 영역 안 정답 벽 점 중 COVER_M 안에 지도 벽 칸이 있는 비율
  처리 시간·메모리 = /usr/bin/time의 CPU 시간(user+sys)과 최대 RSS (기록만)

사용: python3 evaluate_maps.py <결과 폴더> [maze.stl 또는 model.sdf]
"""
import csv
import glob
import json
import math
import os
import re
import statistics
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml
from PIL import Image
from scipy.spatial import cKDTree

MAZE_DIR = os.path.expanduser('~/git/ros_test/waffle-lyrical-slam/src/waffle_slam_course/models/twin_closed_maze')
DEFAULT_SDF = os.path.join(MAZE_DIR, 'meshes', 'maze.stl')   # 기본 정답 (이름은 예전 호출과 맞추려고 유지)
LIDAR_Z = 0.131                # [m] 바닥에서 LiDAR 높이: 시작 높이 0.010 + Waffle Pi SDF lidar_joint 0.121
SPAWN = (-1.835, 2.757, 0.0)   # [m, m, rad] sim/gazebo_maze_waffle_pi.launch.py의 시작 위치
GHOST_M = 0.10
COVER_M = 0.05
REGION_M = 0.10                # 관측 영역: 기준 지도의 빈칸에서 이 거리 안의 정답 벽만 채움률에 넣는다
ALIGN_XY, ALIGN_XY_STEP = 0.30, 0.01   # 시작 위치와 정답 사이 약 0.1 m 어긋남이 있어 넓게 잡는다
ALIGN_DEG, ALIGN_DEG_STEP = 5.0, 0.25



def gt_points(path, step=0.01):
    """정답 벽 점 (world 좌표). .stl이면 LIDAR_Z 단면, .sdf면 충돌 박스 둘레"""
    if path.lower().endswith('.stl'):
        return stl_slice_points(path, LIDAR_Z, step)
    sdf_path = path
    pts = []
    for col in ET.parse(sdf_path).getroot().iter('collision'):
        box = col.find('geometry/box/size')
        if box is None:
            continue
        sx, sy, _ = (float(v) for v in box.text.split())
        pose = col.find('pose')
        x, y, _, _, _, yaw = (float(v) for v in pose.text.split()) if pose is not None else (0,) * 6
        c, s = math.cos(yaw), math.sin(yaw)
        corners = [(-sx / 2, -sy / 2), (sx / 2, -sy / 2), (sx / 2, sy / 2), (-sx / 2, sy / 2)]
        for (ax, ay), (bx, by) in zip(corners, corners[1:] + corners[:1]):
            n = max(1, int(math.hypot(bx - ax, by - ay) / step))
            for t in np.linspace(0, 1, n, endpoint=False):
                lx, ly = ax + (bx - ax) * t, ay + (by - ay) * t
                pts.append((x + c * lx - s * ly, y + s * lx + c * ly))
    return np.array(pts)


def stl_slice_points(stl_path, z0, step):
    """바이너리 STL을 높이 z0 평면으로 잘라 생긴 선분들을 step 간격 점으로 바꾼다"""
    raw = open(stl_path, 'rb').read()
    n = int(np.frombuffer(raw[80:84], np.uint32)[0])
    rec = np.frombuffer(raw[84:84 + 50 * n], dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))
    tri = rec['v'].astype(float)
    z = tri[:, :, 2]
    tri = tri[(z.min(1) < z0) & (z.max(1) > z0)]
    pts = []
    for t in tri:
        cut = []
        for i, j in ((0, 1), (1, 2), (2, 0)):
            a, b = t[i], t[j]
            if (a[2] - z0) * (b[2] - z0) < 0:
                cut.append(a[:2] + (z0 - a[2]) / (b[2] - a[2]) * (b[:2] - a[:2]))
        if len(cut) == 2:
            k = max(1, int(np.linalg.norm(cut[1] - cut[0]) / step))
            pts.extend(cut[0] + (cut[1] - cut[0]) * f for f in np.linspace(0, 1, k + 1))
    pts = np.array(pts)
    return np.unique(np.round(pts / (step / 2)), axis=0) * (step / 2)   # 겹치는 점 정리


def world_to_map(p):
    x0, y0, th = SPAWN
    c, s = math.cos(-th), math.sin(-th)
    d = p - np.array([x0, y0])
    return np.stack([c * d[:, 0] - s * d[:, 1], s * d[:, 0] + c * d[:, 1]], axis=1)


def load_map(stem):
    """map_server와 같은 규칙으로 벽·빈칸을 고른다: 점유 확률 = (255 - 값)/255 (negate 0),
    occupied_thresh보다 크면 벽, free_thresh보다 작으면 빈칸.
    map_saver의 trinary 지도(0/205/254)와 cartographer_pbstream_to_ros_map의 회색조 지도(모름 = 128)를 모두 읽는다."""
    with open(stem + '.yaml') as f:
        meta = yaml.safe_load(f)
    img = np.array(Image.open(os.path.join(os.path.dirname(stem), meta['image']))).astype(float)
    if meta.get('negate', 0):
        img = 255 - img
    p = (255 - img) / 255.0
    res, (ox, oy) = meta['resolution'], meta['origin'][:2]
    h = img.shape[0]

    def centers(mask):
        r, c = np.nonzero(mask)
        return np.stack([ox + (c + 0.5) * res, oy + (h - r - 0.5) * res], axis=1)

    return centers(p > meta.get('occupied_thresh', 0.65)), centers(p < meta.get('free_thresh', 0.196)), res


def transform(p, dx, dy, dth):
    c, s = math.cos(dth), math.sin(dth)
    return np.stack([c * p[:, 0] - s * p[:, 1] + dx, s * p[:, 0] + c * p[:, 1] + dy], axis=1)


def align(occ, gt_tree):
    """정답 벽까지 거리(GHOST_M에서 자름)의 평균이 가장 작은 작은 강체 변환. 거친 격자 → 그 주변 세밀 격자"""
    def search(cx, cy, cth, half_xy, step_xy, half_deg, step_deg):
        best = (float('inf'), cx, cy, cth)
        xs = np.arange(-half_xy, half_xy + 1e-9, step_xy)
        for th in cth + np.radians(np.arange(-half_deg, half_deg + 1e-9, step_deg)):
            rot = transform(occ, 0, 0, th)
            for dx in cx + xs:
                for dy in cy + xs:
                    d, _ = gt_tree.query(rot + np.array([dx, dy]))
                    cost = np.minimum(d, GHOST_M).mean()
                    if cost < best[0]:
                        best = (cost, dx, dy, th)
        return best[1:]

    dx, dy, th = search(0.0, 0.0, 0.0, ALIGN_XY, 2 * ALIGN_XY_STEP, ALIGN_DEG, 2 * ALIGN_DEG_STEP)
    return search(dx, dy, th, 2 * ALIGN_XY_STEP, ALIGN_XY_STEP / 2, 2 * ALIGN_DEG_STEP, ALIGN_DEG_STEP / 2)


def metrics(occ, gt_tree, gt_region_tree):
    d, _ = gt_tree.query(occ)
    covered = cKDTree(occ).query(gt_region_tree.data)[0] <= COVER_M if len(occ) else np.zeros(0, bool)
    return {'ghost_ratio': float((d > GHOST_M).mean()), 'mean_err_cm': float(d.mean() * 100),
            'inlier_err_cm': float(d[d <= GHOST_M].mean() * 100) if (d <= GHOST_M).any() else float('nan'),
            'coverage': float(covered.mean()) if len(covered) else float('nan'), 'n_wall_cells': int(len(occ))}


def read_time(path):
    try:
        t = open(path).read()
    except OSError:
        return float('nan'), float('nan')
    g = lambda k: float(re.search(k + r':\s*([\d.]+)', t).group(1))
    return g(r'User time \(seconds\)') + g(r'System time \(seconds\)'), g(r'Maximum resident set size \(kbytes\)') / 1024


def main():
    out = sys.argv[1]
    sdf = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_SDF
    manifest = json.load(open(os.path.join(out, 'configs', 'manifest.json')))['configs']
    gt = world_to_map(gt_points(sdf))
    gt_tree = cKDTree(gt)
    # 관측 영역은 기준 설정(c1) 첫 실행의 빈칸으로 정해 모든 설정에 같이 쓴다
    base_stem = os.path.join(out, 'runs', manifest[0]['name'] + '_r1', 'map')
    _, base_free, _ = load_map(base_stem)
    in_region = cKDTree(base_free).query(gt)[0] <= REGION_M
    gt_region_tree = cKDTree(gt[in_region])

    rows = []
    for cfg in manifest:
        for run in sorted(glob.glob(os.path.join(out, 'runs', cfg['name'] + '_r*'))):
            stem = os.path.join(run, 'map')
            if not os.path.exists(stem + '.yaml'):
                rows.append({'config': cfg['name'], 'run': os.path.basename(run), 'ok': False})
                continue
            occ, _, res = load_map(stem)
            raw = metrics(occ, gt_tree, gt_region_tree)
            dx, dy, dth = align(occ, gt_tree)
            al = metrics(transform(occ, dx, dy, dth), gt_tree, gt_region_tree)
            cpu, rss = read_time(os.path.join(run, 'time.txt'))
            rows.append({'config': cfg['name'], 'run': os.path.basename(run), 'ok': True, 'resolution': res,
                         **{k: v for k, v in al.items()}, **{'raw_' + k: v for k, v in raw.items()},
                         'align_dx': dx, 'align_dy': dy, 'align_deg': math.degrees(dth),
                         'cpu_s': cpu, 'max_rss_mib': rss})
            print(rows[-1]['run'], {k: round(v, 4) if isinstance(v, float) else v for k, v in al.items()})

    keys = sorted({k for r in rows for k in r}, key=lambda k: (k not in ('config', 'run', 'ok'), k))
    with open(os.path.join(out, 'metrics.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)

    med = {}
    for cfg in manifest:
        ok = [r for r in rows if r['config'] == cfg['name'] and r.get('ok')]
        med[cfg['name']] = {k: statistics.median(r[k] for r in ok) if ok else float('nan')
                            for k in ('ghost_ratio', 'mean_err_cm', 'inlier_err_cm', 'coverage', 'cpu_s', 'max_rss_mib')}
        med[cfg['name']]['n'] = len(ok)
    base = med[manifest[0]['name']]

    lines = ['# Cartographer 설정 비교 (같은 bag, 설정별 중앙값)', '',
             f'정답: `{os.path.basename(sdf)}`' + (f' (높이 {LIDAR_Z} m 단면)' if sdf.endswith('.stl') else ' 충돌 박스') + f', 유령 벽 기준 {GHOST_M} m, 채움 기준 {COVER_M} m, '
             f'정렬 범위 ±{ALIGN_XY} m·±{ALIGN_DEG}°', '',
             '| 설정 | 바꾼 변수 | 실행 수 | 유령 벽 비율 | 벽 위치 오차 [cm] | (참고) 유령 벽 뺀 오차 [cm] | 벽 채움률 | CPU [s] | 최대 메모리 [MiB] | 판정 |',
             '|---|---|---|---|---|---|---|---|---|---|']
    for cfg in manifest:
        m = med[cfg['name']]
        if cfg is manifest[0]:
            verdict = '기준'
        else:
            better = m['ghost_ratio'] <= base['ghost_ratio'] and m['mean_err_cm'] < base['mean_err_cm']
            cover_ok = m['coverage'] >= base['coverage'] - 0.05
            verdict = '기준보다 나음' if better and cover_ok else ('채움률 탈락' if not cover_ok else '나아지지 않음')
        lines.append(f"| {cfg['name']} | {','.join(cfg['changed']) or '-'} | {m['n']} | {m['ghost_ratio']:.3f} | "
                     f"{m['mean_err_cm']:.2f} | {m['inlier_err_cm']:.2f} | {m['coverage']:.3f} | {m['cpu_s']:.1f} | "
                     f"{m['max_rss_mib']:.0f} | {verdict} |")
    lines += ['', '## 변수별 효과 (그 변수만 다른 네 쌍의 B−A 평균)', '',
              '| 변수 | 유령 벽 비율 | 벽 위치 오차 [cm] | 벽 채움률 | CPU [s] |', '|---|---|---|---|---|']
    for var in ('res', 'w', 'ang'):
        diffs = {k: [] for k in ('ghost_ratio', 'mean_err_cm', 'coverage', 'cpu_s')}
        for a in manifest:
            if var in a['changed']:
                continue
            want = sorted(a['changed'] + [var])
            b = [c for c in manifest if sorted(c['changed']) == want][0]
            for k in diffs:
                diffs[k].append(med[b['name']][k] - med[a['name']][k])
        lines.append(f"| {var} | {statistics.mean(diffs['ghost_ratio']):+.3f} | {statistics.mean(diffs['mean_err_cm']):+.2f} | "
                     f"{statistics.mean(diffs['coverage']):+.3f} | {statistics.mean(diffs['cpu_s']):+.1f} |")
    lines += ['', '판정: 유령 벽 비율 ≤ 기준, 벽 위치 오차 < 기준, 벽 채움률이 기준보다 0.05 넘게 낮지 않음 (중앙값 기준).']
    with open(os.path.join(out, 'summary.md'), 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
