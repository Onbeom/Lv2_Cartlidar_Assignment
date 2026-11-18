"""지도 저장 + 미로 도면 맞춤(시스템 python3 · numpy · OpenCV · yaml, ROS 2 서비스는 ros2 CLI).
1) (선택) /finish_trajectory · /write_state → <out>/map.pbstream, cartographer_pbstream_to_ros_map → <out>/map_carto.{pgm,yaml}(Cartographer map frame = 출발 자세)
2) 벽 칸(점유 > occupied_thresh)을 출발 자세 추정(발제 좌표 --start)으로 발제 frame 에 놓고, 미로 도면 벽(--walls, USD 좌표 + --usd-offset)까지
   거리(10 cm 에서 자름) 평균이 가장 작은 작은 강체 보정(출발점 둘레 회전 · 이동)을 거친 격자 → 세밀 격자로 찾음(evaluate_maps.align 과 같은 방식, 거리는 OpenCV 거리 변환)
3) 출력: <out>/align.json(발제 ← Cartographer map 변환, 보정량, 벽 오차 · ghost 전후), <out>/map_mission.{pgm,yaml}(발제 frame 으로 돌려 다시 그린 지도 —
   map_server 로 읽으면 'map' frame = 발제 좌표), 정적 TF 명령 예(발제 frame 이름은 미정 — 기본 mission_map).
사용: python3 save_and_align_map.py --out <폴더> [--skip-services] [--walls …json] [--usd-offset 2.2345 3.2136] [--start 0.734 0.950 3.14159265]"""
import argparse
import json
import math
import subprocess
from pathlib import Path

import cv2
import numpy as np
import yaml

ap = argparse.ArgumentParser()
ap.add_argument('--out', type=Path, required=True)
ap.add_argument('--skip-services', action='store_true', help='map.pbstream 이 이미 있으면 서비스 호출을 건너뜀')
ap.add_argument('--walls', type=Path, default=Path('/home/pa4/git/lv2_next_module/261030_1943/maze_slice_z0.08.json'))
ap.add_argument('--usd-offset', type=float, nargs=2, default=[2.2345, 3.2136], help='발제 = USD + 이 값(회전 0, 사용자 확인 2026-10-30)')
ap.add_argument('--start', type=float, nargs=3, default=[0.734, 0.950, math.pi], help='출발 자세(발제 좌표 m, m, rad)')
ap.add_argument('--resolution', type=float, default=0.025)
ap.add_argument('--pbstream-to-map', default='/home/pa4/git/ros_test/waffle-lyrical-slam/install/cartographer_ros/lib/cartographer_ros/cartographer_pbstream_to_ros_map')
ap.add_argument('--frame', default='mission_map', help='정적 TF 예에 쓸 발제 frame 이름(미정)')
a = ap.parse_args()
a.out.mkdir(parents=True, exist_ok=True)
CLIP = 0.10


def run(cmd, timeout=180):
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    if r.returncode:
        raise SystemExit(f'실패: {" ".join(cmd)}\n{r.stdout[-1500:]}{r.stderr[-1500:]}')
    return r.stdout


pb = a.out / 'map.pbstream'
if not a.skip_services:
    run(['ros2', 'service', 'call', '/finish_trajectory', 'cartographer_ros_msgs/srv/FinishTrajectory', '{trajectory_id: 0}'])
    run(['ros2', 'service', 'call', '/write_state', 'cartographer_ros_msgs/srv/WriteState',
         json.dumps({'filename': str(pb.resolve()), 'include_unfinished_submaps': True})])
run([a.pbstream_to_map, '-pbstream_filename', str(pb), '-map_filestem', str(a.out / 'map_carto'), '-resolution', str(a.resolution)], 120)

meta = yaml.safe_load((a.out / 'map_carto.yaml').read_text())
img = cv2.imread(str(a.out / meta['image']), cv2.IMREAD_UNCHANGED)
if img.ndim == 3:
    img = img[:, :, 0]
p = (255 - img.astype(float)) / 255.0
if meta.get('negate', 0):
    p = 1 - p
res, (ox, oy) = meta['resolution'], meta['origin'][:2]
H, W = img.shape
r_, c_ = np.nonzero(p > meta.get('occupied_thresh', 0.65))
occ = np.stack([ox + (c_ + 0.5) * res, oy + (H - r_ - 0.5) * res], 1)            # Cartographer map frame

# 도면 벽(발제 frame) → 1 cm 래스터 거리 변환
segs = json.loads(a.walls.read_text())['segments']
walls = []
for _, s0, s1 in segs:
    s0, s1 = np.array(s0) + a.usd_offset, np.array(s1) + a.usd_offset
    walls.append(s0 + np.linspace(0, 1, max(2, int(np.linalg.norm(s1 - s0) / 0.005) + 1))[:, None] * (s1 - s0))
walls = np.concatenate(walls)
G = 0.01
gx0, gy0 = walls.min(0) - 1.0
gw, gh = (np.ceil((walls.max(0) + 1.0 - (gx0, gy0)) / G)).astype(int)
grid = np.full((gh, gw), 255, np.uint8)
ij = np.floor((walls - (gx0, gy0)) / G).astype(int)
grid[ij[:, 1], ij[:, 0]] = 0
dist = cv2.distanceTransform(grid, cv2.DIST_L2, 5) * G


def cost(pts):
    k = np.floor((pts - (gx0, gy0)) / G).astype(int)
    inside = (k[:, 0] >= 0) & (k[:, 0] < gw) & (k[:, 1] >= 0) & (k[:, 1] < gh)
    d = np.full(len(pts), CLIP)
    d[inside] = np.minimum(dist[k[inside, 1], k[inside, 0]], CLIP)
    return d


sx, sy, sth = a.start


def to_mission(pts, dx, dy, dth):
    """출발 자세 추정 뒤 출발점 둘레로 dth 회전 + (dx, dy) 이동"""
    th = sth + dth
    c, s = math.cos(th), math.sin(th)
    return np.stack([sx + dx + c * pts[:, 0] - s * pts[:, 1], sy + dy + s * pts[:, 0] + c * pts[:, 1]], 1)


def search(cx, cy, cth, half, step, half_deg, step_deg):
    best = (np.inf, cx, cy, cth)
    xs = np.arange(-half, half + 1e-9, step)
    for th in cth + np.radians(np.arange(-half_deg, half_deg + 1e-9, step_deg)):
        for dx in cx + xs:
            for dy in cy + xs:
                c = cost(to_mission(occ, dx, dy, th)).mean()
                if c < best[0]:
                    best = (c, dx, dy, th)
    return best


before = cost(to_mission(occ, 0, 0, 0))
_, dx, dy, dth = search(0.0, 0.0, 0.0, 0.30, 0.02, 5.0, 0.5)
_, dx, dy, dth = search(dx, dy, dth, 0.04, 0.005, 1.0, 0.125)
after = cost(to_mission(occ, dx, dy, dth))
# 발제 ← Cartographer map 변환: p_m = R(sth+dth)·p + (sx+dx, sy+dy)
T = {'x': sx + dx, 'y': sy + dy, 'yaw': sth + dth}

# 발제 frame 으로 다시 그린 지도(같은 해상도, 최근접, 모르는 칸은 원래 지도의 모름 값)
unknown = int(np.bincount(img[[0, -1], :].ravel()).argmax())
corners = to_mission(np.array([[ox, oy], [ox + W * res, oy], [ox, oy + H * res], [ox + W * res, oy + H * res]]), dx, dy, dth)
mx0, my0 = np.floor(corners.min(0) / res) * res
mx1, my1 = np.ceil(corners.max(0) / res) * res
W2, H2 = int(round((mx1 - mx0) / res)), int(round((my1 - my0) / res))
c, s = math.cos(T['yaw']), math.sin(T['yaw'])
# 원본 화소 (u, v) → 지도 좌표 → 발제 좌표 → 새 화소 (u2, v2) 의 아핀 행렬
A_src = np.array([[res, 0, ox + 0.5 * res], [0, -res, oy + (H - 0.5) * res], [0, 0, 1]])
A_T = np.array([[c, -s, T['x']], [s, c, T['y']], [0, 0, 1]])
A_dst = np.linalg.inv(np.array([[res, 0, mx0 + 0.5 * res], [0, -res, my0 + (H2 - 0.5) * res], [0, 0, 1]]))
M = (A_dst @ A_T @ A_src)[:2]
img2 = cv2.warpAffine(img, M, (W2, H2), flags=cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=unknown)
cv2.imwrite(str(a.out / 'map_mission.pgm'), img2)
meta2 = dict(meta, image='map_mission.pgm', origin=[float(mx0), float(my0), 0.0])
(a.out / 'map_mission.yaml').write_text(yaml.safe_dump(meta2, sort_keys=False))
result = {'mission_from_carto_map': T, 'correction_from_start_guess': {'dx_m': dx, 'dy_m': dy, 'dyaw_deg': math.degrees(dth)},
          'start_guess': {'x': sx, 'y': sy, 'yaw': sth}, 'n_wall_cells': int(len(occ)),
          'wall_err_cm_before': float(before.mean() * 100), 'wall_err_cm_after': float(after.mean() * 100),
          'ghost_ratio_before': float((before >= CLIP).mean()), 'ghost_ratio_after': float((after >= CLIP).mean()),
          'walls': str(a.walls), 'usd_offset': a.usd_offset, 'unknown_value': unknown,
          'static_tf_example': f'ros2 run tf2_ros static_transform_publisher --x {T["x"]:.4f} --y {T["y"]:.4f} --yaw {T["yaw"]:.6f} '
                               f'--frame-id {a.frame} --child-frame-id map',
          'note': '거리는 10 cm 에서 자름. map_mission.yaml 을 map_server 로 읽으면 map frame = 발제 좌표'}
(a.out / 'align.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
print(json.dumps(result, ensure_ascii=False))
