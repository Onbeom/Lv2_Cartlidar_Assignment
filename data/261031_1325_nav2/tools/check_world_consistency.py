"""장애물 자료 일치 검사(읽기만): 세 자료에 같은 장애물이 같은 자리에 있는지 대조한다.
  ① 참값 = 장면 미로 USD 의 모든 충돌 prim(Mesh 삼각형 · Cylinder 원판, 높이 z_range 안) — 모의 물리가 실제로 부딪히는 것
  ② geom = 목표 생성 · G3 채점 · 분석이 쓰는 선분(nodes/geom.py load_world()['body'] 의 벽 · 기둥 부분, layout 물체 제외)
  ③ Nav2 = costmap 이 받는 지도(벽 지도 map_mission_nav + 물체 지도 objects_map 의 점유 칸)
①의 점유 칸마다 ② · ③ 의 가장 가까운 점유까지 거리를 재서 tol 넘게 떨어진 칸(= 그 자료에 빠진 장애물)을 덩어리로 묶어 보고한다. 반대로 ③ 에만 있는 점유(①에서 tol 넘게 떨어짐)도 센다.
261101 설계 25 에서 미로 충돌 원기둥 110 개가 ② 에 빠져 있던 것을 늦게 찾아서 만듦. 실물에서는 ① 대신 발제 도면(벽 · 기둥 · 박스) · ③ 대신 실측 SLAM 지도로 같은 대조를 한다.
사용(pxr 필요): ~/isaacsim-venv/bin/python -I check_world_consistency.py [--objects_map ...] [--out ...]"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import yaml
from pxr import Usd, UsdGeom, UsdPhysics
from scipy import ndimage

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE / 'nodes'))
import geom  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('--maze', default='/home/pa4/git/lv2_next_module/261030_1943/maze_src/maze/maze.usda')
ap.add_argument('--wall_map', default=str(HERE / 'config' / 'map_mission_nav.yaml'))
ap.add_argument('--objects_map', default=str(HERE / 'config' / 'map_objects_fine_posts.yaml'))
ap.add_argument('--z_range', type=float, nargs=2, default=[0.0, 0.50], help='차체 높이 범위(이 안의 충돌 형상만)')
ap.add_argument('--res', type=float, default=0.005)
ap.add_argument('--tol', type=float, default=0.02, help='이보다 멀면 빠진 것으로 봄(m). SLAM 칸 0.025 의 반 + 여유')
ap.add_argument('--geom_posts', type=int, default=1, help='0 이면 geom 기둥 없이(옛 상태 재현)')
ap.add_argument('--out', default=str(HERE / 'tools' / 'check_world_consistency.json'))
a = ap.parse_args()
OX, OY = geom.USD_OFFSET
X0, Y0, X1, Y1 = -0.15, -0.15, 4.65, 6.55            # 발제 좌표 범위
W, H = int((X1 - X0) / a.res) + 1, int((Y1 - Y0) / a.res) + 1


def mark(grid, xs, ys):
    i = np.round((np.asarray(xs) - X0) / a.res).astype(int)
    j = np.round((np.asarray(ys) - Y0) / a.res).astype(int)
    ok = (i >= 0) & (i < W) & (j >= 0) & (j < H)
    grid[j[ok], i[ok]] = True


def seg_points(x0, y0, x1, y1):
    n = max(2, int(np.hypot(x1 - x0, y1 - y0) / (a.res / 2)) + 1)
    t = np.linspace(0, 1, n)
    return x0 + (x1 - x0) * t, y0 + (y1 - y0) * t


# ① 참값: USD 충돌 형상
truth = np.zeros((H, W), bool)
st = Usd.Stage.Open(a.maze)
kinds = {}
for p in Usd.PrimRange(st.GetPseudoRoot()):
    if not p.HasAPI(UsdPhysics.CollisionAPI):
        continue
    kinds[p.GetTypeName()] = kinds.get(p.GetTypeName(), 0) + 1
    xf = np.array(UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
    if p.IsA(UsdGeom.Mesh):
        m = UsdGeom.Mesh(p)
        pts = np.array(m.GetPointsAttr().Get(), float)
        pts = (np.c_[pts, np.ones(len(pts))] @ xf)[:, :3]
        idx, cnt, k = m.GetFaceVertexIndicesAttr().Get(), m.GetFaceVertexCountsAttr().Get(), 0
        for c in cnt:
            poly = pts[list(idx[k:k + c])]
            k += c
            if poly[:, 2].max() < a.z_range[0] or poly[:, 2].min() > a.z_range[1]:
                continue
            for e in range(c):
                xs, ys = seg_points(poly[e, 0] + OX, poly[e, 1] + OY, poly[(e + 1) % c, 0] + OX, poly[(e + 1) % c, 1] + OY)
                mark(truth, xs, ys)
    elif p.IsA(UsdGeom.Cylinder):
        cy = UsdGeom.Cylinder(p)
        sc = np.linalg.norm(xf[:3, :3], axis=1)
        r = cy.GetRadiusAttr().Get() * max(sc[0], sc[1])
        cx, cyy = xf[3, 0] + OX, xf[3, 1] + OY
        th = np.linspace(0, 2 * np.pi, max(16, int(2 * np.pi * r / (a.res / 2))), endpoint=False)
        mark(truth, cx + r * np.cos(th), cyy + r * np.sin(th))      # 윤곽(메시 · geom 과 같게 경계끼리 대조)
    else:
        print('모르는 충돌 prim 종류 — 검사에서 빠짐:', p.GetTypeName(), p.GetPath())

# ② geom(벽 · 기둥 선분만, layout 물체 제외)
gw = geom.load_world(posts=geom.POSTS if a.geom_posts else None)
n_obj = sum(4 for _ in gw['objects'])
segs = gw['body'][:len(gw['body']) - n_obj]
gmap = np.zeros((H, W), bool)
for s in segs:
    mark(gmap, *seg_points(*s))
# 참값 · geom · 지도 모두 경계(윤곽)끼리 대조. 지도 칸은 칸 전체를 칠하므로 안쪽도 들어감(빠짐 검사엔 영향 없음).


# ③ Nav2 지도(벽 지도 + 물체 지도 점유 칸 → 칸 중심 격자에 칸 전체를 칠함)
def load_map(path):
    y = yaml.safe_load(open(path))
    raw = (Path(path).parent / y['image']).read_bytes().split(maxsplit=4)
    w_, h_ = int(raw[1]), int(raw[2])
    img = np.frombuffer(raw[4], np.uint8).reshape(h_, w_)
    p = (255 - img.astype(float)) / 255.0
    return y, p > y['occupied_thresh']


nav, nav_wall = np.zeros((H, W), bool), np.zeros((H, W), bool)
for path in (a.wall_map, a.objects_map):
    y, occ = load_map(path)
    r, (ox, oy) = y['resolution'], y['origin'][:2]
    jj, ii = np.nonzero(occ)
    cxs, cys = ox + (ii + 0.5) * r, oy + (occ.shape[0] - 1 - jj + 0.5) * r
    off = np.arange(-r / 2 + a.res / 2, r / 2, a.res)
    for dx in off:
        for dy in off:
            mark(nav, cxs + dx, cys + dy)
            if path == a.wall_map:
                mark(nav_wall, cxs + dx, cys + dy)


def missing(ref, other, name):
    d = ndimage.distance_transform_edt(~other) * a.res
    bad = ref & (d > a.tol)
    lab, n = ndimage.label(bad, structure=np.ones((3, 3)))
    blobs = []
    for k in range(1, n + 1):
        jj, ii = np.nonzero(lab == k)
        blobs.append({'center': [round(X0 + ii.mean() * a.res, 3), round(Y0 + jj.mean() * a.res, 3)],
                      'n_cells': int(len(ii)), 'max_dist_m': round(float(d[jj, ii].max()), 3)})
    blobs.sort(key=lambda b: -b['max_dist_m'])
    return {'what': name, 'n_cells': int(bad.sum()), 'n_blobs': n, 'max_dist_m': round(float(d[ref].max()), 4) if ref.any() else None, 'blobs_top': blobs[:15]}


res = {'args': vars(a), 'collision_prim_kinds': kinds,
       'geom_missing_truth': missing(truth, gmap, '참값 충돌 형상인데 geom 에 없음'),
       'nav_missing_truth': missing(truth, nav, '참값 충돌 형상인데 Nav2 지도에 없음'),
       'nav_extra_vs_truth': missing(nav_wall, truth, 'Nav2 벽 지도 점유인데 참값 충돌 형상 근처에 없음(물체 지도는 장면에 없는 layout 물체라 뺌)')}
res['pass'] = res['geom_missing_truth']['n_cells'] == 0 and res['nav_missing_truth']['n_cells'] == 0
Path(a.out).write_text(json.dumps(res, ensure_ascii=False, indent=1))
for k in ('geom_missing_truth', 'nav_missing_truth', 'nav_extra_vs_truth'):
    r = res[k]
    print(f"{r['what']}: 칸 {r['n_cells']} · 덩어리 {r['n_blobs']} · 최대 거리 {r['max_dist_m']} m · 상위 {[(b['center'], b['max_dist_m']) for b in r['blobs_top'][:6]]}")
print('충돌 prim 종류', kinds, '| 통과' if res['pass'] else '| 불통과')
