"""Waffle 4WD USD 충돌 형상을 수치로 확인한다(Colliders 화면 확인 대신).

C1 링크마다 충돌 형상 범위와 표시 메시 범위의 차이(축마다, m)
C2 충돌 메시마다 convex hull 부피 / 원래 메시 부피 (PhysX는 convexHull 근사를 쓴다)
C3 관절 0 자세에서 관절로 직접 붙지 않은 링크끼리 충돌 형상(볼록 형상 기준)이 겹치는 쌍

USD 파일은 바꾸지 않는다(Physics variant는 세션 레이어에서만 고른다).
겹침 판정: 두 볼록 형상의 교집합에 들어가는 가장 큰 구의 반지름(선형계획)으로 잰다.

사용 (PC):
  ~/isaacsim-venv/bin/python -I check_colliders_waffle_4wd.py --usd <...>/waffle_4wd.usda [--json 결과.json]
"""
import argparse
import itertools
import json
import sys

import numpy as np
import trimesh
from pxr import Usd, UsdGeom, UsdPhysics
from scipy.optimize import linprog
from scipy.spatial import ConvexHull

TOL_C1_M = 0.005
TOL_C2_RATIO = 1.01
TOL_C3_OVERLAP_M = 1e-6     # 교집합 내접 구 반지름이 이보다 크면 겹침
CYLINDER_SIDES = 64


def world_points(prim, cache):
    """충돌 형상의 꼭짓점(월드 좌표, m)."""
    m = np.array(cache.GetLocalToWorldTransform(prim)).T   # 열 벡터 규약
    if prim.IsA(UsdGeom.Cube):
        h = prim.GetAttribute('size').Get() / 2.0
        local = np.array(list(itertools.product([-h, h], repeat=3)))
    elif prim.IsA(UsdGeom.Cylinder):
        r = prim.GetAttribute('radius').Get()
        hh = prim.GetAttribute('height').Get() / 2.0
        axis = prim.GetAttribute('axis').Get()
        t = np.linspace(0, 2 * np.pi, CYLINDER_SIDES, endpoint=False)
        ring = np.stack([r * np.cos(t), r * np.sin(t)], axis=1)
        pts = [np.insert(ring, 2, z, axis=1) for z in (-hh, hh)]   # z축 원기둥
        local = np.concatenate(pts)
        order = {'X': [2, 0, 1], 'Y': [1, 2, 0], 'Z': [0, 1, 2]}[axis]
        local = local[:, order]
    elif prim.IsA(UsdGeom.Mesh):
        local = np.array(prim.GetAttribute('points').Get(), dtype=float)
    else:
        raise ValueError(f'모르는 형상 {prim.GetTypeName()} {prim.GetPath()}')
    return (m[:3, :3] @ local.T).T + m[:3, 3]


def mesh_volume_ratio(prim):
    pts = np.array(prim.GetAttribute('points').Get(), dtype=float)
    counts = np.array(prim.GetAttribute('faceVertexCounts').Get())
    idx = np.array(prim.GetAttribute('faceVertexIndices').Get())
    faces, k = [], 0
    for c in counts:                         # 다각형 면은 부채꼴로 삼각형 분할
        poly = idx[k:k + c]
        faces += [[poly[0], poly[i], poly[i + 1]] for i in range(1, c - 1)]
        k += c
    tm = trimesh.Trimesh(vertices=pts, faces=np.array(faces), process=True)
    hull_volume = ConvexHull(pts).volume
    return hull_volume / abs(tm.volume), tm.is_watertight, hull_volume


def overlap_radius(points_a, points_b):
    """두 볼록 형상 교집합의 내접 구 반지름. 교집합이 없으면 음수 또는 None."""
    rows, rhs = [], []
    for pts in (points_a, points_b):
        eq = ConvexHull(pts).equations          # n·x + d <= 0
        normal = eq[:, :3]
        norm = np.linalg.norm(normal, axis=1)
        rows.append(np.hstack([normal, norm[:, None]]))   # n·x + |n| r <= -d
        rhs.append(-eq[:, 3])
    res = linprog(c=[0, 0, 0, -1], A_ub=np.vstack(rows), b_ub=np.concatenate(rhs),
                  bounds=[(None, None)] * 3 + [(None, 1.0)], method='highs')
    return res.x[3] if res.status == 0 else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--usd', required=True)
    parser.add_argument('--json')
    args = parser.parse_args()

    stage = Usd.Stage.Open(args.usd)
    robot = stage.GetDefaultPrim()
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        robot.GetVariantSets().GetVariantSet('Physics').SetVariantSelection('physx')
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])

    bodies = {p.GetName(): p for p in Usd.PrimRange(robot) if p.HasAPI(UsdPhysics.RigidBodyAPI)}

    def owner(prim):
        while prim and not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            prim = prim.GetParent()
        return prim.GetName() if prim else None

    shapes = []   # (링크, prim, 월드 꼭짓점)
    for prim in Usd.PrimRange(robot, Usd.TraverseInstanceProxies()):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            shapes.append((owner(prim), prim, world_points(prim, cache)))
    result = {'shapes': len(shapes)}
    print(f'충돌 형상 {len(shapes)}개, 강체 링크 {len(bodies)}개')

    # C1: 링크별 범위 비교
    c1, c1_ok = {}, True
    for name, body in bodies.items():
        pts = np.concatenate([s[2] for s in shapes if s[0] == name]) if any(s[0] == name for s in shapes) else None
        visual = body.GetChild(name)          # 표시 메시 묶음 = 링크 아래 같은 이름의 인스턴스
        if pts is None or not visual.IsValid():
            c1[name] = {'note': f'충돌 {pts is not None}, 표시 {visual.IsValid()}'}
            continue
        vr = bbox.ComputeWorldBound(visual).ComputeAlignedRange()
        vmin, vmax = np.array(vr.GetMin()), np.array(vr.GetMax())
        cmin, cmax = pts.min(0), pts.max(0)
        diff = np.maximum(np.abs(cmin - vmin), np.abs(cmax - vmax))
        ok = bool(np.all(diff <= TOL_C1_M))
        c1_ok &= ok
        c1[name] = {'ok': ok, 'max_diff_mm_xyz': (diff * 1000).round(2).tolist(),
                    'collision_min_max_m': [cmin.round(4).tolist(), cmax.round(4).tolist()],
                    'visual_min_max_m': [vmin.round(4).tolist(), vmax.round(4).tolist()]}
    print(f'[C1] {"성공" if c1_ok else "실패"} (축마다 차이 ≤ {TOL_C1_M * 1000:g} mm)')
    for name, v in c1.items():
        print(f'    {name:20s} {v}')
    result['C1'] = {'ok': c1_ok, 'links': c1}

    # C2: convex hull 부피 비
    ratios, not_watertight, worst = [], 0, None
    for name, prim, _ in shapes:
        if not prim.IsA(UsdGeom.Mesh):
            continue
        ratio, watertight, _ = mesh_volume_ratio(prim)
        if not watertight:
            not_watertight += 1
            continue
        ratios.append(ratio)
        if worst is None or ratio > worst[0]:
            worst = (ratio, str(prim.GetPath()))
    ratios = np.array(ratios)
    c2_ok = bool(len(ratios) and ratios.max() <= TOL_C2_RATIO and not_watertight == 0)
    print(f'[C2] {"성공" if c2_ok else "실패"} (hull/메시 부피 ≤ {TOL_C2_RATIO})')
    print(f'    닫힌 메시 {len(ratios)}개, 닫히지 않아 부피를 못 잰 메시 {not_watertight}개')
    if len(ratios):
        print(f'    부피 비 최소 {ratios.min():.6f}, 중앙값 {np.median(ratios):.6f}, 최대 {ratios.max():.6f} ({worst[1]})')
        print(f'    1.01 초과 {int((ratios > TOL_C2_RATIO).sum())}개')
    result['C2'] = {'ok': c2_ok, 'measured': int(len(ratios)), 'not_watertight': not_watertight,
                    'min': float(ratios.min()) if len(ratios) else None,
                    'max': float(ratios.max()) if len(ratios) else None, 'worst': worst}

    # C3: 관절로 직접 붙지 않은 링크 쌍의 겹침
    adjacent, collision_enabled_joints = set(), []
    for prim in Usd.PrimRange(robot):
        if prim.IsA(UsdPhysics.Joint):
            b0 = prim.GetRelationship('physics:body0').GetTargets()
            b1 = prim.GetRelationship('physics:body1').GetTargets()
            if b0 and b1:
                adjacent.add(frozenset((b0[0].name, b1[0].name)))
            enabled = prim.GetAttribute('physics:collisionEnabled')
            if enabled and enabled.HasAuthoredValue() and enabled.Get():
                collision_enabled_joints.append(prim.GetName())
    print(f'[C3] 관절로 붙은 링크 쌍 {len(adjacent)}개(PhysX가 충돌을 빼는 쌍), '
          f'관절 충돌을 켠(collisionEnabled=true) 관절 {collision_enabled_joints}')
    boxes = [(s[2].min(0), s[2].max(0)) for s in shapes]
    tested, overlaps, nearest = 0, [], []
    for i, j in itertools.combinations(range(len(shapes)), 2):
        a, b = shapes[i], shapes[j]
        if a[0] == b[0] or frozenset((a[0], b[0])) in adjacent:
            continue
        if np.any(boxes[i][1] < boxes[j][0]) or np.any(boxes[j][1] < boxes[i][0]):
            continue
        tested += 1
        r = overlap_radius(a[2], b[2])
        if r is not None and r > TOL_C3_OVERLAP_M:
            overlaps.append((a[0], b[0], str(a[1].GetPath().name), str(b[1].GetPath().name), r))
        elif r is not None:
            nearest.append((a[0], b[0], r))
    pairs = sorted({(o[0], o[1]) for o in overlaps})
    c3_ok = not overlaps
    print(f'[C3] {"성공" if c3_ok else "실패"} (범위가 겹쳐 정밀 검사한 형상 쌍 {tested}개, 겹친 형상 쌍 {len(overlaps)}개, 링크 쌍 {len(pairs)}개)')
    for o in sorted(overlaps, key=lambda x: -x[4])[:20]:
        print(f'    {o[0]} – {o[1]} ({o[2]} / {o[3]}): 내접 구 반지름 {o[4] * 1000:.3f} mm')
    for n in sorted(nearest, key=lambda x: -x[2]):
        # 음수 값은 겹치지 않았다는 뜻이다(두 형상의 면을 그만큼 밀어야 교집합이 생긴다). 거리 자체는 아니다.
        print(f'    겹치지 않음: {n[0]} – {n[1]} (선형계획 값 {n[2] * 1000:.3f} mm)')
    result['C3'] = {'ok': c3_ok, 'tested_shape_pairs': tested, 'overlap_link_pairs': pairs,
                    'overlaps': [list(o) for o in overlaps], 'not_overlapping_tested': [list(n) for n in nearest]}

    all_ok = c1_ok and c2_ok and c3_ok
    print(f'[전체] {"성공" if all_ok else "실패 있음"}')
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as f:
            json.dump(result, f, ensure_ascii=False, indent=2, default=str)
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
