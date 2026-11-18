"""증속 벨트 v3 부품을 URDF base_link 좌표(mm)에 놓는다 — drive_belt.scad의 배치 식을 그대로 옮김(OpenSCAD 없이).

scad 좌표의 브라켓·바퀴 = URDF + x 12.5(앞뒤 대칭면 MIDX 12.5), 판·전장 부품은 scad = URDF(이동 0). 기준 모듈 = 왼쪽 앞, at_corner(f, l): 뒤는 x → 2·12.5 − x, 오른쪽은 y → −y.
출력 부품은 출력용 STEP(→ parts_src/*.stl, 출력 자세)을 print_* 식의 역변환으로 기준 모듈 자리에 놓는다.
구매 부품(축 Ø8 × 56, 688ZZ, GT2 벨트)과 강철 무게추는 scad 치수로 만든다. 모터는 ref/xm430_left_front_at_axis.stl.
결과: parts_belt/<이름>_<모서리>.stl(URDF mm), parts_belt/manifest.json(이름, 모서리, 재료, 부피)
사용(PC): ~/pa-course/IsaacLab-6.1/.venv/bin/python tools/place_belt.py
"""
import json
import math
from pathlib import Path

import numpy as np
import trimesh
from shapely.geometry import Point
from shapely.ops import unary_union

H = Path(__file__).resolve().parents[1]
SRC = H / "parts_src"
REF = Path("/home/pa4/snap/slack/261/Desktop/ref")
OUT = H / "parts_belt"
OUT.mkdir(exist_ok=True)

MIDX, WX, WZ = 12.5, 92.5, 26.25
N_M, N_W, C = 36, 24, 45.85
SLED_T = 6
MZ = WZ + SLED_T
MX = WX - math.sqrt(C * C - (MZ - WZ) ** 2)
FLANGE_TOP, BLK_HX, BLK_COL_Y0 = 15.0, 12.5, 96.2
HORN_Y, BOSS_Y, SPACER_Y0, COLLAR_Y0 = 139.0, 138.5, 105.0, 97.0
T_Y = (140.5, 147.0)
SHAFT_Y = (97.5, 153.5)
B1, B2 = (107.0, 112.0), (133.5, 138.5)
GD_WALL_Y1 = 152.1
STEEL, BAL = (16, 90, 45), (113.4, -87.0, 1.5, 102.0)


def T(x=0, y=0, z=0):
    m = np.eye(4); m[:3, 3] = [x, y, z]; return m


def Rx(deg):
    a = math.radians(deg); c, s = math.cos(a), math.sin(a)
    m = np.eye(4); m[1:3, 1:3] = [[c, -s], [s, c]]; return m


def along_y(y0):          # scad: translate([0, y0, 0]) rotate([-90, 0, 0])
    return T(0, y0, 0) @ Rx(-90)


MIR_X = np.diag([-1.0, 1, 1, 1])
MIR_Y = np.diag([1.0, -1, 1, 1])
CORNERS = {"FL": np.eye(4), "RL": T(2 * MIDX) @ MIR_X, "FR": MIR_Y, "RR": MIR_Y @ T(2 * MIDX) @ MIR_X}
TO_URDF = T(-MIDX)


def load(name):
    return trimesh.load(SRC / f"{name}.stl")


def ring_y(y0, y1, ro, ri, cx, cz, n=48):
    """y 축 원통 고리(축 x = cx, z = cz)."""
    m = trimesh.creation.annulus(r_min=ri, r_max=ro, height=y1 - y0, sections=n) if ri > 0 else \
        trimesh.creation.cylinder(radius=ro, height=y1 - y0, sections=n)
    m.apply_transform(T(cx, (y0 + y1) / 2, cz) @ Rx(-90))
    return m


def belt_mesh():
    pd = lambda n: n * 2 / math.pi   # noqa: E731
    outer = unary_union([Point(WX, WZ).buffer((pd(N_W) + 1.4) / 2, 64), Point(MX, MZ).buffer((pd(N_M) + 1.4) / 2, 64)]).convex_hull
    inner = unary_union([Point(WX, WZ).buffer((pd(N_W) - 0.2) / 2, 64), Point(MX, MZ).buffer((pd(N_M) - 0.2) / 2, 64)]).convex_hull
    # 삼각분할 엔진이 없어서 직접 만든다: 두 볼록 고리를 같은 중심에서 같은 각도로 다시 뽑아 사각형으로 잇고 6 mm 돌출
    from shapely.geometry import LineString
    cx, cz = (WX + MX) / 2, (WZ + MZ) / 2
    n = 180
    def ring_pts(poly):
        pts = []
        for k in range(n):
            a = 2 * math.pi * k / n
            ray = LineString([(cx, cz), (cx + 200 * math.cos(a), cz + 200 * math.sin(a))])
            q = ray.intersection(poly.exterior)
            q = q if q.geom_type == "Point" else list(q.geoms)[0]
            pts.append((q.x, q.y))
        return np.array(pts)
    o, i = ring_pts(outer), ring_pts(inner)
    V, F = [], []
    for w in (0.0, 6.0):
        V += [(x, y, w) for x, y in o] + [(x, y, w) for x, y in i]
    def vid(layer, ring, k):
        return layer * 2 * n + ring * n + (k % n)
    for k in range(n):
        for (r0, l0, r1, l1, flip) in ((0, 0, 1, 0, True), (0, 1, 1, 1, False)):    # 아래·위 면
            a, b, c, d = vid(l0, r0, k), vid(l0, r0, k + 1), vid(l0, r1, k + 1), vid(l0, r1, k)
            F += [(a, c, b), (a, d, c)] if flip else [(a, b, c), (a, c, d)]
        for ring, out_ in ((0, True), (1, False)):                                     # 바깥·안쪽 벽
            a, b, c, d = vid(0, ring, k), vid(0, ring, k + 1), vid(1, ring, k + 1), vid(1, ring, k)
            F += [(a, b, c), (a, c, d)] if out_ else [(a, c, b), (a, d, c)]
    m = trimesh.Trimesh(np.array(V), np.array(F))
    if m.volume < 0:
        m.invert()
    # 2D (x, z_robot) → 로봇: (a, b, w) → (a, T_Y0 + 0.25 + w, b)
    P = np.array([[1, 0, 0, 0], [0, 0, 1, T_Y[0] + 0.25], [0, 1, 0, 0], [0, 0, 0, 1.0]])
    m.apply_transform(P)
    return m


base = {   # 기준 모듈(왼쪽 앞, scad 좌표)의 부품: (메시, 재료)
    "bearing_block": (load("bearing_block_4EA"), T(WX - BLK_HX, BLK_COL_Y0, FLANGE_TOP), "PLA"),
    "motor_sled": (load("motor_sled_A_2EA"), T(0, 0, FLANGE_TOP), "PLA"),
    "wheel_pulley": (load("wheel_pulley_5EA"), T(WX, 0, WZ) @ along_y(BOSS_Y), "PLA"),
    "motor_pulley": (load("motor_pulley_5EA"), T(MX, 0, MZ) @ along_y(HORN_Y), "PLA"),
    "spacer": (load("spacer_5EA"), T(WX, 0, WZ) @ along_y(SPACER_Y0), "PLA"),
    "collar": (load("collar_5EA"), T(WX, 0, WZ) @ along_y(COLLAR_Y0), "PLA"),
    "guard": (load("guard_A_2EA"), Rx(90) @ T(0, 0, -GD_WALL_Y1), "PLA"),
    "motor": (trimesh.load(REF / "xm430_left_front_at_axis.stl"), T(MX, 0, MZ), "XM430"),
    "shaft": (ring_y(*SHAFT_Y, 4.0, 0, WX, WZ), np.eye(4), "steel"),
    "bearing_in": (ring_y(*B1, 8.0, 4.0, WX, WZ), np.eye(4), "bearing"),
    "bearing_out": (ring_y(*B2, 8.0, 4.0, WX, WZ), np.eye(4), "bearing"),
    "belt": (belt_mesh(), np.eye(4), "rubber"),
}
manifest = []
for name, (mesh, Tb, mat) in base.items():
    for cn, Tc in CORNERS.items():
        m = mesh.copy()
        m.apply_transform(TO_URDF @ Tc @ Tb)          # trimesh가 거울 변환에서 면 방향을 스스로 맞춤
        if m.volume < 0:
            m.invert()
        f = OUT / f"{name}_{cn}.stl"
        m.export(f)
        manifest.append({"name": name, "corner": cn, "material": mat, "file": f.name, "volume_mm3": float(m.volume),
                         "bounds_mm": m.bounds.round(2).tolist(), "watertight": bool(m.is_watertight)})
# 무게추 컵 · 강철(한 개, 앞)
cup = load("ballast_cup_1EA")
# 무게추는 상판 아래에 매단다. 판·전장 부품은 scad 좌표 = URDF(ref/chassis_misc_nomotor.stl과 URDF 차체가 이동 0으로 98 % 일치)라
# 12.5 mm 옮기지 않는다(옮기는 것은 브라켓에 붙는 벨트 모듈뿐)
cup.apply_transform(T(0, 0, BAL[3]) @ Rx(180))
cup.export(OUT / "ballast_cup.stl")
steel = trimesh.creation.box(extents=STEEL)
steel.apply_transform(T(BAL[0] + BAL[2] + 0.2 + STEEL[0] / 2, BAL[1] + BAL[2] + 0.2 + STEEL[1] / 2, BAL[3] - STEEL[2] - 0.1 + STEEL[2] / 2))
steel.export(OUT / "ballast_steel.stl")
for nm, m, mat in (("ballast_cup", cup, "PLA"), ("ballast_steel", steel, "steel_block")):
    manifest.append({"name": nm, "corner": "-", "material": mat, "file": f"{nm}.stl", "volume_mm3": float(m.volume),
                     "bounds_mm": m.bounds.round(2).tolist(), "watertight": bool(m.is_watertight)})
(OUT / "manifest.json").write_text(json.dumps({"MX_scad": MX, "MZ": MZ, "parts": manifest}, indent=1))
for r in manifest:
    if r["corner"] in ("FL", "-"):
        print(r["name"], r["corner"], r["bounds_mm"], f"{r['volume_mm3'] / 1000:.2f} cm3", "wt" if r["watertight"] else "NOT watertight")
