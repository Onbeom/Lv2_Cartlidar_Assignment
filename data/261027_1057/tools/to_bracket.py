"""층간 긴 브라켓(왼쪽 Main + Profile Carrier) 위상최적화 — 출력 자세에 맞춘 2.5차원 SIMP(모의 FEM).

설계 판(판 두께 방향으로 같은 밀도 = 출력 방향으로 뚫는 구멍):
  web     Main 벽(y 91~96), 2D (x, z) — Main 출력 때 바닥에 눕는 면이라 관통 구멍에 서포트 없음
  bflange Main 아래 플랜지(z 12~15), 2D (x, y) — 출력 때 세운 벽 → 구멍 위쪽에 45° 지붕(후처리)
  tflange Main 위 플랜지(z 99~102), 2D (x, y) — 같음
  cwall   Carrier 뒷벽(x −154.7~−150.7), 2D (y, z) — Carrier 출력 때 바닥에 눕는 면
나머지(리브·소켓·키 접합·구멍 둘레·판 가장자리·모터 자리)는 남긴다(검토 문서 2-1 ①~⑥).
FEM: 8절점 육면체 복셀(pitch mm), PLA E 2500 MPa · ν 0.35, 단위 N·mm·MPa, GPU PCG(야코비).
모터는 인쇄하지 않는 단단한 블록(E × 20)으로 위·아래 플랜지 모터 구멍 자리를 잇고, 바퀴 힘을 블록 바깥면에 준다.
하중(한쪽, 가정): LC1 바퀴 수직 36 N × 모터 2, LC2 구동 29 N(앞뒤), LC3 기둥 굽힘 1.4 N·m(소켓 위·아래 짝힘 14 N, x·y 각각) +
수직 6.4 N, LC4 비틀림(앞 +36 N · 뒤 −36 N). 경계: 위·아래 플랜지 바깥면의 판넬 볼트 구멍 둘레 고정.
사용(PC): ~/pa-course/IsaacLab-6.1/.venv/bin/python tools/to_bracket.py --pitch 1.5 --out results/to_p15
"""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import scipy.ndimage as ndi
import torch
import trimesh

ap = argparse.ArgumentParser()
ap.add_argument("--pitch", type=float, default=1.5)
ap.add_argument("--out", type=Path, required=True)
ap.add_argument("--vol_frac_total", type=float, default=0.66, help="최종 부피 / 원래 부피(Main + Carrier), B1은 ≤ 0.70")
ap.add_argument("--iters", type=int, default=60)
ap.add_argument("--rfilter", type=float, default=3.0, help="밀도 필터 반경 mm")
ap.add_argument("--cg_tol", type=float, default=1e-5)
ap.add_argument("--eval_only", default="", help="설계 판 2D 밀도 npz를 받아 FEM만(판정용)")
ap.add_argument("--opt_lcs", default="LC1,LC3x,LC3y,LC4,LC5", help="최적화에 쓰는 하중 조건(LC2는 순응도가 작아 검증에서만 봄)")
ap.add_argument("--lc_weight", default="", help="하중 조건별 가중치, 예: LC1=2,LC4=2")
ap.add_argument("--init", default="", help="이어서 시작할 density_maps.npz")
ap.add_argument("--main_stl", default="", help="검증용: 조립 좌표 Main STL(주면 이 형상을 통째로 풀어 비교)")
ap.add_argument("--carr_stl", default="", help="검증용: 조립 좌표 Carrier STL")
ap.add_argument("--bolts_from", default="", help="검증용: 원래 형상 실행의 result.json(고정할 볼트 구멍 위치)")
ap.add_argument("--layout", choices=["motor", "belt"], default="belt",
                help="belt = 증속 벨트 v3(README): 바퀴 축 x ±80(URDF)에 베어링 블록 기둥(아래·위 플랜지와 웹에 닿음), 모터는 x ±34.54 받침")
args = ap.parse_args()
args.out.mkdir(parents=True, exist_ok=True)
dev = torch.device("cuda")
P = Path(__file__).resolve().parents[1] / "placed"
h = args.pitch
E0, NU, EMOTOR, EMIN, PEN = 2500.0, 0.35, 25000.0, 1e-3, 3.0   # 단단한 부재 = PLA × 10(수렴 속도 때문에, 사실상 강체)

# ---------------- 복셀 ----------------
main = trimesh.load(args.main_stl or P / "Floor_Bracket_Left_Main_assembled_mm.stl")
carr = trimesh.load(args.carr_stl or P / "Floor_Bracket_Left_Profile_Carrier_assembled_mm.stl")
EVAL_MESH = bool(args.main_stl)
lo = np.floor(np.minimum(main.bounds[0], carr.bounds[0]) / h) * h
hi = np.ceil(np.maximum(main.bounds[1], carr.bounds[1]) / h) * h
nx, ny, nz = np.round((hi - lo) / h).astype(int)


def voxelize(mesh):
    """(x, y) 열마다 +z 광선 하나로 교점을 구하고 짝수·홀수 규칙으로 칸 가운데가 안쪽인지 정한다(메시 contains보다 메모리가 훨씬 적음)."""
    xs = lo[0] + (np.arange(nx) + 0.5) * h + 1e-4
    ys = lo[1] + (np.arange(ny) + 0.5) * h + 1e-4
    zs = lo[2] + (np.arange(nz) + 0.5) * h
    gx, gy = np.meshgrid(xs, ys, indexing="ij")
    org = np.stack([gx.ravel(), gy.ravel(), np.full(gx.size, lo[2] - 5.0)], 1)
    loc, ridx, _ = mesh.ray.intersects_location(org, np.tile([0.0, 0.0, 1.0], (len(org), 1)), multiple_hits=True)
    occ = np.zeros((nx * ny, nz), bool)
    for r in np.unique(ridx):
        zh = np.sort(loc[ridx == r, 2])
        zh = zh[np.concatenate([[True], np.diff(zh) > 1e-6])]       # 모서리에서 두 번 잡힌 교점 하나로
        for a, b in zip(zh[0::2], zh[1::2]):
            occ[r, (zs > a) & (zs < b)] = True
    return occ.reshape(nx, ny, nz)


t0 = time.time()
in_m = voxelize(main)
in_c = voxelize(carr) & ~in_m
print(f"voxel grid {nx}×{ny}×{nz}, Main {in_m.sum()} Carrier {in_c.sum()} elems, {time.time() - t0:.0f} s", flush=True)
X, Y, Z = (lo[i] + (np.arange(n) + 0.5) * h for i, n in enumerate((nx, ny, nz)))
XX, YY, ZZ = np.meshgrid(X, Y, Z, indexing="ij")

# 모터 블록(인쇄 안 함): x c ± 14, y 108~133, z 15~99
motor = np.zeros_like(in_m)
MOTORS = (-80.0, 80.0)
for c in MOTORS:
    if args.layout == "belt":      # 베어링 블록 기둥: x c ± 12.5, |y| 96~138.7, z 15~99(웹 바깥면·두 플랜지에 닿음)
        motor |= (np.abs(XX - c) < 12.5) & (YY > 96) & (YY < 138.7) & (ZZ > 15) & (ZZ < 99)
    else:
        motor |= (np.abs(XX - c) < 14) & (YY > 108) & (YY < 133) & (ZZ > 15) & (ZZ < 99)
motor &= ~in_m & ~in_c
# 알루미늄 프로파일(20 × 20, 인쇄 안 함): 소켓 안 + 위로 50 mm(z 172까지). 위·아래 칼라를 잇는 실제 부재
ZTOP = 172.0
nz_ext = int(np.ceil((ZTOP - hi[2]) / h))
if nz_ext > 0:
    pad = ((0, 0), (0, 0), (0, nz_ext))
    in_m, in_c, motor = np.pad(in_m, pad), np.pad(in_c, pad), np.pad(motor, pad)
    nz += nz_ext
    Z = lo[2] + (np.arange(nz) + 0.5) * h
    XX, YY, ZZ = np.meshgrid(X, Y, Z, indexing="ij")
seat_z = ZZ[in_c & (XX > -150.7) & (XX < -130.3) & (YY > 119.3) & (YY < 139.7)].min() if (in_c & (XX > -150.7) & (XX < -130.3) & (YY > 119.3) & (YY < 139.7)).any() else 12.0
profile = (XX > -150.5) & (XX < -130.5) & (YY > 119.5) & (YY < 139.5) & (ZZ > seat_z + h) & (ZZ < ZTOP)
profile &= ~in_m & ~in_c
print(f"profile block {profile.sum()} elems, seat z {seat_z:.2f}", flush=True)

# ---------------- 설계 판과 남길 곳 ----------------
web = in_m & (YY > 91) & (YY < 96) & (ZZ > 15) & (ZZ < 99)
bfl = in_m & (ZZ > 12) & (ZZ < 15) & (YY > 96)
tfl = in_m & (ZZ > 99) & (ZZ < 102) & (YY > 96)
cwl = in_c & (XX > -154.7) & (XX < -150.7)
PLATES = {"web": (web, 1, (0, 2)), "bflange": (bfl, 2, (0, 1)), "tflange": (tfl, 2, (0, 1)), "cwall": (cwl, 0, (1, 2))}


def mm(r):
    return max(1, int(round(r / h)))


def disk(r):
    k = mm(r)
    a = np.arange(-k, k + 1)
    return (a[:, None] ** 2 + a[None, :] ** 2) <= k * k


plate_info = {}
for name, (mask, ax, keep_axes) in PLATES.items():
    solid2d = mask.any(axis=ax)
    filled = ndi.binary_fill_holes(solid2d)
    holes = filled & ~solid2d
    lab, n = ndi.label(holes)
    keep = np.zeros_like(solid2d)
    for k in range(1, n + 1):
        comp = lab == k
        big = comp.sum() * h * h > 100.0        # 케이블 통과구(20×20)는 테두리 2 mm, 볼트·모터·키 구멍은 4 mm
        keep |= ndi.binary_dilation(comp, disk(1.5 if big else 3.0))
    keep |= solid2d & ~ndi.binary_erosion(solid2d, disk(2.0))        # 판 가장자리 띠
    A, B = np.meshgrid([X, Y, Z][keep_axes[0]], [X, Y, Z][keep_axes[1]], indexing="ij")
    if name == "web":
        keep |= (B < 17.5) | (B > 97.0) | (A < -100.0)                  # 플랜지 이음, Carrier 키 접합 자리
        if args.layout == "belt":   # README: 베어링 블록이 기대는 웹 x 77.5~107.5 · −82.5~−52.5(README 좌표) z 40~99 → URDF −12.5
            keep |= ((np.abs(A - 80.0) < 15.0) | (np.abs(A + 80.0) < 15.0)) & (B > 40.0)
            keep |= (np.abs(A - 80.0) < 12.5) | (np.abs(A + 80.0) < 12.5)          # 블록이 닿는 면
    if name in ("bflange", "tflange"):
        keep |= B < 98.5                                               # 벽 이음(필렛)
        if args.layout == "belt":
            for c in (-80.0, 0.0, 80.0):                               # 베어링 블록 닿는 면 · 중간 장착점
                keep |= (np.abs(A - c) < 12.5) & (B > 96.0)
            if name == "bflange":                                      # 모터 받침(README x 16.5~70 → URDF 4~57.5, 뒤는 거울)
                keep |= ((A > 4.0) & (A < 57.5) | (A > -57.5) & (A < -4.0)) & (B > 99.0)
        else:
            for c in (-80.0, 0.0, 80.0):                               # 모터 · 중간 장착점 닿는 면
                keep |= (np.abs(A - c) < 12.0) & (B > 110.0)
        keep |= A < -100.0
    if name == "cwall":
        keep |= (B < 30.0) | (B > 100.0)                               # 소켓 위·아래 칼라
    design = solid2d & ~keep
    plate_info[name] = {"solid2d": solid2d, "design": design, "axis": ax, "keep_axes": keep_axes, "holes": holes}
    print(f"plate {name}: solid {solid2d.sum()} cells, design {design.sum()} cells", flush=True)

# 요소 목록
solid = in_m | in_c | motor | profile
eid = -np.ones(solid.shape, np.int64)
eid[solid] = np.arange(solid.sum())
ne = int(solid.sum())
e_ijk = np.argwhere(solid)
is_motor = (motor | profile)[solid]      # 인쇄 안 하는 단단한 부재(모터·프로파일)
# 요소 → 설계 변수(판, 2D 칸) 대응
var_offsets, var_count = {}, 0
elem_var = -np.ones(ne, np.int64)
for name, inf in plate_info.items():
    d2 = inf["design"]
    vid = -np.ones(d2.shape, np.int64)
    vid[d2] = np.arange(d2.sum()) + var_count
    var_offsets[name] = (var_count, vid)
    var_count += int(d2.sum())
    mask3 = PLATES[name][0]
    ax = inf["axis"]
    sel = mask3[solid]
    ijk = e_ijk[sel]
    idx2 = np.delete(ijk, ax, axis=1)
    v = vid[idx2[:, 0], idx2[:, 1]]
    elem_var[np.where(sel)[0]] = v
print(f"elements {ne} (motor {is_motor.sum()}), design vars {var_count}, design elems {(elem_var >= 0).sum()}", flush=True)

# 노드
nodes_shape = (nx + 1, ny + 1, nz + 1)
off = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0], [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]])
corner = (e_ijk[:, None, :] + off[None]).reshape(-1, 3)
nid_full = np.ravel_multi_index(corner.T, nodes_shape)
uniq, inv = np.unique(nid_full, return_inverse=True)
enodes = inv.reshape(ne, 8)
nn = len(uniq)
node_xyz = lo + np.stack(np.unravel_index(uniq, nodes_shape), -1) * h
edof = (enodes[:, :, None] * 3 + np.arange(3)).reshape(ne, 24)
ndof = nn * 3
print(f"nodes {nn}, dofs {ndof}", flush=True)


# ---------------- 요소 강성(육면체, 2×2×2 가우스) ----------------
def hex_ke(E, nu, a):
    C = E / ((1 + nu) * (1 - 2 * nu)) * np.array([
        [1 - nu, nu, nu, 0, 0, 0], [nu, 1 - nu, nu, 0, 0, 0], [nu, nu, 1 - nu, 0, 0, 0],
        [0, 0, 0, (1 - 2 * nu) / 2, 0, 0], [0, 0, 0, 0, (1 - 2 * nu) / 2, 0], [0, 0, 0, 0, 0, (1 - 2 * nu) / 2]])
    xi = off * 2 - 1
    K = np.zeros((24, 24))
    g = 1 / math.sqrt(3)
    for s in (-g, g):
        for t in (-g, g):
            for u in (-g, g):
                dN = np.zeros((8, 3))
                for i in range(8):
                    dN[i, 0] = xi[i, 0] * (1 + xi[i, 1] * t) * (1 + xi[i, 2] * u) / 8
                    dN[i, 1] = xi[i, 1] * (1 + xi[i, 0] * s) * (1 + xi[i, 2] * u) / 8
                    dN[i, 2] = xi[i, 2] * (1 + xi[i, 0] * s) * (1 + xi[i, 1] * t) / 8
                dN *= 2 / a                     # 자연 좌표 → 실제(한 변 a)
                Bm = np.zeros((6, 24))
                for i in range(8):
                    Bm[0, 3 * i] = dN[i, 0]; Bm[1, 3 * i + 1] = dN[i, 1]; Bm[2, 3 * i + 2] = dN[i, 2]
                    Bm[3, 3 * i] = dN[i, 1]; Bm[3, 3 * i + 1] = dN[i, 0]
                    Bm[4, 3 * i + 1] = dN[i, 2]; Bm[4, 3 * i + 2] = dN[i, 1]
                    Bm[5, 3 * i] = dN[i, 2]; Bm[5, 3 * i + 2] = dN[i, 0]
                K += Bm.T @ C @ Bm * (a / 2) ** 3
    return K, C


KE_np, C_np = hex_ke(1.0, NU, h)
KE = torch.tensor(KE_np, device=dev, dtype=torch.float64)
EDOF = torch.tensor(edof, device=dev)

# ---------------- 경계 · 하중 ----------------
fixed = np.zeros(ndof, bool)
bolt_c = []
if args.bolts_from:
    for name, cxy in json.loads(Path(args.bolts_from).read_text())["bolt_holes_fixed"]:
        zface = 12.0 if name == "bflange" else 102.0
        bolt_c.append((name, cxy))
        sel = (np.abs(node_xyz[:, 2] - zface) < 1e-6) & (np.hypot(node_xyz[:, 0] - cxy[0], node_xyz[:, 1] - cxy[1]) < 5.0)
        for d in range(3):
            fixed[np.where(sel)[0] * 3 + d] = True
for name, zface in ((("bflange", 12.0), ("tflange", 102.0)) if not args.bolts_from else ()):
    inf = plate_info[name]
    lab, n = ndi.label(inf["holes"])
    for k in range(1, n + 1):
        comp = np.argwhere(lab == k)
        if len(comp) * h * h > 100.0:
            continue
        cxy = (np.array([X[comp[:, 0]].mean(), Y[comp[:, 1]].mean()]))
        if any(abs(cxy[0] - c) < 16 and cxy[1] > 104 for c in (-80.0, 0.0, 80.0)):
            continue                                            # 모터 구멍은 고정하지 않음
        bolt_c.append((name, cxy.tolist()))
        sel = (np.abs(node_xyz[:, 2] - zface) < 1e-6) & (np.hypot(node_xyz[:, 0] - cxy[0], node_xyz[:, 1] - cxy[1]) < 5.0)
        for d in range(3):
            fixed[np.where(sel)[0] * 3 + d] = True
print(f"bolt holes fixed: {len(bolt_c)}, fixed dofs {fixed.sum()}", flush=True)


def nodes_where(cond):
    return np.where(cond)[0]


loads = {}
f = np.zeros((ndof,))
wheel = {}
for c in MOTORS:
    yo = 138.0 if args.layout == "belt" else 133.0                   # 블록 바깥면(축이 지나는 곳)
    sel = nodes_where((np.abs(node_xyz[:, 1] - yo) < h * 0.67) & (np.abs(node_xyz[:, 0] - c) < 12) &
                      (node_xyz[:, 2] > 20) & (node_xyz[:, 2] < 33))
    wheel[c] = sel
    selb = nodes_where((np.abs(node_xyz[:, 1] - yo) < h * 0.67) & (np.abs(node_xyz[:, 0] - c) < 12) &
                       (node_xyz[:, 2] > 15) & (node_xyz[:, 2] < 21))
    wheel[(c, "b")] = selb


def add(fv, sel, d, F):
    fv[sel * 3 + d] += F / len(sel)


f1 = np.zeros(ndof); [add(f1, wheel[c], 2, 36.0) for c in MOTORS]
f2 = np.zeros(ndof); [add(f2, wheel[(c, "b")], 0, 29.0) for c in MOTORS]
# LC3: 기둥 위 저장소 0.5 g → 소켓 굽힘 1.44 N·m(3.2 N × 0.45 m). 프로파일 블록 위 끝(z 172)에 옆 힘 F = M / (172 − 소켓 가운데 높이) + 수직 6.4 N
ptop = (node_xyz[:, 0] > -150.5) & (node_xyz[:, 0] < -130.5) & (node_xyz[:, 1] > 119.5) & (node_xyz[:, 1] < 139.5) & (np.abs(node_xyz[:, 2] - node_xyz[:, 2].max()) < 1e-6)
pt = nodes_where(ptop)
Flat = 1440.0 / (node_xyz[:, 2].max() - 67.0)
f3x = np.zeros(ndof); add(f3x, pt, 0, Flat); add(f3x, pt, 2, -6.4)
f3y = np.zeros(ndof); add(f3y, pt, 1, Flat); add(f3y, pt, 2, -6.4)
print(f"LC3 lateral at profile top {Flat:.2f} N on {len(pt)} nodes", flush=True)
f4 = np.zeros(ndof); add(f4, wheel[80.0], 2, 36.0); add(f4, wheel[-80.0], 2, -36.0)
LCS = {"LC1": f1, "LC2": f2, "LC3x": f3x, "LC3y": f3y, "LC4": f4}
if args.layout == "belt":   # LC5 벨트 장력 40 N(README): 블록은 모터 쪽으로, 받침 자리(아래 플랜지 윗면)는 바퀴 쪽으로
    f5 = np.zeros(ndof)
    for c, m in ((80.0, 34.54), (-80.0, -34.54)):
        sgn = np.sign(m - c)
        add(f5, wheel[c], 0, 40.0 * sgn)
        sled = nodes_where((np.abs(node_xyz[:, 2] - 15.0) < 1e-6) & (np.abs(node_xyz[:, 0] - m) < 12) & (node_xyz[:, 1] > 105) & (node_xyz[:, 1] < 133))
        add(f5, sled, 0, -40.0 * sgn)
    LCS["LC5"] = f5
for k, v in LCS.items():
    print(f"{k}: |F| sum {np.abs(v).sum():.1f} N, nodes {np.count_nonzero(v)}", flush=True)
FREE = torch.tensor(~fixed, device=dev)
FS = {k: torch.tensor(v, device=dev, dtype=torch.float64) * FREE for k, v in LCS.items()}


def matvec(u, Ee):
    ue = u[EDOF]                                # ne × 24
    fe = (ue @ KE) * Ee[:, None]
    out = torch.zeros_like(u)
    out.index_add_(0, EDOF.reshape(-1), fe.reshape(-1))
    return out * FREE


DIAG_KE = torch.diagonal(KE)


def solve(F, Ee, u0=None, tol=None, maxit=20000):
    tol = tol or args.cg_tol
    d = torch.zeros(ndof, device=dev, dtype=torch.float64)
    d.index_add_(0, EDOF.reshape(-1), (DIAG_KE[None, :] * Ee[:, None]).reshape(-1))
    Minv = torch.where(FREE, 1.0 / d.clamp(min=1e-30), torch.zeros_like(d))
    u = torch.zeros_like(F) if u0 is None else u0.clone()
    r = F - matvec(u, Ee)
    z = Minv * r
    p = z.clone()
    rz = (r * z).sum()
    bn = F.norm()
    for it in range(maxit):
        Ap = matvec(p, Ee)
        alpha = rz / (p * Ap).sum()
        u += alpha * p
        r -= alpha * Ap
        if r.norm() < tol * bn:
            return u, it + 1
        z = Minv * r
        rz_new = (r * z).sum()
        p = z + (rz_new / rz) * p
        rz = rz_new
    return u, maxit


# ---------------- 2D 필터(판마다 원뿔) ----------------
def make_filter(name):
    _, vid = var_offsets[name]
    pts = np.argwhere(vid >= 0)
    ids = vid[vid >= 0]
    R = args.rfilter / h
    from scipy.spatial import cKDTree
    tr = cKDTree(pts)
    rows, cols, vals = [], [], []
    for i, nb in enumerate(tr.query_ball_point(pts, R)):
        w = R - np.linalg.norm(pts[nb] - pts[i], axis=1)
        rows += [ids[i]] * len(nb); cols += list(ids[nb]); vals += list(w)
    return rows, cols, vals


rows, cols, vals = [], [], []
for name in plate_info:
    r_, c_, v_ = make_filter(name)
    rows += r_; cols += c_; vals += v_
Hf = torch.sparse_coo_tensor(torch.tensor([rows, cols]), torch.tensor(vals, dtype=torch.float64), (var_count, var_count)).coalesce().to(dev)
Hs = torch.sparse.sum(Hf, 1).to_dense()

EV = torch.tensor(elem_var, device=dev)
DES = EV >= 0
MOT = torch.tensor(is_motor, device=dev)
vol_elem = h ** 3
V_orig = float((~is_motor).sum()) * vol_elem
V_fixed = float(((~is_motor) & (elem_var < 0)).sum()) * vol_elem
n_per_var = torch.zeros(var_count, device=dev, dtype=torch.float64).index_add_(0, EV[DES], torch.ones(int(DES.sum()), device=dev, dtype=torch.float64))
V_des_full = float(n_per_var.sum()) * vol_elem
V_target = args.vol_frac_total * V_orig
vf_des = (V_target - V_fixed) / V_des_full
print(f"V_orig {V_orig/1000:.1f} cm3 (STL 208.4), fixed {V_fixed/1000:.1f}, designable {V_des_full/1000:.1f}, "
      f"target {V_target/1000:.1f} → design fraction {vf_des:.3f}", flush=True)


def elem_E(xphys):
    rho = torch.ones(ne, device=dev, dtype=torch.float64)
    rho[DES] = xphys[EV[DES]]
    Ee = EMIN + rho ** PEN * (1 - EMIN)
    Ee = Ee * E0
    Ee[MOT] = EMOTOR
    return Ee, rho


def evaluate(xphys, us=None, tol=None):
    Ee, rho = elem_E(xphys)
    out, us_new = {}, {}
    for k, F in FS.items():
        u, it = solve(F, Ee, None if us is None else us.get(k), tol=tol)
        us_new[k] = u
        out[k] = {"compliance": float((F * u).sum()), "cg_iters": it}
    return out, us_new, Ee, rho


PRINTED_NODE = torch.zeros(nn, dtype=torch.bool, device=dev)
PRINTED_NODE[torch.tensor(np.unique(enodes[~is_motor]), device=dev)] = True


def disp_stats(u):
    U = u.reshape(-1, 3).norm(dim=1)
    return float(U[PRINTED_NODE].max())


def vm_stress(u, rho):
    """요소 가운데 응력(밀도로 줄인 강성, 설계 요소는 ρ³)."""
    Bc = np.zeros((6, 24))
    dN = off * 2 - 1
    dN = dN / 8 * 2 / h
    for i in range(8):
        Bc[0, 3 * i] = dN[i, 0]; Bc[1, 3 * i + 1] = dN[i, 1]; Bc[2, 3 * i + 2] = dN[i, 2]
        Bc[3, 3 * i] = dN[i, 1]; Bc[3, 3 * i + 1] = dN[i, 0]
        Bc[4, 3 * i + 1] = dN[i, 2]; Bc[4, 3 * i + 2] = dN[i, 1]
        Bc[5, 3 * i] = dN[i, 2]; Bc[5, 3 * i + 2] = dN[i, 0]
    S = torch.tensor(C_np * E0 @ Bc, device=dev, dtype=torch.float64)
    s = (u[EDOF] @ S.T) * (rho ** PEN)[:, None]
    vm = torch.sqrt(0.5 * ((s[:, 0] - s[:, 1]) ** 2 + (s[:, 1] - s[:, 2]) ** 2 + (s[:, 2] - s[:, 0]) ** 2)
                    + 3 * (s[:, 3] ** 2 + s[:, 4] ** 2 + s[:, 5] ** 2))
    vm[MOT] = 0
    return vm


def report(xphys, tag, us=None):
    res, us, Ee, rho = evaluate(xphys, us, tol=1e-7)
    printed = (~MOT) & (rho > 0.5)
    for k in res:
        res[k]["max_disp_mm"] = disp_stats(us[k])
        vm = vm_stress(us[k], rho)[printed]
        res[k]["vm_max_MPa"] = float(vm.max())
        res[k]["vm_p999_MPa"] = float(torch.quantile(vm.float(), 0.999))
    vol = float(printed.sum()) * vol_elem
    print(f"[{tag}] vol {vol/1000:.1f} cm3 ({vol/V_orig*100:.1f} %) " +
          " ".join(f"{k}: C {v['compliance']:.3f} d {v['max_disp_mm']:.4f} vm {v['vm_max_MPa']:.2f}/{v['vm_p999_MPa']:.2f}" for k, v in res.items()), flush=True)
    return {"volume_cm3": vol / 1000, "volume_frac": vol / V_orig, "lc": res}, us


def to_maps(xphys):
    maps = {}
    x = xphys.cpu().numpy()
    for name, inf in plate_info.items():
        _, vid = var_offsets[name]
        m = inf["solid2d"].astype(float)
        m[vid >= 0] = x[vid[vid >= 0]]
        maps[name] = m
    return maps


meta = {"pitch": h, "grid": [int(nx), int(ny), int(nz)], "lo": lo.tolist(), "elements": ne, "design_vars": var_count,
        "V_orig_cm3": V_orig / 1000, "V_fixed_cm3": V_fixed / 1000, "V_designable_cm3": V_des_full / 1000,
        "bolt_holes_fixed": bolt_c, "args": {k: str(v) for k, v in vars(args).items()}}
np.savez_compressed(args.out / "geometry.npz", X=X, Y=Y, Z=Z, **{f"{n}_solid2d": inf["solid2d"] for n, inf in plate_info.items()},
                    **{f"{n}_design": inf["design"] for n, inf in plate_info.items()},
                    **{f"{n}_holes": inf["holes"] for n, inf in plate_info.items()})

if EVAL_MESH:
    full, _ = report(torch.ones(var_count, device=dev, dtype=torch.float64), "mesh")
    meta.update({"evaluated_mesh": full, "main_stl": args.main_stl, "carr_stl": args.carr_stl})
    (args.out / "eval_mesh.json").write_text(json.dumps(meta, indent=2, ensure_ascii=False))
    raise SystemExit
if args.eval_only:
    d = np.load(args.eval_only)
    x = np.zeros(var_count)
    for name in plate_info:
        _, vid = var_offsets[name]
        x[vid[vid >= 0]] = d[name][vid >= 0]
    rep_final, _ = report(torch.tensor(x, device=dev), "eval")
    full, _ = report(torch.ones(var_count, device=dev, dtype=torch.float64), "original")
    meta.update({"original": full, "evaluated": rep_final, "eval_file": args.eval_only})
    (args.out / "eval.json").write_text(json.dumps(meta, indent=2, ensure_ascii=False))
    raise SystemExit

orig, us = report(torch.ones(var_count, device=dev, dtype=torch.float64), "original")
C0 = {k: v["compliance"] for k, v in orig["lc"].items()}
x = torch.full((var_count,), max(min(vf_des, 1.0), 0.05), device=dev, dtype=torch.float64)
if args.init:
    d0 = np.load(args.init)
    x0 = np.zeros(var_count)
    for name in plate_info:
        _, vid = var_offsets[name]
        x0[vid[vid >= 0]] = d0[name][vid >= 0]
    x = torch.tensor(np.clip(x0, 0.001, 1.0), device=dev)
    print(f"init from {args.init}", flush=True)
W = {k: 1.0 for k in FS}
for kv in filter(None, args.lc_weight.split(",")):
    k, v = kv.split("=")
    W[k] = float(v)
hist = []
for it in range(args.iters):
    xphys = torch.sparse.mm(Hf, x[:, None])[:, 0] / Hs
    Ee, rho = elem_E(xphys)
    dc_e = torch.zeros(ne, device=dev, dtype=torch.float64)
    ctot = 0.0
    its_all = {}
    for k in args.opt_lcs.split(","):
        F = FS[k]
        u, its = solve(F, Ee, us.get(k))
        its_all[k] = its
        us[k] = u
        c = float((F * u).sum())
        ctot += W[k] * c / C0[k]
        ue = u[EDOF]
        ce = ((ue @ KE) * ue).sum(1)
        dc_e += -W[k] * PEN * rho ** (PEN - 1) * (1 - EMIN) * E0 * ce / C0[k]
    dc = torch.zeros(var_count, device=dev, dtype=torch.float64).index_add_(0, EV[DES], dc_e[DES])
    dv = n_per_var.clone()
    dc = torch.sparse.mm(Hf, (dc / Hs)[:, None])[:, 0]
    dv = torch.sparse.mm(Hf, (dv / Hs)[:, None])[:, 0]
    l1, l2, move = 1e-9, 1e9, 0.15
    while (l2 - l1) / (l1 + l2) > 1e-4:
        lm = 0.5 * (l1 + l2)
        xn = (x * torch.sqrt((-dc).clamp(min=0) / dv / lm)).clamp(x - move, x + move).clamp(0.001, 1.0)
        xp = torch.sparse.mm(Hf, xn[:, None])[:, 0] / Hs
        if float((xp * n_per_var).sum()) * vol_elem > V_target - V_fixed:
            l1 = lm
        else:
            l2 = lm
    change = float((xn - x).abs().max())
    x = xn
    hist.append({"it": it, "obj": ctot, "change": change})
    print(f"it {it:3d} obj {ctot:.4f} change {change:.3f} vol_des {float((xp * n_per_var).sum()) / float(n_per_var.sum()):.3f} "
          f"cg {its_all} t {time.time() - t0:.0f}s", flush=True)
    np.savez_compressed(args.out / "density_maps.npz", **to_maps(torch.sparse.mm(Hf, x[:, None])[:, 0] / Hs))
    if it > 15 and change < 0.01:
        break
xphys = torch.sparse.mm(Hf, x[:, None])[:, 0] / Hs
maps = to_maps(xphys)
np.savez_compressed(args.out / "density_maps.npz", **maps)
xbin = (xphys > 0.5).double()
final_bin, _ = report(xbin, "thresholded 0.5", us)
meta.update({"original": orig, "thresholded": final_bin, "history": hist})
(args.out / "result.json").write_text(json.dumps(meta, indent=2, ensure_ascii=False))
print("done", flush=True)
