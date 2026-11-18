"""위상최적화 밀도 지도 → 출력 가능한 구멍 다각형(조립 좌표 mm, 왼쪽 브라켓).

판마다(web·bflange·tflange·cwall):
 1. 설계 칸 밀도를 가우스로 살짝 고르게 한 뒤 0.5 미만을 구멍 후보로(남길 곳은 늘 살).
 2. 0.2 mm 고운 격자로 올려 모폴로지: 살 열기(지름 2.4 mm 원) → 2.4 mm보다 얇은 살은 구멍으로, 구멍 열기·닫기(R 2 mm)로 모서리 둥글게.
    면적 20 mm² 미만 구멍은 없앰. 구멍은 설계 칸 안쪽 0.4 mm까지만.
 3. 세워 출력하는 플랜지(출력 위 = 조립 +y): 위에서 아래로 훑으며 아래 세 칸(왼쪽 아래·바로 아래·오른쪽 아래)이 모두 구멍인
    살 칸이 있으면 바로 아래 칸을 살로 채움 → 구멍 천장이 45°보다 가파른 지붕(서포트 없음).
 4. OpenCV 윤곽 → 다각형(0.1 mm로 단순화) → cuts.json(판, 돌출 축, 범위, 다각형).
사용(PC): ~/pa-course/IsaacLab-6.1/.venv/bin/python tools/post_bracket.py results/to_p15
"""
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import scipy.ndimage as ndi

R = Path(sys.argv[1])
g = np.load(R / "geometry.npz")
dm = np.load(R / "density_maps.npz")
meta = json.loads((R / "result.json").read_text())
h = float(meta["pitch"])
FINE = 0.2
X, Y, Z = g["X"], g["Y"], g["Z"]
# 판: (2D 축 a, b), 돌출 축, 돌출 범위(판 두께보다 0.5 mm씩 넓게), 세워 출력(지붕 필요) 여부
PLATES = {"web": ((X, Z), 1, (90.5, 96.5), False), "bflange": ((X, Y), 2, (11.5, 15.5), True),
          "tflange": ((X, Y), 2, (98.5, 102.5), True), "cwall": ((Y, Z), 0, (-155.2, -150.2), False)}


def disk(r_mm, step):
    k = int(round(r_mm / step))
    a = np.arange(-k, k + 1)
    return (a[:, None] ** 2 + a[None, :] ** 2) <= k * k


# 판 가운데 단면(실제 형상) → 깎아도 되는 영역: 판 바깥 윤곽 안쪽 2 mm, 원래 구멍 둘레 3 mm(큰 구멍 1.5 mm) 밖
import trimesh  # noqa: E402
from shapely.geometry import LineString, Polygon  # noqa: E402
from shapely.ops import polygonize, unary_union  # noqa: E402

PL = Path(__file__).resolve().parents[1] / "placed"
MESH = {"main": trimesh.load(PL / "Floor_Bracket_Left_Main_assembled_mm.stl"),
        "carr": trimesh.load(PL / "Floor_Bracket_Left_Profile_Carrier_assembled_mm.stl")}
MID = {"web": ("main", 1, 93.5), "bflange": ("main", 2, 13.5), "tflange": ("main", 2, 100.5), "cwall": ("carr", 0, -152.7)}


def allowed_region(name):
    mesh_key, ax, c = MID[name]
    n = [0, 0, 0]; n[ax] = 1
    o = [0, 0, 0]; o[ax] = c
    sec = MESH[mesh_key].section(plane_origin=o, plane_normal=n)
    keep_ax = [i for i in range(3) if i != ax]
    lines = [LineString(seg[:, keep_ax]) for seg in sec.discrete]
    faces = list(polygonize(unary_union(lines)))
    region = faces[0]
    for f in faces[1:]:
        region = region.symmetric_difference(f)
    outer = unary_union([Polygon(p.exterior) for p in getattr(region, "geoms", [region])])
    holes = unary_union([outer.difference(region)])
    hole_keep = unary_union([g.buffer(3.0) for g in getattr(holes, "geoms", [holes]) if 0.5 < g.area <= 100])   # 큰 케이블 구멍 둘레는 보호 안 함
    allow = outer.buffer(-2.0).difference(hole_keep)
    from shapely.geometry import box
    # 구멍 돌출이 판 밖의 다른 살(벽 안쪽 모서리 보강띠 z 15~18 · 96~99, Carrier 소켓 칼라 z ≤ 35 · ≥ 94)에 닿으면
    # 출력 자세에서 평평한 천장(아랫면)이 생기므로 그 높이는 깎지 않음
    # 판 바로 바깥(공기 쪽) 0.6 mm 단면에 다른 살이 있으면 그 자리는 깎지 않음(그 살이 출력 때 공중에 뜸)
    side = {"web": 96.6, "bflange": 15.6, "tflange": 98.4, "cwall": -150.1}[name]
    o2 = [0, 0, 0]; o2[ax] = side
    sec2 = MESH[mesh_key].section(plane_origin=o2, plane_normal=n)
    if sec2 is not None:
        l2 = [LineString(seg[:, keep_ax]) for seg in sec2.discrete]
        f2 = list(polygonize(unary_union(l2)))
        if f2:
            att = f2[0]
            for f_ in f2[1:]:
                att = att.symmetric_difference(f_)
            allow = allow.difference(att.buffer(1.0))
    if name == "web":
        allow = allow.intersection(box(-1e3, 19.0, 1e3, 95.5))
    if name == "cwall":
        allow = allow.intersection(box(-1e3, 36.5, 1e3, 92.5))
    return allow


out = {"pitch": h, "fine": FINE, "plates": {}}
diag = {}
for name, ((A, B), axis, rng, roof) in PLATES.items():
    m = dm[name]
    design = g[f"{name}_design"]
    sm = ndi.gaussian_filter(m, 0.7)
    sm[~design] = 1.0
    hole = design & (sm < 0.5)
    # 고운 격자(가운데 맞춤)
    f = int(round(h / FINE))
    FS = h / f                         # 실제 고운 격자 간격(1.5 / 8 = 0.1875 mm). 좌표 변환은 반드시 이 값으로
    hole_f = np.kron(hole, np.ones((f, f), bool))
    des_f = np.kron(design, np.ones((f, f), bool))
    des_f = ndi.binary_erosion(des_f, disk(0.4, FS))
    a0, b0 = A[0] - h / 2, B[0] - h / 2
    hole_f = ndi.gaussian_filter(hole_f.astype(float), 1.5 / FS) > 0.5        # 계단 없애기(반경 약 1.5 mm)
    hole_f &= des_f
    solid_f = ~hole_f
    solid_f = ndi.binary_opening(solid_f, disk(1.2, FS)) | ~des_f              # 2.4 mm보다 얇은 살 → 구멍
    hole_f = ~solid_f & des_f
    hole_f = ndi.binary_opening(hole_f, disk(2.0, FS))                         # 구멍 모서리 둥글게, 좁은 틈 없앰
    hole_f = ndi.binary_closing(hole_f, disk(2.0, FS)) & des_f
    # 큰 기존 구멍(케이블 구멍) 둘레 테두리: 주변이 거의 다 구멍이면 테두리도 비움(떠 있는 고리 방지)
    holes2d = g[f"{name}_holes"]
    lab2, n2 = ndi.label(holes2d)
    for k2 in range(1, n2 + 1):
        comp = lab2 == k2
        if comp.sum() * h * h <= 100.0:
            continue
        ring = ndi.binary_dilation(comp, iterations=int(round(3.5 / h))) & ~comp
        comp_f = np.kron(comp, np.ones((f, f), bool))
        ring_f = np.kron(ring, np.ones((f, f), bool))
        outer = ndi.binary_dilation(ring_f, disk(2.5, FS)) & ~ring_f & ~comp_f
        if outer.sum() and hole_f[outer].mean() > 0.6:
            hole_f |= ring_f | comp_f
    lab, n = ndi.label(hole_f)
    sizes = ndi.sum(hole_f, lab, range(1, n + 1)) * FS * FS
    for k, s in enumerate(sizes, 1):
        if s < 20.0:
            hole_f[lab == k] = False
    if roof:
        solid_f = ~hole_f
        nb = solid_f.shape[1]
        for j in range(nb - 1, 0, -1):                       # 위(+b)에서 아래로
            below = solid_f[:, j - 1]
            sup = below | np.roll(below, 1) | np.roll(below, -1)
            uns = solid_f[:, j] & ~sup
            if uns.any():
                solid_f[uns, j - 1] = True
        hole_f = ~solid_f & des_f
    # 최소 살 두께 확인(구멍 사이·구멍과 남길 곳 사이)
    dist = ndi.distance_transform_edt(~hole_f) * FS
    thin = 0.0
    # 윤곽 → 다각형
    img = (hole_f.T[::-1].astype(np.uint8)) * 255          # 행 = b(위가 큰 값), 열 = a
    cnts, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    polys = []
    ALLOW = allowed_region(name)
    H_img = img.shape[0]
    for c in cnts:
        c = cv2.approxPolyDP(c, 0.1 / FS, True)[:, 0, :]
        if len(c) < 3:
            continue
        a = a0 + (c[:, 0] + 0.5) * FS
        b = b0 + (H_img - 1 - c[:, 1] + 0.5) * FS
        from shapely.geometry import Polygon
        pg = Polygon(np.stack([a, b], 1)).buffer(0).intersection(ALLOW)   # 자기 교차 없앰 + 실제 구멍·가장자리 비킴
        for q in getattr(pg, "geoms", [pg]):
            if q.geom_type != "Polygon" or q.area < 20.0:
                continue
            q = q.buffer(-0.6).buffer(0.6).simplify(0.05)                    # 0.6 mm보다 얇은 뾰족한 끝 없앰
            if q.is_empty or q.geom_type != "Polygon" or q.area < 20.0:
                continue
            polys.append(np.round(np.asarray(q.exterior.coords)[:-1], 3).tolist())
    # 구멍 사이 살이 2.4 mm보다 얇으면 하나로 이음(다각형 단계에서 다시 확인), 다시 허용 영역으로 자름
    if polys:
        U = unary_union([Polygon(p) for p in polys]).buffer(1.2).buffer(-1.2).intersection(ALLOW)
        polys = []
        for q in getattr(U, "geoms", [U]):
            q = q.buffer(-0.6).buffer(0.6).simplify(0.05)
            for r in getattr(q, "geoms", [q]):
                if r.geom_type == "Polygon" and r.area >= 20.0:
                    polys.append(np.round(np.asarray(r.exterior.coords)[:-1], 3).tolist())
    if roof:   # 세워 출력하는 플랜지: 400 mm² 미만 구멍은 줄이는 부피(1 cm³ 안팎)보다 천장 처짐 위험이 커서 뺌
        polys = [p for p in polys if Polygon(p).area >= 400.0]
    area = float(hole_f.sum()) * FS * FS
    plate_thk = rng[1] - rng[0] - 1.0
    out["plates"][name] = {"axis": axis, "range": rng, "roof_up_plus_b": roof, "polygons": polys,
                           "hole_area_mm2": area, "removed_volume_cm3_est": area * plate_thk / 1000}
    diag[name] = (hole_f, des_f, (a0, b0), FS)
    print(f"{name}: holes {len(polys)}, area {area:.0f} mm², removed ≈ {area * plate_thk / 1000:.1f} cm³", flush=True)
(R / "cuts.json").write_text(json.dumps(out, indent=1))

# 확인 그림
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

plt.rcParams["font.family"] = ["Noto Sans CJK JP", "DejaVu Sans"]
fig, axs = plt.subplots(2, 4, figsize=(18, 7), gridspec_kw={"height_ratios": [1, 1]})
for k, (name, (hole_f, des_f, (a0, b0), FS)) in enumerate(diag.items()):
    m = dm[name]
    (A, B), *_ = PLATES[name]
    ext = [A[0] - h / 2, A[-1] + h / 2, B[0] - h / 2, B[-1] + h / 2]
    axs[0, k].imshow(m.T, origin="lower", extent=ext, cmap="Greys", vmin=0, vmax=1)
    axs[0, k].set_title(f"{name} 밀도", fontsize=9)
    show = np.where(hole_f, 0.0, np.where(des_f, 0.6, 1.0))
    axs[1, k].imshow(show.T, origin="lower", extent=[a0, a0 + hole_f.shape[0] * FS, b0, b0 + hole_f.shape[1] * FS],
                     cmap="Blues_r", vmin=0, vmax=1)
    axs[1, k].set_title(f"{name} 구멍(흰) · 설계칸(연파랑)", fontsize=9)
    for ax in axs[:, k]:
        ax.set_aspect("equal")
plt.tight_layout()
plt.savefig(R / "post_maps.png", dpi=90)
print("saved", R / "post_maps.png")
