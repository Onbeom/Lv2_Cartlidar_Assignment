"""구멍 뚫은 저장소 부품(housing·magazine·rail, 출력용 STEP → parts_src/*.stl)을 옛 부품(조립 STEP 고체) 자리에 맞춘다.

형상·크기는 같고 구멍만 다르다(사용자). 주축(PCA) 방향 4가지(오른손 좌표) 후보로 시작해 ICP로 다듬고,
새 부품 표면 점 → 옛 부품 표면 거리의 중앙값이 가장 작은 것을 고른다(구멍 벽은 옛 부품 안이라 중앙값으로 봄).
결과: parts_storage/<이름>.stl(URDF base_link mm), parts_storage/placement.json
사용(PC): ~/pa-course/IsaacLab-6.1/.venv/bin/python tools/place_storage.py
"""
import itertools
import json
from pathlib import Path

import numpy as np
import trimesh

H = Path(__file__).resolve().parents[1]
OUT = H / "parts_storage"
OUT.mkdir(exist_ok=True)
JOBS = {   # 이름: (새 부품 STL, 옛 고체 번호(asm/solids))
    "housing_1": ("housing_3EA", 28), "housing_2": ("housing_3EA", 61), "housing_3": ("housing_3EA", 62),
    "magazine_B": ("magazine_3EA", 29), "magazine_G": ("magazine_3EA", 30), "magazine_R": ("magazine_3EA", 31),
    "rail_hi": ("rail_hi_1EA", 32), "rail_lo": ("rail_lo_1EA", 33),
}


def pca(m):
    p, _ = trimesh.sample.sample_surface(m, 20000, seed=0)
    c = p.mean(0)
    w, v = np.linalg.eigh(np.cov((p - c).T))
    return c, v[:, ::-1]


res = {}
for name, (src, sid) in JOBS.items():
    new = trimesh.load(H / "parts_src" / f"{src}.stl")
    old = trimesh.load(H / "asm" / "solids" / f"s{sid:03d}.stl")
    cn, vn = pca(new)
    co, vo = pca(old)
    pts_old, _ = trimesh.sample.sample_surface(old, 4000, seed=1)
    best = None
    for sg in itertools.product((1, -1), repeat=3):
        S = np.diag(sg)
        R = vo @ S @ vn.T
        if np.linalg.det(R) < 0:
            continue
        T0 = np.eye(4); T0[:3, :3] = R; T0[:3, 3] = co - R @ cn
        Ti, _, cost = trimesh.registration.icp(pts_old, new, initial=np.linalg.inv(T0), max_iterations=60, reflection=False, scale=False)
        T = np.linalg.inv(Ti)       # 새 → 옛
        m = new.copy(); m.apply_transform(T)
        p, _ = trimesh.sample.sample_surface(m, 4000, seed=2)
        d = np.abs(trimesh.proximity.signed_distance(old, p))
        score = float(np.median(d))
        if best is None or score < best[0]:
            best = (score, T, float(np.percentile(d, 90)), m)
    score, T, p90, m = best
    m.export(OUT / f"{name}.stl")
    res[name] = {"src": src, "old_solid": sid, "median_mm": score, "p90_mm": p90, "T_print_to_base_link_mm": T.tolist(),
                 "volume_new_cm3": m.volume / 1000, "volume_old_cm3": old.volume / 1000,
                 "bounds_new": m.bounds.round(2).tolist(), "bounds_old": old.bounds.round(2).tolist()}
    print(f"{name}: 중앙 {score:.3f} mm, 90 % {p90:.3f} mm, 부피 새 {m.volume / 1000:.1f} / 옛 {old.volume / 1000:.1f} cm3, "
          f"외곽 차 {np.abs(m.bounds - old.bounds).max():.2f} mm", flush=True)
(OUT / "placement.json").write_text(json.dumps(res, indent=1))
