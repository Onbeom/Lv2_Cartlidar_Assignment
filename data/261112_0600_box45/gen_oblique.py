"""비스듬히 서기 (안 O) 렌더 장면 만들기 (검토 261111 1 승인 기준, 11-12).

로봇 좌표 (base_footprint). 로봇은 잰 상자 중심 쪽으로 돌아 그 축 위, 가장 가까운 면 기준 틈 0.35 (상자 중심 x R0 = 0.1328 + 0.35 + 0.1025) 에 서려 함.
  β (로봇 기준 상자 요) 꼭짓점 {0, ±15, ±30, ±45}° + 5 단계 뒤 방향 어긋남 ±2° (장면마다 부호 번갈아).
  보는 점 오차 18 mm 대각 네 방향: 팔은 잰 중심 (= 참 중심 + 오차) 을 겨눔 → q = box_look(잰 중심, 0.44 m · 50°).
  서는 거리 오차 δ (참 중심이 R0 + δ): |β| ≤ 30° 는 TFmini ±0.010 m, |β| > 30° 는 D435 + odom ±0.015 m (가정: D435 중심 ≤ 2.1 mm 렌더 +
    장착 피치 1° 일 때 앞뒤 7.4 mm + odom 이동 약 0.1 m 의 2 %) — 부호는 보는 점 오차 x 부호와 같게 (나쁜 쪽).
  퍽 0 · 2 (시작 박스처럼 상자 축 ±40 mm) · 6 · 9 개 (3 x 3 칸 0.06, 서 있는 것 2/3).
  사전값 (perception prior) = 잰 중심, 사전 요 = 지도 축 (로봇 방향 어긋남 ±0.08 rad 균등).
    python3 gen_oblique.py --seed 2 --out oblique_scenes.json
"""
import argparse
import itertools
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--seed", type=int, default=2)
ap.add_argument("--out", type=Path, required=True)
ap.add_argument("--dist", type=float, default=0.44)
ap.add_argument("--ang", type=float, default=50.0)
ap.add_argument("--gap", type=float, default=0.35)
ap.add_argument("--limit", type=int, default=0, help="도구 확인용: 앞에서 몇 장면만")
a = ap.parse_args()

U3 = Path(__file__).resolve().parent.parent / "261112_0130_u3_render"
ns = {"__name__": "x", "__file__": str(U3 / "approach_resid.py")}
src = open(U3 / "approach_resid.py").read().split("\nrng = np.random.default_rng(0)\nprint(")[0]
sys.argv = ["approach_resid.py", "--dist", str(a.dist), "--ang", str(a.ang)]
_o = sys.stdout
sys.stdout = open(os.devnull, "w")
exec(src, ns)
sys.stdout = _o

FRONT, HALF, CELL = 0.1328, 0.1025, 0.06
R0 = FRONT + a.gap + HALF
E = 0.018 / math.sqrt(2)
rng = np.random.default_rng(a.seed)
scenes = []
i = 0
for beta, (sx, sy), k in itertools.product((0, 15, -15, 30, -30, 45, -45), itertools.product((-1, 1), repeat=2), (0, 2, 6, 9)):
    resid = 2.0 if i % 2 == 0 else -2.0
    b_true = math.radians(beta + resid)
    delta = (0.015 if abs(beta) > 30 else 0.010) * sx
    c = np.array([R0 + delta, 0.0])
    aim = c + np.array([sx * E, sy * E])
    q = ns["look"](aim)
    assert q is not None, (beta, sx, sy)
    Rb = np.array([[math.cos(b_true), -math.sin(b_true)], [math.sin(b_true), math.cos(b_true)]])
    if k == 2:
        locs = [np.array([-0.04, 0.0]), np.array([0.04, 0.0])]
        cells = [[0, 0], [0, 0]]
    else:
        allc = list(itertools.product((-1, 0, 1), repeat=2))
        pick = rng.choice(9, size=k, replace=False) if k else []
        locs = [np.array(allc[j], float) * CELL for j in pick]
        cells = [list(allc[j]) for j in pick]
    ids = rng.permutation(12)[:k]
    pucks = []
    for j, loc in enumerate(locs):
        xy = c + Rb @ (loc + rng.uniform(-0.003, 0.003, 2))
        kind = str(rng.choice(["stand_cylinder", "stand_cuboid", "stand_cylinder", "stand_cuboid", "lie_cuboid", "lie_cylinder"]))
        pucks.append({"kind": kind, "xy": xy.tolist(), "yaw": float(b_true + rng.uniform(-math.pi, math.pi)),
                      "color": str(rng.choice(["red", "green"])), "tag_id": int(ids[j]), "cell": cells[j]})
    scenes.append({"id": f"scene_{i:03d}", "kind": "judge", "cam_dz": 0.0, "corner": True, "beta_deg": beta, "resid_deg": resid,
                   "aim_err": [sx * E, sy * E], "delta": delta, "dist_by": "D435+odom" if abs(beta) > 30 else "TFmini",
                   "stop_err": [delta, 0.0, 0.0], "box_yaw_deg": beta + resid, "center": c.tolist(), "yaw": b_true,
                   "prior": aim.tolist(), "prior_yaw": float(rng.uniform(-0.08, 0.08)), "pucks": pucks, "q": [float(v) for v in q]})
    i += 1
if a.limit:
    scenes = scenes[:: max(1, len(scenes) // a.limit)][: a.limit]
a.out.write_text(json.dumps({"dist": a.dist, "ang": a.ang, "gap": a.gap, "R0": R0, "seed": a.seed, "scenes": scenes}, indent=1) + "\n")
print(f"장면 {len(scenes)} 개 → {a.out}, q 범위 j1 {min(s['q'][0] for s in scenes):+.3f} ~ {max(s['q'][0] for s in scenes):+.3f}")
