"""ALIGN 후보 다시 찾기 — 실은 퍽 조건 더함 (계산, 2026-11-11 미션 파트 261111 1).

align_search.py 의 A1 · A2 · A3 · A4(상자 · 바닥 · 회전 반지름) 그대로 +
  P: 팔레트에 실은 퍽 12 개 (loaded URDF 실제 모양) 와 팔 · 집게 최소 거리 ≥ 15 mm
  B: 차체 CoACD (full URDF base_link) 와 최소 거리 ≥ 15 mm (Isaac 측정 전 거름. 처짐 j2 −0.02 도 같이 봄)
j1 · j5(손목 롤, --j5) 도 훑음.
  python search2.py <dev urdf> <isaac urdf 폴더> [--step 0.04] [--j1 -0.2 0.2 0.1]
"""
import argparse
import math
import os
import sys

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("dev")
ap.add_argument("isaac")
ap.add_argument("--step", type=float, default=0.04)
ap.add_argument("--j1", type=float, nargs=3, default=[-0.2, 0.2, 0.1])
ap.add_argument("--j2", type=float, nargs=2, default=[-1.60, -0.80])
ap.add_argument("--top", type=int, default=20)
ap.add_argument("--j3", type=float, nargs=2, default=[-1.5, 1.5])
ap.add_argument("--rc", type=float, default=None, help="제자리 회전 중심 x (base_link). 없으면 계획 중심 C_X. 한계 = 그 중심에서 차체 모서리까지 거리")
ap.add_argument("--j4", type=float, nargs=2, default=[-1.82, 1.9])
ap.add_argument("--j5", type=float, nargs=3, default=[0.0, 0.0, 1.0], help="손목 롤 격자 lo hi step (팔 카메라도 같이 돎)")
ap.add_argument("--a2_aimed", action="store_true", help="A2 를 조준 퍽 (옆 배치의 가운데 퍽) · inline 두 퍽만으로 (옆 배치의 옆 퍽 가림은 기록만)")
ap.add_argument("--no_a2", action="store_true", help="A2 (앞 D435 가림) 를 거르지 않고 기록만")
ap.add_argument("--puck_min", type=float, default=0.015)
a = ap.parse_args()

HERE = os.path.dirname(os.path.abspath(__file__))
sys.argv = ["align_search.py", a.dev, "--pose", "-1.62", "0.21", "1.88"]
ns = {"__name__": "align_search"}
_out = sys.stdout
sys.stdout = open(os.devnull, "w")
exec(open(os.path.join(HERE, "align_search.py")).read(), ns)
sys.stdout = _out
fk, ALLh, scenes, lim, a2_blocked = ns["fk"], ns["ALLh"], ns["scenes"], ns["lim"], ns["a2_blocked"]
FX, FY, CX, CY, W, H = ns["FX"], ns["FY"], ns["CX"], ns["CY"], ns["W"], ns["H"]
FRONT, C_X, ZTOP, LINKS = ns["FRONT"], ns["C_X"], ns["ZTOP"], ns["LINKS"]

sys.path.insert(0, os.path.join(HERE, "rl_probe"))
from align_geom import Geom  # noqa: E402
import xml.etree.ElementTree as ET  # noqa: E402
from align_geom import box_pts, tf  # noqa: E402

LG = Geom(os.path.join(a.isaac, "waffle_4wd_loaded.urdf"))
FG = Geom(os.path.join(a.isaac, "waffle_4wd_full.urdf"))
r = ET.parse(os.path.join(a.isaac, "waffle_4wd_loaded.urdf")).getroot()
shape = {l.get("name"): l.find("visual").find("geometry")[0].tag for l in r.findall("link") if l.get("name").startswith("puck_")}
cyl = np.array([[0.015 * math.cos(t), 0.015 * math.sin(t), z] for t in np.linspace(0, 2 * math.pi, 32, endpoint=False) for z in (-0.03, 0.03)])
PUCKS = []
for j in r.findall("joint"):
    ch = j.find("child").get("link")
    if ch.startswith("puck_"):
        o = j.find("origin")
        T = tf(list(map(float, o.get("xyz").split())), list(map(float, o.get("rpy").split())))
        B = cyl if shape[ch] == "cylinder" else box_pts([0.03, 0.03, 0.06])
        P = (T[:3, :3] @ B.T).T + T[:3, 3]
        PUCKS.append((ch, P, P.min(0), P.max(0)))


def puck_min(q5, cut=0.03):
    best = (9.0, None)
    for ln, an, A in LG.arm_pts(q5, 0.010):
        amn, amx = A.min(0), A.max(0)
        for pn, B, bmn, bmx in PUCKS:
            if np.linalg.norm(np.maximum(0, np.maximum(bmn - amx, amn - bmx))) > min(cut, best[0]):
                continue
            d = Geom.dist(A, B)
            if d < best[0]:
                best = (d, f"{an}–{pn}")
    return best


def a1_margin(q):
    T = fk("arm_camera_optical_frame", q)
    if T[2, 2] > -0.05:
        return -1, T
    c = ALLh @ np.linalg.inv(T).T
    zc = c[..., 2]
    if (zc <= 0.03).any():
        return -1, T
    u = FX * c[..., 0] / zc + CX
    v = FY * c[..., 1] / zc + CY
    return np.minimum.reduce([u, W - u, v, H - v]).min(), T


RC = C_X if a.rc is None else a.rc
_rear, _hw = 2 * C_X - FRONT, math.sqrt(0.218 ** 2 - (FRONT - C_X) ** 2)   # 차체 외곽 (계획 중심에서 모서리 0.218)
RLIM = max(math.hypot(x - RC, y) for x in (FRONT, _rear) for y in (_hw, -_hw))
print(f"회전 중심 x {RC:.4f}, 차체 모서리 반지름 {RLIM:.4f} (팔 점이 이 안, 명령 · 처짐 둘 다)")
res = []
n_a1 = n_a2 = n_p = 0
for q5r, q1 in [(x, y) for x in np.arange(a.j5[0], a.j5[1] + 1e-9, a.j5[2]) for y in np.arange(a.j1[0], a.j1[1] + 1e-9, a.j1[2])]:
    for q2 in np.arange(a.j2[0], a.j2[1] + 1e-9, a.step):
        for q3 in np.arange(a.j3[0], a.j3[1] + 1e-9, a.step):
            for q4 in np.arange(a.j4[0], a.j4[1] + 1e-9, a.step):
                q = {"joint1": q1, "joint2": q2, "joint3": q3, "joint4": q4, "joint5": q5r}
                if any(not (lim[k][0] + 0.1 <= v <= lim[k][1] - 0.1) for k, v in q.items() if k in lim):
                    continue
                m, T = a1_margin(q)
                if m < 30:
                    continue
                # 처짐 (j2 0.02 더 접힘) 에서도 A1
                q_s = dict(q, joint2=q2 - 0.02)
                m_s, _ = a1_margin(q_s)
                if m_s < 30:
                    continue
                Tg = fk("gripper_roll_link", q)
                pts = np.array([fk(l, q)[:3, 3] for l in LINKS] + [(Tg @ np.array([x, y, 0, 1]))[:3] for x, y in ((0.14, 0.0), (0.17, 0.031), (0.17, -0.031))])
                if pts[:, 2].min() < 0.02 or ((pts[:, 0] > FRONT + 0.127) & (pts[:, 2] < ZTOP + 0.04)).any():
                    continue
                Tgs = fk("gripper_roll_link", q_s)
                pts_s = np.array([fk(l, q_s)[:3, 3] for l in LINKS] + [(Tgs @ np.array([x, y, 0, 1]))[:3] for x, y in ((0.14, 0.0), (0.17, 0.031), (0.17, -0.031))])
                rad = max(np.hypot(pts[:, 0] - RC, pts[:, 1]).max(), np.hypot(pts_s[:, 0] - RC, pts_s[:, 1]).max())
                if rad > RLIM:
                    continue
                n_a1 += 1
                blk = a2_blocked(q)
                if a.a2_aimed:
                    blk = [b for b in blk if not (b[1] != "inline" and tuple(b[2]) != (0.0, 0.0))]
                if blk and not a.no_a2:
                    continue
                n_a2 += 1
                q5 = [q1, q2, q3, q4, q5r]
                pm = puck_min(q5)
                pm_s = puck_min([q1, q2 - 0.02, q3, q4, q5r])
                if min(pm[0], pm_s[0]) < a.puck_min:
                    continue
                n_p += 1
                bd = FG.min_dist([q1, q2 - 0.02, q3, q4, q5r])  # 처짐 쪽이 차체에 가까움
                if bd[0] < 0.015:
                    continue
                down = math.degrees(math.acos(max(-1.0, min(1.0, -T[2, 2]))))
                res.append((min(m, m_s), q5, down, rad, pm, pm_s, bd, len(blk)))
print(f"A1·A3·A4(상자·회전) 통과 {n_a1}, + A2 {n_a2}, + 실은 퍽 ≥15 mm {n_p}, + 차체 CoACD ≥15 mm {len(res)}")
res.sort(key=lambda x: -x[0])
for m, q5, down, rad, pm, pm_s, bd, nb in res[:a.top]:
    print(f"A1 여유 {m:.0f} px  q {np.round(q5, 2).tolist()}  광축 아래 {down:.1f}°  회전 반지름 {rad:.3f}  "
          f"실은 퍽 {pm[0] * 1000:.1f}/{pm_s[0] * 1000:.1f} mm ({pm_s[1]})  차체 {bd[0] * 1000:.1f} mm ({bd[1]} – {bd[2]})  A2 가림 시선 {nb}")
