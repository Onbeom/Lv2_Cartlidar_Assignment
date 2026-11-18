"""5번 계획 ② · ③: survey · place_box · verify 상자 보기 자세 판정 (계산, 2026-11-11 미션 파트 261111 1).

자세 = manipulator box_look 과 같은 계산 (look_tool: targetLookCandidates + ArmIK, 관절 한계), 보는 점 = 상자 윗면 가운데.
판정 (기준 3판 B1 · B2 · B3 = A3 · A4, 4판 처짐 · 실은 퍽):
  B1 화면 안: survey · verify = 3 × 3 칸 중심에 놓인 퍽 윗면(30 mm, 서 있는 +0.06 · 누운 +0.03) 전부, place_box = 윗면 네 모서리 + 15 mm (윗면 높이).
     화면 = 편 영상 newK (fx 602.35 · fy 793.36 · cx 530.64 · cy 359.53, 1280 × 720), 가장자리 30 px 넘게.
     흔들림: 로봇 yaw ±0.05 rad (FINE 허용), 상자 회전 0 · ±10°.
  B2 태그: 검은 사각형 16 mm, 반으로 줄인 영상(640 폭)에서 투영된 짧은 변 ≥ 16 px. 태그 면 = 누운 퍽 윗면(+0.03, 먼 면) · 서 있는 퍽(+0.06),
     자리 = 3 × 3 칸 중심, 태그 돌림 0 · 45°.
  A3 관절 한계 여유 > 0.1 rad (명령 · 처짐 j2 −0.02 · j4 +0.02).
  A4 집게 끝 윗면 + 0.04 위(후보 생성에서 거름), 팔 점이 상자 영역(윗면 + 0.04 아래)에 안 듦, 차체 CoACD ≥ 15 mm · 실은 퍽 ≥ 15 mm (place_box 만 퍽 실음).
  python box_view_eval.py <dev urdf> <isaac urdf 폴더> [--face 0.127]
"""
import argparse
import itertools
import math
import os
import subprocess
import sys

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("dev")
ap.add_argument("isaac")
ap.add_argument("--face", type=float, default=0.127, help="차체 앞 끝 → 상자 면 (FINE 0.127, 다시 보기 0.20)")
ap.add_argument("--dists", type=float, nargs="+", default=[round(0.18 + 0.02 * i, 2) for i in range(10)])
ap.add_argument("--angles", type=float, nargs="+", default=[90, 85, 80, 75, 70, 65, 60])
ap.add_argument("--detect_w", type=float, default=640, help="태그를 찾는 영상 폭 (팔 1280 을 이 폭으로 줄임)")
ap.add_argument("--no_body", action="store_true", help="CoACD · 실은 퍽 거리 계산 생략 (빠름)")
a = ap.parse_args()

AV = "/home/pa4/git/lv2_next_module/261111_1700_align_view"
sys.argv = ["align_search.py", a.dev, "--pose", "-1.37", "-0.01", "1.85"]
ns = {"__name__": "x"}
_o = sys.stdout
sys.stdout = open(os.devnull, "w")
exec(open(os.path.join(AV, "align_search.py")).read(), ns)
sys.stdout = _o
fk, LINKS = ns["fk"], ns["LINKS"]
FX, FY, CX, CY, W, H = 602.35, 793.36, 530.64, 359.53, 1280, 720
FRONT, HALF, ZTOP = 0.1328, 0.1025, 0.07
LIM_LO = np.array([-3.14159, -2.14, -1.6, -1.92, -2.8])
LIM_HI = np.array([3.14159, 1.6, 1.6, 2.0, 2.8])

T_ab = fk("arm_base_link", {})          # base_footprint ← arm_base_link
xc = FRONT + a.face + HALF
aim_fp = np.array([xc, 0.0, ZTOP, 1.0])
aim_ab = np.linalg.inv(T_ab) @ aim_fp

line = f"{aim_ab[0]} {aim_ab[1]} {aim_ab[2]} 0.04 {len(a.dists)} {' '.join(map(str, a.dists))} {len(a.angles)} {' '.join(map(str, a.angles))}\n"
out = subprocess.run([os.path.join(os.path.dirname(os.path.abspath(__file__)), "look_tool")], input=line, capture_output=True,
                     text=True, check=True).stdout.split("\n")
cands = []
for l in out:
    if not l or l == "END":
        continue
    v = l.split()
    cands.append(dict(d=float(v[0]), ang=float(v[1]), ok=v[2] == "1", q=[float(x) for x in v[3:8]]))

sys.path.insert(0, os.path.join(AV, "rl_probe"))
if not a.no_body:
    from align_geom import Geom, box_pts, tf  # noqa: E402
    import xml.etree.ElementTree as ET  # noqa: E402
    FG = Geom(os.path.join(a.isaac, "waffle_4wd_full.urdf"))
    LG = Geom(os.path.join(a.isaac, "waffle_4wd_loaded.urdf"))
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
            PUCKS.append((ch, (T[:3, :3] @ B.T).T + T[:3, 3]))

    def puck_min(q5):
        best = 9.0
        for ln, an, A in LG.arm_pts(q5, 0.010):
            for pn, B in PUCKS:
                best = min(best, Geom.dist(A, B))
        return best


def rot_about(p, yaw, c):
    """로봇이 yaw 돌았을 때 장면 점 (로봇 기준) = c 둘레로 −yaw"""
    s, k = math.sin(-yaw), math.cos(-yaw)
    x, y = p[..., 0] - c[0], p[..., 1] - c[1]
    out = p.copy()
    out[..., 0] = c[0] + k * x - s * y
    out[..., 1] = c[1] + s * x + k * y
    return out


def box_pts_plane(half, z, byaw):
    P = np.array([[sx * half, sy * half] for sx in np.linspace(-1, 1, 5) for sy in np.linspace(-1, 1, 5)])
    c, s = math.cos(byaw), math.sin(byaw)
    R = np.c_[c * P[:, 0] - s * P[:, 1], s * P[:, 0] + c * P[:, 1]]
    return np.c_[R[:, 0] + xc, R[:, 1], np.full(len(R), z)]


def proj(Ti, P):
    k = np.c_[P, np.ones(len(P))] @ Ti.T
    z = k[:, 2]
    return FX * k[:, 0] / z + CX, FY * k[:, 1] / z + CY, z


ICR = np.array([-0.0658, 0.0])
CELL = 0.0683
YAWS = (-0.05, 0.0, 0.05)
BYAWS = (math.radians(-10), 0.0, math.radians(10))


def cell_pucks(z, byaw):
    """3 × 3 칸 중심에 놓인 퍽 윗면 (30 mm 정사각) 꼭짓점, 높이 z"""
    c, s = math.cos(byaw), math.sin(byaw)
    P = []
    for cx_, cy_ in itertools.product((-CELL, 0, CELL), repeat=2):
        for dx, dy in itertools.product((-0.015, 0.015), repeat=2):
            x, y = cx_ + dx, cy_ + dy
            P.append([xc + c * x - s * y, s * x + c * y, z])
    return np.array(P)


def b1(Ti, half, z, cells=False):
    worst = 1e9
    for yw, by in itertools.product(YAWS, BYAWS):
        P = rot_about(cell_pucks(z, by) if cells else box_pts_plane(half, z, by), yw, ICR)
        u, v, zz = proj(Ti, P)
        if (zz <= 0.02).any():
            return -1e9
        worst = min(worst, np.minimum.reduce([u, W - u, v, H - v]).min())
    return worst


TAG = 0.016


def b2(Ti):
    worst = 1e9
    for z in (ZTOP + 0.03, ZTOP + 0.06):
        for cx_, cy_ in itertools.product((-CELL, 0, CELL), repeat=2):
            for tr in (0.0, math.pi / 4):
                c, s = math.cos(tr), math.sin(tr)
                P = np.array([[xc + cx_ + c * dx - s * dy, cy_ + s * dx + c * dy, z] for dx, dy in ((-TAG / 2, -TAG / 2), (TAG / 2, -TAG / 2), (TAG / 2, TAG / 2), (-TAG / 2, TAG / 2))])
                u, v, _ = proj(Ti, P)
                uv = np.c_[u, v] * (a.detect_w / W)   # 검출 폭 영상
                side = min(np.linalg.norm(uv[(i + 1) % 4] - uv[i]) for i in range(4))
                worst = min(worst, side)
    return worst


def arm_in_box(q):
    Tg = fk("gripper_roll_link", q)
    pts = np.array([fk(l, q)[:3, 3] for l in LINKS] + [(Tg @ np.array([x, y, 0, 1]))[:3] for x, y in ((0.14, 0.0), (0.17, 0.031), (0.17, -0.031))])
    inside = (pts[:, 0] > FRONT + a.face) & (np.abs(pts[:, 1]) < HALF + 0.03) & (pts[:, 2] < ZTOP + 0.04)
    return bool(inside.any()), pts[:, 2].min()


print(f"상자 면 {a.face} m (상자 가운데 base x {xc:.4f}), 보는 점 arm_base_link ({aim_ab[0]:.3f}, {aim_ab[1]:.3f}, {aim_ab[2]:.3f})")
print("d  각  | IK | B1 survey·verify(px) B1 place(px) | B2 태그(px, 640) | 관절 여유 명령/처짐 | 상자 침범 | 차체 mm | 실은 퍽 mm")
rows = []
for c in cands:
    if not c["ok"]:
        print(f"{c['d']:.2f} {c['ang']:.0f} | IK 없음 · 한계 밖")
        continue
    q5 = c["q"]
    qs = [q5[0], q5[1] - 0.02, q5[2], q5[3] + 0.02, q5[4]]
    res = {}
    for tag, qq in (("cmd", q5), ("sag", qs)):
        q = {f"joint{i + 1}": qq[i] for i in range(5)}
        Ti = np.linalg.inv(fk("arm_camera_optical_frame", q))
        res[tag] = dict(sv=min(b1(Ti, 0, ZTOP + 0.06, True), b1(Ti, 0, ZTOP + 0.03, True)), pl=b1(Ti, HALF + 0.015, ZTOP), tag=b2(Ti),
                        jm=float(min(np.min(np.array(qq) - LIM_LO), np.min(LIM_HI - np.array(qq)))), box=arm_in_box(q))
    sv = min(res["cmd"]["sv"], res["sag"]["sv"])
    pl = min(res["cmd"]["pl"], res["sag"]["pl"])
    tg = min(res["cmd"]["tag"], res["sag"]["tag"])
    jm = min(res["cmd"]["jm"], res["sag"]["jm"])
    inbox = res["cmd"]["box"][0] or res["sag"]["box"][0]
    body = pk = float("nan")
    if not a.no_body:
        body = min(FG.min_dist(q5)[0], FG.min_dist(qs)[0]) * 1000
        pk = min(puck_min(q5), puck_min(qs)) * 1000
    rows.append(dict(d=c["d"], ang=c["ang"], q=q5, sv=sv, pl=pl, tg=tg, jm=jm, inbox=inbox, body=body, pk=pk))
    print(f"{c['d']:.2f} {c['ang']:.0f} | ok | {sv:7.0f} {pl:7.0f} | {tg:5.1f} | {res['cmd']['jm']:.3f}/{res['sag']['jm']:.3f} | {'침범' if inbox else '-'} | "
          f"{body:6.1f} | {pk:6.1f} | q {np.round(q5, 3).tolist()}")
print("\n판정 (B1 > 30 px, B2 ≥ 16 px, 관절 > 0.1, 상자 침범 없음, 차체 ≥ 15, 실은 퍽 ≥ 15 = place_box 만):")
for r_ in rows:
    common = r_["jm"] > 0.1 and not r_["inbox"] and (a.no_body or r_["body"] >= 15)
    sv_ok = common and r_["sv"] > 30 and r_["tg"] >= 16
    pl_ok = common and r_["pl"] > 30 and (a.no_body or r_["pk"] >= 15)
    print(f"  {r_['d']:.2f} m {r_['ang']:.0f}°: survey · verify {'통과' if sv_ok else '-'} | place_box {'통과' if pl_ok else '-'}")
