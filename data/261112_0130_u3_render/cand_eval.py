"""5번 통합안 후보 다시 계산 (계산, 11-12 미션 파트 261112 1 — 검토 261111 1 ① 조건부 승인 반영).

FK = dev URDF(waffle_4wd_description, c689a38) 를 urdf_chain 으로 (base_footprint 기준 바르게: align_search.fk 는 base_link 에서
멈춰 높이가 6.75 mm 낮았음). 카메라 = 편 영상 newK (fx 602.35 · fy 793.36 · cx 530.64 · cy 359.53, 1280 x 720).
자세 = look_tool (노드 box_look 과 같은 계산), 보는 점 = 계획 상자 중심 + 앞뒤 o (윗면 높이).
단계별 밀림 (검토 261111 1, 11-12):
  place_box : 앞뒤 dx ±0.01 (approach_box 가 TFmini 로 면까지 재서 섬 → 정지 허용 drive align default_tolerance), 옆 dy ±0.03 (밀림 + 정지), ±0.05 기록
  survey    : 지도 상자 자리를 겨눔 → dx · dy ±0.03
  verify    : 잰 중심을 겨눔 → dx · dy ±0.01
  공통      : 로봇 요 ±0.05 rad (회전 중심 icr_x −0.0658), 상자 요 0 · ±10°, 관절 명령 · 처짐 (j2 −0.02 · j4 +0.02) 둘 다
판정: B1 place = 윗면 네 모서리 + 15 mm 가 화면 가장자리에서 30 px 넘게, B1 survey · verify = 3 x 3 칸 중심 퍽 윗면 30 mm (서 있는 +0.06 · 누운 +0.03) 30 px 넘게,
      B2 = 태그 16 mm 짧은 변 (960 폭 = 1280 투영 x 0.75, 1280 폭 그대로) ≥ 16 px, 태그 돌림 0 · 45°, 서 있는 · 누운 높이 (survey · verify 밀림 중 최소),
      A3 관절 한계 여유 > 0.1 rad, A4 팔 점이 상자 영역 (윗면 + 0.04 아래, 옆 밀림 더함) 밖 · 차체 CoACD ≥ 15 mm · 실은 퍽 ≥ 15 mm (place_box 만).
    python3 cand_eval.py unified      # 한 번 보기 후보 격자
    python3 cand_eval.py two_view     # 예비안 (면 0.127 두 번 보기 + 면 0.20 place_box) 같은 밀림 조건
"""
import itertools
import math
import os
import subprocess
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(Path.home() / "git/lv2_next_module/Cartlidar/sim_render"))
from urdf_chain import fixed_transform  # noqa: E402

AV = Path.home() / "git/lv2_next_module/261111_1700_align_view"
sys.path.insert(0, str(AV / "rl_probe"))
ISAAC = HERE / "dev_src/workspace/urdf_and_usd/isaac/urdf"   # dev c689a38 정밀 URDF (CoACD) · 실은 퍽
_argv = sys.argv
sys.argv = ["loaded_pucks.py", str(ISAAC)]
from align_geom import Geom, box_pts, tf  # noqa: E402
sys.argv = _argv
import xml.etree.ElementTree as ET  # noqa: E402

URDF = str(HERE / "waffle_4wd_dev_c689a38.urdf")
LOOK = str(Path.home() / "git/lv2_next_module/261111_2240_box_view/look_tool")
FX, FY, CX, CY, W, H = 602.35, 793.36, 530.64, 359.53, 1280, 720
FRONT, HALF, ZTOP, CELL, TAG = 0.1328, 0.1025, 0.07, 0.0683, 0.016
LO = np.array([-3.14159, -2.14, -1.6, -1.92, -2.8])
HI = np.array([3.14159, 1.6, 1.6, 2.0, 2.8])
ICR = np.array([-0.0658, 0.0])
T_ab = fixed_transform(URDF, "arm_base_link", root="base_footprint", q={})
FG = Geom(str(ISAAC / "waffle_4wd_full.urdf"))
LG = Geom(str(ISAAC / "waffle_4wd_loaded.urdf"))
_r = ET.parse(str(ISAAC / "waffle_4wd_loaded.urdf")).getroot()
_shape = {l.get("name"): l.find("visual").find("geometry")[0].tag for l in _r.findall("link") if l.get("name").startswith("puck_")}
_cyl = np.array([[0.015 * math.cos(t), 0.015 * math.sin(t), z] for t in np.linspace(0, 2 * math.pi, 32, endpoint=False) for z in (-0.03, 0.03)])
PUCKS = []
for j in _r.findall("joint"):
    ch = j.find("child").get("link")
    if ch.startswith("puck_"):
        o = j.find("origin")
        T = tf(list(map(float, o.get("xyz").split())), list(map(float, o.get("rpy").split())))
        B = _cyl if _shape[ch] == "cylinder" else box_pts([0.03, 0.03, 0.06])
        PUCKS.append((T[:3, :3] @ B.T).T + T[:3, 3])
LINKS = ["link2", "link3", "link4", "link5", "gripper_roll_link", "arm_camera_link"]


def qd(q):
    return {f"joint{i + 1}": q[i] for i in range(5)}


def look(xc, o, d, ang):
    aim = np.linalg.inv(T_ab) @ np.array([xc + o, 0.0, ZTOP, 1.0])
    out = subprocess.run([LOOK], input=f"{aim[0]} {aim[1]} {aim[2]} 0.04 1 {d} 1 {ang}\n", capture_output=True, text=True).stdout.split()
    if len(out) < 8 or out[2] != "1":
        return None
    return [float(x) for x in out[3:8]]


def sag(q):
    return [q[0], q[1] - 0.02, q[2], q[3] + 0.02, q[4]]


def Ti_of(q):
    return np.linalg.inv(fixed_transform(URDF, "arm_camera_optical_frame", root="base_footprint", q=qd(q)))


def place(local, z, xc, d, byaw, ryaw):
    c, s = math.cos(byaw), math.sin(byaw)
    R = np.c_[c * local[:, 0] - s * local[:, 1], s * local[:, 0] + c * local[:, 1]]
    v = np.c_[R[:, 0] + xc + d[0], R[:, 1] + d[1]] - ICR
    cr, sr = math.cos(-ryaw), math.sin(-ryaw)
    xy = ICR + np.c_[cr * v[:, 0] - sr * v[:, 1], sr * v[:, 0] + cr * v[:, 1]]
    return np.c_[xy, np.full(len(xy), z)]


def proj(Ti, P):
    k = np.c_[P, np.ones(len(P))] @ Ti.T
    return np.c_[FX * k[:, 0] / k[:, 2] + CX, FY * k[:, 1] / k[:, 2] + CY], k[:, 2]


def margin(Ti, P):
    uv, z = proj(Ti, P)
    if (z <= 0.02).any():
        return -1e9
    return float(min(uv[:, 0].min(), W - uv[:, 0].max(), uv[:, 1].min(), H - uv[:, 1].max()))


HB = HALF + 0.015
BOX_LOCAL = np.array([[sx * HB, sy * HB] for sx in (-1, 1) for sy in (-1, 1)])


def cell_local(cells):
    return np.array([[cx_ + dx, cy_ + dy] for cx_, cy_ in cells for dx, dy in itertools.product((-0.015, 0.015), repeat=2)])


ALL_CELLS = list(itertools.product((-CELL, 0, CELL), repeat=2))


def shifts(sx, sy):
    return list(itertools.product((-sx, 0.0, sx), (-sy, 0.0, sy)))


def b1(Ti, local, zs, xc, sh):
    return min(margin(Ti, place(local, z, xc, d, math.radians(b), r))
               for z in zs for d in sh for r in (-0.05, 0.0, 0.05) for b in (-10.0, 0.0, 10.0))


def b2(Ti, cells, xc, sh, scale):
    best = 1e9
    for (cx_, cy_), z, tr, d, r, b in itertools.product(cells, (ZTOP + 0.03, ZTOP + 0.06), (0.0, math.pi / 4), sh, (-0.05, 0.0, 0.05), (-10.0, 0.0, 10.0)):
        sq = np.array([[math.cos(tr + k * math.pi / 2), math.sin(tr + k * math.pi / 2)] for k in range(4)]) * TAG / math.sqrt(2)
        P = place(sq + np.array([cx_, cy_]), z, xc, d, math.radians(b), r)
        uv, _ = proj(Ti, P)
        best = min(best, min(np.linalg.norm(uv[i] - uv[(i + 1) % 4]) for i in range(4)) * scale)
    return best


def arm_in_box(q, xc, face, sy):
    pts = [fixed_transform(URDF, l, root="base_footprint", q=qd(q))[:3, 3] for l in LINKS]
    Tg = fixed_transform(URDF, "gripper_roll_link", root="base_footprint", q=qd(q))
    pts += [(Tg @ np.array([x, y, 0, 1]))[:3] for x, y in ((0.14, 0.0), (0.17, 0.031), (0.17, -0.031))]
    pts = np.array(pts)
    return bool(((pts[:, 0] > FRONT + face - 0.01) & (np.abs(pts[:, 1]) < HALF + 0.03 + sy) & (pts[:, 2] < ZTOP + 0.04)).any())


def body(q, loaded):
    d = min(FG.min_dist(q)[0], FG.min_dist(sag(q))[0]) * 1000
    pk = min(Geom.dist(A, B) for qq in (q, sag(q)) for _, _, A in LG.arm_pts(qq, 0.010) for B in PUCKS) * 1000 if loaded else float("nan")
    return d, pk


def jm(q):
    return float(min(np.min(np.array(qq) - LO) for qq in (q, sag(q))) if False else
                 min(min(np.min(np.array(qq) - LO), np.min(HI - np.array(qq))) for qq in (q, sag(q))))


def eval_unified(o, d, ang, face=0.20, full=True):
    xc = FRONT + face + HALF
    q = look(xc, o, d, ang)
    if q is None:
        return None
    r = dict(o=o, d=d, ang=ang, q=[round(v, 4) for v in q], jm=jm(q))
    Tis = [Ti_of(q), Ti_of(sag(q))]
    r["place"] = min(b1(Ti, BOX_LOCAL, (ZTOP,), xc, shifts(0.01, 0.03)) for Ti in Tis)
    r["survey"] = min(b1(Ti, cell_local(ALL_CELLS), (ZTOP + 0.06, ZTOP + 0.03), xc, shifts(0.03, 0.03)) for Ti in Tis)
    r["verify"] = min(b1(Ti, cell_local(ALL_CELLS), (ZTOP + 0.06, ZTOP + 0.03), xc, shifts(0.01, 0.01)) for Ti in Tis)
    if not full:
        return r
    r["place5"] = min(b1(Ti, BOX_LOCAL, (ZTOP,), xc, shifts(0.01, 0.05)) for Ti in Tis)
    tag = min(b2(Ti, ALL_CELLS, xc, shifts(0.03, 0.03), 1.0) for Ti in Tis)
    r["tag960"], r["tag1280"] = tag * 0.75, tag
    r["inbox"] = arm_in_box(q, xc, face, 0.03) or arm_in_box(sag(q), xc, face, 0.03)
    r["body"], r["puck"] = body(q, True)
    return r


def verdict(r):
    # 검토 11-12: 960 이 16 에 붙거나 (±1 px) 못 미치면 1280 을 씀 → 960 ≥ 17 이면 960, 아니면 1280 ≥ 16
    tag_ok = r["tag960"] >= 17.0 or r["tag1280"] >= 16.0
    return (r["place"] > 30 and r["survey"] > 30 and r["verify"] > 30 and tag_ok and r["jm"] > 0.1 and not r["inbox"]
            and r["body"] >= 15 and r["puck"] >= 15)


def fmt(r):
    s = (f"o {r['o']:+.2f} {r['d']:.2f} m {r['ang']:.0f}° q {r['q']} | place {r['place']:.1f} (±0.05 {r.get('place5', float('nan')):.1f}) · "
         f"survey {r['survey']:.1f} · verify {r['verify']:.1f} px")
    if "tag960" in r:
        s += (f" | 태그 960 {r['tag960']:.1f} · 1280 {r['tag1280']:.1f} px | 관절 {r['jm']:.2f} | 상자 침범 {r['inbox']} | "
              f"차체 {r['body']:.1f} · 실은 퍽 {r['puck']:.1f} mm | {'통과 (폭 ' + ('960' if r['tag960'] >= 17 else '1280') + ')' if verdict(r) else '불통과'}")
    return s


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "unified"
    if mode == "unified":
        rows = []
        for o, d, ang in itertools.product((-0.06, -0.04, -0.02, 0.0, 0.02, 0.04), (0.34, 0.36, 0.38, 0.40, 0.42, 0.44), (45, 50, 55, 60)):
            r = eval_unified(o, d, ang, full=False)
            if r and r["place"] > 30 and r["survey"] > 30 and r["verify"] > 30 and r["jm"] > 0.1:
                rows.append(r)
        print(f"화면 기준(B1 셋) · 관절 여유 통과 {len(rows)} 개 → 나머지 계산", flush=True)
        full = [eval_unified(r["o"], r["d"], r["ang"]) for r in rows]
        full.sort(key=lambda r: (not verdict(r), -min(r["place"] / 30, r["tag960"] / 16, r["body"] / 15, r["puck"] / 15)))
        for r in full:
            print(fmt(r))
        print("기준 비교 (참고): 이전 U1 자세 (보는 점 높이가 6.75 mm 낮게 계산된 q) 와 바른 FK 로 낸 같은 0.36 · 60°:")
        print(" ", fmt(eval_unified(0.0, 0.36, 60)))
    else:
        # 예비안: 면 0.127 두 번 보기 (0.24 m 60°, 보는 점 −0.02 · +0.06) + 면 0.20 place_box (0.38 m 55°, 보는 점 0)
        xc0 = FRONT + 0.127 + HALF
        for name, o in (("앞 보기", -0.02), ("뒤 보기", 0.06)):
            q = look(xc0, o, 0.24, 60)
            Tis = [Ti_of(q), Ti_of(sag(q))]
            per = []
            for c in ALL_CELLS:
                sv = min(b1(Ti, cell_local([c]), (ZTOP + 0.06, ZTOP + 0.03), xc0, shifts(0.03, 0.03)) for Ti in Tis)
                vf = min(b1(Ti, cell_local([c]), (ZTOP + 0.06, ZTOP + 0.03), xc0, shifts(0.01, 0.01)) for Ti in Tis)
                tg = min(b2(Ti, [c], xc0, shifts(0.03, 0.03), 0.5) for Ti in Tis)
                per.append((c, sv, vf, tg))
            print(f"{name}: q {np.round(q, 4).tolist()} 관절 {jm(q):.2f} 상자 침범 {arm_in_box(q, xc0, 0.127, 0.03)} 차체 {body(q, False)[0]:.1f} mm")
            for c, sv, vf, tg in per:
                print(f"   칸 ({c[0]:+.3f}, {c[1]:+.3f}): survey {sv:7.1f} · verify {vf:7.1f} px · 태그(640) {tg:5.1f} px")
        r = eval_unified(0.0, 0.38, 55)
        print("place_box 0.38 m 55° (면 0.20):", fmt(r))


if __name__ == "__main__":
    main()
