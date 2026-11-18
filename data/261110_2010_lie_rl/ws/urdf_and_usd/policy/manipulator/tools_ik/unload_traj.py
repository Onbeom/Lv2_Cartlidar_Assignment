"""스토리지 (팔레트 12 칸) 에서 퍽 꺼내기 경로 (2026-11-09, 넣기 경로와 별개). base_link 좌표, 팀 URDF FK.

칸 안 퍽 (바닥 = floor_z, 위 = floor_z + 60 mm, 벽 위 = floor_z + 30 mm) 을 집게 피치 p (60° = 상자 L1 · L3 놓기, 45° = L2 놓기) 로 쥠.
잡는 점 = 퍽 중심 + 15 mm (놓기 가정과 같음) = 벽 위 + 15 mm -> 손가락 패드 (±15 mm) 가 벽 위로 나온 퍽 윗부분만 쥠 (칸 안으로 안 들어감).
집게 수평 방향 = 관절1 축 -> 칸 (바깥), 손가락 닫는 축 수평 (직육면체 퍽 면이 넣을 때처럼 맞음).
경유 자세 (관절 선형 보간으로 따라감):
  0 칸 위 (잡는 점 + 6 cm 수직)  1 쥐기 (내려감)  2 들기 (퍽 바닥이 칸 바닥 + 10 cm)  3 높이 (관절1 은 칸 쪽, 퍽 바닥 0.21)
  4 앞 높이 (3 과 같은 자세에서 관절1 만 0)  5 놓기 넘김 자세 (놓기 환경 시작 자세)
검사: IK 오차 1 mm · 피치 1°, 관절 여유, 팔 자기 충돌 거리 (selfcol, 퍽 포함) > 55 mm, 손가락 · 쥔 퍽이 다른 칸 퍽 (꽉 찬 팔레트 가정) 위 8 mm 위.
    ~/.venvs/mujoco/bin/python unload_traj.py   -> ../unload_traj.json
"""
import json
import math
import os

import numpy as np
from scipy.optimize import least_squares

from selfcol import self_dist
from td_ik import LIM, fk

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
J1X = 0.096
GRASP_DZ, PUCK_H = 0.015, 0.060
HAND = {60: [0.0, -0.2454, 0.2999, 0.9927, 0.0], 45: [0.0, -0.0204, 0.2807, 0.5251, 0.0]}  # 놓기 환경 시작 자세 (place_env_cfg)
LIFT = 0.10
HIGH_BOTTOM = (0.21, 0.20, 0.19)  # 높이 자세 퍽 바닥 후보 (꽉 찬 팔레트 퍽 위 0.174 ~ 0.196)
HIGH_R = (0.10, 0.12, 0.08)


def pose_res(p, gd, pitch):
    def res(q):
        P, R = fk(q)
        return list((P - p) * 100) + list((R[:, 0] - gd) * 3) + [R[2, 1] * 3]
    return res


def solve(p, h, pitch, seeds):
    gd = math.cos(math.radians(pitch)) * h - math.sin(math.radians(pitch)) * np.array([0, 0, 1.0])
    res = pose_res(p, gd, pitch)
    best = None
    lim = LIM.copy()
    for sd in seeds:
        s = least_squares(res, np.clip(sd, lim[:, 0] + 1e-4, lim[:, 1] - 1e-4), bounds=(lim[:, 0], lim[:, 1]), max_nfev=300)
        q = s.x
        P, R = fk(q)
        if np.linalg.norm(P - p) < 0.001 and R[:, 0] @ gd > 0.9998:
            m = min(min(q[i] - lim[i, 0], lim[i, 1] - q[i]) for i in range(4))
            if m > 0.05 and (best is None or m > best[0]):
                best = (m, q)
    return None if best is None else best[1]


def seeds_for(th, q_prev=None):
    out = [] if q_prev is None else [q_prev]
    for a in (-1.3, -0.8, -0.3, 0.3):
        for b in (-0.4, 0.4, 1.0):
            for c in (-1.5, 0.0, 1.0, 1.7):
                out.append(np.array([th, a, b, c, 0.0]))
    return out


def held_pts(q, pc_h, e_h):
    """손가락 끝 · 쥔 퍽 아래쪽 점 (base)."""
    P, R = fk(q)
    gx, fy, fz = R[:, 0], R[:, 1], R[:, 2]
    pts = [P + gx * da + fy * db + fz * dc for da in (-0.02, 0.0, 0.015) for db in (-0.025, 0.025) for dc in (-0.03, 0.0)]
    c = P + R @ pc_h
    a = R @ e_h
    pts += [c - a * 0.03 + d for d in (np.array([0.015, 0, 0]), np.array([-0.015, 0, 0]), np.array([0, 0.015, 0]), np.array([0, -0.015, 0]))]
    return pts


def main():
    tr = json.load(open(os.path.join(HERE, "pallet_place_traj.json")))
    slots = tr["slots"]
    tops = np.array([[s["x"], s["y"], s["floor_z"] + PUCK_H] for s in slots])  # 꽉 찬 팔레트 퍽 위
    out = {"note": __doc__.strip().splitlines()[0], "pitches": {}}
    for pitch in (60, 45):
        rows = []
        for si, s in enumerate(slots):
            x, y, fl = s["x"], s["y"], s["floor_z"]
            h = np.array([x - J1X, y, 0.0]); h /= np.linalg.norm(h)
            th = math.atan2(h[1], h[0])
            pc = np.array([x, y, fl + PUCK_H / 2])
            G = pc + np.array([0, 0, GRASP_DZ])
            q1 = solve(G, h, pitch, seeds_for(th))
            row = {"slot": f"{s['row']}-{s['index']}", "shape": s["shape"], "x": x, "y": y, "floor_z": fl, "ok": False}
            if q1 is None:
                row["why"] = "쥐기 IK"; rows.append(row); continue
            q0 = solve(G + np.array([0, 0, 0.06]), h, pitch, seeds_for(th, q1))
            q2 = None
            for lift_ in (LIFT, 0.085, 0.07):  # 퍽 바닥이 다른 칸 퍽 위 (칸 바닥 + 60) 보다 10 mm 넘게 위
                q2 = solve(G + np.array([0, 0, lift_]), h, pitch, seeds_for(th, q1))
                if q2 is not None:
                    break
            if q0 is None or q2 is None:
                row["why"] = "위 · 들기 IK"; rows.append(row); continue
            q3 = None
            for hb in HIGH_BOTTOM:
                for hr in HIGH_R:
                    p3 = np.array([J1X, 0, 0]) + h * hr; p3[2] = hb + PUCK_H / 2 + GRASP_DZ
                    q3 = solve(p3, h, pitch, seeds_for(th, q2))
                    if q3 is not None:
                        break
                if q3 is not None:
                    break
            if q3 is None:
                row["why"] = "높이 자세 IK"; rows.append(row); continue
            q4 = q3.copy(); q4[0] = 0.0
            q5 = np.array(HAND[pitch])
            path = [q0, q1, q2, q3, q4, q5]
            # 손 안 퍽 (집게 좌표)
            P1, R1 = fk(q1)
            pc_h = R1.T @ (pc - P1); e_h = R1.T @ np.array([0, 0, 1.0])
            # 검사: 자기 충돌 (퍽 포함), 다른 칸 퍽 위 여유 (쥔 뒤 구간)
            worst_self, worst_clr = 9.0, 9.0
            for a in range(len(path) - 1):
                for t in np.linspace(0, 1, 12):
                    q = path[a] + (path[a + 1] - path[a]) * t
                    worst_self = min(worst_self, self_dist(q, (pc_h + np.array([0.14, 0, 0]), e_h) if a >= 1 else None))  # selfcol 은 손목 롤 기준 (잡는 점 = +0.14)
                    if a >= 2:  # 들기 끝부터 (쥐고 드는 동안은 옆 칸 퍽 옆을 지나감)
                        for pt in held_pts(q, pc_h, e_h):
                            dxy = np.hypot(tops[:, 0] - pt[0], tops[:, 1] - pt[1])
                            near = (dxy < 0.035)
                            near[si] = False  # 꺼낸 칸
                            if near.any():
                                worst_clr = min(worst_clr, float(pt[2] - tops[near, 2].max()))
            row.update({"ok": bool(worst_self > 0.055 and worst_clr > 0.008), "self_mm": round(worst_self * 1000), "clear_mm": round(worst_clr * 1000) if worst_clr < 9 else None,
                        "j1": round(th, 4), "q": [[round(float(v), 5) for v in qq] for qq in path]})
            if not row["ok"]:
                row["why"] = "자기 충돌" if worst_self <= 0.055 else "다른 칸 퍽 위 여유"
            rows.append(row)
        out["pitches"][str(pitch)] = rows
        ok = sum(r["ok"] for r in rows)
        print(f"피치 {pitch}°: {ok}/12 칸 | " + " ".join(f"{r['slot']}:{'O' if r['ok'] else 'X(' + r.get('why', '') + ')'}{'' if 'self_mm' not in r else '[' + str(r['self_mm']) + ',' + str(r['clear_mm']) + ']'}" for r in rows))
    json.dump(out, open(os.path.join(HERE, "unload_traj.json"), "w"))


if __name__ == "__main__":
    main()
