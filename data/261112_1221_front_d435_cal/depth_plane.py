#!/usr/bin/env python3
"""앞 D435 깊이로 로봇이 선 면(책상 · 바닥)을 맞춰 카메라 롤 · 피치 오차를 잰다 (PC, 컬러 태그와 따로인 교차 확인).
depth_plane.py <폴더> [<폴더> ...] [--xyz .. --rpy ..] [--zmax 0.04]
- 깊이 → 깊이 광학 → <이름>_link(드라이버 TF) → base_link(URDF) → base_footprint.
- 명목 z 가 −0.15 ~ zmax 인 점(로봇이 선 면 근처, 노트북처럼 높이 놓인 물건은 대부분 빠짐)에서 RANSAC 평면.
- 평면 법선을 z 로 돌리는 각이 오차다(front_cal 과 같은 뜻, 요는 못 잼). 깊이 배율 오차는 평면 방향을 바꾸지 않는다."""
import argparse, json, math, os
import numpy as np
import front_cal as F


def solve(folder, xyz, rpy, zmax, seed=0):
    meta = json.load(open(os.path.join(folder, 'meta.json')))
    D = np.load(os.path.join(folder, 'depth.npz'))['depth'].astype(np.float32) * 1e-3
    d = np.median(np.where(D > 0, D, np.nan), axis=0)
    K = np.array(meta['K']).reshape(3, 3)
    v, u = np.mgrid[0:d.shape[0]:2, 0:d.shape[1]:2]
    z = d[v, u]; ok = np.isfinite(z) & (z > 0.2) & (z < 2.0)
    u, v, z = u[ok], v[ok], z[ok]
    P = np.stack([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z], 1)
    Tn, _ = F.T_bf_opt(xyz, rpy, meta.get('T_link_optical') or F.DEF_LINK_OPT)
    c = Tn[:3, 3]
    Q = (Tn[:3, :3] @ P.T).T + Tn[:3, 3]
    sel = (Q[:, 2] > -0.15) & (Q[:, 2] < zmax)
    Q = Q[sel]
    rng = np.random.default_rng(seed); best = None
    for _ in range(400):
        a, b, e = Q[rng.choice(len(Q), 3, replace=False)]
        n = np.cross(b - a, e - a); nn = np.linalg.norm(n)
        if nn < 1e-9: continue
        n /= nn; inl = np.abs((Q - a) @ n) < 0.006
        if best is None or inl.sum() > best.sum(): best = inl
    R = Q[best]
    for _ in range(3):
        m = R.mean(0); _, _, Vt = np.linalg.svd(R - m, full_matrices=False); n = Vt[2] * (1 if Vt[2][2] > 0 else -1)
        res = (Q - m) @ n; inl = np.abs(res) < 0.004; R = Q[inl]
    m = R.mean(0); _, _, Vt = np.linalg.svd(R - m, full_matrices=False); n = Vt[2] * (1 if Vt[2][2] > 0 else -1)
    RD = F.rot_a_to_b(n, np.array([0, 0, 1.0]))
    e = [math.degrees(a) for a in F.R2rpy(RD)]
    h = float(n @ (m - c))   # 카메라 → 평면 거리 (법선 방향, 음수)
    rms = float(np.sqrt(np.mean(((R - m) @ n) ** 2)) * 1e3)
    return {'folder': folder, 'n_pts': int(len(R)), 'frac': float(len(R) / len(Q)), 'roll': e[0], 'pitch': e[1],
            'cam_height_mm': -h * 1e3, 'plane_rms_mm': rms, 'x_range': [float(R[:, 0].min()), float(R[:, 0].max())]}


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('folders', nargs='+')
    ap.add_argument('--xyz', type=float, nargs=3, default=F.URDF_XYZ)
    ap.add_argument('--rpy', type=float, nargs=3, default=F.URDF_RPY)
    ap.add_argument('--zmax', type=float, default=0.04)
    a = ap.parse_args()
    for f in a.folders:
        r = solve(f, a.xyz, a.rpy, a.zmax)
        print(f"{os.path.basename(f.rstrip('/'))}: 롤 {r['roll']:+.2f}° 피치 {r['pitch']:+.2f}° | 평면 점 {r['n_pts']} ({r['frac']*100:.0f} %), "
              f"잔차 {r['plane_rms_mm']:.1f} mm, 카메라 높이 {r['cam_height_mm']:.1f} mm, x {r['x_range'][0]:.2f} ~ {r['x_range'][1]:.2f} m")
