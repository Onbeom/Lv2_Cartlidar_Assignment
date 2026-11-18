"""이전(261022_2119)·새(261027_1530) MuJoCo 모델의 평면 외곽·무게중심·회전 쓸림 반경 비교(정적 계산, 시험 아님).
사용(PC): $MJ_PY tools/geom_compare.py <이전 model.xml> <새 model.xml>
R_full(c) = |c| + 회전 중심 c(차체 x축 위)에서 가장 먼 외곽점까지 평면 거리. c는 이전 V1에서 맞춘 회전 중심 값을 넣어 본다.
"""
import sys
import numpy as np
import mujoco

ARM = ["RMX52_Base_Yaw", "RMX52_Shoulder", "RMX52_Elbow", "RMX52_Wrist_Pitch", "RMX52_Wrist_Roll_Reversed"]
S0 = [0.0, 1.4, -1.0, -1.3, 0.0]
PIVOTS = (0.0, -0.035, -0.052, -0.063, -0.0797)   # 이전 V1 회전 중심 x: MuJoCo −52.0 mm, Isaac −79.7 mm; −35·−63 = 무게중심 이동만큼 앞으로 온 경우(가정)


def info(path, label):
    m = mujoco.MjModel.from_xml_path(path)
    d = mujoco.MjData(m)
    adr = {n: m.jnt_qposadr[m.joint(n).id] for n in ARM}
    print(label, "질량", round(float(sum(m.body_mass)), 4), "kg")
    for pose_name, q in (("0", [0] * 5), ("S0", S0)):
        d.qpos[:] = 0
        d.qpos[3] = 1
        for n, v in zip(ARM, q):
            d.qpos[adr[n]] = v
        mujoco.mj_forward(m, d)
        pts = []
        for g in range(m.ngeom):
            if m.geom_bodyid[g] == 0:
                continue
            R, c, s, t = d.geom_xmat[g].reshape(3, 3), d.geom_xpos[g], m.geom_size[g], m.geom_type[g]
            if t == mujoco.mjtGeom.mjGEOM_MESH:
                mid = m.geom_dataid[g]
                a, k = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
                pts.append(m.mesh_vert[a:a + k] @ R.T + c)
            elif t == mujoco.mjtGeom.mjGEOM_BOX:
                pts.append(np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]) * s @ R.T + c)
            else:
                ang = np.linspace(0, 2 * np.pi, 72)
                ring = np.stack([np.cos(ang) * s[0], np.sin(ang) * s[0], np.zeros_like(ang)], 1)
                pts.append(np.vstack([ring + [0, 0, s[1]], ring - [0, 0, s[1]]]) @ R.T + c)
        P = np.vstack(pts)[:, :2]
        com = d.subtree_com[1]
        rf = {round(cx * 1000, 1): round((abs(cx) + float(np.max(np.hypot(P[:, 0] - cx, P[:, 1])))) * 1000, 1) for cx in PIVOTS}
        front = (0.08 + com[0]) / 0.16   # 정적 앞바퀴 하중 비율(바퀴 x ±0.08)
        print(f"  자세 {pose_name}: x {P[:, 0].min() * 1000:.1f}~{P[:, 0].max() * 1000:.1f} mm, "
              f"y {P[:, 1].min() * 1000:.1f}~{P[:, 1].max() * 1000:.1f} mm, 무게중심 {np.round(com * 1000, 1)} mm, "
              f"앞바퀴 하중 비율 {front:.3f}, R_full(회전 중심 x mm → mm) {rf}")


info(sys.argv[1], "이전 261022_2119")
info(sys.argv[2], "새 261027_1530")
