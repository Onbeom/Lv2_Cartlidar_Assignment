"""Isaac 미로 USD 충돌 형상을 라이다 높이 근처(z 0.06~0.11)에서 잘라 윤곽 점으로 저장(발제 좌표 = USD + USD_OFFSET). pxr 필요."""
import sys, numpy as np
from pxr import Usd, UsdGeom, UsdPhysics
OX, OY = 2.2345, 3.2136
maze, out = sys.argv[1], sys.argv[2]
ZL, ZH = 0.06, 0.11
st = Usd.Stage.Open(maze)
pts_out, kinds = [], {}
def seg(a, b):
    n = max(2, int(np.hypot(*(b - a)) / 0.0025) + 1)
    t = np.linspace(0, 1, n)[:, None]
    return a + (b - a) * t
for p in Usd.PrimRange(st.GetPseudoRoot()):
    if not p.HasAPI(UsdPhysics.CollisionAPI):
        continue
    kinds[p.GetTypeName()] = kinds.get(p.GetTypeName(), 0) + 1
    xf = np.array(UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
    if p.IsA(UsdGeom.Mesh):
        m = UsdGeom.Mesh(p)
        v = np.array(m.GetPointsAttr().Get(), float)
        v = (np.c_[v, np.ones(len(v))] @ xf)[:, :3]
        idx, cnt, k = m.GetFaceVertexIndicesAttr().Get(), m.GetFaceVertexCountsAttr().Get(), 0
        for c in cnt:
            poly = v[list(idx[k:k + c])]; k += c
            if poly[:, 2].max() < ZL or poly[:, 2].min() > ZH:
                continue
            for e in range(c):
                pts_out.append(seg(poly[e, :2], poly[(e + 1) % c, :2]))
    elif p.IsA(UsdGeom.Cylinder):
        cy = UsdGeom.Cylinder(p)
        r = cy.GetRadiusAttr().Get() * max(np.linalg.norm(xf[:3, :3], axis=1)[:2])
        th = np.linspace(0, 2 * np.pi, max(16, int(2 * np.pi * r / 0.0025)), endpoint=False)
        pts_out.append(np.c_[xf[3, 0] + r * np.cos(th), xf[3, 1] + r * np.sin(th)])
pts = np.concatenate(pts_out) + [OX, OY]
np.save(out, pts)
print('kinds', kinds, 'n', len(pts), 'x', pts[:, 0].min(), pts[:, 0].max(), 'y', pts[:, 1].min(), pts[:, 1].max())
