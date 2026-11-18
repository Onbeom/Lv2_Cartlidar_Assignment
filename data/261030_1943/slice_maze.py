"""새 maze USD 충돌 메시를 높이 z에서 잘라 벽 선분과 기둥 점을 낸다(읽기만). 이전 발제 도면과 겹쳐 그린다."""
import json, sys
import numpy as np
from pxr import Usd, UsdGeom
SRC = 'maze_src/maze/maze.usda'
Z = float(sys.argv[1]) if len(sys.argv) > 1 else 0.2
st = Usd.Stage.Open(SRC)
segs = []
groups = {}
for p in Usd.PrimRange(st.GetDefaultPrim()):
    if not p.IsA(UsdGeom.Mesh) or '/collision/' not in str(p.GetPath()):
        continue
    m = UsdGeom.Mesh(p)
    pts = np.array(m.GetPointsAttr().Get(), float)
    xf = np.array(UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
    pts = (np.c_[pts, np.ones(len(pts))] @ xf)[:, :3]
    counts = m.GetFaceVertexCountsAttr().Get(); idx = m.GetFaceVertexIndicesAttr().Get()
    k = 0
    g = str(p.GetPath()).split('/')[2]
    for c in counts:
        poly = pts[list(idx[k:k + c])]; k += c
        for i in range(1, c - 1):
            tri = poly[[0, i, i + 1]]
            d = tri[:, 2] - Z
            if (d > 0).all() or (d < 0).all():
                continue
            cut = []
            for a, b in [(0, 1), (1, 2), (2, 0)]:
                if (d[a] > 0) != (d[b] > 0):
                    t = d[a] / (d[a] - d[b]); cut.append(tri[a, :2] + t * (tri[b, :2] - tri[a, :2]))
            if len(cut) == 2:
                segs.append((g, cut[0].tolist(), cut[1].tolist()))
out = {'source': SRC, 'z': Z, 'n_segments': len(segs), 'segments': segs}
json.dump(out, open(f'maze_slice_z{Z:.2f}.json', 'w'))
a = np.array([s[1] + s[2] for s in segs])
print('segments', len(segs), 'x', a[:, [0, 2]].min(), a[:, [0, 2]].max(), 'y', a[:, [1, 3]].min(), a[:, [1, 3]].max())
