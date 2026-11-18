"""새 maze USD 충돌 원기둥(벽 끝 · 모서리 프로파일 기둥)의 중심 · 반지름 · 높이를 낸다(읽기만, USD 좌표).
slice_maze.py 는 충돌 Mesh 만 잘라서 이 원기둥 110 개가 maze_slice_z*.json 에 없었음(261101 설계 25 출발 통로 막힘 조사에서 찾음)."""
import json
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics
SRC = 'maze_src/maze/maze.usda'
st = Usd.Stage.Open(SRC)
out = []
for p in Usd.PrimRange(st.GetDefaultPrim()):
    if not (p.IsA(UsdGeom.Cylinder) and p.HasAPI(UsdPhysics.CollisionAPI)):
        continue
    c = UsdGeom.Cylinder(p)
    xf = np.array(UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
    ax = {'X': 0, 'Y': 1, 'Z': 2}[c.GetAxisAttr().Get()]
    sc = np.linalg.norm(xf[:3, :3], axis=1)
    centre = xf[3, :3]
    axis_w = xf[ax, :3] / sc[ax]
    r_scale = max(sc[i] for i in range(3) if i != ax)
    out.append({'name': str(p.GetPath()).split('/')[2], 'prim': str(p.GetPath()), 'center': [float(centre[0]), float(centre[1])],
                'z': [float(centre[2] - c.GetHeightAttr().Get() * sc[ax] / 2), float(centre[2] + c.GetHeightAttr().Get() * sc[ax] / 2)],
                'radius': float(c.GetRadiusAttr().Get() * r_scale), 'axis_world': [round(float(v), 4) for v in axis_w]})
json.dump({'source': SRC, 'n_posts': len(out), 'posts': out}, open('maze_posts.json', 'w'), indent=1)
r = np.array([o['radius'] for o in out])
print('posts', len(out), 'radius', r.min().round(4), r.max().round(4), 'axis', sorted({tuple(o['axis_world']) for o in out}))
