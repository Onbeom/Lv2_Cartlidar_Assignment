"""maze USD 구조·범위 읽기(읽기만)."""
import sys, json
from pxr import Usd, UsdGeom
st = Usd.Stage.Open(sys.argv[1])
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'render', 'proxy', 'guide'])
root = st.GetDefaultPrim()
print('root', root.GetPath(), 'mpu', UsdGeom.GetStageMetersPerUnit(st), 'up', UsdGeom.GetStageUpAxis(st))
def box(p):
    r = cache.ComputeWorldBound(p).ComputeAlignedRange()
    return [round(v, 4) for v in list(r.GetMin()) + list(r.GetMax())]
print('all', box(root))
for c in root.GetChildren():
    n = sum(1 for _ in Usd.PrimRange(c))
    meshes = [p for p in Usd.PrimRange(c) if p.IsA(UsdGeom.Mesh)]
    print(c.GetName(), c.GetTypeName(), 'prims', n, 'meshes', len(meshes), 'bbox', box(c))
    for g in c.GetChildren():
        print('   ', g.GetName(), g.GetTypeName(), box(g), [a.GetName() for a in g.GetAppliedSchemas()][:3] if hasattr(g,'GetAppliedSchemas') else '')
