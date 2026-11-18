from pathlib import Path
import sys,json
sys.path.insert(0,str(Path('.caddeps').resolve()))
import numpy as np,trimesh
D=Path('sim_work/convex');rows=json.loads(Path('sim_work/decomposition.json').read_text());repairs=[]
for r in rows:
    for name in r['files']:
        p=D/name;m=trimesh.load(p,force='mesh')
        if m.is_watertight and m.is_convex and m.volume>0:continue
        old=m.vertices.copy()
        for resolution in [1e-7,1e-6,1e-5]:
            v=np.unique(np.round(old/resolution)*resolution,axis=0)
            try:h=trimesh.Trimesh(vertices=v,process=False).convex_hull;h.export(p);m=trimesh.load(p,force='mesh')
            except Exception:continue
            if m.is_watertight and m.is_convex and m.volume>0:break
        assert m.is_watertight and m.is_convex and m.volume>0,name
        repairs.append(dict(file=name,rounding_grid_mm=resolution*1000,bounds_change_mm=float(np.max(abs(np.array([old.min(0),old.max(0)])-m.bounds))*1000)))
Path('sim_work/convex_mesh_repairs.json').write_text(json.dumps(repairs,indent=2));print('Repaired',len(repairs),'hulls')
