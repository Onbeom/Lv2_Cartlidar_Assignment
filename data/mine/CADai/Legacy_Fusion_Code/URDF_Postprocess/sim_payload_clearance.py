"""Remove convex approximation intrusion into CAD-verified payload columns only."""
from pathlib import Path
import sys,json
sys.path.insert(0,str(Path('.caddeps').resolve()))
import numpy as np,trimesh
from scipy.spatial import ConvexHull,QhullError
from scipy.optimize import linprog
W=Path('sim_work');D=W/'convex';rows=json.loads((W/'decomposition.json').read_text());native=json.loads((W/'native_sources/manifest.json').read_text());bodies=json.loads(Path('Waffle_Handoff_20261020/body_physics_geometry.json').read_text(encoding='utf8'));payload=json.loads(Path('Waffle_Handoff_20261020/MAX_12_PAYLOAD_AUDIT.json').read_text())['actual_12_payload_centers']
cols=[]
for channel,idx in [('B',22),('G',140),('R',141)]:
    src=next(x for x in native if x['link']=='base_link'and x['component_index']==idx);b=next(x for x in bodies if x['path']==src['path']);R=np.array(b['transform_cm']).reshape(4,4)[:3,:3]
    pts=np.array([r['center_world_mm']for r in payload if r['channel']==channel]);center=pts.mean(0)/1000;half=np.array([15.05,15.05,120.05])/1000
    axes=np.r_[R.T,-R.T];eq=np.c_[axes,-axes@center-np.r_[half,half]];cols.append((channel,eq))
def overlap(V,eq):
    h=ConvexHull(V);a=np.r_[h.equations,eq];res=linprog([0,0,0,-1],A_ub=np.c_[a[:,:3],np.ones(len(a))],b_ub=-a[:,3],bounds=[(None,None)]*3+[(0,None)],method='highs')
    return res.success and res.x[3]>1e-8
def clip(V,eq):
    dist=V@eq[:3]+eq[3];inside=dist<=1e-10
    if inside.all():return V
    if not inside.any():return None
    h=ConvexHull(V);edges=set(tuple(sorted((int(a),int(b))))for t in h.simplices for a,b in zip(t,np.roll(t,1)))
    points=list(V[inside])
    for a,b in edges:
        if inside[a]!=inside[b]:points.append(V[a]+dist[a]/(dist[a]-dist[b])*(V[b]-V[a]))
    p=np.unique(np.round(points,12),axis=0)
    try:h=ConvexHull(p)
    except QhullError:return None
    return p[h.vertices]if h.volume>1e-14 else None
changed=[]
for row in rows:
    if row['link']!='base_link':continue
    new=[]
    for filename in row['files']:
        V=np.array(trimesh.load(D/filename,force='mesh').vertices);parts=[V];hits=[]
        for name,eqs in cols:
            output=[]
            for v in parts:
                if not overlap(v,eqs):output.append(v);continue
                hits.append(name);remaining=v
                for eq in eqs:
                    if remaining is None:break
                    outside=clip(remaining,-eq)
                    if outside is not None:output.append(outside)
                    remaining=clip(remaining,eq)
            parts=output
        if not hits:new.append(filename);continue
        files=[]
        for i,p in enumerate(parts):
            m=trimesh.Trimesh(vertices=p,process=False).convex_hull;name=Path(filename).stem+f'_free{i:02}.stl';m.export(D/name);new.append(name);files.append(name)
        changed.append(dict(original=filename,channels=sorted(set(hits)),replacement=files))
    row['files']=new;row['hulls']=len(new)
# Post-check all remaining base hulls avoid the column with positive overlap radius.
for row in rows:
    if row['link']=='base_link':
        for name in row['files']:
            v=np.array(trimesh.load(D/name,force='mesh').vertices)
            assert not any(overlap(v,eq)for _,eq in cols),name
(W/'decomposition.json').write_text(json.dumps(rows,indent=2))
out=dict(status='PASS_CONVEX_STORAGE_COLUMN_CLEARANCE',scope='Collision approximation only, not CAD modification or continuous grasp validation',basis='12 native CAD payload placements had no solid intersections. Clear vertical 30.1 x30.1 x240.1mm corridor per channel from these placements.',changed_hulls=changed,clearance_xy_per_side_mm=.05,real_geometry_altered=False)
(W/'payload_collision_audit.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))

