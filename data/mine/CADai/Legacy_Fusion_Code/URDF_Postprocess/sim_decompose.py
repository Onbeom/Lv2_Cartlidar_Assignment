from pathlib import Path
import sys,json,time,os
os.environ.setdefault('OMP_NUM_THREADS','4')
import numpy as np
sys.path.insert(0,str(Path('.caddeps').resolve()))
import coacd,trimesh
coacd.set_log_level('warn')
root=Path('sim_work');out=root/'convex';out.mkdir(exist_ok=True)
meta=json.loads((root/'mesh_components.json').read_text(encoding='utf8'));tasks=[]
for l in meta:
    for p in l['components']:
        path=p['nearest_path'];vol=p['volume_mm3']
        if l['link']=='base_link'and vol>10000 and any(x in path for x in ['housing:','magazine_','Chassis_Bracket_','rail_','translator 8.0 1:2']):tasks.append((l['link'],p))
        if l['link'].startswith('gripper_')and vol>1000:tasks.append((l['link'],p))
results=[]
for link,p in tasks:
    key=link+f"_part{p['index']:03}";marker=out/(key+'.json')
    if marker.exists():results.append(json.loads(marker.read_text(encoding='utf8')));continue
    source=root/'native_sources'/(key+'.stl')
    if not source.exists():raise RuntimeError('Need watertight native CAD source '+str(source))
    M=trimesh.load_mesh(source,process=True);M.apply_scale(.001)
    M.merge_vertices(digits_vertex=8);M.update_faces(M.unique_faces());M.remove_unreferenced_vertices()
    if not M.is_watertight:raise RuntimeError('Native source is not closed '+key)
    start=time.monotonic();print('decompose',key,p['nearest_path'],M.is_watertight,flush=True)
    parts=coacd.run_coacd(coacd.Mesh(np.asarray(M.vertices,dtype=np.float64),np.asarray(M.faces,dtype=np.int32)),threshold=.001,real_metric=True,max_convex_hull=32,preprocess_mode='off',resolution=1000,mcts_nodes=5,mcts_iterations=16,mcts_max_depth=3,decimate=True,max_ch_vertex=64,seed=0)
    files=[]
    for i,(v,f)in enumerate(parts):
        hull=trimesh.Trimesh(v,f,process=True).convex_hull
        path=out/(key+f'_hull{i:02}.stl');hull.export(path);files.append(path.name)
    row=dict(link=link,component_index=p['index'],source_path=p['nearest_path'],files=files,hulls=len(files),time_s=time.monotonic()-start,threshold_m=.001,native_closed_source=True)
    marker.write_text(json.dumps(row,ensure_ascii=False,indent=2),encoding='utf8');results.append(row)
    (root/'decomposition_progress.json').write_text(json.dumps(dict(completed=len(results),total=len(tasks),last=row),ensure_ascii=False,indent=2),encoding='utf8')
(root/'decomposition.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf8')
print('complete',len(results),'parts',sum(x['hulls']for x in results),'hulls',flush=True)
