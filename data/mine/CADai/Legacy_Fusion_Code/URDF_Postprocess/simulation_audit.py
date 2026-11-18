"""Independent static audit; no ROS/Isaac runtime claims. Python + NumPy + trimesh."""
from pathlib import Path
import sys,json,math,xml.etree.ElementTree as ET
import numpy as np
HERE=Path(__file__).resolve().parent
for dep in [HERE/'.caddeps',Path.cwd()/'.caddeps']:
    if dep.exists():sys.path.insert(0,str(dep))
import trimesh
ROOT=Path(r'C:\Users\qjaqj\OneDrive\Desktop\Project2\261020\braket\submit\urdf')
if HERE.name=='scripts':ROOT=HERE.parent
PKG=ROOT/'waffle_4wd_description_261021'
CAND=Path(sys.argv[1])if len(sys.argv)>1 else PKG/'urdf'
COL=Path(sys.argv[2])if len(sys.argv)>2 else PKG/'meshes/collision'
review=json.loads((ROOT/'data/ROBOT_REVIEW_PENDING.json').read_text(encoding='utf8'))
root=ET.parse(CAND/'waffle_4wd.urdf').getroot()
L={n.get('name'):n for n in root.findall('link')};J={n.get('name'):n for n in root.findall('joint')}
def T(n):
    out=np.eye(4)
    if n is None:return out
    r,p,y=map(float,n.get('rpy','0 0 0').split());cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    out[:3,:3]=[[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]
    out[:3,3]=list(map(float,n.get('xyz','0 0 0').split()));return out
children=[j.find('child').get('link')for j in J.values()]
assert len(L)==23 and len(J)==22 and len(set(children))==22
assert set(L)-set(children)=={'base_footprint'}
F={'base_footprint':np.eye(4)}
for _ in range(23):
    for j in J.values():
        p,c=j.find('parent').get('link'),j.find('child').get('link')
        if p in F:F[c]=F[p]@T(j.find('origin'))
assert set(F)==set(L)
shift=F['base_link'][:3,3]*1000;mass=0;physical=[];meshes=[];visual=[]
def tensor(v):
    a,b,c,d,e,f=v;return np.array([[a,d,e],[d,b,f],[e,f,c]])
refs={r['name']:r for r in review['links']}
physics=json.loads((ROOT/'data/PHYSICS_ESTIMATE_G_MM.json').read_text(encoding='utf8'))['bodies']
ns={'c':'http://www.collada.org/2005/11/COLLADASchema'}
for name,node in L.items():
    inert=node.find('inertial')
    if name not in refs:
        assert inert is None and node.find('visual')is None and node.find('collision')is None
        continue
    assert inert is not None
    m=float(inert.find('mass').get('value'))*1000;mass+=m;IF=F[name]@T(inert.find('origin'));ref=refs[name]
    a=inert.find('inertia');I=tensor([float(a.get(k))for k in ['ixx','iyy','izz','ixy','ixz','iyz']])*1e9;I=IF[:3,:3]@I@IF[:3,:3].T
    me=abs(m-ref['mass_g']);ce=float(np.max(abs(IF[:3,3]*1000-ref['com_world_mm']-shift)));ie=float(np.max(abs(I-tensor(ref['inertia_com_world_g_mm2']))))
    eig=np.linalg.eigvalsh(I);assert me<1e-8 and ce<1e-6 and ie<1e-5 and min(eig)>0 and max(eig)<=sum(eig)-max(eig)+1e-6,(name,me,ce,ie)
    physical.append(dict(link=name,mass_g=m,mass_error_g=me,com_error_mm=ce,inertia_error_g_mm2=ie))
    assert node.findall('visual')and node.findall('collision'),name
    vis=node.find('visual');mesh=vis.find('geometry/mesh');file=PKG/mesh.get('filename').split('/',3)[3];dae=ET.parse(file).getroot();arrays=[]
    assert float(dae.find('c:asset/c:unit',ns).get('meter'))==1
    for g in dae.findall('c:library_geometries/c:geometry',ns):
        gm=g.find('c:mesh',ns);src=gm.find('c:vertices/c:input',ns).get('source')[1:];arr=next(a for a in gm.findall('c:source',ns)if a.get('id')==src).find('c:float_array',ns);arrays.append(np.fromstring(arr.text,sep=' ').reshape(-1,3))
    V=np.concatenate(arrays)*np.array(list(map(float,mesh.get('scale','1 1 1').split())));A=F[name]@T(vis.find('origin'));V=(V@A[:3,:3].T+A[:3,3])*1000
    rr=[r['bounds']for r in physics if r['link']==name];lo=np.min([r['min']for r in rr],0)+shift;hi=np.max([r['max']for r in rr],0)+shift;err=float(max(np.max(abs(V.min(0)-lo)),np.max(abs(V.max(0)-hi))));assert err<.002,(name,err)
    visual.append(dict(link=name,bounds_error_mm=err))
    for c in node.findall('collision'):
        g=c.find('geometry');assert len(g)==1
        if g[0].tag=='mesh':
            p=COL/Path(g[0].get('filename')).name;M=trimesh.load(p,force='mesh');assert M.is_watertight and M.is_convex and len(M.vertices)<=255 and M.volume>0,(p,len(M.vertices))
            assert np.max(M.extents)<1 and g[0].get('scale')=='1 1 1'
            meshes.append(dict(file=p.name,vertices=len(M.vertices),volume_mm3=float(M.volume*1e9)))
        else:
            vals=list(map(float,g[0].get('size').split()))if g[0].tag=='box'else[float(g[0].get('radius')),float(g[0].get('length'))];assert min(vals)>0
for ref in review['joints']:
    j=J[ref['name']];assert j.find('parent').get('link')==ref['parent']and j.find('child').get('link')==ref['child']
    if j.get('type')=='fixed':continue
    axis=F[ref['child']][:3,:3]@np.array(list(map(float,j.find('axis').get('xyz').split())))
    expected=np.array(refs[ref['child']]['link_frame_world_mm'])[:3,:3]@np.array(ref['axis_child']);assert max(abs(axis-expected))<1e-6
    lim=j.find('limit');assert float(lim.get('effort'))>0 and float(lim.get('velocity'))>0
    if ref['name'].startswith('RMX52_'):
        assert j.get('type')=='revolute'and np.allclose([float(lim.get(k))for k in ['lower','upper']],ref['proposed_limits_rad'])and float(lim.get('effort'))==3.8
    if ref['name'].startswith('Wheel_'):assert float(lim.get('effort'))==2.7
mimic=J['gripper_right_joint'].find('mimic');assert mimic.get('joint')=='gripper_left_joint'and float(mimic.get('multiplier'))==-1
gap0=19.62773936566
gap=[gap0+2000*float(J['gripper_left_joint'].find('limit').get(k))for k in ['lower','upper']];assert np.allclose(gap,[16.8,72],atol=1e-8)
# All optical frames use z-forward with an orthonormal right-handed basis.
for n in ['camera_optical_frame','top_camera_front_optical_frame','top_camera_rear_optical_frame']:
    assert np.allclose(F[n][:3,:3].T@F[n][:3,:3],np.eye(3))and np.linalg.det(F[n][:3,:3])>.999
assert np.dot(F['camera_optical_frame'][:3,2],[.7566434194854301,0,-.6538277569432146])>.999999
# Native source wheel minimum is lifted to the footprint's ground plane.
ground=min(r['bounds']['min'][2]for r in physics if r['link'].startswith('wheel_'))+shift[2];assert abs(ground)<1e-8
macro=ET.parse(CAND/'main.urdf.xacro'if (CAND/'main.urdf.xacro').exists()else CAND/'assemblies/main.urdf.xacro').getroot().find('{http://www.ros.org/wiki/xacro}macro')
for tag in ['link','joint']:
    assert len(macro.findall(tag))==len(root.findall(tag))
    for a,b in zip(root.findall(tag),macro.findall(tag)):
        assert b.get('name')=='${prefix}'+a.get('name')
        for k in ['inertial','limit','geometry','axis','origin']:
            aa,bb=a.find(k),b.find(k)
            if aa is not None:assert ET.tostring(aa)==ET.tostring(bb)
assert abs(mass-review['total_mass_g'])<1e-8
out=dict(status='PASS_STATIC_NUMERIC_CHECKS_RUNTIME_PENDING',links=len(L),joints=len(J),physical_links=len(physical),root='base_footprint',total_mass_g=mass,engineering_origin_shift_under_footprint_mm=shift.tolist(),physical=physical,visual=visual,convex_mesh_count=len(meshes),convex_meshes=meshes,gripper_gap_limits_mm=gap,ros_runtime_tested=False,isaac_runtime_tested=False,collision_accuracy_certified=False)
destination=ROOT/'data/simulation_audit.json'if HERE.name=='scripts'else Path('sim_work/simulation_audit.json')
destination.parent.mkdir(parents=True,exist_ok=True)
destination.write_text(json.dumps(out,indent=2),encoding='utf8')
print(json.dumps({k:v for k,v in out.items()if k not in ['physical','visual','convex_meshes']},indent=2))
