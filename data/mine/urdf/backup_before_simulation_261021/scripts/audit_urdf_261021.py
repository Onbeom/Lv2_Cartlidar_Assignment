import json, math, pathlib, xml.etree.ElementTree as ET
import numpy as np

ROOT=pathlib.Path(__file__).resolve().parent.parent if pathlib.Path(__file__).resolve().parent.name=='scripts' else pathlib.Path(r'C:\Users\qjaqj\OneDrive\Desktop\Project2\261020\braket\submit\urdf')
PKG=ROOT/'waffle_4wd_description_261021'
DATA=ROOT/'data' if (ROOT/'data/ROBOT_REVIEW_PENDING.json').exists()else pathlib.Path('Waffle_Handoff_20261020')
review=json.loads((DATA/'ROBOT_REVIEW_PENDING.json').read_text(encoding='utf8'))
def transform(node):
    T=np.eye(4)
    if node is None:return T
    xyz=[float(x) for x in node.get('xyz','0 0 0').split()]
    r,p,y=[float(x) for x in node.get('rpy','0 0 0').split()]
    cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    T[:3,:3]=[[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]
    T[:3,3]=xyz
    return T
def tensor(v):
    a,b,c,d,e,f=v
    return np.array([[a,d,e],[d,b,f],[e,f,c]])
paths=list((PKG/'urdf').glob('*.urdf'))
assert len(paths)==1,paths
tree=ET.parse(paths[0]);robot=tree.getroot()
links={x.get('name'):x for x in robot.findall('link')};joints={x.get('name'):x for x in robot.findall('joint')}
children=[x.find('child').get('link')for x in joints.values()]
roots=set(links)-set(children)
assert len(links)==14 and len(joints)==13 and roots=={'base_link'}
assert len(set(children))==len(children)
F={'base_link':np.eye(4)}
for _ in range(14):
    for j in joints.values():
        p=j.find('parent').get('link');c=j.find('child').get('link')
        if p in F:F[c]=F[p]@transform(j.find('origin'))
assert set(F)==set(links)
by={l['name']:l for l in review['links']};rows=[];missing=[];mass=0
for name,node in links.items():
    inert=node.find('inertial');m=float(inert.find('mass').get('value'))*1000;mass+=m
    IF=F[name]@transform(inert.find('origin'));com=IF[:3,3]*1000
    a=inert.find('inertia');I=tensor([float(a.get(k))for k in ['ixx','iyy','izz','ixy','ixz','iyz']])*1e9
    I=IF[:3,:3]@I@IF[:3,:3].T
    ref=by[name];error=float(np.max(np.abs(I-tensor(ref['inertia_com_world_g_mm2']))))
    rows.append(dict(link=name,mass_g=m,mass_error_g=m-ref['mass_g'],com_error_mm=float(np.max(np.abs(com-ref['com_world_mm']))),inertia_error_g_mm2=error,positive_inertia=bool(np.all(np.linalg.eigvalsh(I)>0))))
    for tag in ['visual','collision']:
        assert node.findall(tag),(name,tag)
        for mesh in node.findall(tag+'/geometry/mesh'):
            uri=mesh.get('filename');rel=uri.split('/',3)[3]if uri.startswith('package://')else uri
            file=PKG/rel
            if not file.is_file()or file.stat().st_size==0:missing.append(str(file))
joint_rows=[]
for ref in review['joints']:
    j=joints[ref['name']];c=j.find('child').get('link');p=j.find('parent').get('link')
    assert(c,p)==(ref['child'],ref['parent'])
    if ref['current_type']=='fixed':continue
    axis=np.array([float(x)for x in j.find('axis').get('xyz').split()]);axis=F[c][:3,:3]@axis
    expected=np.array(by[c]['link_frame_world_mm'])[:3,:3]@np.array(ref['axis_child'])
    pos=F[c][:3,3]*1000;expectedpos=np.array(by[c]['link_frame_world_mm'])[:3,3]
    joint_rows.append(dict(joint=ref['name'],axis_world=axis.tolist(),axis_error=float(np.max(np.abs(axis-expected))),origin_error_mm=float(np.max(np.abs(pos-expectedpos)))))
mimic=joints['gripper_right_joint'].find('mimic');assert mimic is not None and mimic.get('joint')=='gripper_left_joint' and float(mimic.get('multiplier'))==-1
physics=json.loads((DATA/'PHYSICS_ESTIMATE_G_MM.json').read_text(encoding='utf8'))
mesh_checks=[];ns={'c':'http://www.collada.org/2005/11/COLLADASchema'}
for name,node in links.items():
    visual=node.find('visual');mesh=visual.find('geometry/mesh');file=PKG/mesh.get('filename').split('/',3)[3]
    dae=ET.parse(file);droot=dae.getroot();unit=float(droot.find('c:asset/c:unit',ns).get('meter'))
    assert unit==1 and droot.find('c:asset/c:up_axis',ns).text=='Z_UP'
    assert not droot.findall('.//c:node/c:matrix',ns)and not droot.findall('.//c:node/c:translate',ns)
    arrays=[]
    for geom in droot.findall('c:library_geometries/c:geometry',ns):
        gm=geom.find('c:mesh',ns);src=gm.find('c:vertices/c:input',ns).get('source')[1:]
        arr=next(x for x in gm.findall('c:source',ns)if x.get('id')==src).find('c:float_array',ns)
        arrays.append(np.fromstring(arr.text,sep=' ').reshape(-1,3))
    V=np.concatenate(arrays);scale=np.array([float(x)for x in mesh.get('scale','1 1 1').split()])
    T=F[name]@transform(visual.find('origin'));V=(V*scale)@T[:3,:3].T+T[:3,3];V*=1000
    refs=[r['bounds']for r in physics['bodies']if r['link']==name]
    rmin=np.min([r['min']for r in refs],axis=0);rmax=np.max([r['max']for r in refs],axis=0)
    err=max(np.max(np.abs(V.min(0)-rmin)),np.max(np.abs(V.max(0)-rmax)))
    mesh_checks.append(dict(link=name,world_bounds_error_mm=float(err),vertices=len(V)))
macro=ET.parse(PKG/'urdf/assemblies/main.urdf.xacro').getroot()
mj=next(x for x in macro.iter('joint')if x.get('name')=='${prefix}gripper_right_joint')
assert mj.find('mimic').get('joint')=='${prefix}gripper_left_joint'and mj.find('mimic').get('multiplier')=='-1'
assert robot.get('name')=='waffle_4wd'
result=dict(urdf=str(paths[0]),links=len(links),joints=len(joints),root='base_link',mass_g=mass,link_checks=rows,joint_checks=joint_rows,mesh_checks=mesh_checks,missing_meshes=missing,xacro_mimic_present=True,ros_runtime_tested=False)
result['passed']=not missing and abs(mass-review['total_mass_g'])<.002 and all(abs(r['mass_error_g'])<.002 and r['com_error_mm']<.02 and r['inertia_error_g_mm2']<100 and r['positive_inertia']for r in rows)and all(r['axis_error']<1e-5 and r['origin_error_mm']<.02 for r in joint_rows)and all(r['world_bounds_error_mm']<.25 for r in mesh_checks)
(DATA/'urdf_independent_audit.json').write_text(json.dumps(result,indent=2),encoding='utf8')
print(json.dumps(result,indent=2))
assert result['passed']
