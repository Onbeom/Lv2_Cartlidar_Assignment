from pathlib import Path
import json, math, xml.etree.ElementTree as ET, types
import numpy as np
out=Path(__file__).resolve().parent.parent if Path(__file__).resolve().parent.name=='scripts'else Path(r'C:\Users\qjaqj\OneDrive\Desktop\Project2\261020\braket\submit\urdf')
data=out/'data' if (out/'data/ROBOT_REVIEW_PENDING.json').exists()else Path('Waffle_Handoff_20261020')
pkg=out/'waffle_4wd_description_261021';review=json.loads((data/'ROBOT_REVIEW_PENDING.json').read_text(encoding='utf8'))
def tensor(v):
    a,b,c,d,e,f=v
    return np.array([[a,d,e],[d,b,f],[e,f,c]])
def transform(node):
    T=np.eye(4)
    if node is None:return T
    r,p,y=[float(x)for x in node.get('rpy','0 0 0').split()];cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    T[:3,:3]=[[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]];T[:3,3]=[float(x)for x in node.get('xyz','0 0 0').split()]
    return T
input_urdf=next((pkg/'urdf').glob('*.urdf'));robot=ET.parse(input_urdf).getroot();F={'base_link':np.eye(4)}
for _ in range(14):
    for j in robot.findall('joint'):
        p=j.find('parent').get('link');c=j.find('child').get('link')
        if p in F:F[c]=F[p]@transform(j.find('origin'))
check=types.SimpleNamespace(review=review,F=F,by={l['name']:l for l in review['links']},tensor=tensor)
NS='http://www.ros.org/wiki/xacro';ET.register_namespace('xacro',NS)
old=pkg/'urdf/.urdf';new=pkg/'urdf/waffle_4wd.urdf'
if old.exists():old.rename(new)
old=pkg/'urdf/.urdf.xacro';wrapper=pkg/'urdf/waffle_4wd.urdf.xacro'
if old.exists():old.rename(wrapper)
refs={j['name']:j for j in check.review['joints']}
for path in [new,wrapper,pkg/'urdf/assemblies/main.urdf.xacro']:
    tree=ET.parse(path);root=tree.getroot();root.set('name','waffle_4wd')
    for link in root.iter('link'):
        name=link.get('name').replace('${prefix}','')
        if name not in check.by:continue
        ref=check.by[name];F=check.F[name];R=F[:3,:3]
        com=R.T@(np.array(ref['com_world_mm'])/1000-F[:3,3])
        I=R.T@check.tensor(ref['inertia_com_world_g_mm2'])@R*1e-9
        inert=link.find('inertial');inert.find('mass').set('value',format(ref['mass_g']/1000,'.17g'))
        origin=inert.find('origin');origin.set('xyz',' '.join(format(x,'.17g')for x in com));origin.set('rpy','0 0 0')
        for key,v in zip(['ixx','iyy','izz','ixy','ixz','iyz'],[I[0,0],I[1,1],I[2,2],I[0,1],I[0,2],I[1,2]]):inert.find('inertia').set(key,format(v,'.17g'))
    for joint in root.iter('joint'):
        name=joint.get('name','').replace('${prefix}','');ref=refs.get(name)
        if ref is None:continue
        if name=='gripper_right_joint':
            for old in list(joint.findall('mimic')):joint.remove(old)
            prefix='${prefix}'if '${prefix}'in joint.get('name')else ''
            ET.SubElement(joint,'mimic',joint=prefix+'gripper_left_joint',multiplier='-1',offset='0')
        limit=joint.find('limit')
        if limit is not None:
            # Visualization-only export: no unverified actuator force rating.
            limit.set('effort','0')
            limit.set('velocity',str(ref.get('proposed_start_velocity_rad_per_s',.01)))
            if ref['current_type']=='prismatic':
                for key,v in zip(['lower','upper'],ref['travel_limits_mm']):limit.set(key,format(v/1000,'.17g'))
    if path==wrapper:
        for old in list(root):
            if old.tag=='{'+NS+'}arg'and old.get('name')=='prefix':root.remove(old)
        root.insert(0,ET.Element('{'+NS+'}arg',name='prefix',default=''))
        for node in root:
            if node.tag=='{'+NS+'}main':node.set('prefix','$(arg prefix)')
    tree.write(path,encoding='utf-8',xml_declaration=True)
launch=pkg/'launch/display.launch.py';s=launch.read_text(encoding='utf8').replace("'urdf', '.urdf.xacro'","'urdf', 'waffle_4wd.urdf.xacro'").replace('Launch file for .','Launch file for waffle_4wd.')
compile(s,str(launch),'exec');launch.write_text(s,encoding='utf8')
robot=ET.parse(new).getroot()
supplement=dict(metadata=dict(design_name='waffle_4wd',package_name=pkg.name,total_mass_kg=check.review['total_mass_g']/1000,link_count=14,joint_count=13,root_link='base_link',units='SI',effort_status='0 means unconfigured; visualization only'))
supplement['links']={n.get('name'):dict(mass_kg=float(n.find('inertial/mass').get('value')),com_link_m=n.find('inertial/origin').get('xyz'),inertia_com_link_kg_m2=n.find('inertial/inertia').attrib)for n in robot.findall('link')}
supplement['joints']={n.get('name'):dict(type=n.get('type'),parent=n.find('parent').get('link'),child=n.find('child').get('link'),origin=n.find('origin').attrib,axis=n.find('axis').attrib if n.find('axis')is not None else None,limits=n.find('limit').attrib if n.find('limit')is not None else None,mimic=n.find('mimic').attrib if n.find('mimic')is not None else None)for n in robot.findall('joint')}
(pkg/'robot_data.yaml').write_text(json.dumps(supplement,ensure_ascii=False,indent=2),encoding='utf8')
state=pkg/'config/joint_state.yaml';s=state.read_text(encoding='utf8').split('    # Joints to publish')[0];state.write_text(s,encoding='utf8')
package_xml=pkg/'package.xml';tree=ET.parse(package_xml);tree.getroot().find('description').text='Waffle 4WD CAD-derived visualization description, 2026-10-21';tree.write(package_xml,encoding='utf8',xml_declaration=True)
for path in [pkg/'README.md',pkg/'config/transforms.md']:
    if not path.exists():continue
    s=path.read_text(encoding='utf8').replace('/.urdf','/waffle_4wd.urdf').replace('name=""','name="waffle_4wd"')
    path.write_text(s,encoding='utf8')
probes=json.loads((data/'modular_urdf_native_motion_probe.json').read_text(encoding='utf8'))
assert len(probes)==11 and max(r['error']for r in probes)<1e-5
for path in [data/'modular_urdf_export.json',out/'export_status.json']:
    r=json.loads(path.read_text(encoding='utf8'));r['native_motion_passed']=True;r['native_probe_normalization']='Actual Fusion-applied increment; original test raw data retained.';r['mimic_patched_files']=[str(new),str(pkg/'urdf/assemblies/main.urdf.xacro')];r['postprocessing']=['Nonempty robot and file names','Finger mimic in both URDF and xacro','Engineering-reference inertials at full precision','Effort=0 until hardware limits are configured'];path.write_text(json.dumps(r,ensure_ascii=False,indent=2),encoding='utf8')
print('Finalized robot name, xacro mimic, SI inertials and unconfigured effort limits.')
