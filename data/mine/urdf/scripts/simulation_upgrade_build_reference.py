"""Build an in-place URDF upgrade candidate, preserving CAD and inertials."""
from pathlib import Path
import sys,json,math,copy,collections,shutil,xml.etree.ElementTree as ET
import numpy as np
sys.path.insert(0,str(Path('CADai_preparation').resolve()))
from portable_fk import Robot,origin
ROOT=Path(r'C:\Users\qjaqj\OneDrive\Desktop\Project2\261020\braket\submit\urdf');PKG=ROOT/'waffle_4wd_description_261021';WORK=Path('sim_work');CAND=WORK/'candidate';CAND.mkdir(exist_ok=True)
robot=Robot(PKG/'urdf/waffle_4wd.urdf');F={n:np.array(T)for n,T in robot.fk().items()};tree=ET.parse(PKG/'urdf/waffle_4wd.urdf');root=tree.getroot()
review=json.loads((ROOT/'data/ROBOT_REVIEW_PENDING.json').read_text(encoding='utf8'));physics=json.loads((ROOT/'data/PHYSICS_ESTIMATE_G_MM.json').read_text(encoding='utf8'))['bodies']
refs={j['name']:j for j in review['joints']};links={l.get('name'):l for l in root.findall('link')};decomp=json.loads((WORK/'decomposition.json').read_text(encoding='utf8'));components=json.loads((WORK/'mesh_components.json').read_text(encoding='utf8'))
def fmt(v):return ' '.join(format(float(x),'.17g')for x in v)
def rpy(R):
    p=math.asin(max(-1,min(1,-R[2,0])))
    if abs(math.cos(p))>1e-8:return [math.atan2(R[2,1],R[2,2]),p,math.atan2(R[1,0],R[0,0])]
    return [0,p,math.atan2(-R[0,1],R[1,1])]
def add_origin(node,T):ET.SubElement(node,'origin',xyz=fmt(T[:3,3]),rpy=fmt(rpy(T[:3,:3])))
def Tframe(p,R=None):
    T=np.eye(4);T[:3,3]=np.array(p)/1000
    if R is not None:T[:3,:3]=R
    return T
def forwardR(forward):
    f=np.array(forward,dtype=float);f/=np.linalg.norm(f);left=np.cross([0,0,1],f);left/=np.linalg.norm(left);up=np.cross(f,left);return np.column_stack((f,left,up))
def fixed(name,parent,worldT,basis=None):
    ET.SubElement(root,'link',name=name)
    j=ET.SubElement(root,'joint',name=name+'_fixed',type='fixed');ET.SubElement(j,'parent',link=parent);ET.SubElement(j,'child',link=name)
    add_origin(j,np.linalg.inv(F[parent]if basis is None else basis)@worldT);F[name]=worldT
def box(link,label,bounds_world_mm):
    lo,hi=np.array(bounds_world_mm).reshape(2,3)
    # F from portable FK has translations in mm; normalize that convention here.
    parent=F[link].copy();parent[:3,3]/=1000;T=np.linalg.inv(parent)@Tframe((lo+hi)/2)
    c=ET.SubElement(links[link],'collision',name=label);add_origin(c,T);g=ET.SubElement(c,'geometry');ET.SubElement(g,'box',size=fmt(np.maximum((hi-lo)/1000,.0002)))
def hulls(link,row):
    for i,name in enumerate(row['files']):
        c=ET.SubElement(links[link],'collision',name=f"convex_{row['component_index']}_{i}");ET.SubElement(c,'origin',xyz='0 0 0',rpy='0 0 0');g=ET.SubElement(c,'geometry');ET.SubElement(g,'mesh',filename='package://'+PKG.name+'/meshes/collision/'+name,scale='1 1 1')
for link in links.values():
    for c in list(link.findall('collision')):link.remove(c)
covered={(r['link'],r['component_index'])for r in decomp}
for row in decomp:hulls(row['link'],row)
G=collections.defaultdict(list)
for p in physics:
    if p['link']=='base_link':G[p['path'].split('+')[0]].append(p)
skip=[r['source_path'].split('+')[0]for r in decomp if r['link']=='base_link']
for name,rows in G.items():
    if name in skip:continue
    lo=np.min([x['bounds']['min']for x in rows],0);hi=np.max([x['bounds']['max']for x in rows],0)
    if max(hi-lo)<3:continue
    box('base_link','static_'+str(len(links['base_link'].findall('collision'))),np.r_[lo,hi])
for row in components:
    link=row['link']
    if link.startswith('wheel_'):
        d=np.load(WORK/(link+'.npz'));v=d['vertices'];lo=v.min(0);hi=v.max(0);center=(lo+hi)/2
        radius=float(np.max(np.linalg.norm(v[:,:2]-center[:2],axis=1)))
        c=ET.SubElement(links[link],'collision',name='tire_cylinder');ET.SubElement(c,'origin',xyz=fmt(center),rpy='0 0 0');g=ET.SubElement(c,'geometry');ET.SubElement(g,'cylinder',radius=str(radius),length=str(hi[2]-lo[2]))
    elif link.startswith('lidar_'):
        rows=[x['bounds']for x in physics if x['link']==link];box(link,'lidar_carrier_envelope',np.r_[np.min([x['min']for x in rows],0),np.max([x['max']for x in rows],0)])
    elif link.startswith('arm_'):
        for p in row['components']:
            if p['volume_mm3']>1000:box(link,'part_'+str(p['index']),p['bounds_world_mm'])
for j in root.findall('joint'):
    name=j.get('name');ref=refs[name];limit=j.find('limit')
    if name.startswith('RMX52_'):
        j.set('type','revolute')
        for key,v in zip(['lower','upper'],ref['proposed_limits_rad']):limit.set(key,str(v))
        limit.set('effort','3.8');limit.set('velocity','0.5')
    elif name.startswith('Wheel_'):limit.set('effort','2.7');limit.set('velocity','2.0')
    elif name.startswith('gripper_'):limit.set('effort','10');limit.set('velocity','0.01')
    # Passive model damping is a simulation starting assumption.
    if j.get('type')!='fixed':ET.SubElement(j,'dynamics',damping='0.01'if name.startswith('RMX52_')else '0.001',friction='0')

# F currently uses mm; normalize for TF construction below.
for T in F.values():T[:3,3]/=1000
sensor_meta=[]
old_cam=json.loads(Path('Waffle_Handoff_20261020/camera_installed.json').read_text(encoding='utf8'));a=np.array(old_cam['optical_axis']);pcb=np.array(old_cam['pcb_back_center_world_mm'])+[-12.5,0,0]
lens=pcb+(30.756377471373103-5.75637747137313)*a;camT=Tframe(lens,forwardR(a))
fixed('camera_link','arm_link_5',camT)
optR=np.array([[0,0,1],[-1,0,0],[0,-1,0]]);optT=camT.copy();optT[:3,:3]=camT[:3,:3]@optR;fixed('camera_optical_frame','camera_link',optT)
sensor_meta.append(dict(frame='camera_optical_frame',home_world_mm=lens.tolist(),forward_world=a.tolist(),basis='UC684 CAD front lens face center and optical axis, not calibrated entrance pupil',calibration_verified=False))
cam=json.loads(Path('Waffle_Assembly_Build/Oct19/camera.json').read_text(encoding='utf8'));native=json.loads(Path('Waffle_Handoff_20261020/body_physics_geometry.json').read_text(encoding='utf8'))
for p in cam['parts']:
    if not p['name'].startswith('D435')or 'cover'in p['name']:continue
    body=next(x for x in native if x['path'].endswith('+'+p['name']));M=np.array(body['transform_cm']).reshape(4,4);M[:3,3]*=10
    lens_cyl=[c for c in p['bodies'][0]['cylinders']if abs(c['radius']-.7995)<1e-4][:2]
    point=np.mean([c['origin']for c in lens_cyl],0)*10;pos=M[:3,:3]@point+M[:3,3];f=M[:3,:3]@np.array(lens_cyl[0]['axis']);front=pos[0]>-140.5
    if (f[0]>0)!=front:f=-f
    name='top_camera_front'if front else 'top_camera_rear';CT=Tframe(pos,forwardR(f));fixed(name+'_link','base_link',CT);OT=CT.copy();OT[:3,:3]=CT[:3,:3]@optR;fixed(name+'_optical_frame',name+'_link',OT)
    sensor_meta.append(dict(frame=name+'_optical_frame',home_world_mm=pos.tolist(),forward_world=f.tolist(),basis='CAD stereo lens cylinder midpoint, not calibrated RGB optical center',calibration_verified=False))
for side,x,y,yaw in [('front',185.5,50,0),('rear',-185.5,-50,math.pi)]:
    R=np.array([[math.cos(yaw),-math.sin(yaw),0],[math.sin(yaw),math.cos(yaw),0],[0,0,1]])
    fixed('lidar_'+side+'_scan','lidar_'+side,Tframe([x,y,60.85],R))
    sensor_meta.append(dict(frame='lidar_'+side+'_scan',home_world_mm=[x,y,60.85],basis='CAD scan shaft XY; Z midpoint of earlier assumed scan band 54.7..67mm; yaw outward, uncalibrated',calibration_verified=False))
wheel_bottom=min(x['bounds']['min'][2]for x in physics if x['link'].startswith('wheel_'));ground_offset=-wheel_bottom
ET.SubElement(root,'link',name='base_footprint');j=ET.SubElement(root,'joint',name='base_footprint_to_base_link',type='fixed');ET.SubElement(j,'parent',link='base_footprint');ET.SubElement(j,'child',link='base_link');ET.SubElement(j,'origin',xyz=fmt([0,0,ground_offset/1000]),rpy='0 0 0')
tree.write(CAND/'waffle_4wd.urdf',encoding='utf8',xml_declaration=True)
NS='http://www.ros.org/wiki/xacro';ET.register_namespace('xacro',NS);mr=ET.Element('robot',name='waffle_4wd');macro=ET.SubElement(mr,'{'+NS+'}macro',name='main',params="prefix:=''")
for node in list(root):
    if node.tag not in ['link','joint']:continue
    n=copy.deepcopy(node)
    for e in n.iter():
        if e.tag in ['link','joint']:e.set('name','${prefix}'+e.get('name'))
        elif e.tag in ['parent','child']:e.set('link','${prefix}'+e.get('link'))
        elif e.tag=='mimic':e.set('joint','${prefix}'+e.get('joint'))
    macro.append(n)
ET.ElementTree(mr).write(CAND/'main.urdf.xacro',encoding='utf8',xml_declaration=True)
meta=dict(status='SIMULATION_INPUTS_ADDED_RUNTIME_VALIDATION_PENDING',motor_voltage_nominal_V=11.1,power={'opencr_1':'one TurtleBot 11.1V nominal battery','opencr_2':'one separate TurtleBot 11.1V nominal battery','jetson':'separate 12V supply; does not define motor voltage'},arm_model='XM430-W350',wheel_model='XM430-W210',arm_peak_effort_N_mm=3800,wheel_peak_effort_N_mm=2700,effort_basis='Manufacturer 11.1V stall torque: peak envelope, NOT continuous/thermal safe rating.',finger_equivalent_effort_N=10,finger_effort_basis='Simulation assumption per finger, not derived motor torque or validated grip force.',limits_basis='Previous sign-corrected reference intervals, not calibrated hardware stops.',ground_offset_mm=ground_offset,root='base_footprint',links=len(root.findall('link')),joints=len(root.findall('joint')),sensor_frames=sensor_meta,collision=dict(primitives=len(root.findall('.//collision/geometry/box'))+len(root.findall('.//collision/geometry/cylinder')),convex_meshes=len(root.findall('.//collision/geometry/mesh')),threshold_mm=1,limitations='Simplified approximate contacts; tiny fasteners/internal geometry omitted. Runtime collision filtering and grasp clearance require validation.'),sources=['https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/','https://emanual.robotis.com/docs/en/dxl/x/xm430-w210/'],isaac_runtime_tested=False,ros_runtime_tested=False)
(CAND/'simulation_metadata.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2),encoding='utf8')
print(json.dumps(meta,ensure_ascii=False,indent=2))
