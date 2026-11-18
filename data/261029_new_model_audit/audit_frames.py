from pathlib import Path
import json,hashlib,sys,xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from model_frames import ModelFrames
ROOT=Path('/home/pa4/git/lv2_next_module');M=ROOT/'261029_1030/urdf_261028';U=M/'waffle_4wd_description_261028/urdf/waffle_4wd.urdf';O=ROOT/'Cartlidar/sim_render/results/new_model_261029';pkg=U.parent.parent
sys.path.insert(0,str(ROOT/'Cartlidar/sim_world'));from robot_geom import Robot
sys.path.insert(0,str(ROOT/'Cartlidar/sim_render'));from urdf_chain import fixed_transform
f=ModelFrames(U);sensor_path=M/'isaac/urdf/sensors.json';sens=json.loads(sensor_path.read_text());params=json.loads((M/'isaac/robot_params.json').read_text());camera={k:v for k,v in sens['sensors'].items() if v['kind']=='camera'};missing=[]
for e in ET.parse(U).getroot().findall('.//mesh'):
 name=e.attrib['filename'];path=pkg/name.split('://',1)[1].split('/',1)[1] if name.startswith('package://') else (pkg/name)
 if not path.exists():missing.append(str(path))
rows=[];independent=[];rb=Robot(U,pkg)
def angle(R):return float(np.linalg.norm(Rotation.from_matrix(R).as_rotvec()))
for name,s in camera.items():
 T=f.transform(s['body'],s['frame'],params['poses']['drive']['value']);expected=np.eye(4);expected[:3,3]=s['pos'];expected[:3,:3]=Rotation.from_quat(s['quat_xyzw']).as_matrix();pe=float(np.linalg.norm(T[:3,3]-expected[:3,3]));ae=angle(T[:3,:3].T@expected[:3,:3]);rows.append(dict(sensor=name,frame=s['frame'],body=s['body'],position_error_m=pe,angle_error_rad=ae,pass_metadata=pe<=2e-6 and ae<=5e-6,T_body_optical=T.tolist()))
for pose in ('drive','stow','zero'):
 q=params['poses'][pose]['value'];a=f.poses(q);b=rb.link_poses(q)
 for name,s in camera.items():
  pe=float(np.linalg.norm(a[s['frame']][:3,3]-b[s['frame']][:3,3]));ae=angle(a[s['frame']][:3,:3].T@b[s['frame']][:3,:3]);roundtrip=np.linalg.inv(a['base_footprint'])@a[s['frame']];Tback=np.linalg.inv(roundtrip);err=float(np.max(abs(roundtrip@Tback-np.eye(4))))
  independent.append(dict(pose=pose,sensor=name,position_error_m=pe,angle_error_rad=ae,roundtrip_error=err,pass_fk=pe<=1e-9 and ae<=1e-7 and err<=1e-12))
compat={}
try:fixed_transform(U,'top_camera_front_color_optical_frame');compat['old_fixed_transform']='unexpected_success'
except (KeyError,ValueError) as e:compat['old_fixed_transform']=dict(fails=True,error_type=type(e).__name__,reason=str(e))
try:f.poses({'RMX52_Base_Yaw':0});compat['old_joint_rejected']=False
except ValueError:compat['old_joint_rejected']=True
compat['old_arm_camera_frame_exists']='camera_optical_frame' in f.links;compat['new_arm_camera_frame_exists']='arm_camera_optical_frame' in f.links
provenance={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [U,sensor_path,M/'isaac/robot_params.json',M/'isaac/usd/full/waffle_4wd_full/waffle_4wd_full.usda',M/'isaac/usd/light/waffle_4wd_light/waffle_4wd_light.usda']}
result=dict(model=str(M),root=f.root,link_count=len(f.links),joint_count=len(f.joints),camera_count=len(camera),missing_meshes=missing,metadata=rows,independent_fk=independent,compatibility=compat,provenance=provenance,pass_preflight=not missing and all(x['pass_metadata'] for x in rows) and all(x['pass_fk'] for x in independent),note='정적 인지TF점검만. 실제보정/카메라가시성/주행성공 아님')
(O/'frame_audit.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
profile=dict(urdf=str(U),package_dir=str(pkg),source_sha256=provenance[str(U.relative_to(ROOT))],root_link=f.root,ground_frame='base_footprint',root_height_m=sens['root_height_m'],cameras={k:dict(frame=v['frame'],body=v['body']) for k,v in camera.items()},joint_names=sorted(f.joints),preview_pose=params['poses']['drive']['value'],label='모의 모델 TF 프로필, 실물 보정 파일 아님')
(O/'perception_model_profile.json').write_text(json.dumps(profile,indent=2,ensure_ascii=False)+'\n')
print(json.dumps(dict(root=f.root,links=len(f.links),joints=len(f.joints),cameras=len(camera),missing_meshes=len(missing),metadata_ok=sum(x['pass_metadata'] for x in rows),independent_fk_ok=sum(x['pass_fk'] for x in independent),metadata_max_position_error_m=max(x['position_error_m'] for x in rows),metadata_max_angle_error_rad=max(x['angle_error_rad'] for x in rows),compatibility=compat,pass_preflight=result['pass_preflight']),ensure_ascii=False))
