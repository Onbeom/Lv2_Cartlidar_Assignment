import sys,json,math,ast
from pathlib import Path
import numpy as np,yaml
ROOT=Path('/home/pa4/git/lv2_next_module');MODEL=ROOT/'261029_1030/urdf_261028';URDF=MODEL/'waffle_4wd_description_261028/urdf/waffle_4wd.urdf';PKG=URDF.parent.parent;USD=MODEL/'isaac/usd/full/waffle_4wd_full/waffle_4wd_full.usda';OUT=ROOT/'Cartlidar/sim_render/results/a1_new_model_261029'
sys.path.insert(0,str(ROOT/'261029_new_model_audit'));from model_frames import ModelFrames
FRAMES=ModelFrames(URDF);SENSORS=json.loads((MODEL/'isaac/urdf/sensors.json').read_text());PARAMS=json.loads((MODEL/'isaac/robot_params.json').read_text());Q=PARAMS['poses']['drive']['value'];SPEC=yaml.safe_load((PKG/'config/sensors.yaml').read_text());CAM={}
for name,s,frame in [('d435',SPEC['d435']['sim'],'top_camera_front_color_optical_frame'),('arm',SPEC['arm_camera']['sim'],'arm_camera_optical_frame')]:
 w,h=s['width'],s['height'];fx=w/(2*math.tan(s.get('color_hfov_rad',s.get('hfov_rad'))/2));CAM[name]=dict(width=w,height=h,K=dict(fx=fx,fy=fx,cx=w/2,cy=h/2),frame=frame,near=.105 if name=='d435' else .02)
# 기존 채점의 표면점·가림 정의를 바꾸지 않고 해상도별로 사용.
tree=ast.parse((ROOT/'261028_1459/render_a1_view.py').read_text());funcs=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in ('puck_surface','cam_points','in_frac','truth_box','project')]
for c in CAM.values():
 ns=dict(np=np,math=math,HALF_XY=.015,HALF_Z=.03,W=c['width'],H=c['height']);exec(compile(ast.Module(body=funcs,type_ignores=[]),'<truth geometry>','exec'),ns);c['geometry']=ns

def base_pose(x,y,yaw):
 T=np.eye(4);T[:3,:3]=[[math.cos(yaw),-math.sin(yaw),0],[math.sin(yaw),math.cos(yaw),0],[0,0,1]];T[:3,3]=[x,y,SENSORS['root_height_m']];return T
