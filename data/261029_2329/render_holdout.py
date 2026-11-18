"""같은 URDF/USD 카메라의 새 모델 near·로봇숨김 대조. 원본 모델 저장/수정 없음."""
from pathlib import Path
import json,sys,hashlib,os
import numpy as np
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'Cartlidar/sim_render'))
meta=json.loads((root/'Cartlidar/sim_render/results/mission_261028/render_r12_gpu/render_r12.json').read_text());pose=meta['poses'][0]
model=root/'261029_1030/urdf_261028'; urdf=model/'waffle_4wd_description_261028/urdf/waffle_4wd.urdf'; model_usd=model/'isaac/usd/full/waffle_4wd_full/waffle_4wd_full.usda'; sensors=json.loads((model/'isaac/urdf/sensors.json').read_text()); params=json.loads((model/'isaac/robot_params.json').read_text()); pose_name=os.environ['SELF_FILTER_POSE']; q=params['poses'][pose_name]['value']; meta['urdf']=str(urdf)
out=root/'Cartlidar/sim_render/results/followup_261028_1457'/('camera_new_model_'+pose_name+'_holdout');out.mkdir(exist_ok=False)
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':640,'height':480,'anti_aliasing':0,'multi_gpu':False})
import omni.usd,omni.replicator.core as rep
from pxr import Gf,Usd,UsdGeom
import cv2
from robot_usd import use_robot_usd,pose_robot,ROBOT
from urdf_chain import fixed_transform
ctx=omni.usd.get_context();ctx.open_stage(str(Path(pose['world'])/'task_world.usda'))
def updates(n):
 for _ in range(n):app.update()
updates(60);stage=ctx.get_stage();usd=use_robot_usd(stage, model_usd);updates(10)
from robot_usd import set_xform
T_w_base=np.array(pose['T_world_base_footprint']);T_w_base=T_w_base @ np.array([[1,0,0,0],[0,1,0,0],[0,0,1,sensors['root_height_m']],[0,0,0,1]])
set_xform(stage.GetPrimAtPath(ROBOT),T_w_base)
fk_checks=[]
for link,path in sorted(sensors['moving_bodies'].items(),key=lambda item:item[1].count('/')):
 prim=stage.GetPrimAtPath(ROBOT+'/'+path); parent=prim.GetParent().GetName()
 if not prim.IsValid():raise RuntimeError('USD 링크 없음: '+path)
 T0=np.linalg.inv(fixed_transform(str(urdf),parent,root='base_link',q={})) @ fixed_transform(str(urdf),link,root='base_link',q={})
 actual=np.array(UsdGeom.Xformable(prim).GetLocalTransformation()).T
 fk_checks.append({'link':link,'zero_local_max_abs_error':float(np.max(np.abs(T0-actual)))})
 if np.max(np.abs(T0-actual))>1e-4:raise RuntimeError('URDF/USD 기본 TF 불일치: '+link)
 T=np.linalg.inv(fixed_transform(str(urdf),parent,root='base_link',q=q)) @ fixed_transform(str(urdf),link,root='base_link',q=q)
 set_xform(prim,T)
updates(10)
robot=UsdGeom.Imageable(stage.GetPrimAtPath(ROBOT));camera=UsdGeom.Camera.Define(stage,'/World/CameraAudit/camera')
camera.CreateHorizontalApertureAttr(20.955);camera.CreateVerticalApertureAttr(20.955*480/640)
rp=rep.create.render_product(str(camera.GetPath()),(640,480));rgb=rep.AnnotatorRegistry.get_annotator('rgb');dep=rep.AnnotatorRegistry.get_annotator('distance_to_image_plane');rgb.attach(rp);dep.attach(rp);rows=[]
for side in ('front','rear'):
 for sensor in ('color','infra1','infra2'):
  frame=f'top_camera_{side}_{sensor}_optical_frame';T=T_w_base@fixed_transform(str(urdf),frame,root='base_link',q=q);spec=sensors['spec']['d435'];hfov=spec['color_hfov_rad'] if sensor=='color' else spec['ir_hfov_rad'];fx=640/(2*np.tan(hfov/2));K={'fx':float(fx),'fy':float(fx),'cx':320.,'cy':240.};f=K['fx']*20.955/640
  camera.CreateFocalLengthAttr(f);camera.CreateVerticalApertureAttr(f*480/K['fy']);camera.CreateHorizontalApertureOffsetAttr(-(K['cx']-320)/640*20.955);camera.CreateVerticalApertureOffsetAttr((K['cy']-240)/640*20.955)
  R,t=T[:3,:3],T[:3,3];xf=UsdGeom.Xformable(camera);xf.ClearXformOpOrder();xf.AddTransformOp().Set(Gf.Matrix4d(*R[:,0].tolist(),0,*(-R[:,1]).tolist(),0,*(-R[:,2]).tolist(),0,*t.tolist(),1))
  for label,near,visible,double in [('specified_near105mm',.105,True,False),('hidden_near105mm',.105,False,False)]:
   if visible:robot.MakeVisible()
   else:robot.MakeInvisible()
   camera.CreateClippingRangeAttr(Gf.Vec2f(near,10.));updates(12)
   for _ in range(3):rep.orchestrator.step(rt_subframes=8)
   image=np.asarray(rgb.get_data())[:,:,:3].astype(np.uint8);depth=np.asarray(dep.get_data()).astype(np.float32);name=frame+'_'+label;cv2.imwrite(str(out/(name+'.png')),cv2.cvtColor(image,cv2.COLOR_RGB2BGR));np.save(out/(name+'_depth.npy'),depth);finite=np.isfinite(depth)&(depth>0)
   row={'frame':frame,'K':K,'T_world_cam':T.tolist(),'case':label,'near_m':near,'visible':visible,'forced_double_sided':double,'finite_fraction':float(finite.mean()),'under10mm_fraction':float((finite&(depth<.01)).mean()),'under150mm_fraction':float((finite&(depth<.15)).mean()),'depth_percentile_m':np.percentile(depth[finite],[0,1,50,99,100]).tolist() if finite.any() else None,'center_depth_m':float(depth[240,320]) if np.isfinite(depth[240,320]) else None,'image':name+'.png','depth':name+'_depth.npy'};rows.append(row);print(json.dumps(row,ensure_ascii=False),flush=True)
robot.MakeVisible();rgb.detach();dep.detach();rp.destroy();updates(4)
(out/'summary.json').write_text(json.dumps({'urdf':meta['urdf'],'urdf_sha256':hashlib.sha256(Path(meta['urdf']).read_bytes()).hexdigest(),'usd':usd,'pose':pose_name,'arm_q':q,'root':'base_link','root_height_m':sensors['root_height_m'],'fk_checks':fk_checks,'K_source':'새 sensors.json HFOV의 중심 pinhole K, 실물 보정값 아님','rows':rows,'note':'정적 렌더 대조; 원본 모델 수정/저장 없음'},ensure_ascii=False,indent=2));app.close()
